#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale, double clearance_scale,
                               double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , clearance_scale_(clearance_scale)
  , min_frontier_size_(min_frontier_size)
{
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // todo:在这里判断是否可达，然后就选择第一个可到达点就彳亍
        // 先修改frontier的结构，加入 可达点 和 可达标识位

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;

  // // ① 将 world 坐标转换为 map 索引
  // unsigned int mx, my;
  // if (costmap_->worldToMap(output.centroid.x, output.centroid.y, mx, my)) {
  //   int bad_count = 0;
  //   int total_check = 0;
  //   // 新增变量：记录最安全的格子
  //   int best_score = -1;
  //   unsigned int best_nx = mx, best_ny = my;

  //   // ② 检查 centroid 点周围 5x5 区域
  //   for (int dx = -2; dx <= 2; ++dx) {
  //     for (int dy = -2; dy <= 2; ++dy) {
  //       int nx = mx + dx;
  //       int ny = my + dy;
  //       if (nx >= 0 && ny >= 0 &&
  //           nx < static_cast<int>(costmap_->getSizeInCellsX()) &&
  //           ny < static_cast<int>(costmap_->getSizeInCellsY())) {
  //         unsigned char cost = costmap_->getCost(nx, ny);

  //         if (cost >= costmap_2d::LETHAL_OBSTACLE) {
  //           bad_count++;
  //         } else {
  //           // 非障碍区域，计分：得分 = 周围9格非障碍数（越大越好）
  //           int score = 0;
  //           for (int ddx = -1; ddx <= 1; ++ddx) {
  //             for (int ddy = -1; ddy <= 1; ++ddy) {
  //               int ex = nx + ddx;
  //               int ey = ny + ddy;
  //               if (ex >= 0 && ey >= 0 &&
  //                   ex < static_cast<int>(costmap_->getSizeInCellsX()) &&
  //                   ey < static_cast<int>(costmap_->getSizeInCellsY())) {
  //                 unsigned char ecost = costmap_->getCost(ex, ey);
  //                 if (ecost < costmap_2d::LETHAL_OBSTACLE) score++;
  //               }
  //             }
  //           }
  //           // 记录分数最高的格子
  //           if (score > best_score) {
  //             best_score = score;
  //             best_nx = nx;
  //             best_ny = ny;
  //           }
  //         }

  //         total_check++;
  //       }
        
  //     }
  //   }

  //   // ③ 如果 70% 以上是障碍物，认为被包围
  //   if ((double)bad_count / total_check > 0.7) {
  //     // ROS_WARN("Frontier rejected: middle point appears to be in cluttered/obstacle-surrounded area.");
  //     // output.can_achieve = false;
  //     // return output;

  //     if (best_score >= 0) {
  //       double new_x, new_y;
  //       costmap_->mapToWorld(best_nx, best_ny, new_x, new_y);
  //       output.centroid.x = new_x;
  //       output.centroid.y = new_y;
  //       ROS_WARN("centroid changed to safe");
  //     } else {
  //       output.can_achieve = false;
  //       ROS_WARN("centroid is unsafe");
  //     }
  //   }
  // }

  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}
