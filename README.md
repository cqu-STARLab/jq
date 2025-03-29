jq项目

启动顺序：
roslaunch simple_lio lio.launch （建图与雷达驱动）
roslaunch scout_navigation scout_move_base.launch （规划与底盘驱动）
roslaunch explore_lite explore.launch （导航和探索）

导航和探索 需要 单独给 explore.launch 发送topic，才能开始。
导航给定map坐标系下的目标点。
探索给定区域左下角和右上角。
