#include <ros/ros.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include <ctime>
#include <chrono>



using namespace std;

//城市参数初始化
std::vector<int> city_x;
std::vector<int> city_y;
std::vector<int> city_p0;
std::vector<int> city_p1;
std::vector<int> city_s;
//小车参数初始化
std::vector<int> car_x;
std::vector<int> car_y;
std::vector<int> car_p;
std::vector<int> car_q;

std::vector<int> car_q_real(12,0);
struct Point2D {
    float x;
    float y;
};
std::vector<std::vector<Point2D>> city_vertex=
{{{0,0},{3,5}},{{1,2},{4,7}},{{2,1},{5,6}},{{3,3},{6,8}},
 {{4,0},{7,5}},{{5,2},{8,7}},{{6,1},{9,6}},{{7,3},{10,8}},
 {{8,0},{11,5}},{{9,2},{13,7}},{{10,1},{13,5}},{{11,3},{13,5}},
 {{12,0},{15,5}},{{13,2},{16,7}},{{14,1},{17,6}},{{15,3},{18,8}},
 {{16,0},{19,5}},{{17,2},{20,7}},{{18,1},{20,5}},{{19,3},{22,5}},
};

//任务调度参数初始化
float c1 = 0.5;
float c2 = 0.3;
float cost = 0;
int goal[12];
int cost_w[6][6];
int cost_L[6][6];
int count_w = 0;
int count_L = 0;
int count_w2 = 0;
int count_L2 = 0;
int n;//元素个数
int *assign;//分配结果
int **mat;//代价矩阵
int **matRcd;//代价矩阵，计算更改
int totalCost;//总成本
std::vector<int> task_point;
bool param_loaded = false;

//矩阵行归约
void rowSub()
{
	int *minEmt=new int[n];for(int i=0;i<n;i++)minEmt[i]=(int)1e8;

	for(int i=0;i<n;i++)for(int j=0;j<n;j++)if(mat[i][j]<minEmt[i])minEmt[i]=mat[i][j];

	for(int i=0;i<n;i++)for(int j=0;j<n;j++)mat[i][j]-=minEmt[i];
            // 输出mat矩阵的值行归约
    // std::cout << "行归约后Matrix mat:" << std::endl;
    // for (int i = 0; i < n; ++i) {
    //     for (int j = 0; j < n; ++j) {
    //         std::cout << mat[i][j] << " ";
    //     }
    //     std::cout << std::endl; // 每行后换行
    // }


	delete []minEmt;
}

//矩阵列归约
void columnSub()
{
	int *minEmt=new int[n];for(int j=0;j<n;j++)minEmt[j]=(int)1e8;

	for(int j=0;j<n;j++)for(int i=0;i<n;i++)if(mat[i][j]<minEmt[j])minEmt[j]=mat[i][j];

	for(int j=0;j<n;j++)for(int i=0;i<n;i++)mat[i][j]-=minEmt[j];
    //             // 输出mat矩阵的值列归约
    // std::cout << "列归约后Matrix mat:" << std::endl;
    // for (int i = 0; i < n; ++i) {
    //     for (int j = 0; j < n; ++j) {
    //         std::cout << mat[i][j] << " ";
    //     }
    //     std::cout << std::endl; // 每行后换行
    // }

	delete []minEmt;
    
}

//检验最优
bool isOptimal(int *assign)
{

	int *tAssign=new int[n];for(int i=0;i<n;i++)tAssign[i]=-1;
	int *nZero=new int[n];
	bool *rowIsUsed=new bool[n];
	bool *columnIsUsed=new bool[n];
	for(int i=0;i<n;i++)rowIsUsed[i]=columnIsUsed[i]=0;

	int nLine=0;
	while(nLine<n){
		for(int i=0;i<n;i++)nZero[i]=0;
		for(int i=0;i<n;i++){
			if(rowIsUsed[i]==1)continue;
			for(int j=0;j<n;j++){
				if(columnIsUsed[j]!=1&&mat[i][j]==0)nZero[i]++;
			}
		}

		int minZeros=n;
		int rowId=-1;
		for(int i=0;i<n;i++){
			if(rowIsUsed[i]==0&&nZero[i]<minZeros&&nZero[i]>0){
				minZeros=nZero[i];
				rowId=i;
			}
		}
		if(rowId==-1)break;
		for(int j=0;j<n;j++){
			if(mat[rowId][j]==0&&columnIsUsed[j]==0){
				rowIsUsed[rowId]=1;
				columnIsUsed[j]=1;
				tAssign[rowId]=j;
				break;
			}
		}
		nLine++;
	}
	for(int i=0;i<n;i++)assign[i]=tAssign[i];
	delete []tAssign;
	delete []nZero;
	delete []rowIsUsed;
	delete []columnIsUsed;

	for(int i=0;i<n;i++)if(assign[i]==-1)return false;
	return true;
}

//矩阵变换
void matTrans()
{
	bool *rowTip=new bool[n];
	bool *columnTip=new bool[n];
	bool *rowLine=new bool[n];
	bool *columnLine=new bool[n];
	for(int i=0;i<n;i++)rowTip[i]=columnTip[i]=rowLine[i]=columnLine[i]=0;

	//标记
	for(int i=0;i<n;i++)if(assign[i]==-1)rowTip[i]=1;

	while(1){
		int preTip=0;
		for(int i=0;i<n;i++)preTip=preTip+rowTip[i]+columnTip[i];
		for(int i=0;i<n;i++){
			if(rowTip[i]==1){
				for(int j=0;j<n;j++){
					if(mat[i][j]==0)columnTip[j]=1;
				}
			}
		}
		for(int j=0;j<n;j++){
			if(columnTip[j]==1){
				for(int i=0;i<n;i++){
					if(assign[i]==j)rowTip[i]=1;
				}
			}
		}
		int curTip=0;
		for(int i=0;i<n;i++)curTip=curTip+rowTip[i]+columnTip[i];
		if(preTip==curTip)break;
	}
	
	//
	for(int i=0;i<n;i++){
		if(rowTip[i]==0)rowLine[i]=1;
		if(columnTip[i]==1)columnLine[i]=1;
	}

	//找最小
	int minElmt=(int)1e8;
	for(int i=0;i<n;i++)for(int j=0;j<n;j++)if(rowLine[i]==0&&columnLine[j]==0&&mat[i][j]<minElmt)minElmt=mat[i][j];
	//变换
	for(int i=0;i<n;i++)if(rowTip[i]==1)for(int j=0;j<n;j++)mat[i][j]-=minElmt;
	for(int j=0;j<n;j++)if(columnTip[j]==1)for(int i=0;i<n;i++)mat[i][j]+=minElmt;

	delete []rowTip;
	delete []columnTip;
	delete []rowLine;
	delete []columnLine;
}

//算法求解
void compute(const int Num,const int values[][6])
{
    //分配结果
    n=Num;
    assign=new int[n];for(int i=0;i<n;i++)assign[i]=-1;
    //
    // int values[n][n]={  { 20, 30, 40,40,3},
    //     {15, 25, 35, 45,6},
    //     {10, 20, 30, 50,9},
    //     {5, 15, 25, 35,20},
    //     {25, 5, 15, 5,20}};
    // int values[n][n] = {
    // {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120},
    // {15, 25, 35, 45, 55, 65, 75, 85, 95, 105, 115},
    // {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110},
    // {5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 105},
    // {25, 5, 15, 25, 35, 45, 55, 65, 75, 85, 95},
    // {40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40},
    // {35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35},
    // {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30},
    // {25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25},
    // {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20},
    // {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15},
    // {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}};

	mat=new int*[n];for(int i=0;i<n;i++)mat[i]=new int[n];

	matRcd=new int*[n];for(int i=0;i<n;i++)matRcd[i]=new int[n];
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++){
			mat[i][j]=values[i][j];
			matRcd[i][j]=mat[i][j];
		}
	}    // city_y = {76,85,62,54,12,32,54,76,94,10,23,24,26,33,45,74,82,55,43,66};

    // for (int i = 0; i < n; ++i) {
    //     for (int j = 0; j < n; ++j) {
    //         std::cout << mat[i][j] << " ";
    //     }
    //     std::cout << std::endl; // 每行后换行
    // }

	
	// read("cost.dat");//读取数据
	rowSub();//行归约
	columnSub();//列归约

	//如果不能找到n个独立的0元素，则对矩阵进行变换
	while(!isOptimal(assign)){
		matTrans();
	}

	for(int i=0;i<n;i++)totalCost+=matRcd[i][assign[i]];
	for(int i=0;i<n;i++)delete []mat[i];delete []mat;
	for(int i=0;i<n;i++)delete []matRcd[i];delete []matRcd;
}

int main(int argc, char *argv[])
{
    //----------------------------------------------------------------第一次启动协同任务调度计算节点

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"cost");
    ros::NodeHandle nh;

    // 城市参数载入
    nh.getParam("city_x",city_x); 
    nh.getParam("city_y",city_y); 
    nh.getParam("city_p0",city_p0); 
    nh.getParam("city_p1",city_p1); 
    // nh.getParam("city_s",city_s); 
    std::cout<<"ok go on"<<endl;
    // 小车参数载入
    nh.getParam("car_x",car_x); 
    nh.getParam("car_y",car_y); 
    nh.getParam("car_p",car_p); 
    nh.getParam("car_q",car_q); 
    // 
    for (const auto& rectangle : city_vertex){
    if (rectangle.size() == 2) {
        const Point2D& bottomLeft = rectangle[0];
        const Point2D& topRight = rectangle[1];
        int area = static_cast<int>((topRight.x - bottomLeft.x) * (topRight.y - bottomLeft.y));
        city_s.push_back(area);
    }
    }

    //判断是否成功加载参数服务器中的参数
    while (!param_loaded && ros::ok())
    {
        // 尝试从参数服务器获取参数
        if (nh.getParam("car_p", car_p))
        {
            param_loaded = true; // 参数加载成功
            std::cout << "参数 'car_p' 加载成功，包含 " << car_p.size() << " 个元素。" << std::endl;
        }
        else
        {
            // 参数加载失败，打印错误信息，并进行等待重连
            ROS_WARN("等待参数服务器加载...");
            ros::Duration(5).sleep();
            nh.getParam("city_x",city_x); 

            nh.getParam("city_y",city_y); 
            nh.getParam("city_p0",city_p0); 
            nh.getParam("city_p1",city_p1); 
            // nh.getParam("city_s",city_s); 
            nh.getParam("car_x",car_x); 
            nh.getParam("car_y",car_y); 
            // nh.getParam("car_p",car_p); 
            nh.getParam("car_q",car_q); 
        }
    }
    while (car_q_real[0]==0){
            nh.getParam("car_q_real",car_q_real); 
    }
    car_q[0] = car_q_real[0];

    // for (size_t i = 0; i < car_q_real.size(); ++i) {
    //     if (car_q_real[i] != 0) {
    //         car_q[i] = car_q_real[i];
    //     }
    // }


    //----------------------------------------------------------------第一次启动协同任务调度计算节点结束
    //数据加载成功，满足任务调度算法输入，开始执行任务调度，记录此时系统的实时时间
    auto start = std::chrono::steady_clock::now();

    //     // 打印行向量
    std::cout << "car_qxxxxx:";
    for (size_t i = 0; i < car_q.size(); ++i) {
        std::cout << car_q[i]<<endl;}
    
    // std::cout<<"ok go on"<<endl;
    //根据城市和无人车集群的实时信息计算代价矩阵
   for (int i = 0; i < 6; i++)
   {
    /* code */
        for (int j = 0; j < 12; j++)
            {
    // std::cout<<"输出"<<endl;


                if(car_p[j] == 1){
                    cost = static_cast<int>(c1*sqrt(float((car_x[j]-city_x[i])*(car_x[j]-city_x[i])+(car_y[j]-city_y[i])*(car_y[j]-city_y[i])))+c2*(car_q[j]-0.7*city_s[i])); 
                    // cout<<cost<<endl;
                      cost_w[count_w][count_w2]=cost;
                    // cout<<c2*(a[i].q-c[i].q)<<endl;
                
                    count_w2++;
                    if(count_w2==6){
                        count_w2=0;
                        count_w++;
                    }
                }
                else if(car_p[j] == 0){
                    cost = static_cast<int>(c1*sqrt(float((car_x[j]-city_x[i])*(car_x[j]-city_x[i])+(car_y[j]-city_y[i])*(car_y[j]-city_y[i])))+c2*(car_q[j]-0.7*city_s[i]));  
                      cost_L[count_L][count_L2]=cost;
                    count_L2++;
                    if(count_L2==6){
                        count_L2=0;
                        count_L++;   
                    }
                }
            /* code */
            }  
       /* code */
   }
           // 输出cost_w矩阵的值
    // std::cout << "cost_w mat:" << std::endl;
    // for (int i = 0; i < 6; ++i) {
    //     for (int j = 0; j < 6; ++j) {
    //         std::cout << cost_w[i][j] << " ";
    //     }
    //     std::cout << std::endl; // 每行后换行
    // }
    //        // 输出cost_L矩阵的值
    // std::cout << "cost_L mat:" << std::endl;
    // for (int i = 0; i < 6; ++i) {
    //     for (int j = 0; j < 6; ++j) {
    //         std::cout << cost_L[i][j] << " ";
    //     }
    //     std::cout << std::endl; // 每行后换行
    // }
    //     // 遍历float矩阵，并将每个元素转换为int
    // for (size_t i = 0; i < 6; ++i) {
    //     for (size_t j = 0; j < 6; ++j) {
    //         // 将float元素转换为int，这里使用静态_cast<int>()进行转换
    //         intMatrix[i][j] = static_cast<int>(floatMatrix[i][j]);
    //     }
    // }

	//调用算法对成本矩阵进行求解实现无人车的最优任务分配
	compute(6,cost_w);

	for(int i=0;i<n;i++)cout<<"<<-----******无人车"<<i+1<<"-->"<<"任务"<<assign[i]+1<<"******----->>"<<endl;
    nh.getParam("task_point",task_point);
    for (size_t i = 0; i < 6&& i < task_point.size(); ++i)
    {
        task_point[i] = assign[i]+1;
    }

	compute(6,cost_L);
	for(int i=0;i<n;i++)cout<<"<<-----******无人车"<<i+7<<"-->"<<"任务"<<assign[i]+1<<"******----->>"<<endl;
    for (size_t i = 0; i < 6&& i < task_point.size(); ++i)
    {
        task_point[i+6] = assign[i]+1;
    }
    for (const auto& element : task_point) {
        std::cout << element << " ";
    }
    nh.setParam("task_point",task_point);

	// //输出结果
	// for(int i=0;i<n;i++)cout<<"员工"<<i+1<<"-->"<<"任务"<<assign[i]+1<<endl;
    //-------------------------------------------------------输出协同任务调度的响应时间
    auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 
	std::cout << "response time: " << elapsed.count() << "us" << std::endl;//
    //回调函数，当有消息传入即可触发回调
    ros::spin();
}
