
#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
using namespace std;

/* 包含需要的库文件 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>

/* 用于全局路径规划器接口 */
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>


//  cells 表示一个珊格的结构体，包含 当前珊格索引值 和 当前珊格到目标珊格的总距离代价f值
struct cells {
	int currentCell;
	double fCost;
};
//重载<,用于cells在openList表中比较大小进行排序
bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }

double infinity = std::numeric_limits<double>::infinity();//定义一个无穷大的数

int clock_gettime(clockid_t clk_id, struct timespect *tp);//用于记录时间戳

//时间结构体
timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp; 
}



namespace Astar_planner {
  class AstarPlannerROS : public nav_core::BaseGlobalPlanner 
  {
    public:
      
      AstarPlannerROS (ros::NodeHandle &); 
      AstarPlannerROS ();
      AstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap);
      
      ~AstarPlannerROS();

      // 重写父类nav_core::BaseGlobalPlanner中的两个关键函数
      /* 
      1、初始化函数initialize()
        参数1：规划器的名字
        参数2：指向用于规划的代价地图的 ROS 包装器的指针
      */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap);

      /*  
      2、给定地图上的一个目标姿势，计算一个路径
        参数1：起始位姿
        参数2：目标位姿
        参数3：位姿容器
      */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan
                  );

      void BAstarPlanner(int startCell, int goalCell,vector<int> &bestPath);
    
      bool isCellInsideMap(double x, double y);//判断是否在珊格地图中
      bool isStartAndGoalCellsValid(int startCell,int goalCell);//开始和目标单元格是否有效
      int convertToCellIndex (double x, double y);//获取要在 Path 中使用的单元格的索引
      vector <int> findFreeNeighborCell (int CellID);//找到空闲的邻居单元
      void convertToCoordinate(int index, double& x, double& y);//将珊格索引转换成物理坐标,通过引用的方式传给x、y

      //函数的返回值得到一个容器
      vector<int> findPath(int startCell, int goalCell, double g_score[]);
      vector<int> constructPath(int startCell, int goalCell, double g_score[]);

      double calculateHCost(int cellID, int goalCell);//计算两珊格的距离，也就是计算A*(f = g + h)算法中的 h 值，也是代价距离


      //计算领域珊格的f值，并把领域珊格对象插入openList表中
      void addNeighborCellToOpenList(multiset<cells> & OPL, int neighborCell, int goalCell, double g_score[]);


      double getMoveCost(int CellID1, int CellID2);//判断移动到领域珊格的移动代价

      bool isFree(int CellID); //returns true if the cell is Free
      bool isFree(int i, int j); 

      int getCellIndex(int x,int y) //get the index of the cell to be used in Path
      {
        //return (i*width)+j;  
        return x + y * width;
      }

      int getCellRowID(int index)//得到珊格的x值
      {
        return index%width;
      }

      int getCellColID(int index)//得到珊格的y值
      {
        return index/width;
      }

    private:
      //ros::NodeHandle ROSNodeHandle;//节点句柄
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      std::string _frame_id;
      ros::Publisher _plan_pub;
      double originX;
      double originY;
      double resolution;
      int width;//珊格地图宽
      int height;//珊格地图高
      bool initialized_;//标志：是否初始化了
      int value;//记录寻路的次数
      int mapSize;//珊格地图大小，也就是珊格个数
      bool *MCI;
  };

};
#endif
