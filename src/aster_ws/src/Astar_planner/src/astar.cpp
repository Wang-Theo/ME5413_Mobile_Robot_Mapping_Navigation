

#include "astar.h"
#include <pluginlib/class_list_macros.h>

//把自己写的全局规划器注册为一个插件
PLUGINLIB_EXPORT_CLASS(Astar_planner::AstarPlannerROS, nav_core::BaseGlobalPlanner)

namespace Astar_planner
{
  AstarPlannerROS::AstarPlannerROS(){}
  AstarPlannerROS::AstarPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap)
  {
    initialize(name, costmap);//通过构造函数初始化并同时调用初始化函数
  }

  AstarPlannerROS::~AstarPlannerROS(){}

  void AstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap)
  {
    setlocale(LC_ALL,"");

    if (!initialized_)
    {
      costmap_ros_ = costmap;//初始化动态代价地图
      costmap_ = costmap_ros_->getCostmap();//获取静态代价地图
      _frame_id = costmap->getGlobalFrameID();;//获取全局地图坐标系的ID

      ros::NodeHandle private_nh("~/" + name);//给节点加命名空间，变为私有节点
      _plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1);//用于发布全局路径

      //珊格地图原点对应在物理坐标系下的位置
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();

      width = costmap_->getSizeInCellsX();//地图宽度
      height = costmap_->getSizeInCellsY();//地图高度
      resolution = costmap_->getResolution();//地图分辨率
      mapSize = width * height;//地图大小
      value = 0;//路径规划的次数

      MCI = new bool[mapSize];//创建一个bool类型的数组，数组大小为珊格的个数，存储每个珊格的状态

      /*  
      遍历所有珊格的代价值，并把其分为 空闲(true) 和 不空闲(false)，存在一个bool数组中
        乘以 width 的原因：从珊格地图的左下角第一个珊格开始数，依次存入OGM数组中
      */
      for (unsigned int ix = 0; ix < width; ix++)
      {
        for (unsigned int iy = 0; iy < height; iy++)
        {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
          if (cost == 0)
            MCI[ix + iy * width] = true;//代价为0存1
          else
            MCI[ix + iy * width] = false;//代价不为0存0
        }
      }

      //MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

      ROS_INFO("BAstar 全局规划器初始化成功！");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool AstarPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!initialized_)//判断规划器是否初始化
    {
      ROS_ERROR("全局规划器未初始化......");
      return false;
    }

    plan.clear();//获得一个干净的容器

    if (goal.header.frame_id != _frame_id)//判断目标点的父级坐标系是否是全局地图坐标系
    {
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.It is instead in the %s frame.",
                _frame_id.c_str(), goal.header.frame_id.c_str());
      return false;
    }
    if (start.header.frame_id != _frame_id)//判断起始点的父级坐标系是否是全局地图坐标系
    {
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.It is instead in the %s frame.",
                _frame_id.c_str(), start.header.frame_id.c_str());
      return false;
    }

    double startX = start.pose.position.x;
    double startY = start.pose.position.y;

    double goalX = goal.pose.position.x;
    double goalY = goal.pose.position.y;

    ROS_INFO("地图原点:(%.2f, %.2f)", originX, originY);
    ROS_INFO("起始点:(%.2f, %.2f), 目标点:(%.2f, %.2f)", startX, startY, goalX, goalY);
    /* cout << "珊格地图宽度：" << width << endl;
    cout << "珊格地图高度：" << height << endl;
    cout << "珊格地图大小" << mapSize << endl; */

    int startCell;
    int goalCell;

    if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))//判断点是否在珊格地图中
    {
      startCell = convertToCellIndex(startX, startY); //得到一个索引，这个索引用于在OGM数组中寻找坐标点所在珊格对应的代价信息
      goalCell = convertToCellIndex(goalX, goalY);
    }
    else
    {
      ROS_WARN("the start or goal is out of the map");
      return false;
    }
    


    if (isStartAndGoalCellsValid(startCell, goalCell))//判断起始点和目标点是否有效
    {

      vector<int> bestPath;
      bestPath.clear();

      BAstarPlanner(startCell, goalCell, bestPath);//进行全局路径规划，更新了bestPath

      if (bestPath.size() > 0)//如果已经找到全局路径
      {

        // convert the path

        for (int i = 0; i < bestPath.size(); i++)
        {

          double x = 0.0;
          double y = 0.0;

          int index = bestPath[i];//通过循环拿到路径中每个珊格的索引

          convertToCoordinate(index, x, y);//将珊格索引转换成物理坐标,通过引用的方式传给x、y

          geometry_msgs::PoseStamped pose = goal;//目标点坐标系

          //通过循环把bestPath中的每个珊格依次设置为目标点，依次插入plan容器中
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;
          //四元数
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;

          plan.push_back(pose);//得到格式符合ROS的最终物理坐标路径
        }

        
        double path_length = 0.0;//记录路径长度
        std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
        geometry_msgs::PoseStamped last_pose;
        last_pose = *it;//记录起始点位姿
        it++;
        for (; it != plan.end(); ++it)
        {
          //先算第一个和第二个位姿的距离，再算二、三位姿的距离，依次计算并累加,这里应该少计算了最后两个位姿的距离
          path_length += hypot((*it).pose.position.x - last_pose.pose.position.x,
                               (*it).pose.position.y - last_pose.pose.position.y);
          last_pose = *it;
        }
        cout << "全局路径的长度: " << path_length << " 米" << endl;

        //创建一个可以用于发布的路径对象
        nav_msgs::Path path;
        path.poses.resize(plan.size());//设置路径大小，用于存储位姿

        path.header.frame_id = _frame_id;//设置父级坐标系
        path.header.stamp = ros::Time::now();//设置时间戳

        // 为path填充数据
        for (unsigned int i = 0; i < plan.size(); i++)
        {
          path.poses[i] = plan[i];
        }

        _plan_pub.publish(path);//进行路径发布
        return true;
      }

      else
      {
        ROS_WARN("规划器找不到路径，请选择其他目标位置!");
        return false;
      }
    }

    else
    {
      ROS_WARN("起始点或目标点无效！");
      return false;
    }
  }

/* ------------------------------------------------------------------------------------------------------------------ */

  //将物理坐标转换成珊格索引
  int AstarPlannerROS::convertToCellIndex(double x, double y)
  {
    //计算对应的珊格个数
    int newX = (x - originX) / resolution - 0.5;//珊格地图是从0开始的所以要减1,自动取整时会去掉小数部分，故而减0.5即可
    int newY = (y - originY) / resolution - 0.5;

    int cellIndex = getCellIndex(newX, newY);//得到索引值
    return cellIndex;
  }

  //将珊格索引转换成物理坐标,通过引用的方式传给x、y
  void AstarPlannerROS::convertToCoordinate(int index, double &x, double &y)
  {
    x = (getCellRowID(index) + 0.5) * resolution;
    y = (getCellColID(index) + 0.5) * resolution;

    x = x + originX;
    y = y + originY;
  }

  //判断某点的珊格是否在全局珊格地图中
  bool AstarPlannerROS::isCellInsideMap(double x, double y)
  {
    if(x < originX || y < originY)
      return false;
    
    double mx = (x - originX) / resolution - 0.5;
    double my = (y - originY) / resolution - 0.5;

    if(mx < width && my < height)
      return true;

    return false;
  }

  //路径规划器的主函数，传入起始点、目标点索引，通过引用的方式更新最优路径bestPath
  void AstarPlannerROS::BAstarPlanner(int startCell, int goalCell,vector<int> &bestPath)
  {

    double g_score[mapSize];

    for (uint i = 0; i < mapSize; i++)
      g_score[i] = infinity;//先把所有珊格的g值赋为无穷大做一个标记，表示没有被计算过

    timespec time1, time2;//初始化两个时间
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);//记录路径规划前的时间

    bestPath = findPath(startCell, goalCell, g_score);//进行路径规划

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);//记录路径规划后的时间
    cout << "New_A* 规划完成所需时间 = " << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6 << " 微秒" << endl;
    cout << "路径规划的次数为:" << value << endl;
  }

  /*  
  找到一条最优路径，返回一个存有路径经过所有珊格索引的容器bestpath
    参数1：起始点珊格索引
    参数2：目标珊格的索引
    参数3：用于存储所有珊格的g值(起始珊格到该珊格的距离代价)数组
  */
  vector<int> AstarPlannerROS::findPath(int startCell, int goalCell, double g_score[])
  {
    value++;//计数加一
    vector<int> bestPath;
    vector<int> emptyPath;
    cells CP;//创建一个cell对象

    multiset<cells> OPL;//创建一个特殊集合，集合中的元数是有序的，并且允许重复，这是一个openList表
    int currentCell;

    /*  
    A*算法中的路径：f = g + h
      g：起始点到其中一个邻域(旁边八个格子的一个)的距离
      h：该邻域到目标点的距离
    */
    g_score[startCell] = 0; //计算起始位置的g值
    CP.currentCell = startCell;//起始珊格的索引
    CP.fCost = g_score[startCell] + calculateHCost(startCell, goalCell);//计算起始珊格的f = g + h

    //把起始珊格插入poenList表
    OPL.insert(CP);
    currentCell = startCell;

    //不停的循环，直到邻域搜索到目标点的g值，此时的h值为0，故g值就是f值
    //直到计算到目标点的g值停止，即寻路完成
    while (!OPL.empty() && g_score[goalCell] == infinity)
    {
      //选择openList表中具有最低成本 fCost 的单元格，它是多重集的开始
      currentCell = OPL.begin()->currentCell;
      //擦除openList表中被选走的珊格
      OPL.erase(OPL.begin());

      vector<int> neighborCells;
      neighborCells = findFreeNeighborCell(currentCell);//得到当前点的空闲邻域珊格索引

      for (uint i = 0; i < neighborCells.size(); i++) //遍历该邻域，并计算邻域的f值，把邻域珊格全部插入openList表中
        if (g_score[neighborCells[i]] == infinity)//判断邻域是否被计算过，如果没被计算过，则计算g值
        {
          //计算邻域所有珊格的g值 = 当前珊格的g值 + 当前珊格移动到邻域珊格的移动代价
          g_score[neighborCells[i]] = g_score[currentCell] + getMoveCost(currentCell, neighborCells[i]);

          //计算邻域珊格的f值，并把邻域珊格对象插入openList表中
          addNeighborCellToOpenList(OPL, neighborCells[i], goalCell, g_score);
        } 
    }

    //A*算法计算过的节点个数
    int node = 0;//记录搜索过的节点数
    for(uint i = 0; i < mapSize; i++)//遍历存储g值的数组，只要被搜索过，珊格的g值就不等于无穷
    {
        if(g_score[i] != infinity)
          node++;
    }
    cout << "A*算法计算过的节点个数:" << node << endl;

    if (g_score[goalCell] != infinity) //如果找到了目标点
    {
      bestPath = constructPath(startCell, goalCell, g_score);
      return bestPath;
    }
    else
    {
      cout << "无法找到一个路径 !" << endl;
      return emptyPath;//返回一个空容器
    }
  }

  /*  
  反向比较g值，记录g值小的领域索引，形成一条最优路径path，最后再反过来给bestpath
    参数1：起始珊格的索引
    参数2：目标珊格的索引
    参数3：所有珊格的g值(起始珊格到该珊格的距离代价)数组
  */
  vector<int> AstarPlannerROS::constructPath(int startCell, int goalCell, double g_score[])
  {
    vector<int> bestPath;
    vector<int> path;

    path.insert(path.begin() + bestPath.size(), goalCell);//初始化，path的第一个元素的目标珊格的索引
    int currentCell = goalCell;

    while (currentCell != startCell)//当前点(从目标点开始)不等于起始点进行循环
    {
      vector<int> neighborCells;
      neighborCells = findFreeNeighborCell(currentCell);//找到目标点附近的空闲邻域的索引

      vector<double> gScoresNeighbors;//用于存储邻域珊格的g值
      for (uint i = 0; i < neighborCells.size(); i++)//遍历该领域，并把g值存入gScoresNeighbors容器中
        gScoresNeighbors.push_back(g_score[neighborCells[i]]);

      /*
      posMinGScore是领域中最小g值在neighborCells容器中的位置
      从gScoresNeighbors容器里的开头开始搜索，直到找到g值最小的邻域，记录中间包含元素的个数(不含最后一个元数)给distance
        参数1：存储邻域g值的容器的开始地址
        参数2:min_element(first,last): 返回数组区间[first，last）中最小元素的位置，这里用于获取最小的g值
      */
      int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
      
      //通过该位置找到领域珊格最小g值对应的珊格索引，用来更新当前珊格
      currentCell = neighborCells[posMinGScore];

      //将最小g值的领域珊格通过循环依次插入path中
      path.insert(path.begin() + path.size(), currentCell);
    }

    /*
    把path容器里的元素全部倒过来给到bestpath
      path里的路径是从目标点到起点
      bestpath里的路径是从起点到目标点
    */
    for (uint i = 0; i < path.size(); i++)
      bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);
    return bestPath;
  }

  /*  
  计算两珊格的距离，也就是计算A*(f = g + h)算法中的 h 值，也是距离代价
    参数1:去向目标点之间某点的索引
    参数2:目标点的索引
  */
  double AstarPlannerROS::calculateHCost(int cellID, int goalCell)
  {    
    double h;    //h值
    double distance;//两珊格的距离
    double w;    //权重

    int x1=getCellRowID(goalCell);
    int y1=getCellColID(goalCell);
    int x2=getCellRowID(cellID);
    int y2=getCellColID(cellID);

    w = 3.0;
    distance = abs(x1-x2)+abs(y1-y2);         //曼哈顿距离
    //distance = sqrt(pow((x1-x2),2) + pow((y1-y2),2));   //欧几里得距离
    //distance = min(abs(x1-x2),abs(y1-y2)) * sqrt(2)  + max(abs(x1-x2),abs(y1-y2)) - min(abs(x1-x2),abs(y1-y2));//对角线距离
    
    h = w * distance;//乘以权重，扩大数字，有利于比较

    return h;
  }

  /*  
  计算邻域珊格的f值，并把邻域珊格对象插入openList表中
    参数1：openList表，通过引用的方式传入，会更新openList表
    参数2：邻域珊格索引
    参数3：目标珊格索引
    参数4：邻域珊格的g值数组
  */
  void AstarPlannerROS::addNeighborCellToOpenList(multiset<cells> &OPL, int neighborCell, int goalCell, double g_score[])
  {
    cells CP;//实例化一个珊格对象
    CP.currentCell = neighborCell;//邻域珊格的索引值 
    CP.fCost = g_score[neighborCell] + calculateHCost(neighborCell, goalCell);//计算邻域的f值
    //CP.fCost = g_score[neighborCell];//D*算法

    OPL.insert(CP);//把该邻域插入到openList表中
  }

  /*  
  遍历某点一个邻域的珊格是否有障碍物，并返回一个没有障碍物的邻域索引容器
    参数：某点的索引值
  */
  vector<int> AstarPlannerROS::findFreeNeighborCell(int CellID)
  {

    int rowID = getCellRowID(CellID);//得到珊格的x值
    int colID = getCellColID(CellID);//得到珊格的y值
    int neighborIndex;//记录邻域珊格的索引
    vector<int> freeNeighborCells;//存空闲邻域索引的数组

    for (int i = -1; i <= 1; i++)
    {
      if(rowID + i < 0 || rowID + i > width)
        continue;
      for (int j = -1; j <= 1; j++)
      {
        if(colID + j < 0 || colID + j > height || (i ==0 && j == 0))
          continue;

        neighborIndex = getCellIndex(rowID + i, colID + j);
        if (isFree(neighborIndex))//判断邻域是否空闲
          freeNeighborCells.push_back(neighborIndex);//把空闲的邻域索引依次存入数组中
      }
    }
    return freeNeighborCells;
  }

  /*  
  判断起始点和目标点是否有效
    参数1：起始点对应的代价索引(OGM的小标)
    参数2：目标点对应的代价索引(OGM的小标)
  */
  bool AstarPlannerROS::isStartAndGoalCellsValid(int startCell, int goalCell)
  {
    bool isFreeStartCell = isFree(startCell);
    bool isFreeGoalCell = isFree(goalCell);
    if (startCell == goalCell)
    {
      ROS_WARN("起始点与目标点位置相同...");
      return false;
    }

    if(findFreeNeighborCell(goalCell).size() == 0 || findFreeNeighborCell(startCell).size() == 0)
    {
      ROS_WARN("起始点和目标点附近有不可绕过的障碍物，无法规划路径...");
      return false; 
    }
    
    if (isFreeStartCell && isFreeGoalCell)
    {
      cout << "起始点与目标点均是空闲!" << endl;
      return true;
    }
    return false;
  }

  /*  
  判断移动到邻域珊格的移动代价，并返回移动到该邻域珊格的代价
    参数1：当前珊格索引
    参数2：邻域珊格索引
  */
  double AstarPlannerROS::getMoveCost(int CellID1, int CellID2)
  {
    //计算珊格坐标
    int i1 = getCellRowID(CellID1);
    int j1 = getCellColID(CellID1);
    int i2 = getCellRowID(CellID2);
    int j2 = getCellColID(CellID2);

    double moveCost = infinity; //先把代价置为无穷
    //如果该邻域是八邻域中的: 左上、右上、右下、左下，则移动代价为根号2
    if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
    {
      moveCost = 1.414;
    }
    //如果该邻域是八邻域中的: 右、下、左、上，则移动代价为1
    else
    {
      if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
      {
        moveCost = 1;
      }
    }
    return moveCost;
  }

  //通过珊格的坐标判断该珊格是否空闲
  bool AstarPlannerROS::isFree(int i, int j)
  {
    int CellID = getCellIndex(i, j);
    return MCI[CellID];
  }

  //通过索引得到珊格是否空闲
  bool AstarPlannerROS::isFree(int CellID)
  {
    return MCI[CellID];
  }
};
