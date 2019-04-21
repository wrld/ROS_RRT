#ifndef _RRT_H
#define _RRT_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <algorithm>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <set>

long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000+tv.tv_usec/1000;
}
namespace rrt_plan
{
struct PointCell
{
  float x;
  float y;
};
 
struct TreeNode
{
  PointCell cell;
  TreeNode* father;
  std::vector<TreeNode*> children;
};


class RRTPlanner : public nav_core::BaseGlobalPlanner
{
public:
  void getCorrdinate (float &x, float &y)
    {
        x = x - originX;
        y = y - originY;
    }
  int mapSize;
  int choice;
  int starR;
  int smooth;
  RRTPlanner();
  ~RRTPlanner();
  bool initialized_;
  RRTPlanner (ros::NodeHandle &); //this constructor is may be not needed
  RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  sensor_msgs::PointCloud t;
  ros::NodeHandle ROSNodeHandle;
  void clean_all(std::vector<geometry_msgs::PoseStamped>& plan);
    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  void transfer_map_to_cell(PointCell tree);

private:
  bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
  // bool rrt_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
  //                          std::vector<geometry_msgs::PoseStamped>& plan)

  bool publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
 
  TreeNode* root_;//这是整棵树的根部，即起点位置
  TreeNode* root_1;
  TreeNode* goal_node_;//终点位置
  TreeNode* goal_connect_;
  bool buildRRT(int choice);
  bool drawSample(int choice);
  bool extendNearestNode(PointCell random);
  bool extendSingleStep(TreeNode* rrtnode, TreeNode* &node, const PointCell random);
  void probSample();
  void addGoalOrientNode();
  void addRandomNode();
  bool goalReached(const TreeNode* nearestNode);
  void map_initial(costmap_2d::Costmap2DROS* costmap_ros);
  void initialPoseCB(costmap_2d::Costmap2DROS* costmap_ros);
  void goalCB(costmap_2d::Costmap2DROS* costmap_ros);
  bool use_rrt_base(int choice);
  bool use_rrt_connect(int choice);
  bool use_rrt_star(int choice);
  int getconnected(std::vector<TreeNode*> startNode,TreeNode* endnode);
  bool extendConnectNode(PointCell random);
  bool getnear(TreeNode* &node);
  double calcul_cost(TreeNode* rrtnode,TreeNode* node);
  double calcul_cost( TreeNode* rrtnode);
  bool extendstarNode(PointCell random);
  bool check_if_on_obstacle(TreeNode* rrtnode,TreeNode* node);
  bool checkIfOnObstacles(TreeNode* &node);
  void pinghua();

  //map_info_[node->cell.y * map_sizex_ + node->cell.x]
  /* map information */
  double map_resolution_;
  int map_sizex_;
  int map_sizey_;
  double map_origin_x_, map_origin_y_;
  std::vector<int> map_info_;
  std::string frame_;
  std::map<int, PointCell> freespace_;//在这里，我用一个map来表示栅格地图中没有障碍物的部分，我们可以只在空闲位置随机撒点试试看
 costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;
    
  /* rrt elements */
  double distance_, probability_;//判断到终点的容忍度，以及向终点方向拉扯的撒点概率
  double threshold_;
  double grav;
  int gravity;
  /* random number generator */
  std::default_random_engine gen_;
  std::random_device rd_;//随机数生成器
  /* sampling parameters */
  int iteration_;//最大迭代次数
  int sample_num_;//每次随机撒点的数量
  double extend_step_;//树枝扩展的步长
  std::vector<TreeNode*> rrt_nodes_;
  std::vector<TreeNode*> rrt_nodes_0;
  std::vector<TreeNode*> rrt_nodes_1;
  std::vector<TreeNode*> rrt_new_;
  std::vector<int> near;
  std::vector<PointCell> current_sampling_0;//本次撒点的样本
  std::vector<PointCell> current_sampling_1;//本次撒点的样本
  
  geometry_msgs::PoseStamped start_;
  float originX;
  float originY;
  int start_x_;
  int start_y_;
  geometry_msgs::PoseStamped goal_;
  int goal_x_;
  int goal_y_;
  std::vector<geometry_msgs::PoseStamped> plan_;
 
  /* ros publisher */
  ros::Publisher plan_pub_, tree_pub_,sample_pub;
  ros::Subscriber map_sub_, pose_sub_, goal_sub_;
 
};


//end of namespace 
}
#endif