#include "rrt2.h"
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <sensor_msgs/PointCloud.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <algorithm>
#include <pluginlib/class_list_macros.h>

// namespace global_planner
// {

PLUGINLIB_EXPORT_CLASS(rrt_plan::RRTPlanner, nav_core::BaseGlobalPlanner)
int k=0;
int choice=1;
namespace rrt_plan
{
RRTPlanner::RRTPlanner():initialized_(false)
{

}
RRTPlanner::RRTPlanner(ros::NodeHandle &nh)
{
    ROSNodeHandle = nh;
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}
void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    iteration_=1000;
    sample_num_=30; 
    extend_step_=0.1;
    threhold_=0.3;
     probability_=0.15;

    ros::NodeHandle nd("~/" + name);
    //initialize the map
     costmap_ros_ = costmap_ros;//initialize the cost map
    //get the costmap from costmap_ros
    costmap_ = costmap_ros_->getCostmap();
    //initialize the planner parameterss
    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();
    map_origin_x_ = originX;
    map_origin_y_ = originY;
    map_sizex_ = costmap_->getSizeInCellsX();
    map_sizey_ = costmap_->getSizeInCellsY();
    map_resolution_ = costmap_->getResolution();
    mapSize = map_sizex_*map_sizey_;
    ROS_WARN("---mapsizeX=%d,mapsizeY=%d---",map_sizex_,map_sizey_);
    ROS_WARN("---map_resolution_=%f",map_resolution_);

    //freespace_.clear();
    int j = 0;
    int index;
      //initial the costmap to adjust the bizhangtiaojian
    //map_info_ = new bool [mapSize];
      for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
               // ROS_WARN("---map_cost_=%d",cost);

                   if(cost<=0)  
                    map_info_.push_back(true);
                else
                    map_info_.push_back(false);

              
            }
        }
   for(int i = 0; i < map_sizex_ * map_sizey_; i++)
    {
          if(map_info_[i] == 0)
        {
            index = i + 1;
            ++j;
            
            PointCell cell;
            cell.x = index % map_sizex_;
            cell.y = index / map_sizex_;
            freespace_[j] = cell;
        }
        
    }
     ROS_WARN("num:%d,mapsize%d,biji%f,",j,mapSize,j/mapSize);
 
    //map_initial(costmap_ros);
    ROS_INFO("---width=%d,height=%d---",map_sizex_,map_sizey_);        
    //pub the plan of path out 
    plan_pub_ = nd.advertise<nav_msgs::Path>("rrt_path", 1);
    tree_pub_ = nd.advertise<sensor_msgs::PointCloud>("rrt_tree", 1);
    //pub the sample points
    sample_pub= nd.advertise<sensor_msgs::PointCloud>("sample", 1);
    initialized_ = true;
    ROS_WARN("hello chunlao");
    // map_initial(costmap_ros);

   
}
RRTPlanner::~RRTPlanner()
{
    //delete the rrt nodes
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        delete rrt_nodes_[i];
    }
}
 
bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan)
{    
     if (!initialized_)
    {
        ROS_ERROR("---The planner has not been initialized, please call initialize() to use the planner---");
        return false;
    }

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("---This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.---",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }
    k=0;
    plan.clear();
    rrt_nodes_.clear();
    rrt_nodes_0.clear();
    rrt_nodes_1.clear();
    
    root_ = new TreeNode;
    root_->father = NULL;
    root_->children.clear();
    root_1 = new TreeNode;
    root_1->father = NULL;
    root_1->children.clear();
 
    

    tf::Stamped < tf::Pose > goal_tf;
    tf::Stamped < tf::Pose > start_tf;
    //transform the coordinate
    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);
    
    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();
    // transform the world to the cell
    //map_resolution:bilichi
    
    start_x_ = (start.pose.position.x- map_origin_x_) / map_resolution_;
    start_y_ = (start.pose.position.y- map_origin_y_) / map_resolution_;
    goal_x_ = (goal.pose.position.x - map_origin_x_) / map_resolution_;
    goal_y_ = (goal.pose.position.y - map_origin_y_) / map_resolution_;
    root_->cell.x = start_x_;
    root_->cell.y = start_y_;
    root_1->cell.x = goal_x_;
    root_1->cell.y = goal_y_;
    
    //put the start root to the rrt_nodes
    rrt_nodes_.push_back(root_);
    rrt_nodes_0.push_back(root_);
    rrt_nodes_1.push_back(root_1);
    ROS_WARN("size=%d",rrt_nodes_.size());
    // rrt_nodes[0]->cell.x=start_x_;
    // rrt_nodes[0]->cell.y=start_y_;
    ROS_INFO("---startX=%f,startY=%f---",rrt_nodes_0[0]->cell.x,rrt_nodes_0[0]->cell.y);
    ROS_INFO("---goalX=%f,goalY=%f---",rrt_nodes_1[0]->cell.x,rrt_nodes_1[0]->cell.y);

   if(buildRRT(choice))
    {
        ROS_WARN("ddd");
        plan.clear();
        std::vector<geometry_msgs::PoseStamped> temp_plan;
        temp_plan.clear();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.pose.position.x = rrt_nodes_0[0]->cell.x * map_resolution_ + map_origin_x_;
        pose.pose.position.y = rrt_nodes_0[0]->cell.y * map_resolution_ + map_origin_y_;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1;
        TreeNode* father1;
        father1=NULL;
        if(choice==0){
        temp_plan.push_back(pose);
        TreeNode* father = goal_node_->father;
        //huisu
        while(father != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father->cell.y * map_resolution_ + map_origin_y_;
           // ROS_WARN("x=%f,y=%f",father->cell.x,father->cell.y);
                
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father = father->father;
        }}
        else if(choice==1){
        //temp_plan.push_back(pose);
        //TreeNode* father0 = goal_node_->father;
        // ROS_WARN("hihi");
        if(goal_connect_!=NULL)
        {father1 = goal_connect_->father;}
        int i=0;
        // ROS_WARN("1x=%f,1y=%f",rrt_nodes_0[0]->cell.x,rrt_nodes_0[0]->cell.y);
        // ROS_WARN("rrt_nodes_0:%d",rrt_nodes_0.size());  
        while(i<rrt_nodes_1.size())
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = rrt_nodes_1[i]->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = rrt_nodes_1[i]->cell.y * map_resolution_ + map_origin_y_;
           // ROS_WARN("1x=%f,1y=%f",rrt_nodes_1[i]->cell.x,rrt_nodes_1[i]->cell.y);
                
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            i++;
        }
        // ROS_WARN("father!!!");
        while(father1 != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father1->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father1->cell.y * map_resolution_ + map_origin_y_;
         //    ROS_WARN("x=%f,y=%f",father1->cell.x,father1->cell.y);
                
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father1 = father1->father;
        }
        }
       // ROS_WARN("greathea!!!");
       
        for(int i =temp_plan.size() - 1; i > -1; i--)
        {
            plan.push_back(temp_plan[i]);
        }
        //ROS_I("temp_plan.size():%d",temp_plan.size());
       
        publishPlan(plan);
        initialized_ = true;
        return true;
    }
    else
    {
        ROS_ERROR("We can not find a plan in %d cycles.", iteration_);
        return false;
    }

    
}
 
bool RRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
   // gui_path.header.stamp = ros::Time::now();
   // ROS_INFO("Plan found:%d",path.size());
        
    if(!path.empty())
    {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
    }
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
      //  ROS_WARN("path_x:%f",(path[i].pose.position.x-map_origin_x_)/map_resolution_);
      //  ROS_WARN("path_y:%f",(path[i].pose.position.y-map_origin_y_)/map_resolution_);
      
    }
  // ROS_WARN("path_size:%d",path.size());
    // ROS_WARN("great pub!!!");
    plan_pub_.publish(gui_path);
    // ROS_WARN("great pub!!!");
}
 
bool RRTPlanner::buildRRT(int choice)
{
switch (choice){
    case 0:
        if(use_rrt_base(choice))
       {     return true;}
        else{
            return false;
        }
    break;
    case 1:
        if(use_rrt_connect(choice))
          {     return true;}
        else{
            return false;
        }
    break;
}
}
 
bool RRTPlanner::use_rrt_base(int choice){
    for(int i = 0; i < iteration_; i++)
        {
            if(drawSample(choice))//suiji sadian
            {
                for(int j = 0; j < current_sampling_0.size(); j++)
                {   
                    // ROS_INFO("Sample time: %d",current_sampling_.size());

                    if(extendNearestNode(current_sampling_0[j]))
                   {   //  ROS_WARN("success!!!!");
                        return true;}
                }
            }
        }
    return false;
}
bool RRTPlanner::use_rrt_connect(int choice){
    for(int i = 0; i < iteration_; i++)
    {
        if(drawSample(choice))//suiji sadian
        {
            for(int j = 0; j < current_sampling_0.size(); j++)
            {   
                 ROS_INFO("Sample time: %d",current_sampling_0.size());

                if(extendConnectNode(current_sampling_0[j]))
               {     ROS_WARN("success!!!!");
                    return true;}
            }
        }
    }
    return false;
}
bool RRTPlanner::drawSample(int choice)
{

    if(freespace_.size() == 0)
        return false;
    else
    {
        probSample();
    }
 //   ROS_INFO("success drawSample");
    return true;
}
 
bool RRTPlanner::extendConnectNode(PointCell random)
{
    int j;
    double path_length = 1000000000000;
    //find the nearest nodes of sample points
    //rrt_nodes_.clear();
    ROS_WARN("start extending");
    rrt_nodes_0[0]->cell.x=start_x_;
    rrt_nodes_0[0]->cell.y=start_y_; 
    for(int i = 0; i < rrt_nodes_0.size(); i++)
    {
        if(hypot(rrt_nodes_0[i]->cell.x - random.x, rrt_nodes_0[i]->cell.y - random.y) <= path_length)
        {
            path_length = hypot(rrt_nodes_0[i]->cell.x - random.x, rrt_nodes_0[i]->cell.y - random.y);
       //     ROS_WARN("extend nearest");
           j = i;
        }
    }
     ROS_WARN("starting");
   
    //add new node to extend from current node to the nearest sample node
    TreeNode* node;
    node = new TreeNode;
    if(extendSingleStep(rrt_nodes_0[j], node, random))
    {
       // ROS_WARN("start extending");
        node->father = rrt_nodes_0[j];
        rrt_nodes_0[j]->children.push_back(node);
        rrt_nodes_0.push_back(node);
         if(goalReached(node))
        {
          //ROS_WARN("goalreach");
            goal_node_ = node;
            return true;
        }}
        TreeNode* node1;
        node1 = new TreeNode;
        PointCell exten;
        exten.x=node->cell.x;
        exten.y=node->cell.y;
        ROS_WARN("connect");
            
        if(extendSingleStep(rrt_nodes_1[k], node1, exten)){
         node1->father = rrt_nodes_1[k];
        rrt_nodes_1[k]->children.push_back(node1);
        rrt_nodes_1.push_back(node1);
        ROS_WARN("ext2222endsinglestep");
   //     ROS_WARN("---goal1X=%f,goal1Y=%f---",rrt_nodes_1[k]->cell.x,rrt_nodes_1[k]->cell.y);
        // if(goalReached(node1))
        // {
        //     ROS_WARN("goalreach");
        //     goal_node_ = node1;//weilejianyan!!!!!
        //     return true;
        // }
        k++;
        int connect=getconnected(rrt_nodes_0,node1);

//        ROS_WARN("consize:%d",rrt_nodes_1.size());
        
  //      ROS_WARN("con:%d",connect);
        if(connect!=-1)
        {
            ROS_WARN("goalconnect");
            goal_node_ = node1;//weilejianyan!!!!!
            goal_connect_=rrt_nodes_0[connect];

            return true;
        }
        }
    
     return false;
}
int RRTPlanner::getconnected(std::vector<TreeNode*> startNode,TreeNode* endnode){
      for(int i = 0; i < startNode.size(); i++)
    {
        float dist = sqrt(pow(endnode->cell.x - startNode[i]->cell.x,2) + pow(endnode->cell.y - startNode[i]->cell.y,2))* map_resolution_;
    //    ROS_WARN("cossn:%f",dist);
   
        if(dist < threhold_)
            return i;
    }
            return -1;
}
bool RRTPlanner::extendNearestNode(PointCell random)
{
    int j;
    double path_length = 1000000000000;
    //find the nearest nodes of sample points
    //rrt_nodes_.clear();
    rrt_nodes_[0]->cell.x=start_x_;
    rrt_nodes_[0]->cell.y=start_y_; 
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        if(hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y) <= path_length)
        {
            path_length = hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y);
        //    ROS_WARN("extend nearest");
           j = i;
        }
    }
    //add new node to extend from current node to the nearest sample node
    TreeNode* node;
    node = new TreeNode;
    if(extendSingleStep(rrt_nodes_[j], node, random))
    {
        node->father = rrt_nodes_[j];
        rrt_nodes_[j]->children.push_back(node);
        rrt_nodes_.push_back(node);
       // ROS_WARN("extendsinglestep");
        if(goalReached(node))
        {
          //ROS_WARN("goalreach");
            goal_node_ = node;
            return true;
        }
    }
    //ROS_WARN("fail extend");
    return false;
}
 //check if the goal has been reached
bool RRTPlanner::goalReached(const TreeNode* nearestNode)
{
   // ROS_WARN("threhold_should%f",sqrt(pow(nearestNode->cell.x - goal_x_, 2) + pow(nearestNode->cell.y - goal_y_, 2)) * map_resolution_);
    if(sqrt(pow(nearestNode->cell.x - goal_x_, 2) + pow(nearestNode->cell.y - goal_y_, 2)) * map_resolution_ < threhold_)
    {   // ROS_WARN("this is %d",nearestNode->cell.x);
        return true;
    }else{
        return false;
    }
}
bool RRTPlanner::extendSingleStep(const TreeNode* rrtnode, TreeNode* &node, const PointCell random)
{
    double sintheta, costheta;
    
    sintheta = (random.y - rrtnode->cell.y) / sqrt(pow((random.x - rrtnode->cell.x), 2) + pow((random.y - rrtnode->cell.y), 2));
    costheta = (random.x - rrtnode->cell.x) / sqrt(pow((random.x - rrtnode->cell.x), 2) + pow((random.y - rrtnode->cell.y), 2));
    node->cell.x = rrtnode->cell.x + (extend_step_ / map_resolution_) * costheta;
    node->cell.y = rrtnode->cell.y + (extend_step_ / map_resolution_) * sintheta;
  //  ROS_WARN("start extend");
    //detect if it is the obstacles
    if(map_info_[node->cell.y * map_sizex_ + node->cell.x] == 0)
        return true;
    else
        return false;
}
 
void RRTPlanner::probSample()
{    //generate the probability
    double prob0 = rd_() % 10 * 0.1;
    double prob1 = rd_() % 10 * 0.1;
    
    if(prob0 > probability_)
    {
       // ROS_WARN("add goal");
        addGoalOrientNode();
    }
    else
       // ROS_WARN("add random");
        addRandomNode();

     sample_pub.publish(t);
}
 
void RRTPlanner::addGoalOrientNode()
{
    current_sampling_0.clear();
    current_sampling_1.clear();
    
    for(int i = 0;i < sample_num_;i++)
    {
        PointCell p;
        //guass around the goal node
        p.x = goal_x_ + (rd_() % 200 - 100) * 0.08;
        p.y = goal_y_ + (rd_() % 200 - 100) * 0.08;
               //in case of too big numbers
        if(p.x<500 && p.y<500){
              //  ROS_WARN("goalrandomx :%f,goalrandomy:%f",p.x,p.y);

        current_sampling_0.push_back(p);
    }}
    for(int i = 0;i < sample_num_;i++)
    {
        PointCell p;
        //guass around the goal node
        p.x = goal_x_ + (rd_() % 200 - 100) * 0.08;
        p.y = goal_y_ + (rd_() % 200 - 100) * 0.08;     
        //in case of too big numbers
        if(p.x<500 && p.y<500){
              //  ROS_WARN("goalrandomx :%f,goalrandomy:%f",p.x,p.y);

        current_sampling_1.push_back(p);
    }}
}
//add some qrandoms
void RRTPlanner::addRandomNode()
{
    current_sampling_0.clear();
    current_sampling_1.clear();
    sensor_msgs::PointCloud t;
    for(int i = 0;i < sample_num_;i++)
    {
        float x = rd_() % map_sizex_;
        float y = rd_() % map_sizey_;
        
        PointCell p;
        
        p.x = x;
        p.y = y;
     //   ROS_INFO("---randomsampleX=%f,sampleY=%f---",x,y);
       
        if(p.x<500 && p.y<500){
        current_sampling_0.push_back(p);
    }}
    for(int i = 0;i < sample_num_;i++)
    {
        float x = rd_() % map_sizex_;
        float y = rd_() % map_sizey_;
        
        PointCell p;
        
        p.x = x;
        p.y = y;
     //   ROS_INFO("---randomsampleX=%f,sampleY=%f---",x,y);
       
        if(p.x<500 && p.y<500){
        current_sampling_1.push_back(p);
    }}
}

//initial the map 
void RRTPlanner::map_initial(costmap_2d::Costmap2DROS* costmap_ros)
{
    //mutex.lock();
        
    costmap_ros_ = costmap_ros;//initialize the cost map
    //get the costmap from costmap_ros
    costmap_ = costmap_ros_->getCostmap();
    //initialize the planner parameterss
    map_sizex_ = costmap_->getSizeInCellsX();
    map_sizey_ = costmap_->getSizeInCellsY();
    map_resolution_ = costmap_->getResolution();
    mapSize = map_sizex_*map_sizey_;
    ROS_WARN("---mapsizeX=%d,mapsizeY=%d---",map_sizex_,map_sizey_);
    ROS_WARN("---map_resolution_=%f",map_resolution_);

    freespace_.clear();
    int j = 0;
    int index=0;
    
    for(int i = 0; i < map_sizex_ * map_sizey_; i++)
    {   
        PointCell cell;
            cell.x = i % map_sizex_;
            cell.y = i / map_sizex_;
             // ROS_WARN("---mapsizeX=%d,mapsizeY=%d---",map_sizex_,map_sizey_);
    
             // ROS_WARN("numx:%f,numy:%f",cell.x,cell.y);
            unsigned int cost = static_cast<int>(costmap_->getCost( cell.x, cell.y));
         //   ROS_WARN("num:%d",cost);
                if(cost==0)  
                    map_info_.push_back(false);
                else
                    map_info_.push_back(true);

          if(map_info_[i] == 50)
        {
            index = i + 1;
            ++j;
            
            PointCell cell;
            cell.x = index % map_sizex_;
            cell.y = index / map_sizex_;

            freespace_[j] = cell;
            
        }
        
    }
    ROS_WARN("num:%f",j/mapSize);
    //ROS_WARN("num:%d",j);
   
}}