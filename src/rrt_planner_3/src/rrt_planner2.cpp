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
#include <time.h>
#include <ctime>
#include <iostream>
using namespace std;
PLUGINLIB_EXPORT_CLASS(rrt_plan::RRTPlanner, nav_core::BaseGlobalPlanner)
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
     if (!initialized_)
    {
    
    ros::NodeHandle nd("~/" + name);
    //initialize the parameters and map
    nd.getParam("iteration_", iteration_);
    nd.getParam("sample_num_", sample_num_);
    nd.getParam("extend_step_", extend_step_);
    nd.getParam("distance_", distance_);
    nd.getParam("probability_", probability_);
    nd.getParam("starR", starR);   
    nd.getParam("choice", choice);   
    nd.getParam("smooth", smooth);   
    nd.getParam("threshold_", threshold_);   
    nd.getParam("gravity", gravity);   
    nd.getParam("grav", grav); 
    nd.getParam("extend_check", extend_check);  
    map_initial(costmap_ros);
    ROS_INFO("---width=%d,height=%d---",map_sizex_,map_sizey_);        
    //pub the plan of path out 
    plan_pub_ = nd.advertise<nav_msgs::Path>("rrt_path", 1);
    tree_pub_ = nd.advertise<sensor_msgs::PointCloud>("rrt_tree", 1);
    //pub the sample points
    sample_pub= nd.advertise<sensor_msgs::PointCloud>("sample", 1);
    initialized_ = true;
    ROS_INFO("Initialized!!!!");
}
    else{
    ROS_WARN("Fail in initializing");
}
}

RRTPlanner::~RRTPlanner()
{
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        delete rrt_nodes_[i];
    }
    for(int i = 0; i < rrt_nodes_0.size(); i++)
    {
        delete rrt_nodes_0[i];
    }
    for(int i = 0; i < rrt_nodes_1.size(); i++)
    {
        delete rrt_nodes_1[i];
    }
    for(int i = 0; i < rrt_new_.size(); i++)
    {
        delete rrt_new_[i];
    }

}
void RRTPlanner::clean_all(std::vector<geometry_msgs::PoseStamped>& plan){
    plan.clear();
    rrt_new_.clear();
    rrt_nodes_.clear();
    rrt_nodes_0.clear();
    rrt_nodes_1.clear();
    k=0;
    root_ = new TreeNode;
    root_->father = NULL;
    root_->children.clear();
    root_1 = new TreeNode;
    root_1->father = NULL;
    root_1->children.clear();

} 

void RRTPlanner::transfer_map_to_cell(PointCell tree){
    tree.x=(tree.x- map_origin_x_) / map_resolution_;
    tree.y=(tree.y- map_origin_y_) / map_resolution_;
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
    start_t=clock();
    //t1=getCurrentTime();       
    clean_all(plan);
    tf::Stamped < tf::Pose > goal_tf;
    tf::Stamped < tf::Pose > start_tf;
    //transform the coordinate
    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);
    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();
    // transform the world to the cell  
    start_x_ = (start.pose.position.x- map_origin_x_) / map_resolution_;
    start_y_ = (start.pose.position.y- map_origin_y_) / map_resolution_;
    goal_x_ = (goal.pose.position.x - map_origin_x_) / map_resolution_;
    goal_y_ = (goal.pose.position.y - map_origin_y_) / map_resolution_;
    root_->cell.x = start_x_;
    root_->cell.y = start_y_;
    root_1->cell.x = goal_x_;
    root_1->cell.y = goal_y_;
    if(goal_x_<0||goal_x_>500||goal_y_<0||goal_y_>500){
        ROS_ERROR("Goal point out of the map!!!");
    }
    unsigned int cost_goal = static_cast<int>(costmap_->getCost(int(goal_x_) , int(goal_y_)));
    if(cost_goal>=threshold_){
        ROS_ERROR("Goal in the barrier!!!");
        
    }
    //put the start root to the rrt_nodes
    rrt_nodes_.push_back(root_);
    rrt_nodes_0.push_back(root_);
    rrt_nodes_1.push_back(root_1);
    ROS_INFO("---startX=%f,startY=%f---",rrt_nodes_0[0]->cell.x,rrt_nodes_0[0]->cell.y);
    ROS_INFO("---goalX=%f,goalY=%f---",rrt_nodes_1[0]->cell.x,rrt_nodes_1[0]->cell.y);
    //buildRRT: mainly construct the rrt tree
   if(buildRRT(choice))
    {
        ROS_WARN("ddd");
        plan.clear();
        std::vector<geometry_msgs::PoseStamped> temp_plan;
        temp_plan.clear();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        TreeNode* father1;
        father1=NULL;
        /*0---base RRT planner
          1---connect RRT planner
          2---RRT star planner*/
        if(choice==2){
        TreeNode* father = goal_node_;
        while(father != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father->cell.y * map_resolution_ + map_origin_y_;
                
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father = father->father;
      
        }}
        else if(choice==1){
        if(goal_connect_!=NULL)
        {father1 = goal_connect_->father;}
        int i=0;
        while(i<rrt_nodes_1.size())
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = rrt_nodes_1[i]->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = rrt_nodes_1[i]->cell.y * map_resolution_ + map_origin_y_;                
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            i++;
        }
        while(father1 != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father1->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father1->cell.y * map_resolution_ + map_origin_y_;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father1 = father1->father;
        }
        }else if(choice==0){
        TreeNode* father = goal_node_;
          while(father != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father->cell.y * map_resolution_ + map_origin_y_;
            unsigned int cost = static_cast<int>(costmap_->getCost(int(father->cell.x) , int(father->cell.y )));
            // cout<<"1"<<" "<<pose.pose.position.x<<" "<<pose.pose.position.y<<endl;
            // cout<<"2"<< " "<<father->cell.x<<" "<<father->cell.y<<endl;
            // cout<<"cost"<<cost<<endl;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father = father->father;
        }
        }
        finish_t=clock();
        for(int i =temp_plan.size() - 1; i > -1; i--)
        {
            plan.push_back(temp_plan[i]);
        }
        publishPlan(plan);
        totaltime=(double)(finish_t-start_t)/CLOCKS_PER_SEC;
        cout<<"\n此程序的运行时间为"<<totaltime<<"秒！"<<endl;
        initialized_ = true;
        return true;
    }
    else
    {
        ROS_ERROR("We can not find a plan in %d cycles.", iteration_);
        return false;
    }    
}
//pub the path out
bool RRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    if(!path.empty())
    {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
    }
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];     
    }
    plan_pub_.publish(gui_path);
}
 
bool RRTPlanner::buildRRT(int choice)
{
 for(int i = 0; i < iteration_; i++)
        {
            if(drawSample(choice))
            {
                for(int j = 0; j < current_sampling_0.size(); j++)
                {   
                    /*0---base RRT planner
                      1---connect RRT planner
                      2---RRT star planner*/
                    switch(choice){
                    case 0:
                        if(extendNearestNode(current_sampling_0[j]))
                        {    return true;}
                    break;
                    case 1:
                        if(extendConnectNode(current_sampling_0[j]))
                        {    return true;}
                    break; 
                    case 2:
                        if(extendstarNode(current_sampling_0[j]))
                        {    return true;}
                    break;                     
                }
                }
            }
        }
    return false;

} 
//RRT base
bool RRTPlanner::extendNearestNode(PointCell random)
{
    int j;
    double path_length = 1000000000000;
    //find the nearest nodes of sample points
    rrt_nodes_[0]->cell.x=start_x_;
    rrt_nodes_[0]->cell.y=start_y_; 
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        if(hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y) <= path_length)
        {
            path_length = hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y);
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
        if(goalReached(node))
        {
            goal_node_ = node;
            if(smooth==1){
            pinghua();     
            }
            return true;
        }
    }
    return false;
}
//RRT connect
bool RRTPlanner::extendConnectNode(PointCell random)
{
    int j;
    double path_length = 1000000000000;
    //find the nearest nodes of sample points
    rrt_nodes_0[0]->cell.x=start_x_;
    rrt_nodes_0[0]->cell.y=start_y_; 
    for(int i = 0; i < rrt_nodes_0.size(); i++)
    {
        if(hypot(rrt_nodes_0[i]->cell.x - random.x, rrt_nodes_0[i]->cell.y - random.y) <= path_length)
        {
          path_length = hypot(rrt_nodes_0[i]->cell.x - random.x, rrt_nodes_0[i]->cell.y - random.y);
           j = i;
        }
    }
    TreeNode* node;
    node = new TreeNode;
    if(extendSingleStep(rrt_nodes_0[j], node, random))
    {
        node->father = rrt_nodes_0[j];
        rrt_nodes_0[j]->children.push_back(node);
        rrt_nodes_0.push_back(node);
         if(goalReached(node))
        {
            goal_node_ = node;
            return true;
        }}
        TreeNode* node1;
        node1 = new TreeNode;
        PointCell exten;
        exten.x=node->cell.x;
        exten.y=node->cell.y;
        if(extendSingleStep(rrt_nodes_1[k], node1, exten)){
         node1->father = rrt_nodes_1[k];
        rrt_nodes_1[k]->children.push_back(node1);
        rrt_nodes_1.push_back(node1);
        k++;
        int connect=getconnected(rrt_nodes_0,node1);
        
        if(connect!=-1)
        {
            goal_node_ = node1;//weilejianyan!!!!!
            goal_connect_=rrt_nodes_0[connect];
           
            return true;
        }
        }
    
     return false;
}
//RRT star
bool RRTPlanner::extendstarNode(PointCell random)
{
      int j,small=0;
    double path_length = 1000000000000;
    double cost_least=10000000;
    //find the nearest nodes of sample points
    rrt_nodes_[0]->cell.x=start_x_;
    rrt_nodes_[0]->cell.y=start_y_; 
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        if(hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y) <= path_length)
        {
            path_length = hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y);
           j = i;
        }
    }
    //add new node to extend from current node to the nearest sample node
    TreeNode* node;
    node = new TreeNode;
    node->father=NULL;
    if(extendSingleStep(rrt_nodes_[j], node, random))
    {   
        if(getnear(node)){
        //node_nearest to node_init plus node_nearest to node_new 
        cost_least=calcul_cost(rrt_nodes_[j],node);
        for(int i = 0;i<near.size();i++){
        if(calcul_cost(rrt_nodes_[near[i]],node)<cost_least){
            small=i;
            cost_least=calcul_cost(rrt_nodes_[near[i]],node);
        }
        }
        node->father=NULL;
        node->father=rrt_nodes_[near[small]];
        rrt_nodes_[near[small]]->children.clear();
        rrt_nodes_[near[small]]->children.push_back(node);
        rrt_nodes_.push_back(node);
        //END FIRST STEP
        near.erase(near.begin()+small);
        if(j>=5){
        for(int i = 0;i<near.size();i++){
        if(calcul_cost(rrt_nodes_[near[i]])>calcul_cost(node)+calcul_cost(node,rrt_nodes_[near[i]])){
            rrt_nodes_[near[i]]->father=NULL;
            rrt_nodes_[near[i]]->father=node;
            rrt_nodes_[near[i]]->children.clear();
            node->children.clear();
            node->children.push_back(rrt_nodes_[near[i]]);
        }
        }
        }
        
    }else{
              node->father = rrt_nodes_[j];
        rrt_nodes_[j]->children.push_back(node);
        rrt_nodes_.push_back(node);
  
        }
        if(goalReached(node))
        {
            goal_node_ = node;
             if(smooth==1){
            pinghua();     
            }
            return true;
        }
    }
    return false;
} 
//smooth the path
void RRTPlanner::pinghua(){
     int i=rrt_nodes_.size()-3;
     int t=0;
     TreeNode *nod;
     nod=new TreeNode;
     TreeNode *nod2;
     nod2=new TreeNode;
     nod=goal_node_;
     nod2=goal_node_->father;
     rrt_new_.push_back(goal_node_);
     //ROS_ERROR("hello");
     while(nod2->father!=NULL){
     {
        if(check_if_on_obstacle(nod,nod2)){
        nod2=nod2->father;
    }else{
        nod->father=NULL;
        rrt_new_.push_back(nod2);
        nod->father = nod2;
        nod2->children.push_back(nod);
        nod=nod->father;
        nod2=nod2->father;
    
    }
}
}     
        nod=rrt_new_[0];
        i=rrt_new_.size()-1;
        ROS_INFO("lengthnew=%d",i); 
        goal_node_=rrt_new_[0];
}
//get all nodes in the tree near the new node(R=starR)
bool  RRTPlanner::getnear(TreeNode* &node){
    near.clear();
    for(int i = 0;i<rrt_nodes_.size();i++){
        if(hypot(rrt_nodes_[i]->cell.x - node->cell.x, rrt_nodes_[i]->cell.y - node->cell.y)<starR){
            near.push_back(i);
        }
    }
    if(near.size()==0)return false;
    else return true;
}
//calculate the cost from the init node to rrtnode and the cost from rrtnode to node
double  RRTPlanner::calcul_cost( TreeNode* rrtnode,TreeNode* node){
    double cost=0;
    TreeNode* rrt;
    rrt=rrtnode;
    cost=hypot(rrtnode->cell.x - node->cell.x, rrtnode->cell.y - node->cell.y);
    while(rrt->father!=NULL){
        cost=cost+hypot(rrt->cell.x - (rrt->father)->cell.x, rrt->cell.y - (rrt->father)->cell.y);
        rrt=rrt->father;
    }
    return cost;
}
//calculate the cost from the init node to rrtnode
double  RRTPlanner::calcul_cost(TreeNode* rrtnode){
    double cost=0;
    TreeNode* rrt = rrtnode;
    while(rrt->father!=NULL){
        cost=cost+hypot(rrt->cell.x - (rrt->father)->cell.x, rrt->cell.y - (rrt->father)->cell.y);
        rrt=rrt->father;
    }
    return cost;
}
//detect if the two node could be connected
int RRTPlanner::getconnected(std::vector<TreeNode*> startNode,TreeNode* endnode){
      for(int i = 0; i < startNode.size(); i++)
    {
        float dist = sqrt(pow(endnode->cell.x - startNode[i]->cell.x,2) + pow(endnode->cell.y - startNode[i]->cell.y,2))* map_resolution_;
         if(dist < distance_)
            return i;
    }
            return -1;
}
//detect if the goal node has been reached
bool RRTPlanner::goalReached(const TreeNode* nearestNode)
{
    if(sqrt(pow(nearestNode->cell.x - goal_x_, 2) + pow(nearestNode->cell.y - goal_y_, 2)) * map_resolution_ < distance_)
    {   return true;
    }else{
        return false;
    }
}
//extend a single step from the nearest node to the random node
bool RRTPlanner::extendSingleStep(TreeNode* rrtnode, TreeNode* &node, const PointCell random)
{
    float sintheta, costheta;
    float sintheta_g, costheta_g;
    float theta,sintheta_t, costheta_t;
    sintheta = (random.y - rrtnode->cell.y) / sqrt(pow((random.x - rrtnode->cell.x), 2) + pow((random.y - rrtnode->cell.y), 2));
    costheta = (random.x - rrtnode->cell.x) / sqrt(pow((random.x - rrtnode->cell.x), 2) + pow((random.y - rrtnode->cell.y), 2));
    //use gravity points
    if(gravity==1){
    sintheta_g = (goal_y_ - rrtnode->cell.y) / sqrt(pow((goal_x_ - rrtnode->cell.x), 2) + pow((goal_y_ - rrtnode->cell.y), 2));
    costheta_g = (goal_x_ - rrtnode->cell.x) / sqrt(pow((goal_x_ - rrtnode->cell.x), 2) + pow((goal_x_ - rrtnode->cell.y), 2));
    node->cell.x = rrtnode->cell.x + (1-grav)*(extend_step_ / map_resolution_) * costheta+grav*(extend_step_ / map_resolution_) * costheta;
    node->cell.y = rrtnode->cell.y + (1-grav)*(extend_step_ / map_resolution_) * sintheta+grav*(extend_step_ / map_resolution_) * sintheta;
    }
    else{
    node->cell.x = rrtnode->cell.x + (extend_step_ / map_resolution_) * costheta;
    node->cell.y = rrtnode->cell.y + (extend_step_ / map_resolution_) * sintheta;
    }
    //if(check_if_on_obstacle(rrtnode,node))
    if(checkIfOnObstacles(node))
      {  
        return true;
          }    
    else
        return false;
}
//generate some sample points, use probablity rrt
void RRTPlanner::probSample()
{    //generate the probability
    double prob0 = rd_() % 10 * 0.1;
    double prob1 = rd_() % 10 * 0.1;
    //use p probability rrt
    if(prob0 > probability_)
    {
        addGoalOrientNode();
    }
    else
        addRandomNode();

     sample_pub.publish(t);
}
//mainly draw the sample points
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
//check if a road is on obstacle 
bool RRTPlanner::check_if_on_obstacle(TreeNode* rrtnode,TreeNode* node){
    TreeNode* test;
    float p=0.00;
    test=NULL;
    test = new TreeNode;   
    float dist=sqrt(pow(rrtnode->cell.x-node->cell.x,2)+pow(rrtnode->cell.y-node->cell.y,2));
    float theta=atan2(node->cell.y-rrtnode->cell.y,node->cell.x-node->cell.x);
    int k=0;
    while(p<=dist) {
    test->cell.x=rrtnode->cell.x+p*cos(theta);
    test->cell.y=rrtnode->cell.y+p*sin(theta);
        k++;
        if(!checkIfOnObstacles(test))
        {
            return false;
            break;
        }
        p=p+extend_check;
    }
    delete test;
    return true;
}
//check if a point is on obstacle
bool RRTPlanner::checkIfOnObstacles(TreeNode* &node){
    unsigned int cost = static_cast<int>(costmap_->getCost(int(node->cell.x) , int(node->cell.y )));            
    if (cost<=threshold_){
        return true;
    }
    else{
        return false;
    }
}
//generate some sample points around the goal node
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
        current_sampling_1.push_back(p);
    }}
}
//generate some random points on the map
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
       
        if(p.x<500 && p.y<500){
        current_sampling_1.push_back(p);
    }}
}

//initial the map 
void RRTPlanner::map_initial(costmap_2d::Costmap2DROS* costmap_ros)
{
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
    freespace_.clear();
    int j = 0;
    int index;
      //initial the costmap to adjust the bizhangtiaojian
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
                   if(cost<=60)  
                   {map_info_.push_back(true);
                    }
                else
                    {
                    map_info_.push_back(false);
                    }
            }
            cout<<endl;
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
   ROS_WARN("freespace num:%d,mapsize%d,freespace in a map%f,",j,mapSize,j/mapSize);
 
}
}