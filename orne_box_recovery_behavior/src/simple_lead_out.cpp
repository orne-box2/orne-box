#include "simple_lead_out.h"
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <memory>
#include <algorithm>
#include <cmath>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <rotate_recovery/rotate_recovery.h>

PLUGINLIB_EXPORT_CLASS(simple_lead_out::SimpleLeadOut, nav_core::RecoveryBehavior)

namespace simple_lead_out
{
static bool samePose(const geometry_msgs::Pose& a,const geometry_msgs::Pose& b)
{
  auto square=[](double value) { return value*value; };
  double position=square(a.position.x-b.position.x)+square(a.position.y-b.position.y)+square(a.position.z-b.position.z);
  double direct=square(a.orientation.x-b.orientation.x)+square(a.orientation.y-b.orientation.y)+
      square(a.orientation.z-b.orientation.z)+square(a.orientation.w-b.orientation.w);
  double opposite=square(a.orientation.x+b.orientation.x)+square(a.orientation.y+b.orientation.y)+
      square(a.orientation.z+b.orientation.z)+square(a.orientation.w+b.orientation.w);
  return position<=1e-12&&std::min(direct,opposite)<=1e-12;
}
static bool sameFrame(std::string a,std::string b)
{
  while (!a.empty()&&a.front()=='/') a.erase(a.begin());
  while (!b.empty()&&b.front()=='/') b.erase(b.begin());
  return a==b;
}
void RecoveryCancellation::initialize()
{
  ros::NodeHandle n;
  std::string input;
  if (!ros::param::get("/move_base/queue_wait_cmd_vel_topic", input)||input!="/queue_wait/cmd_vel_in") return;
  cancel_sub_=n.subscribe<actionlib_msgs::GoalID>("/move_base/cancel",10,
      [this](const actionlib_msgs::GoalID::ConstPtr&) { ++generation_; });
  active_sub_=n.subscribe<std_msgs::Bool>("/queue_wait/active",1,
      [this](const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data&&!active_.exchange(msg->data)) ++generation_;
        else active_=msg->data;
      });
  running_sub_=n.subscribe<std_msgs::Bool>("/waypoint_manager/waypoint_server/running",1,
      [this](const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data&&running_.exchange(msg->data)) ++generation_;
        else running_=msg->data;
      });
  current_goal_sub_=n.subscribe<geometry_msgs::PoseStamped>("/move_base/current_goal",1,
      [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(mutex_); current_goal_=*msg; goal_ready_=true;
      });
  goal_sub_=n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,
      [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(mutex_);
        if (goal_ready_&&(!sameFrame(msg->header.frame_id,current_goal_.header.frame_id)||!samePose(msg->pose,current_goal_.pose))) ++generation_;
      });
}

SimpleLeadOut::SimpleLeadOut(): local_costmap_(NULL), tf_(NULL), initialized_(false),
                                width_min_(0.0), width_max_(0.0), back_(0.0), front_(0.0) {}

SimpleLeadOut::~SimpleLeadOut(){
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

void SimpleLeadOut::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  local_costmap_ = local_costmap;
  tf_ = tf;

  geometry_msgs::Polygon footprint = local_costmap_->getRobotFootprintPolygon();

  if(!initialized_){
    for(int i=0; i<footprint.points.size(); i++){
      front_<footprint.points.at(i).x?front_=footprint.points.at(i).x:front_=front_;
      back_>footprint.points.at(i).x?back_=footprint.points.at(i).x:back_=back_;
      width_max_<footprint.points.at(i).y?width_max_=footprint.points.at(i).y:width_max_=width_max_;
      width_min_>footprint.points.at(i).y?width_min_=footprint.points.at(i).y:width_min_=width_min_;
    }

    private_nh_.param("base_frame_id", base_frame_id_, std::string("/base_link"));
    private_nh_.param("scan_frame_id", scan_frame_id_, std::string("/scan"));

    ros::NodeHandle private_nh_("~/" + name);
    std::string planner_namespace;
    private_nh_.param("planner_namespace", planner_namespace, std::string("TrajectoryPlannerROS"));
    private_nh_.param("move_dist", move_dist_, front_-back_);

    planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(planner_nh_, scan_frame_id_, 10);
    scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, *tf_, base_frame_id_, 10, 0);
    scan_filter_->registerCallback(boost::bind(&SimpleLeadOut::scanCallback, this, _1));

    planner_nh_.param("vel_x", vel_x_, 0.2);
    if(vel_x_ < 0.0){
      ROS_WARN_STREAM("vel_x can not be negative.");
      vel_x_ = 0.0;
    }

    planner_nh_.param("stop_dist", stop_dist_, 0.2);
    if(stop_dist_ < 0.0){
      ROS_WARN_STREAM("stop_dist can not be negative.");
      stop_dist_ = 0.0;
    }

    cancellation_.initialize();
    initialized_ = true;

    frame_to_scan_.clear();
    scans_.clear();
  }else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void SimpleLeadOut::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);

  if(frame_to_scan_.find(scan->header.frame_id) == frame_to_scan_.end()){
    int laser_index = frame_to_scan_.size();
    frame_to_scan_[scan->header.frame_id] = laser_index;
    scans_.push_back(*scan);
  }else{
    scans_.at(frame_to_scan_[scan->header.frame_id]) = *scan;
  }
}

pcl::PointCloud<pcl::PointXYZ> SimpleLeadOut::mergeCloud()
{
  boost::recursive_mutex::scoped_lock mc(configuration_mutex_);

  std::vector< sensor_msgs::LaserScan > scans = scans_;

  pcl::PointCloud<pcl::PointXYZ> cloud_merged;
  cloud_merged.clear();
  for(int i=0;i<scans.size();i++){
    if(ros::Time(0) - scans.at(i).header.stamp > ros::Duration(0.5)) continue;
    sensor_msgs::PointCloud2 scan_cloud;
    projector_.projectLaser(scans.at(i), scan_cloud);

    sensor_msgs::PointCloud2 tf_cloud;
    try{
      tf_->lookupTransform(scan_cloud.header.frame_id, base_frame_id_, scan_cloud.header.stamp, ros::Duration(0.5));
      pcl_ros::transformPointCloud(base_frame_id_, scan_cloud, tf_cloud, *tf_);
    }catch(tf::TransformException e){
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      continue;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(tf_cloud, *pcl_point_cloud);
    cloud_merged += *pcl_point_cloud;
  }

  return cloud_merged;
}

bool SimpleLeadOut::isObstacle(double x_min, double x_max, double y_min, double y_max, const pcl::PointCloud<pcl::PointXYZ> scan_cloud)
{
  for(int i=0;i<scan_cloud.points.size();i++)
    if((x_min < scan_cloud.points.at(i).x) && (scan_cloud.points.at(i).x < x_max) && (y_min < scan_cloud.points.at(i).y) && (scan_cloud.points.at(i).y < y_max)) return true;

  return false;
}

void SimpleLeadOut::runBehavior()
{
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  const auto generation=cancellation_.generation();
  if (cancellation_.stopped(generation)) return;
  ROS_WARN_STREAM("Lead out behavior start");

  ros::NodeHandle n;
  ros::Rate r(20.0);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  geometry_msgs::Twist cmd_vel;

  geometry_msgs::PoseStamped global_pose;

  pcl::PointCloud<pcl::PointXYZ> scan_cloud = mergeCloud();

  if(!mergeCloud().size()){
    ROS_WARN("can't move");
    return;
  }

  if (isObstacle(front_, front_+move_dist_, width_min_, width_max_, scan_cloud)){
    ROS_WARN("can't move");
    double now_x = global_pose.pose.position.x, now_y = global_pose.pose.position.y;
    double prev_x = now_x, prev_y = now_y;
    double init_x = now_x, init_y = now_y;
    double move = 0.0;
    while((move < 0.3)&&(n.ok())&&!cancellation_.stopped(generation)){
      cmd_vel.linear.x = -vel_x_;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      
      vel_pub.publish(cmd_vel);

      local_costmap_->getRobotPose(global_pose);
      prev_x = now_x;
      prev_y = now_y;
      now_x = global_pose.pose.position.x;
      now_y = global_pose.pose.position.y;
      move += sqrt((now_x-prev_x)*(now_x-prev_x) + (now_y-prev_y)*(now_y-prev_y));
      
      r.sleep();
    }

    vel_pub.publish(geometry_msgs::Twist{});
    return;
  }

  // ROS_WARN_STREAM("lead out behavior pub_twist :" < vel_x_);
  local_costmap_->getRobotPose(global_pose);
  double now_x = global_pose.pose.position.x, now_y = global_pose.pose.position.y;
  double prev_x = now_x, prev_y = now_y;
  double init_x = now_x, init_y = now_y;
  double move = 0.0;

  ROS_WARN("move!!!!");
  
  // while((sqrt((now_x-init_x)*(now_x-init_x) + (now_y-init_y)*(now_y-init_y)) < move_dist_)&&(n.ok()))
  while((move < move_dist_)&&(n.ok())&&!cancellation_.stopped(generation)){
    pcl::PointCloud<pcl::PointXYZ> scan_cloud = mergeCloud();
    // ROS_WARN_STREAM("test :" << direction * move_dist_  <<  ' ' << d);
    if(isObstacle(front_, front_+stop_dist_, width_min_, width_max_, scan_cloud)) {
      ROS_WARN("can't move");
      break;
    }

    cmd_vel.linear.x = vel_x_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    vel_pub.publish(cmd_vel);

    local_costmap_->getRobotPose(global_pose);
    prev_x = now_x;
    prev_y = now_y;
    now_x = global_pose.pose.position.x;
    now_y = global_pose.pose.position.y;
    move += sqrt((now_x-prev_x)*(now_x-prev_x) + (now_y-prev_y)*(now_y-prev_y));
    
    r.sleep();
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  vel_pub.publish(cmd_vel);
  ROS_WARN_STREAM("lead out end");

  return;
  
}
};

namespace simple_lead_out
{

class DefaultClear : public clear_costmap_recovery::ClearCostmapRecovery
{
public:
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global,
                  costmap_2d::Costmap2DROS* local) override
  {
    ros::NodeHandle parameters("~");
    double distance;
    if (name=="conservative_reset") parameters.param("conservative_reset_dist",distance,3.0);
    else { parameters.param("local_costmap/circumscribed_radius",distance,.46); distance*=4; }
    parameters.setParam(name+"/reset_distance",distance);
    clear_costmap_recovery::ClearCostmapRecovery::initialize(name,tf,global,local);
  }
};
class InterruptibleRotate : public nav_core::RecoveryBehavior
{
  costmap_2d::Costmap2DROS* costmap_=nullptr;
  std::unique_ptr<base_local_planner::CostmapModel> model_;
  std::unique_ptr<rotate_recovery::RotateRecovery> original_;
  RecoveryCancellation cancellation_;
  double frequency_, granularity_, acceleration_, maximum_, minimum_, tolerance_;
  bool allowed_=true;
public:
  void initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*,
                  costmap_2d::Costmap2DROS* local) override
  {
    ros::NodeHandle("~").param("clearing_rotation_allowed",allowed_,true);
    const std::string parameter_name=name=="rotate_recovery_after_reset"?"rotate_recovery":name;
    std::string input;
    if (!ros::param::get("/move_base/queue_wait_cmd_vel_topic",input)||input!="/queue_wait/cmd_vel_in") {
      original_.reset(new rotate_recovery::RotateRecovery);
      original_->initialize(parameter_name,nullptr,nullptr,local);
      return;
    }
    costmap_=local;
    model_.reset(new base_local_planner::CostmapModel(*local->getCostmap()));
    ros::NodeHandle parameters("~/"+parameter_name), planner("~/TrajectoryPlannerROS");
    parameters.param("frequency",frequency_,20.0);
    parameters.param("sim_granularity",granularity_,.017);
    acceleration_=nav_core::loadParameterWithDeprecation(planner,"acc_lim_theta","acc_lim_th",3.2);
    maximum_=nav_core::loadParameterWithDeprecation(planner,"max_vel_theta","max_rotational_vel",1.0);
    minimum_=nav_core::loadParameterWithDeprecation(planner,"min_in_place_vel_theta","min_in_place_rotational_vel",.4);
    planner.param("yaw_goal_tolerance",tolerance_,.1);
    if (!std::isfinite(frequency_)||frequency_<=0||!std::isfinite(granularity_)||granularity_<=0||
        !std::isfinite(acceleration_)||acceleration_<=0||!std::isfinite(maximum_)||maximum_<=0||
        !std::isfinite(minimum_)||minimum_<0||!std::isfinite(tolerance_)||tolerance_<=0)
    {
      ROS_ERROR("Rotation recovery disabled: invalid or zero rotation parameters");
      allowed_=false;
      return;
    }
    cancellation_.initialize();
  }
  void runBehavior() override
  {
    if (!allowed_) return;
    if (original_) { original_->runBehavior(); return; }
    const auto generation=cancellation_.generation();
    if (!costmap_||cancellation_.stopped(generation)) return;
    ros::NodeHandle n;
    auto publisher=n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    geometry_msgs::PoseStamped pose;
    if (!costmap_->getRobotPose(pose)) return;
    double previous=tf2::getYaw(pose.pose.orientation), turned=0;
    ros::WallRate rate(frequency_);
    ROS_WARN("Interruptible rotate recovery started");
    while (ros::ok()&&!cancellation_.stopped(generation)) {
      if (!costmap_->getRobotPose(pose)) break;
      double yaw=tf2::getYaw(pose.pose.orientation);
      turned+=angles::shortest_angular_distance(previous,yaw); previous=yaw;
      double remaining=2*M_PI-turned;
      if (remaining<=tolerance_) break;
      bool blocked=false;
      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getCostmap()->getMutex());
        for (double angle=0;angle<remaining;angle+=granularity_) {
          if (cancellation_.stopped(generation)||
              model_->footprintCost(pose.pose.position.x,pose.pose.position.y,yaw+angle,
                                    costmap_->getRobotFootprint(),0,0)<0) { blocked=true; break; }
        }
      }
      if (blocked||cancellation_.stopped(generation)) break;
      geometry_msgs::Twist command;
      command.angular.z=std::min(maximum_,std::max(minimum_,std::sqrt(2*acceleration_*remaining)));
      publisher.publish(command);
      rate.sleep();
    }
    publisher.publish(geometry_msgs::Twist{});
  }
};
}
PLUGINLIB_EXPORT_CLASS(simple_lead_out::InterruptibleRotate, nav_core::RecoveryBehavior)
PLUGINLIB_EXPORT_CLASS(simple_lead_out::DefaultClear, nav_core::RecoveryBehavior)
