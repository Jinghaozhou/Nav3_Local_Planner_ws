#include "planner_node.hpp"
#include "input_adapter.hpp"
#include "execution_guard.hpp"
#include "scan_planner/core/planning/planner_manager.hpp"
#include "scan_planner/core/planning/reference_path.hpp"
#include <nav_msgs/msg/path.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/planner_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <future>
#include <optional>

namespace scan_planner {
using Status=scan_planner_msgs::msg::PlannerStatus;
using Steady=std::chrono::steady_clock;
struct PlannerNode::Impl {
  PlannerNode &node;
  FrameTransformer frames;
  InputAdapter input;
  std::unique_ptr<PlannerManager> planner;
  GridMap::Ptr validation_map;
  std::optional<Trajectory> active_trajectory;
  PlannerParameters parameters;
  RobotState robot;
  Path cloud;
  ReferencePath reference;
  builtin_interfaces::msg::Time odom_stamp,cloud_stamp;
  bool have_odom{false},have_cloud{false},have_path{false},permission{false};
  bool permission_required{false};
  bool task_completed{false};
  std::optional<Steady::time_point> odom_received,cloud_received,permission_received;
  double odom_timeout,map_timeout,tf_timeout,lease,replan_period,translation_jump,rotation_jump;
  double path_offset,body_height;
  std::string height_mode;
  uint64_t path_id{0},tf_version{0};
  int64_t trajectory_id{0};
  uint8_t state{Status::WAITING_INPUT};
  std::string reason{"Waiting for odometry, local map, TF and path"};
  std::optional<Eigen::Isometry3d> last_map_to_odom;
  std::optional<rclcpp::Time> valid_until;
  std::future<PlanResult> future;
  uint64_t request_path{0},request_tf{0};
  builtin_interfaces::msg::Time request_cloud_stamp;
  Eigen::Vector3d request_position;
  Steady::time_point last_plan{Steady::now()-std::chrono::seconds(10)};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub;
  rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr trajectory_pub;
  rclcpp::Publisher<Status>::SharedPtr status_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub;
  rclcpp::TimerBase::SharedPtr timer;

  explicit Impl(PlannerNode &n):node(n),frames(n),input(n,frames) {
    const auto positive=[this](const std::string &key,double fallback) {
      const double value=node.declare_parameter<double>(key,fallback);
      if(!std::isfinite(value)||value<=0) throw std::invalid_argument(key+" must be positive");
      return value;
    };
    odom_timeout=positive("odom_timeout",0.5); map_timeout=positive("map_timeout",1.5);
    tf_timeout=positive("tf_max_age_for_navigation",0.5);
    lease=positive("trajectory_lease",0.8); replan_period=positive("replan_period",0.3);
    if(lease>2.0 || replan_period>=lease)
      throw std::invalid_argument("trajectory_lease must be <=2 seconds and exceed replan_period");
    translation_jump=positive("localization_translation_jump",0.3);
    rotation_jump=positive("localization_rotation_jump",0.15);
    height_mode=node.declare_parameter<std::string>("path_height_mode","reference");
    path_offset=node.declare_parameter<double>("path_z_offset",0.0);
    body_height=node.declare_parameter<double>("body_height",0.4);
    if((height_mode!="reference"&&height_mode!="ground")||!std::isfinite(path_offset)||!std::isfinite(body_height))
      throw std::invalid_argument("Invalid path height configuration");
    MapParameters map;
    map.resolution=positive("map.resolution",0.1);
    const auto size=node.declare_parameter<std::vector<double>>("map.size",{10.0,10.0,5.0});
    if(size.size()!=3) throw std::invalid_argument("map.size requires three dimensions");
    map.size={size[0],size[1],size[2]};
    map.radius=positive("robot.radius",0.25);
    map.cylinder_offset=node.declare_parameter<double>("robot.cylinder_offset",0.18);
    map.body_above=positive("robot.body_above",0.1); map.body_below=positive("robot.body_below",0.4);
    const auto unknown=node.declare_parameter<std::string>("unknown_space_policy","occupied");
    if(unknown!="occupied"&&unknown!="free_inside_bounds") throw std::invalid_argument("Invalid unknown_space_policy");
    map.allow_unobserved_space=unknown=="free_inside_bounds";
    parameters.horizon=positive("planner.horizon",3.0);
    parameters.point_spacing=positive("planner.point_spacing",0.15);
    parameters.reference_corridor=positive("planner.reference_corridor",0.8);
    parameters.max_velocity=positive("planner.max_velocity",0.75);
    parameters.max_acceleration=positive("planner.max_acceleration",0.5);
    parameters.start_velocity_xy_deadband=node.declare_parameter<double>("planner.start_velocity_xy_deadband",0.0);
    parameters.start_velocity_z_deadband=node.declare_parameter<double>("planner.start_velocity_z_deadband",0.0);
    parameters.goal_xy_tolerance=positive("goal.xy_tolerance",0.15);
    parameters.goal_z_tolerance=positive("goal.z_tolerance",0.15);
    parameters.stopped_velocity=positive("goal.stopped_velocity",0.1);
    OptimizationParameters optimization;
    optimization.max_velocity=parameters.max_velocity; optimization.max_acceleration=parameters.max_acceleration;
    optimization.smoothness=positive("optimization.smoothness",1.0);
    optimization.collision=positive("optimization.collision",2.0);
    optimization.feasibility=positive("optimization.feasibility",0.1);
    optimization.clearance=positive("optimization.clearance",0.2);
    planner=std::make_unique<PlannerManager>(map,parameters,optimization);
    validation_map=std::make_shared<GridMap>(map);
    trajectory_pub=node.create_publisher<scan_planner_msgs::msg::Bspline>("scan_planner/trajectory",rclcpp::QoS(1).reliable());
    status_pub=node.create_publisher<Status>("scan_planner/status",rclcpp::QoS(1).reliable().transient_local());
    local_path_pub=node.create_publisher<nav_msgs::msg::Path>("scan_planner/local_path",rclcpp::QoS(1).reliable().transient_local());
    odom_sub=node.create_subscription<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){onOdometry(*m);});
    cloud_sub=node.create_subscription<sensor_msgs::msg::PointCloud2>("local_map",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){onCloud(*m);});
    path_sub=node.create_subscription<nav_msgs::msg::Path>("global_path",rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m){onPath(*m);});
    permission_required=node.declare_parameter<bool>("require_localization_permission",false);
    if(permission_required) permission_sub=node.create_subscription<std_msgs::msg::Bool>("localization_permission",rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m){permission=m->data; permission_received=Steady::now();});
    // Time-based instance token avoids accepting stale controller data after a restart.
    path_id=static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
    timer=node.create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    RCLCPP_INFO(node.get_logger(),"SCAN integration ready: local planning frame=%s, path height=%s, unknown=%s",
      input.planningFrame().c_str(),height_mode.c_str(),unknown.c_str());
  }
  ~Impl() { if(future.valid()) future.wait(); }
  void stop(uint8_t next,const std::string &message) {
    const bool changed=state!=next || reason!=message;
    state=next; reason=message; valid_until.reset(); active_trajectory.reset();
    if(changed) {
      nav_msgs::msg::Path empty; empty.header.frame_id=input.planningFrame(); empty.header.stamp=node.now();
      local_path_pub->publish(empty);
    }
  }
  void publishStatus() {
    Status status; status.header.stamp=node.now(); status.header.frame_id=input.globalFrame();
    status.path_id=path_id; status.traj_id=trajectory_id; status.state=state; status.reason=reason;
    status.motion_allowed=state==Status::RUNNING && valid_until && node.now()<*valid_until;
    if(have_path) {const auto &goal=reference.goal(); status.goal.x=goal.x(); status.goal.y=goal.y(); status.goal.z=goal.z();}
    status_pub->publish(status);
  }
  void onOdometry(const nav_msgs::msg::Odometry &m) {
    try {
      if(!fresh(node.now(),m.header.stamp,odom_timeout)) throw std::invalid_argument("Odometry timestamp is stale or future dated");
      RobotState value;
      const bool valid=input.odometry(m,value);
      if(!valid) { have_odom=false; stop(Status::WAITING_INPUT,"Velocity estimate is initializing or rejected"); publishStatus(); return; }
      robot=value; odom_stamp=m.header.stamp; odom_received=Steady::now(); have_odom=true;
    } catch(const std::exception &e) {have_odom=false; stop(Status::DATA_STALE,e.what()); publishStatus();}
  }
  void onCloud(const sensor_msgs::msg::PointCloud2 &m) {
    try {
      if(!fresh(node.now(),m.header.stamp,map_timeout)) throw std::invalid_argument("Local map acquisition time is stale or future dated");
      if(have_cloud && rclcpp::Time(m.header.stamp)==rclcpp::Time(cloud_stamp)) return;
      auto converted=input.cloud(m);
      cloud=std::move(converted); cloud_stamp=m.header.stamp; cloud_received=Steady::now(); have_cloud=true;
      if(have_odom) {
        // This map belongs to the callback thread, independent of the worker map.
        validation_map->updateSnapshot(cloud,robot.position);
        if(active_trajectory && !collisionFree(*active_trajectory,*validation_map,parameters.max_velocity)) {
          stop(Status::LOCAL_PLAN_FAILED,"Latest snapshot invalidates the active trajectory");
          publishStatus();
        }
      }
    } catch(const std::exception &e) {have_cloud=false; stop(Status::DATA_STALE,e.what()); publishStatus();}
  }
  void onPath(const nav_msgs::msg::Path &m) {
    ++path_id; have_path=false; task_completed=false; last_map_to_odom.reset();
    stop(Status::GLOBAL_PATH_INVALID,"Reference path was replaced; waiting for validation");
    try {
      if(m.header.frame_id!=input.globalFrame()) throw std::invalid_argument("Global path frame must match global_frame");
      if(m.poses.empty() || m.poses.size()>200000) throw std::invalid_argument("Empty or oversized path revokes the previous task");
      Path points; points.reserve(m.poses.size());
      for(const auto &pose:m.poses) {
        if(!pose.header.frame_id.empty() && pose.header.frame_id!=m.header.frame_id)
          throw std::invalid_argument("Inconsistent path pose frame");
        const auto &p=pose.pose.position;
        points.emplace_back(p.x,p.y,p.z+path_offset+(height_mode=="ground"?body_height:0.0));
      }
      reference.set(points); have_path=true;
      state=Status::WAITING_INPUT; reason="Reference accepted; waiting for valid inputs and plan";
      last_plan=Steady::now()-std::chrono::seconds(10);
    } catch(const std::exception &e) {stop(Status::GLOBAL_PATH_INVALID,e.what());}
    publishStatus();
  }
  bool inputsReady() {
    const auto now=node.now(); const auto wall=Steady::now();
    if(!have_odom || !have_cloud) {
      if(state!=Status::DATA_STALE && state!=Status::GLOBAL_PATH_INVALID)
        stop(Status::WAITING_INPUT,"Waiting for valid odometry and local map");
      return false;
    }
    if(!fresh(now,odom_stamp,odom_timeout)||!fresh(now,cloud_stamp,map_timeout)||
       !odom_received||!cloud_received||std::chrono::duration<double>(wall-*odom_received).count()>odom_timeout||
       std::chrono::duration<double>(wall-*cloud_received).count()>map_timeout) {
      stop(Status::DATA_STALE,"Odometry or local map acquisition has timed out"); return false;
    }
    if(permission_required && (!permission||!permission_received||std::chrono::duration<double>(wall-*permission_received).count()>1.0)) {
      stop(Status::LOCALIZATION_UNAVAILABLE,"Localization permission is absent or expired"); return false;
    }
    return true;
  }
  void accept(PlanResult result) {
    if(request_path!=path_id || request_tf!=tf_version || !have_path || !inputsReady()) return;
    if(!fresh(node.now(),request_cloud_stamp,map_timeout) || (robot.position-request_position).norm()>0.25) {
      stop(Status::LOCAL_PLAN_FAILED,"Planning result is stale or its start moved too far"); return;
    }
    if(!result.success) {stop(Status::LOCAL_PLAN_FAILED,result.reason); return;}
    auto &trajectory=result.trajectory;
    validation_map->updateSnapshot(cloud,robot.position);
    if(!collisionFree(trajectory,*validation_map,parameters.max_velocity)) {
      stop(Status::LOCAL_PLAN_FAILED,"Latest snapshot rejects the completed trajectory"); return;
    }
    active_trajectory=trajectory;
    scan_planner_msgs::msg::Bspline message;
    message.header.frame_id=input.planningFrame(); message.header.stamp=node.now();
    message.path_id=path_id; message.traj_id=++trajectory_id; message.order=3; message.start_time=node.now();
    valid_until=node.now()+rclcpp::Duration::from_seconds(std::min(lease,trajectory.duration));
    message.valid_until=*valid_until;
    const auto points=trajectory.position.getControlPoint();
    for(int i=0;i<points.cols();++i) {geometry_msgs::msg::Point p; p.x=points(0,i);p.y=points(1,i);p.z=points(2,i); message.pos_pts.push_back(p);}
    const auto knots=trajectory.position.getKnot();
    for(int i=0;i<knots.size();++i) message.knots.push_back(knots(i));
    trajectory_pub->publish(message);
    nav_msgs::msg::Path path; path.header=message.header;
    for(double t=0;t<trajectory.duration+0.05;t+=0.05) {
      const Eigen::Vector3d p=trajectory.position.evaluateDeBoorT(std::min(t,trajectory.duration));
      geometry_msgs::msg::PoseStamped pose; pose.header=path.header;pose.pose.orientation.w=1;
      pose.pose.position.x=p.x();pose.pose.position.y=p.y();pose.pose.position.z=p.z();path.poses.push_back(pose);
    }
    local_path_pub->publish(path);state=Status::RUNNING;reason=result.reason;
  }
  void tick() {
    try {
      // Drain even obsolete jobs, so the next request can start without races.
      std::optional<PlanResult> completed;
      if(future.valid() && future.wait_for(std::chrono::seconds(0))==std::future_status::ready) completed=future.get();
      if(task_completed) {
        stop(Status::GOAL_REACHED,"Task completed; waiting for a new reference path");
        publishStatus();return;
      }
      if(!inputsReady()) {publishStatus(); return;}
      if(!have_path) {if(state!=Status::GLOBAL_PATH_INVALID) stop(Status::WAITING_PATH,"Waiting for global path");publishStatus();return;}
      const auto zero=rclcpp::Time(0,0,node.get_clock()->get_clock_type());
      const auto map_to_odom=frames.lookup(input.globalFrame(),input.planningFrame(),zero,tf_timeout);
      // Explicitly consume the authoritative TF chain, including robot reference.
      const auto global_robot=frames.lookup(input.globalFrame(),input.robotFrame(),rclcpp::Time(odom_stamp),tf_timeout);
      if((global_robot.translation()-map_to_odom*robot.position).norm()>0.25)
        throw std::runtime_error("Odometry and TF robot poses disagree");
      const Eigen::Matrix3d rotation_error=global_robot.linear().transpose()*
        map_to_odom.linear()*robot.orientation.toRotationMatrix();
      if(Eigen::AngleAxisd(rotation_error).angle()>rotation_jump)
        throw std::runtime_error("Odometry and TF robot orientations disagree");
      if(last_map_to_odom) {
        const auto delta=last_map_to_odom->inverse()*map_to_odom;
        const double angle=Eigen::AngleAxisd(delta.linear()).angle();
        if(delta.translation().norm()>translation_jump || angle>rotation_jump) {
          ++tf_version; stop(Status::TF_CHANGED,"Global localization changed; rebuilding local trajectory");
          reference.reanchor();
          last_map_to_odom=map_to_odom; publishStatus();return;
        }
      }
      last_map_to_odom=map_to_odom;
      const Eigen::Vector3d map_position=map_to_odom*robot.position;
      auto segment=reference.localSegment(map_position,parameters.horizon);
      const auto odom_goal=map_to_odom.inverse()*reference.goal();
      if(reference.remaining()<=parameters.goal_xy_tolerance && planner->reached(robot,odom_goal)) {
        task_completed=true;
        stop(Status::GOAL_REACHED,"Reached original goal with measured position and velocity"); publishStatus();return;
      }
      if(completed) accept(std::move(*completed));
      const auto wall=Steady::now();
      if(!future.valid() && std::chrono::duration<double>(wall-last_plan).count()>=replan_period) {
        for(auto &point:segment) point=map_to_odom.inverse()*point;
        request_path=path_id;request_tf=tf_version;request_cloud_stamp=cloud_stamp;request_position=robot.position;
        const auto state_copy=robot;auto cloud_copy=cloud;
        future=std::async(std::launch::async,[this,state_copy,segment,cloud_copy=std::move(cloud_copy)]{
          try { return planner->plan(state_copy,segment,cloud_copy); }
          catch(const std::exception &e) {PlanResult result;result.reason=e.what();return result;}
        });
        last_plan=wall;
      }
      if(state==Status::RUNNING && (!valid_until||node.now()>=*valid_until)) stop(Status::LOCAL_PLAN_FAILED,"Trajectory lease expired");
    } catch(const std::exception &e) {++tf_version;stop(Status::LOCALIZATION_UNAVAILABLE,e.what());}
    publishStatus();
  }
};
PlannerNode::PlannerNode():Node("scan_planner"),impl_(std::make_unique<Impl>(*this)) {}
PlannerNode::~PlannerNode()=default;
}  // namespace scan_planner
