#include "controller_node.hpp"
#include "input_adapter.hpp"
#include "execution_guard.hpp"
#include "scan_planner/core/control/trajectory_tracker.hpp"
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/planner_status.hpp>
#include <scan_planner_msgs/msg/execution_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <optional>

namespace scan_planner {
using Status=scan_planner_msgs::msg::PlannerStatus;
using Steady=std::chrono::steady_clock;
struct ControllerNode::Impl {
  ControllerNode &node;
  FrameTransformer frames;
  InputAdapter input;
  TrackerParameters parameters;
  RobotState robot;
  Trajectory trajectory;
  Status status;
  scan_planner_msgs::msg::Bspline spline;
  builtin_interfaces::msg::Time odom_stamp;
  bool have_odom{false},have_trajectory{false},have_status{false};
  std::optional<Steady::time_point> odom_received,status_received;
  double odom_timeout,status_timeout,elapsed{0};
  std::string control_frame;
  rclcpp::Time last_ros;
  Steady::time_point clock_advanced{Steady::now()};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub;
  rclcpp::Publisher<scan_planner_msgs::msg::ExecutionState>::SharedPtr execution_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr spline_sub;
  rclcpp::Subscription<Status>::SharedPtr status_sub;
  rclcpp::TimerBase::SharedPtr timer;
  explicit Impl(ControllerNode &n):node(n),frames(n),input(n,frames),last_ros(n.now()) {
    const auto positive=[this](const std::string &key,double fallback) {
      const double value=node.declare_parameter<double>(key,fallback);
      if(!std::isfinite(value)||value<=0) throw std::invalid_argument(key+" must be positive");
      return value;
    };
    odom_timeout=positive("odom_timeout",0.5);status_timeout=positive("status_timeout",0.3);
    parameters.kp_position=positive("controller.kp_position",0.8); parameters.kp_yaw=positive("controller.kp_yaw",1.5);
    parameters.lookahead=positive("controller.lookahead",0.8);parameters.max_vx=positive("controller.max_vx",0.75);
    parameters.heading_preview_distance=node.declare_parameter<double>("controller.heading_preview_distance",0.0);
    if(!std::isfinite(parameters.heading_preview_distance)||parameters.heading_preview_distance<0)
      throw std::invalid_argument("controller.heading_preview_distance must be nonnegative");
    parameters.preview_position_gain=node.declare_parameter<double>("controller.preview_position_gain",0.0);
    if(!std::isfinite(parameters.preview_position_gain)||parameters.preview_position_gain<0 ||
       parameters.preview_position_gain>parameters.kp_position)
      throw std::invalid_argument("controller.preview_position_gain must be between zero and kp_position");
    parameters.max_vy=positive("controller.max_vy",0.35);parameters.max_yaw_rate=positive("controller.max_yaw_rate",1.0);
    parameters.heading_threshold=positive("controller.heading_threshold",0.8);
    control_frame=node.declare_parameter<std::string>("control_frame",input.robotFrame());
    command_pub=node.create_publisher<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(1).reliable());
    execution_pub=node.create_publisher<scan_planner_msgs::msg::ExecutionState>("scan_planner/execution_state",10);
    odom_sub=node.create_subscription<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        try {
          if(!fresh(node.now(),m->header.stamp,odom_timeout)) {have_odom=false;return;}
          RobotState value;have_odom=input.odometry(*m,value);
          if(have_odom) {robot=value;odom_stamp=m->header.stamp;odom_received=Steady::now();}
        }catch(const std::exception &){have_odom=false;}
      });
    status_sub=node.create_subscription<Status>("scan_planner/status",rclcpp::QoS(1).reliable().transient_local(),
      [this](Status::ConstSharedPtr m){status=*m;have_status=true;status_received=Steady::now();});
    spline_sub=node.create_subscription<scan_planner_msgs::msg::Bspline>("scan_planner/trajectory",rclcpp::QoS(1).reliable(),
      [this](scan_planner_msgs::msg::Bspline::ConstSharedPtr m){onTrajectory(*m);});
    timer=node.create_wall_timer(std::chrono::milliseconds(10),[this]{tick();});
  }
  void onTrajectory(const scan_planner_msgs::msg::Bspline &m) {
    try {
      if(m.header.frame_id!=input.planningFrame() || m.order!=3 || m.pos_pts.size()<7 ||
         m.pos_pts.size()>2000 || m.knots.size()!=m.pos_pts.size()+4 ||
         !fresh(node.now(),m.start_time,0.5) || rclcpp::Time(m.valid_until)<=node.now() ||
         (rclcpp::Time(m.valid_until)-node.now()).seconds()>2.0) throw std::invalid_argument("Invalid spline envelope");
      if(have_trajectory && m.path_id==spline.path_id && m.traj_id<=spline.traj_id) return;
      Eigen::MatrixXd points(3,m.pos_pts.size());Eigen::VectorXd knots(m.knots.size());
      for(std::size_t i=0;i<m.pos_pts.size();++i) points.col(i)<<m.pos_pts[i].x,m.pos_pts[i].y,m.pos_pts[i].z;
      for(std::size_t i=0;i<m.knots.size();++i) {
        if(!std::isfinite(m.knots[i]) || (i>0&&m.knots[i]<=m.knots[i-1])) throw std::invalid_argument("Invalid knot vector");
        knots(i)=m.knots[i];
      }
      if(!points.allFinite()) throw std::invalid_argument("Non-finite control points");
      UniformBspline position(points,3,0.1);position.setKnot(knots);
      const double duration=position.getTimeSum();
      if(!std::isfinite(duration)||duration<=0||duration>120) throw std::invalid_argument("Invalid spline duration");
      trajectory={position,duration};spline=m;have_trajectory=true;
      elapsed=std::max(0.0,(node.now()-rclcpp::Time(m.start_time)).seconds());last_ros=node.now();
    }catch(const std::exception &e) {have_trajectory=false;RCLCPP_WARN(node.get_logger(),"Rejected trajectory: %s",e.what());}
  }
  std::string denial(const rclcpp::Time &now,const Steady::time_point &wall) const {
    if(!have_odom||!odom_received||!fresh(now,odom_stamp,odom_timeout)||
       std::chrono::duration<double>(wall-*odom_received).count()>odom_timeout) return "Odometry unavailable or expired";
    if(!have_status||!status_received||!fresh(now,status.header.stamp,status_timeout)||
       std::chrono::duration<double>(wall-*status_received).count()>status_timeout) return "Planner heartbeat expired";
    if(!status.motion_allowed || status.state!=Status::RUNNING)
      return status.reason.empty()?"Planner revoked motion":status.reason;
    if(!have_trajectory||status.path_id!=spline.path_id||status.traj_id!=spline.traj_id) return "Waiting for authorized task trajectory";
    if(now<rclcpp::Time(spline.start_time)) return "Scheduled trajectory has not started";
    if(now>=rclcpp::Time(spline.valid_until)||elapsed>=trajectory.duration) return "Trajectory lease or duration expired";
    if(std::chrono::duration<double>(wall-clock_advanced).count()>odom_timeout) return "ROS clock stopped";
    return {};
  }
  void tick() {
    const auto now=node.now();const auto wall=Steady::now();
    const double dt=(now-last_ros).seconds();last_ros=now;
    if(dt>0) clock_advanced=wall;
    if(dt<0) {have_trajectory=false;have_odom=false;}
    geometry_msgs::msg::Twist command;
    scan_planner_msgs::msg::ExecutionState execution;
    execution.header.stamp=now;execution.header.frame_id=input.robotFrame();
    execution.path_id=spline.path_id;execution.traj_id=spline.traj_id;
    execution.reason=denial(now,wall);
    if(execution.reason.empty()) {
      try {
        const auto value=track(robot,trajectory,elapsed,parameters);
        const auto robot_to_control=frames.lookup(input.robotFrame(),control_frame,rclcpp::Time(odom_stamp),0.5);
        const Eigen::Vector3d angular(0,0,value.yaw_rate);
        const Eigen::Vector3d at_control=robot_to_control.linear().transpose()*
          (Eigen::Vector3d(value.vx,value.vy,0)+angular.cross(robot_to_control.translation()));
        const Eigen::Vector3d angular_control=robot_to_control.linear().transpose()*angular;
        command.linear.x=std::clamp(at_control.x(),-parameters.max_vx,parameters.max_vx);
        command.linear.y=std::clamp(at_control.y(),-parameters.max_vy,parameters.max_vy);
        command.angular.z=std::clamp(angular_control.z(),-parameters.max_yaw_rate,parameters.max_yaw_rate);
        execution.tracking=true;execution.frozen=value.frozen;
        if(!value.frozen) elapsed+=std::clamp(dt,0.0,0.1);
        execution.reason=value.frozen?"Turning before translation":"Tracking";
      }catch(const std::exception &e){command=geometry_msgs::msg::Twist();execution.reason=e.what();}
    }
    execution.elapsed=elapsed;command_pub->publish(command);execution_pub->publish(execution);
  }
};
ControllerNode::ControllerNode():Node("scan_controller"),impl_(std::make_unique<Impl>(*this)) {}
ControllerNode::~ControllerNode()=default;
}  // namespace scan_planner
