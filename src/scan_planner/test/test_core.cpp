#include <gtest/gtest.h>
#include "scan_planner/core/planning/planner_manager.hpp"
#include "scan_planner/core/planning/reference_path.hpp"
#include "scan_planner/core/control/trajectory_tracker.hpp"
using namespace scan_planner;

TEST(SnapshotMap, ReplacesOldObstaclesAndKeepsUnknownExplicit) {
  MapParameters p; p.radius=0.1; p.cylinder_offset=0; p.body_above=0.1; p.body_below=0.2;
  p.allow_unobserved_space=true;
  GridMap map(p);
  map.updateSnapshot({Eigen::Vector3d(1,0,0.5)}, Eigen::Vector3d::Zero());
  EXPECT_EQ(map.getInflateOccupancy({1,0,0.5},0),1);
  map.updateSnapshot({Eigen::Vector3d(2,0,0.5)}, Eigen::Vector3d::Zero());
  EXPECT_EQ(map.getInflateOccupancy({1,0,0.5},0),0);
  EXPECT_EQ(map.getInflateOccupancy({2,0,0.5},0),1);
  EXPECT_EQ(map.getInflateOccupancy({20,0,0.5},0),-1);
  p.allow_unobserved_space=false;
  GridMap unknown(p); unknown.updateSnapshot({}, Eigen::Vector3d::Zero());
  EXPECT_EQ(unknown.getInflateOccupancy({0,0,0},0),-1);
}
TEST(SnapshotMap, BodyBelowInflatesForbiddenReferenceUpwards) {
  MapParameters p; p.resolution=0.05; p.radius=0; p.cylinder_offset=0;
  p.body_above=0.05; p.body_below=0.4; p.allow_unobserved_space=true;
  GridMap map(p); map.updateSnapshot({Eigen::Vector3d(0,0,0)}, Eigen::Vector3d::Zero());
  EXPECT_EQ(map.getInflateOccupancy({0,0,0.3},0),1);
  EXPECT_EQ(map.getInflateOccupancy({0,0,-0.3},0),0);
}
TEST(SnapshotMap, InflationAccountsForPositionsInsideVoxels) {
  MapParameters p; p.resolution=0.1; p.radius=0.25;p.cylinder_offset=0;p.allow_unobserved_space=true;
  GridMap map(p);map.updateSnapshot({Eigen::Vector3d(0.099,0.099,0.6)},Eigen::Vector3d::Zero());
  EXPECT_EQ(map.getInflateOccupancy({0.301,0.201,0.6},0),1);
}
TEST(SnapshotMap, FinerCellsReduceDiscretizationWithoutShrinkingEnvelope) {
  MapParameters p; p.radius=0.25;p.cylinder_offset=0;p.allow_unobserved_space=true;
  p.resolution=0.1;
  GridMap coarse(p);coarse.updateSnapshot({Eigen::Vector3d(0.36,0,0.01)},Eigen::Vector3d::Zero());
  EXPECT_EQ(coarse.getInflateOccupancy(Eigen::Vector3d::Zero(),0),1);
  p.resolution=0.05;
  GridMap fine(p);fine.updateSnapshot({Eigen::Vector3d(0.36,0,0.01)},Eigen::Vector3d::Zero());
  EXPECT_EQ(fine.getInflateOccupancy(Eigen::Vector3d::Zero(),0),0);
  fine.updateSnapshot({Eigen::Vector3d(0.2,0,0.01)},Eigen::Vector3d::Zero());
  EXPECT_EQ(fine.getInflateOccupancy(Eigen::Vector3d::Zero(),0),1);
}
TEST(ReferencePath, PreservesFloorAndBoundsForwardProgress) {
  ReferencePath path;
  path.set({{0,0,0},{2,0,0},{2,0,2},{0,0,2}});
  auto segment=path.localSegment({0,0,2},0.7);
  EXPECT_NEAR(segment.front().z(),2,1e-6);
  EXPECT_NEAR(path.remaining(),0,1e-6);
  path.reanchor();
  segment=path.localSegment({0,0,0},0.7);
  EXPECT_NEAR(segment.back().z(),0,1e-6);
  path.set({{0,0,0},{2,0,0},{2,0,2},{0,0,2}});
  segment=path.localSegment({0,0,0},0.7);
  EXPECT_NEAR(segment.back().z(),0,1e-6);
  segment=path.localSegment({0,0,2},0.7);
  EXPECT_NEAR(segment.back().z(),0,1e-6);
  EXPECT_THROW(path.set({Eigen::Vector3d(NAN,0,0)}),std::invalid_argument);
}
TEST(Planner, FlatPathAndWholeTrajectoryCollisionValidation) {
  MapParameters map; map.allow_unobserved_space=true;
  PlannerParameters parameters; OptimizationParameters optimization;
  PlannerManager planner(map,parameters,optimization);
  RobotState state; state.position={0,0,0.6};
  const Path reference{{0,0,0.6},{1,0,0.6},{2,0,0.6}};
  const Path cloud{{4,4,0}};
  auto result=planner.plan(state,reference,cloud);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(planner.collisionFree(result.trajectory));
  auto position=result.trajectory.position;
  EXPECT_NEAR(position.evaluateDeBoorT(0).x(),0,0.01);
  EXPECT_NEAR(position.evaluateDeBoorT(result.trajectory.duration).x(),2,0.01);
  planner.map()->updateSnapshot({Eigen::Vector3d(1.9,0,0.6)},state.position);
  EXPECT_FALSE(planner.collisionFree(result.trajectory));
  Eigen::Vector3d first=Eigen::Vector3d::Constant(NAN);double collision_time=-1;
  EXPECT_FALSE(collisionFree(result.trajectory,*planner.map(),parameters.max_velocity,0,&first,&collision_time));
  EXPECT_TRUE(first.allFinite());
  EXPECT_GE(collision_time,0);
  EXPECT_LE(collision_time,result.trajectory.duration);
  planner.map()->updateSnapshot({Eigen::Vector3d(0,0,0.6)},state.position);
  EXPECT_FALSE(collisionFree(result.trajectory,*planner.map(),parameters.max_velocity));
  EXPECT_TRUE(collisionFree(result.trajectory,*planner.map(),parameters.max_velocity,
                            result.trajectory.duration*0.8));
  EXPECT_FALSE(planner.reached(state,{0,0,2}));
  state.velocity={0.3,0,0}; EXPECT_FALSE(planner.reached(state,state.position));
}
TEST(Planner, WholeTrajectoryCollisionUsesApproachYawAtEndpoint) {
  MapParameters parameters;
  parameters.resolution=0.05;
  parameters.radius=0.1;
  parameters.cylinder_offset=0.2;
  parameters.body_above=0.1;
  parameters.body_below=0.1;
  parameters.allow_unobserved_space=true;
  GridMap map(parameters);
  map.updateSnapshot({Eigen::Vector3d(0.25,0,0.5)},Eigen::Vector3d(0,-0.5,0.5));

  Eigen::MatrixXd control(3,6);
  control.row(0).setZero();
  control.row(1)<<-1,-1,-1,0,0,0;
  control.row(2).setConstant(0.5);
  Trajectory trajectory{UniformBspline(control,3,0.5),0.0};
  trajectory.duration=trajectory.position.getTimeSum();
  const Eigen::Vector3d end=trajectory.position.evaluateDeBoorT(trajectory.duration);
  EXPECT_EQ(map.getInflateOccupancy(end,0.0),1);
  EXPECT_EQ(map.getInflateOccupancy(end,1.5707963267948966),0);
  EXPECT_TRUE(collisionFree(trajectory,map,1.0));

  map.updateSnapshot({Eigen::Vector3d(0,0,0.5)},Eigen::Vector3d(0,-0.5,0.5));
  EXPECT_FALSE(collisionFree(trajectory,map,1.0));
}
TEST(Planner, OccupiedStartAndUnknownSnapshotCannotMove) {
  MapParameters map; map.allow_unobserved_space=true;
  PlannerManager planner(map,{},{}); RobotState state;
  EXPECT_FALSE(planner.plan(state,{{0,0,0},{1,0,0}},{{0,0,0}}).success);
  map.allow_unobserved_space=false; PlannerManager conservative(map,{},{});
  EXPECT_FALSE(conservative.plan(state,{{0,0,0},{1,0,0}},{{4,4,0}}).success);
}
TEST(Planner, DetoursAroundStaticObstacleWithoutChangingOriginalGoal) {
  MapParameters map; map.allow_unobserved_space=true; map.radius=0.15;
  map.cylinder_offset=0.1;
  PlannerManager planner(map,{},{}); RobotState state; state.position={0,0,0.6};
  const auto result=planner.plan(state,{{0,0,0.6},{1,0,0.6},{2,0,0.6}},{{1,0,0.6}});
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(planner.collisionFree(result.trajectory));
  auto trajectory=result.trajectory.position;
  EXPECT_NEAR(trajectory.evaluateDeBoorT(result.trajectory.duration).x(),2,0.01);
}
TEST(Planner, PreservesPiecewiseHeightReferenceAndMeasuredStartVelocity) {
  MapParameters map; map.allow_unobserved_space=true;
  PlannerManager planner(map,{},{}); RobotState state; state.position={0,0,0.6};state.velocity={0.1,0,0};
  const auto result=planner.plan(state,{{0,0,0.6},{1,0,0.6},{2,0,1.2},{3,0,1.2}},{{4,4,0}});
  ASSERT_TRUE(result.success) << result.reason;
  auto trajectory=result.trajectory.position;
  auto velocity=trajectory.getDerivative();
  EXPECT_NEAR(velocity.evaluateDeBoorT(0).x(),0.1,1e-6);
  for(double t=0;t<result.trajectory.duration;t+=0.03) {
    const auto point=trajectory.evaluateDeBoorT(t);
    if(point.x()<0.5) EXPECT_NEAR(point.z(),0.6,0.04);
    if(point.x()>2.5) EXPECT_NEAR(point.z(),1.2,0.04);
  }
}
TEST(Planner, ReplansWithBoundaryVelocityAndAReferenceProgressGap) {
  MapParameters map;map.allow_unobserved_space=true;
  PlannerManager planner(map,{},{});
  for(double speed:{0.0,0.1,0.4}) {
    RobotState state;state.position={0.776,0,0.6};state.velocity={speed,0,0};
    const auto result=planner.plan(state,{{0.9,0,0.6},{2,0,0.6}},{{4,4,0}});
    ASSERT_TRUE(result.success) << "speed=" << speed << ": " << result.reason;
    auto position=result.trajectory.position;
    auto velocity=position.getDerivative();
    auto acceleration=velocity.getDerivative();
    EXPECT_NEAR(velocity.evaluateDeBoorT(0).x(),speed,1e-6);
    for(double t=0;t<result.trajectory.duration;t+=0.02) {
      EXPECT_LE(velocity.evaluateDeBoorT(t).norm(),0.751);
      EXPECT_LE(acceleration.evaluateDeBoorT(t).norm(),0.501);
    }
  }
}
TEST(Planner, IgnoresContactJitterButKeepsMeasuredMotion) {
  MapParameters map;map.allow_unobserved_space=true;
  PlannerParameters parameters;parameters.max_velocity=0.22;parameters.max_acceleration=0.25;
  parameters.start_velocity_xy_deadband=0.05;
  parameters.start_velocity_z_deadband=0.08;
  OptimizationParameters optimization;
  optimization.max_velocity=parameters.max_velocity;
  optimization.max_acceleration=parameters.max_acceleration;
  PlannerManager planner(map,parameters,optimization);
  RobotState state;state.position={0,0,0.5};state.velocity={-0.02,0,-0.04};
  const Path reference{{0.13,0,0.5},{1,0,0.5},{2,0,0.5}};
  const Path cloud{{4,4,0}};
  auto settled=planner.plan(state,reference,cloud);
  ASSERT_TRUE(settled.success) << settled.reason;
  auto start_velocity=settled.trajectory.position.getDerivative();
  EXPECT_NEAR(start_velocity.evaluateDeBoorT(0).norm(),0.0,1e-6);
  TrackerParameters tracker;tracker.heading_preview_distance=0.10;
  EXPECT_FALSE(track(state,settled.trajectory,0,tracker).frozen);
  EXPECT_NEAR(track(state,settled.trajectory,0,tracker).vx,0.0,1e-6);
  tracker.preview_position_gain=0.40;
  EXPECT_GT(track(state,settled.trajectory,0,tracker).vx,0.015);

  state.velocity={-0.12,0,0};
  auto moving=planner.plan(state,reference,cloud);
  ASSERT_TRUE(moving.success) << moving.reason;
  start_velocity=moving.trajectory.position.getDerivative();
  EXPECT_NEAR(start_velocity.evaluateDeBoorT(0).x(),-0.12,1e-6);

  // Vertical contact jitter is ignored before deciding whether the start
  // tangent exceeds the planner's dynamic limit.
  state.velocity={0.215,0,0.06};
  auto jitter=planner.plan(state,reference,cloud);
  EXPECT_NE(jitter.reason,"Measured start velocity exceeds the configured planning limit");
  state.velocity={0.27,0,0};
  auto overspeed=planner.plan(state,reference,cloud);
  EXPECT_EQ(overspeed.reason,"Measured start velocity exceeds the configured planning limit");
}
TEST(Tracker, TurnsBeforeTranslationAndStopsAfterDuration) {
  Eigen::MatrixXd cp=Eigen::MatrixXd::Zero(3,6);
  for(int i=0;i<6;++i) cp(0,i)=i*0.2;
  Trajectory trajectory{UniformBspline(cp,3,0.5),1.5};
  RobotState state; state.orientation=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ());
  const auto turning=track(state,trajectory,0.1,{});
  EXPECT_TRUE(turning.frozen); EXPECT_DOUBLE_EQ(turning.vx,0); EXPECT_DOUBLE_EQ(turning.vy,0);
  const auto stopped=track(state,trajectory,2,{});
  EXPECT_DOUBLE_EQ(stopped.vx,0); EXPECT_DOUBLE_EQ(stopped.yaw_rate,0);
}
TEST(Tracker, LooksPastShortReverseContactSegmentForHeading) {
  Eigen::MatrixXd cp=Eigen::MatrixXd::Zero(3,7);
  cp.row(0)<<0.052,0,-0.052,-0.025,0.103,0.25,0.4;
  Trajectory trajectory{UniformBspline(cp,3,2.38),9.52};
  RobotState state;
  TrackerParameters tracker;tracker.lookahead=0.8;
  EXPECT_TRUE(track(state,trajectory,0,tracker).frozen);
  tracker.heading_preview_distance=0.10;
  EXPECT_FALSE(track(state,trajectory,0,tracker).frozen);
  tracker.preview_position_gain=0.4;
  state.orientation=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ());
  const auto turning=track(state,trajectory,0,tracker);
  EXPECT_TRUE(turning.frozen);
  EXPECT_DOUBLE_EQ(turning.vx,0);
  EXPECT_DOUBLE_EQ(turning.vy,0);
}
