/**
 * @file tutorial3.cpp
 * @brief SE3とR3の違い
 * @details サンプリングに基づく探索で，(回転を無視した)SE3とR3ではその結果がどう変わるか
 */

#include <ros/ros.h>
#include <tuple>
#include <fstream>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <color_names/color_names.h>
#include <easy_marker/easy_marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;
using namespace easy_marker;
using namespace color_names;
using namespace visualization_msgs;
using namespace nav_msgs;

MarkerArray showRoadmapAsMarker(og::SimpleSetup& ss, vector<string> colors)
{
  MarkerArray ma_msg;
  ob::PlannerData pdat(ss.getSpaceInformation());
  ss.getPlannerData(pdat);

  Marker vertex_mrk, edge_mrk;
  vertex_mrk.header.frame_id  = edge_mrk.header.frame_id = "world";
  vertex_mrk.header.stamp = edge_mrk.header.stamp = ros::Time::now();
  vertex_mrk.type = Marker::POINTS;
  vertex_mrk.id = 0;
  vertex_mrk.color = color_names::makeColorMsg(colors[0]);
  vertex_mrk.scale.x = vertex_mrk.scale.y = 0.010;
  edge_mrk.type = Marker::LINE_LIST;
  edge_mrk.id = 1;
  edge_mrk.color = color_names::makeColorMsg(colors[1]);
  edge_mrk.scale.x = 0.005;
  for (int i = 0; i < pdat.numVertices(); i++)
  {
    // vertex marker
    auto v = pdat.getVertex(i);
    if (v == ob::PlannerData::NO_VERTEX) continue;
    vector<double> reals;
    ss.getStateSpace()->copyToReals(reals, v.getState()); // return 7 item: pos and quot(xyzw)
    geometry_msgs::Point p_msg;
    p_msg.x = reals[0]; p_msg.y = reals[1]; p_msg.z = reals[2];
    vertex_mrk.points.push_back(p_msg);

    // edge marker
    vector<unsigned int> edge_list;
    unsigned int n_edge = pdat.getEdges(i, edge_list);
    for (int j = 0; j < n_edge; j++)
    {
      edge_mrk.points.push_back(p_msg);
      auto ve = pdat.getVertex(edge_list[j]);
      if (ve == ob::PlannerData::NO_VERTEX) continue;
      vector<double> realse;
      ss.getStateSpace()->copyToReals(realse, ve.getState());
      geometry_msgs::Point pe_msg;
      pe_msg.x = realse[0]; pe_msg.y = realse[1]; pe_msg.z = realse[2];
      edge_mrk.points.push_back(pe_msg);

    }
  }
  ma_msg.markers.push_back(vertex_mrk);
  ma_msg.markers.push_back(edge_mrk);

  return ma_msg;
}

// For SE3StateSpace
Path getResultantPath1(og::SimpleSetup& ss)
{
  Path path_msg;
  auto path_states = ss.getSolutionPath().getStates();
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  path_msg.header = header;
  for (auto&& state : path_states)
  {
    // SE3
    geometry_msgs::PoseStamped ps;
    ps.header = header;
    auto st = state->as<ob::SE3StateSpace::StateType>();
    ps.pose.position.x = st->getX();
    ps.pose.position.y = st->getY();
    ps.pose.position.z = st->getZ();
    ps.pose.orientation.w = st->rotation().w;
    ps.pose.orientation.x = st->rotation().x;
    ps.pose.orientation.y = st->rotation().y;
    ps.pose.orientation.z = st->rotation().z;

    // Set data
    path_msg.poses.push_back(ps);
  }

  return path_msg;
}

// For RealVectorStateSpace (R3)
Path getResultantPath2(og::SimpleSetup& ss)
{
  Path path_msg;
  auto path_states = ss.getSolutionPath().getStates();
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  path_msg.header = header;
  for (auto&& state : path_states)
  {
    // R3
    geometry_msgs::PoseStamped ps;
    ps.header = header;
    //auto st = state->as<ob::SE3StateSpace::StateType>();
    auto st = state->as<ob::RealVectorStateSpace::StateType>();
    ps.pose.position.x = (*st)[0];
    ps.pose.position.y = (*st)[1];
    ps.pose.position.z = (*st)[2];
    ps.pose.orientation.w = 1.0;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;

    // Set data
    path_msg.poses.push_back(ps);
  }

  return path_msg;
}

bool validArea(double x, double y, double z)
{
  double radius = 0.10;
  vector<vector<double>> spheres =
  {
    {0., 0., 0.},
    {+0.5, +0.5, +0.5},
    {-0.5, +0.5, +0.5},
    {-0.5, -0.5, +0.5},
    {-0.5, +0.5, -0.5},
    {-0.5, -0.5, -0.5},
    {+0.5, -0.5, +0.5},
    {+0.5, -0.5, -0.5},
    {+0.5, +0.5, -0.5},
  };

  for (auto&& center : spheres)
  {
    double dx = x - center[0];
    double dy = y - center[1];
    double dz = z - center[2];

    if (dx*dx + dy*dy + dz*dz <= radius)
    {
     return false;
    }
  }

  return true;
}

bool isStateValid1(const ob::State *state)
{
  const ob::SE3StateSpace::StateType *state_3d = state->as<ob::SE3StateSpace::StateType>();
  const double &x(state_3d->getX()), &y(state_3d->getY()), &z(state_3d->getZ());

  // If state is invalid,
  //if (std::fabs(x) < 0.5 && std::fabs(y) < 0.5 && std::fabs(z) < 0.5) return false;

  // State is valid
  //return true;

  return validArea(x, y, z);
}

// SE3 without rotation
tuple<MarkerArray, Path> planWithSimpleSetup1()
{
  // Definition of State space
  ob::StateSpacePtr space(new ob::SE3StateSpace());

  // Definition of Boundary and register it to StateSpace
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->as<ob::SE3StateSpace>()->setBounds(bounds);

  // Instance of SimpleSetup for geometric (It is possible to write codes without SimpleSetup)
  og::SimpleSetup ss(space);

  // Register isStateValid to StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid1, _1));

  // Set Start and Goal state
  ob::ScopedState<ob::SE3StateSpace> start(space), goal(space);
  start->setXYZ(-0.9, -0.9, -0.9);
  goal->setXYZ(0.9, 0.9, 0.9);
  start->rotation().x = 0.;
  start->rotation().y = 0.;
  start->rotation().z = 0.;
  start->rotation().w = 1.;
  goal->rotation().x = 0.;
  goal->rotation().y = 0.;
  goal->rotation().z = 0.;
  goal->rotation().w = 1.;
  ss.setStartAndGoalStates(start, goal);

  // Execute planning
  double termination_time = 10.0;
  ob::PlannerStatus solved = ss.solve(termination_time);

  // Show a result
  if (solved)
  {
    // Simplify the solution
    ss.simplifySolution();

    std::cout << "planner name is " << ss.getPlanner()->getName() << std::endl;
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(std::cout);
    // Print the solution path to a file
    std::ofstream ofs("./path_SE3.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  }
  else std::cout << "No solution found" << std::endl;

  Path path_msg = getResultantPath1(ss);
  return make_tuple(showRoadmapAsMarker(ss, {"green", "red"}), path_msg);
}

bool isStateValid2(const ob::State *state)
{
  const ob::RealVectorStateSpace::StateType *stateptr = state->as<ob::RealVectorStateSpace::StateType>();
  const double x((*stateptr)[0]), y((*stateptr)[1]), z((*stateptr)[2]);

  // If state is invalid,
  //if (std::fabs(x) < 0.5 && std::fabs(y) < 0.5 && std::fabs(z) < 0.5) return false;

  // State is valid
  //return true;

  return validArea(x, y, z);
}

// Pure R3 only for position
tuple<MarkerArray, Path> planWithSimpleSetup2()
{
  const int DIMENSION_SIZE = 3;
  // Definition of State space
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(DIMENSION_SIZE));

  // Definition of Boundary and register it to StateSpace
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // Instance of SimpleSetup for geometric (It is possible to write codes without SimpleSetup)
  og::SimpleSetup ss(space);

  // Register isStateValid to StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid2, _1));

  // Set Start and Goal state
  ob::ScopedState<ob::StateSpace> start(space), goal(space);
  start[0] = -0.9;
  start[1] = -0.9;
  start[2] = -0.9;
  goal[0] = +0.9;
  goal[1] = +0.9;
  goal[2] = +0.9;
  ss.setStartAndGoalStates(start, goal);

  // Execute planning
  double termination_time = 10.0;
  ob::PlannerStatus solved = ss.solve(termination_time);

  // Show a result
  if (solved)
  {
    // Simplify the solution
    ss.simplifySolution();

    std::cout << "planner name is " << ss.getPlanner()->getName() << std::endl;
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(std::cout);
    // Print the solution path to a file
    std::ofstream ofs("./path_R3.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  }
  else std::cout << "No solution found" << std::endl;

  Path path_msg = getResultantPath2(ss);
  return make_tuple(showRoadmapAsMarker(ss, {"blue", "purple"}), path_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ompl_tutorial3");
  ros::NodeHandle nh;

  auto msgs1 = planWithSimpleSetup1();
  auto msgs2 = planWithSimpleSetup2();

  auto roadmap_msg1 = std::get<0>(msgs1);
  auto roadmap_msg2 = std::get<0>(msgs2);
  auto path_msg1 = std::get<1>(msgs1);
  auto path_msg2 = std::get<1>(msgs2);

  auto roadmap_pub1 = nh.advertise<MarkerArray>("roadmap1", 1);
  auto roadmap_pub2 = nh.advertise<MarkerArray>("roadmap2", 1);
  auto path_pub1 = nh.advertise<Path>("path1", 1);
  auto path_pub2 = nh.advertise<Path>("path2", 1);
  ros::Rate loop(1);
  while(ros::ok())
  {
    auto stamp = ros::Time::now();
    for (auto&& m : roadmap_msg1.markers) m.header.stamp = stamp;
    for (auto&& m : roadmap_msg2.markers) m.header.stamp = stamp;
    path_msg1.header.stamp = stamp;
    path_msg2.header.stamp = stamp;
    for (auto&& m : path_msg1.poses) m.header.stamp = stamp;
    for (auto&& m : path_msg2.poses) m.header.stamp = stamp;

    roadmap_pub1.publish(roadmap_msg1);
    roadmap_pub2.publish(roadmap_msg2);
    path_pub1.publish(path_msg1);
    path_pub2.publish(path_msg2);
    loop.sleep();
  }
  return 0;
}
