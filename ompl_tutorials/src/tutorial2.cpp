/**
 * @file tutorial2.cpp
 * @brief check Connection between vertices and so on, by PRM
 */

#include <ros/ros.h>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  // If state is invalid,
  if (std::fabs(x) < 0.5 && std::fabs(y) < 0.5) return false;

  // State is valid
  return true;
}

void planWithSimpleSetup()
{
  // Definition of State space
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  // Definition of Boundary and register it to StateSpace
  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // Instance of SimpleSetup for geometric (It is possible to write codes without SimpleSetup)
  og::SimpleSetup ss(space);

  // Register isStateValid to StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

  // Set Start and Goal state
  ob::ScopedState<ob::SE2StateSpace> start(space), goal(space);
  start->setXY(-0.9, -0.9);
  goal->setXY(0.9, 0.9);
  ss.setStartAndGoalStates(start, goal);

  // Set Planner
  auto prm_ptr = new og::PRM(ss.getSpaceInformation());
  /// The your function that can reject a milestone connection. Default function is prepared.
  auto filter_ = [&prm_ptr](const og::PRM::Vertex& v1, const og::PRM::Vertex& v2)
  {
    auto map = boost::get(og::PRM::vertex_state_t(), prm_ptr->getRoadmap());
    auto state1 = map[v1];
    auto state2 = map[v2];
    std::cout << "x:" << state1->as<ob::SE2StateSpace::StateType>()->getX() << std::endl;

    return true;
  };
  /// The function that specifies the milestones that connection attempts will be made to for a given milestone.
  //auto strategy_ = [&prm_ptr](const og::PRM::Vertex v)
  //{
  //  vector<og::PRM::Vertex> milestones;
  //  return milestones;
  //};

  prm_ptr->setConnectionFilter(filter_);
  //prm_ptr->setConnectionStrategy(strategy_);
  ob::PlannerPtr planner(prm_ptr);
  ss.setPlanner(planner);

  // Execute planning
  double termination_time = 1.0;
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
    std::ofstream ofs("./path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  }
  else std::cout << "No solution found" << std::endl;
}

int main(int argc, char** argv)
{
  planWithSimpleSetup(); 
  return 0;
}
