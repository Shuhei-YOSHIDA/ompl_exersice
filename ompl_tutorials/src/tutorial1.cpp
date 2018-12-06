/**
 * @file tutorial1.cpp
 */

#include <ros/ros.h>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>

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

  // Execute planning
  double termination_time = 1.0;
  ob::PlannerStatus solved = ss.solve(termination_time);

  // Show a result
  if (solved)
  {
    // Simplify the solution
    ss.simplifySolution();

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
