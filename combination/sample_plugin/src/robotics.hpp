#pragma once

/*
 * INCLUDES
 */

#include <iostream>
#include <vector>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

/*
 * DEFINES
 */

/*
 * TYPEDEFS
 */

typedef Eigen::Matrix4f Mat;
typedef rw::math::Vector3D<> Vec;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Transform3D<> Pose;
typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::MovableFrame MovFrame;
typedef rw::math::Rotation3D<> Rotm;
typedef rw::math::Q Q;

/*
 * GLOBAL VARIABLES
 */

/*
 * FUNCTIONS
 */

Q getConfiguration(Pose estimatedPose) {
    std::cout << "Calculating configuration to grasp the object" << std::endl;
    Q result;

    // load models and state
    rw::models::WorkCell::Ptr workcell = rw::loaders::WorkCellLoader::Factory::load("combination/workcell/Scene.wc.xml");
    rw::models::SerialDevice::Ptr device = workcell->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::kinematics::State state = workcell->getDefaultState();

    // create collision detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(
        new rw::proximity::CollisionDetector(
            workcell,
            rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()
        )
    );

    // name of frames
    const std::string robotName = device->getName();
    const std::string robotBaseName = robotName + ".Base";
    const std::string robotTcpName = robotName + ".TCP";

    // find frames
    Frame *frameTable = workcell->findFrame("Table");
    MovFrame *frameGoal = workcell->findFrame<MovFrame>("GraspTarget");
    Frame *frameTcp = workcell->findFrame("GraspTCP");
    Frame *frameRobotBase = workcell->findFrame(robotBaseName);
    Frame *frameRobotTcp = workcell->findFrame(robotTcpName);

    // check for existence
    if (frameTable == NULL) { std::cerr << "Could not find Table!" << std::endl; }
    if (frameGoal == NULL) { std::cerr << "Could not find GraspTarget!" << std::endl; }
    if (frameTcp == NULL) { std::cerr << "Could not find GraspTCP!" << std::endl; }
    if (frameRobotBase == NULL) { std::cerr << "Could not find " << robotBaseName << "!" << std::endl; }
    if (frameRobotTcp == NULL) {std::cerr << "Could not find " << robotTcpName << "!" << std::endl; }

    // move grasp target to estimated pose
    Vec pos = estimatedPose.P();
    pos(2) = 0.05;
    Rpy rpy = Rpy(estimatedPose.R());
    rpy(2) = 180 * rw::math::Deg2Rad;
    estimatedPose = Pose(pos, rpy.toRotation3D());
    std::cout << "\nEstimated pose -->"
              << "\nPosition: " << estimatedPose.P()
              << "\nRotation: " << Rpy(estimatedPose.R()) << std::endl,

    frameGoal->moveTo(estimatedPose, state);
    std::cout << "\nGrasptarget pose -->"
              << "\nPosition: " << frameGoal->getTransform(state).P()
              << "\nRotation: " << Rpy(frameGoal->getTransform(state).R()) << std::endl;

    // make helper transformations
    Pose base2Goal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    Pose tcp2RobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    Pose targetAt = base2Goal * tcp2RobotTcp;

    // get configurations for collisions
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSolver = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(device, state));
    std::vector<Q> solutions = closedFormSolver->solve(targetAt, state);
    std::cout << "\tNumber of collision free solutions --> " << solutions.size() << std::endl;

    // check the configurations for a collision free solution
    for (std::size_t i = 0; i < solutions.size(); i++) {
        // set the robot at the configuration
        device->setQ(solutions[i], state);

        // check if it is in collision
        if (!detector->inCollision(state, NULL, true)) {
            result = solutions[i];
            break; // only need one
        }
    }

    std::cout << "\tFound configurations --> " << result << std::endl;
    return result;
}
