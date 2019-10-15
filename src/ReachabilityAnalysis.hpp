//--------------------------------
// Describtion of this header file
//-------------------------------

#ifndef REACHABILITYANALYSIS_HPP
#define REACHABILITYANALYSIS_HPP

//---------
// INCLUDES
//---------
#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

//-----------
// NAMESPACES
//-----------
USE_ROBWORK_NAMESPACE
using namespace robwork;

//----------
// FUNCTIONS
//----------

/**
 * @brief getConfigurations
 * @param nameGoal
 * @param nameTcp
 * @param robot
 * @param wc
 * @param state
 * @return
 */
std::vector<rw::math::Q> getConfigurations(const std::string nameGoal,
                                           const std::string nameTcp,
                                           rw::models::SerialDevice::Ptr robot,
                                           rw::models::WorkCell::Ptr wc,
                                           rw::kinematics::State state) {
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

/**
 * @brief getCollisionFreeSolutions: Get the collision free solution for the robot to grap the object
 * @param wc: workcell
 * @param object: object to grasp
 * @param rwplay_path: path to the rwplay file
 * @return vector with collision free q vectors
 */
std::vector<rw::math::Q> getCollisionFreeSolutions(rw::models::WorkCell::Ptr wc,
                                                   rw::models::SerialDevice::Ptr device,
                                                   rw::kinematics::MovableFrame::Ptr object,
                                                   const std::string target,
                                                   const std::string rwplay_path,
                                                   rw::kinematics::State state) {
    // Create detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    std::vector<rw::math::Q> collisionFreeSolutions;
    // for every degree around the roll axis
    for (double roll_angle = 0; roll_angle < 360.0; roll_angle += 1.0) {
        rw::math::RPY<> rot(roll_angle*rw::math::Deg2Rad, 0, 0);
        rw::math::Vector3D<> pos = object->getTransform(state).P();
        rw::math::Transform3D<> trans(pos, rot);
        object->moveTo(trans, state);
        std::vector<rw::math::Q> solutions = getConfigurations(target, "GraspTCP", device, wc, state);
        for (unsigned int i = 0; i < solutions.size(); i++) {
            // set the robot in that configuration and check if it is in collision
            device->setQ(solutions[i], state);
            if (!detector->inCollision(state, NULL, true)) {
                collisionFreeSolutions.push_back(solutions[i]); // save it
                break; // we only need one
            }
        }
    }
    // visualize them if there are any
    if (collisionFreeSolutions.size() > 0) {
        TimedStatePath tStatePath;
        double time=0;
        for (unsigned int i = 0; i < collisionFreeSolutions.size(); i++) {
            device->setQ(collisionFreeSolutions[i], state);
            tStatePath.push_back(TimedState(time, state));
            time+=0.01;
        }
        // Store at rwplay file
        rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
    }

    return collisionFreeSolutions;
}

#endif // REACHABILITYANALYSIS_HPP
