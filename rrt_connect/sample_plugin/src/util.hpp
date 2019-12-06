#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

typedef Eigen::Matrix4f Mat;
typedef rw::math::Vector3D<> Vec;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Transform3D<> Pose;
typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::MovableFrame MovFrame;
typedef rw::math::Rotation3D<> Rotm;
typedef rw::math::Q Q;

Pose matrix2Transform(const Mat matrix) {
    Vec pos = Vec(matrix(0,3), matrix(1,3), matrix(2,3));
    Rotm rotm = Rotm(matrix(0,0), matrix(0,1), matrix(0,2),
                     matrix(1,0), matrix(1,1), matrix(1,2),
                     matrix(2,0), matrix(2,1), matrix(2,2));
    return Pose(pos, rotm);
}

void save2File(const std::string &filePath,
               const std::vector<float> &noises,
               const std::vector<double> &times,
               const std::vector<float> &diffAngle,
               const std::vector<Vec> &diffPos) {
    std::cout << "Writing data to file: " << filePath << std::endl;
    std::ofstream file;
    file.open(filePath);
    if (times.size() != diffAngle.size() || diffAngle.size() != diffPos.size()) {
        std::cerr << "Vectors do not have the same size!" << std::endl;
        return;
    }
    file << "noise time[ms] angle[Deg] x[m] y[m] z[m]\n";
    for (size_t i = 0; i < times.size(); i++) {
        file << noises[i]    << " "
             << times[i]     << " "
             << diffAngle[i] << " "
             << diffPos[i](0)<< " "
             << diffPos[i](1)<< " "
             << diffPos[i](2)<< "\n";
    }
    file.close();
}

void moveFrame(rw::models::WorkCell::Ptr &workcell, rw::kinematics::State &state) {
    std::cout << "Moving frame.." << std::endl;
    // initialise seed
    rw::math::Math::seed();

    // get pose
    MovFrame *frame = workcell->findFrame<MovFrame>("Cylinder");
    Pose pose = frame->getTransform(state);
    Vec position = pose.P();

    // generate random pose
    position(0) = rw::math::Math::ran(-0.3, 0.3);
    position(1) = rw::math::Math::ran(0.37, 0.53);
    position(2) = 0.15;
    Pose newPose = Pose(position, pose.R());

    std::cout << "\tNew pose -->" << std::endl;
    std::cout << "\tP --> " << newPose.P() << std::endl;
    std::cout << "\tR --> " << Rpy(newPose.R()) << std::endl;

    // move object
    frame->moveTo(newPose, state);
}

bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const Q &q) {
    std::cout << "Checking for collisions.." << std::endl;
    rw::kinematics::State test_state = state;
    device->setQ(q, test_state);
    rw::proximity::CollisionDetector::QueryResult data;
    bool colFrom = detector.inCollision(test_state, &data);
    if (colFrom) {
        std::cerr << "\tConfiguration in collision: " << q << std::endl;
        std::cerr << "\tColliding frames: " << std::endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
        }
        return false;
    }
    return true;
}

Q getConfiguration(rw::models::WorkCell::Ptr &workcell, rw::kinematics::State &state, rw::models::SerialDevice::Ptr &device) {
    std::cout << "Calculating configuration to grasp the object" << std::endl;
    Q result;

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
    Frame *frameGoal = workcell->findFrame("GraspTarget");
    Frame *frameTcp = workcell->findFrame("GraspTCP");
    Frame *frameRobotBase = workcell->findFrame(robotBaseName);
    Frame *frameRobotTcp = workcell->findFrame(robotTcpName);

    // check for existence
    if (frameGoal == NULL) { std::cerr << "Could not find GraspTarget!" << std::endl; }
    if (frameTcp == NULL) { std::cerr << "Could not find GraspTCP!" << std::endl; }
    if (frameRobotBase == NULL) { std::cerr << "Could not find " << robotBaseName << "!" << std::endl; }
    if (frameRobotTcp == NULL) {std::cerr << "Could not find " << robotTcpName << "!" << std::endl; }

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
