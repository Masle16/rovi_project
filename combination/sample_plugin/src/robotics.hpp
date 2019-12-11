#pragma once

/*
 * INCLUDES
 */

#include <iostream>
#include <vector>
#include <map>

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
rw::math::Q ABOVE_PICK = {6, 2.183, -1.620, -1.653, -1.442, 1.573, 0.612}; // Above pick (1)
rw::math::Q INTERMEDIATE_1 = {6, 1.082, -1.128, -1.854, -1.917, 0.367, 0.650}; // Intermediate (1)
rw::math::Q INTERMEDIATE_2 = {6, -0.239, -1.911, -0.651, -1.546, 1.514, -0.918}; // Intermediate (2)
rw::math::Q ABOVE_PLACE = {6, -1.139, -1.613, -1.662, -1.436, 1.569, -2.710}; // Above place (1)

/*
 * FUNCTIONS
 */

double constant_vel(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}

rw::math::Q parabolic_blend(double tmT, rw::math::Q p_i, rw::math::Q v1, rw::math::Q v2, double tau) {
    return (v2-v1)/(4*tau)*pow(tmT+tau, 2)+v1*tmT+p_i;
}

rw::math::Vector3D<double> parabolic_blend(double tmT, rw::math::Vector3D<double> p_i, rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2, double tau) {
    return (v2-v1)/(4*tau)*pow(tmT+tau, 2)+v1*tmT+p_i;
}

void getAboveGraspQ(Pose estimatedPose) {
    std::cout << "\nCalculating above pick configuration.." << std::endl;

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

    Vec pos = estimatedPose.P();
    pos(2) = 0.1;
    Rpy rpy = Rpy(estimatedPose.R());
    rpy(2) = 180 * rw::math::Deg2Rad;
    estimatedPose = Pose(pos, rpy.toRotation3D());
//    std::cout << "\nEstimated pose -->"
//              << "\nPosition: " << estimatedPose.P()
//              << "\nRotation: " << Rpy(estimatedPose.R()) << std::endl,

    // move grasp target to estimated pose
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
    std::cout << "\nNumber solutions --> " << solutions.size() << std::endl;

    // check the configurations for a collision free solution
    for (std::size_t i = 0; i < solutions.size(); i++) {
        // set the robot at the configuration
        device->setQ(solutions[i], state);
        // check if it is in collision
        if (detector->inCollision(state, NULL, true)) {
            solutions.erase(solutions.begin()+i);
        }
    }

    // find solutions with smallest l2 dist to above pick
    std::cout << "\nCollision free solutions --> " << solutions.size() << std::endl;
    rw::math::EuclideanMetric<Q> metric;
    double bestDist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < solutions.size(); i++) {
        double dist = metric.distance(INTERMEDIATE_1, solutions[i]);
        if (dist < bestDist) {
            bestDist = dist;
            ABOVE_PICK = solutions[i];
        }
    }

    std::cout << "\nFound configurations --> " << ABOVE_PICK
              << "\nwhich has a distance of " << bestDist << " to intermediate 1." << std::endl;
}

Q getGraspQ(Pose estimatedPose) {
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
    std::cout << "\nNumber solutions --> " << solutions.size() << std::endl;

    // check the configurations for a collision free solution
    for (std::size_t i = 0; i < solutions.size(); i++) {
        // set the robot at the configuration
        device->setQ(solutions[i], state);

        // check if it is in collision
        if (detector->inCollision(state, NULL, true)) {
            solutions.erase(solutions.begin()+i);
        }
//        if (!detector->inCollision(state, NULL, true)) {
//            result = solutions[i];
//            break; // only need one
//        }
    }

    // find solutions with smallest l2 dist to above pick
    std::cout << "\nCollision free solutions --> " << solutions.size() << std::endl;
    rw::math::EuclideanMetric<Q> metric;
    double bestDist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < solutions.size(); i++) {
        double dist = metric.distance(ABOVE_PICK, solutions[i]);
        if (dist < bestDist) {
            bestDist = dist;
            result = solutions[i];
        }
    }

    std::cout << "\nFound configurations --> " << result
              << "\nwhich has a distance of " << bestDist << " to above pick configuration." << std::endl;
    return result;
}

std::vector<double> getTimeBetweenQs(const std::vector<Q> &qs) {
    std::cout << "\nCalculating time between configurations with a constant velocity of 0.25 m/s" << std::endl;

    std::vector<double> result;

    // load models and state
    rw::models::WorkCell::Ptr workcell = rw::loaders::WorkCellLoader::Factory::load("combination/workcell/Scene.wc.xml");
    rw::models::SerialDevice::Ptr device = workcell->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::kinematics::State state = workcell->getDefaultState();

    for (std::size_t i = 0; i < (qs.size()-1); i++) {
        // position 1
        device->setQ(qs[i], state);
        Vec pos1 = device->baseTend(state).P();

        // position 2
        device->setQ(qs[i+1], state);
        Vec pos2 = device->baseTend(state).P();

        // calc difference
        rw::math::EuclideanMetric<Vec> metric;
        double diff = metric.distance(pos1, pos2);
        double time = std::round(diff / 0.25);
        time = (int(time) == 0) ? 1 : time;

        std::cout << "Time " << i << " --> " << time << std::endl;
        result.push_back(time);
    }

    return result;
}

std::map<int, Q> q_interpolation(std::vector<Q> points, std::vector<double> times) {
    std::cout << "\nComputing interpolation with parabolic blend with time step of 0.1.." << std::endl;

    double time_step = 0.1;
    double time = 0;
    double t = 0;
    int time_index = 0;

    // Linear interpolation between the points:
    std::map<int, rw::math::Q> interpolation;

    for (size_t j = 0; j < times.size(); j++) {

        // Calculate the number of steps for the next loop.
        int interval = int(std::round((time+times[j])-time));
        std::cout << "\nNumber of steps for the next loop --> " << interval << std::endl;

        for (int i = 0; i <= interval/time_step; i++) {
            //std::cout << points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j]) << ", " << t << ", " << i << std::endl;
            //std::cout << points[j] << points[j+1] << constant_vel(i, time, times[j])*(points[j+1]-points[j]) << std::endl;
            interpolation.insert(std::pair<int, rw::math::Q>(time_index, points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j])));
            t += time_step;
            time_index++;
        }
        time_index--;
        time += times[j];
        t = time;
    }


    double T = 0;
    for (size_t i = 1; i < points.size()-1; i++) {
        T += times[i-1];
        double tau = times[i-1] < times[i] ? times[i-1]/3 : times[i]/3;
        tau = std::round(tau*10)/10;

        double blend_time = T-tau;

        rw::math::Q v1 = (points[i]-points[i-1])/times[i-1];
        rw::math::Q v2 = (points[i+1]-points[i])/times[i];

        //std::cout << T << ", " << blend_time << ", " << tau << std::endl;
        //std::cout << interpolation.find(int((T-tau)*10))->first << ", " << interpolation.find(int((T-tau)*10))->second << std::endl;

        int blend_steps = 0;
        while (blend_steps++ <= int(std::round((2*tau)/time_step))) {
            // Insert the blend into the interpolation.
            interpolation.find(int(std::round(blend_time*10)))->second = parabolic_blend(blend_time-T, points[i], v1, v2, tau);
            //std::cout << parabolic_blend(blend_time-T, points[i], v1, v2, tau) << ", " << blend_time << ", " << T+tau << std::endl;
            blend_time += time_step;
        }
        //std::cout << interpolation.find(int((T+tau)*10))->first << ", " << interpolation.find(int((T+tau)*10))->second << std::endl;
        //std::cout << "Next point" << std::endl;
    }

    std::map<int, rw::math::Q>::iterator it;
    for (it = interpolation.begin(); it != interpolation.end(); it++) {
        std::cout << it->first << ", " << it->second << std::endl;
    }

    return interpolation;
}

std::vector<Q> parabolicBlendHome2Object(Q home, Q object) {

    std::vector<Q> result {
        home,
        INTERMEDIATE_2,
        INTERMEDIATE_1,
        ABOVE_PICK,
        object
    };
    return result;
}

std::vector<Q> parabolicBlendObject2Goal(Q object, Q goal) {
    std::vector<Q> result {
        object,
        ABOVE_PICK,
        INTERMEDIATE_1,
        INTERMEDIATE_2,
        ABOVE_PLACE,
        goal
    };
    return result;
}
