/*
 * INCLUDES
 */
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <map>
#include <iterator>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

/*
 * DEFINES
 */
#define MAXTIME 60.
#define MAX_STEP_SIZE 2.0
#define MAX_ITERATIONS 50

/*
 * TYPEDEFS
 */
typedef rw::math::Transform3D<> Pose;
typedef rw::math::Rotation3D<> Rotation;
typedef rw::math::Vector3D<> Vector;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Q Q;
typedef rw::kinematics::Frame Frame;

/*
 * GLOBAL VARIABLES
 */
const std::string WC_FILE = "../../workcell/Scene.wc.xml";
const std::string DEVICE_NAME = "UR-6-85-5-A";
const double VELOCITY = 0.25; // m/s

/*
 * FUNCTIONS
 */
double constant_vel(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}

std::string convertDoub2Str(const double &input) {
    std::ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(2);
    streamObj << input;
    return streamObj.str();
}

Pose forwardKinematics(const std::vector<Pose>& tRefs,
                       const unsigned int idx,
                       const Q& q) {
    if (tRefs.size() != q.size()) {
        std::cerr << "The number of local transformations must be equal to the length of the configuration vector." << std::endl;
    }
    Pose baseTi;
    for (unsigned int i = 0; i < idx; i++) {
        Pose T(Rpy(q[i], 0, 0).toRotation3D());
        baseTi = baseTi * tRefs[i] * T;
    }
    return baseTi;
}

/**
 * The functions was given in the exercises in lecture 6 of robotics.
 * @brief checkCollisions
 * @param device
 * @param state
 * @param detector
 * @param q
 * @return
 */
bool checkCollisions(rw::models::Device::Ptr device,
                     const rw::kinematics::State &state,
                     const rw::proximity::CollisionDetector &detector,
                     const rw::math::Q &q) {
    rw::kinematics::State test_state = state;
    device->setQ(q, test_state);
    rw::proximity::CollisionDetector::QueryResult data;
    bool colFrom = detector.inCollision(test_state, &data);
	if (colFrom) {
        std::cerr << "Configuration in collision: " << q << std::endl;
        std::cerr << "Colliding frames: " << std::endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
		}
		return false;
	}
    return true;
}

/**
 * @brief getQ
 * @param robot
 * @param workcell
 * @param state
 * @param translation
 * @param cylinder_frame
 * @return
 */
rw::math::Q getQ(rw::models::SerialDevice::Ptr robot,
                 rw::models::WorkCell::Ptr workcell,
                 rw::kinematics::State &state,
                 rw::math::Transform3D<> translation,
                 rw::kinematics::MovableFrame::Ptr cylinder_frame) {
    // create detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(
        new rw::proximity::CollisionDetector(
            workcell,
            rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()
        )
    );

    // move cylinder to translation
    cylinder_frame->moveTo(translation, state);

    // create name of frames
    const std::string robot_name = robot->getName();
    const std::string robot_base_name = robot_name + ".Base";
    const std::string robot_tcp_name = robot_name + ".TCP";

    // find frames
    rw::kinematics::Frame *frame_goal = workcell->findFrame("GraspTarget");
    rw::kinematics::Frame *frame_tcp = workcell->findFrame("GraspTCP");
    rw::kinematics::Frame *frame_robot_base = workcell->findFrame(robot_base_name);
    rw::kinematics::Frame *frame_robot_tcp = workcell->findFrame(robot_tcp_name);

    // check for existence
    if (frame_goal == NULL) { std::cerr << "Could not find GraspTarget!" << std::endl; }
    if (frame_tcp == NULL) { std::cerr << "Could not find GraspTCP!" << std::endl; }
    if (frame_robot_base == NULL) { std::cerr << "Could not find " << robot_base_name << "!" << std::endl; }
    if (frame_robot_tcp == NULL) {std::cerr << "Could not find " << robot_tcp_name << "!" << std::endl; }

    // make helper transformations
    rw::math::Transform3D<> frame_base2goal = rw::kinematics::Kinematics::frameTframe(frame_robot_base, frame_goal, state);
    rw::math::Transform3D<> frame_tcp2robot_tcp = rw::kinematics::Kinematics::frameTframe(frame_tcp, frame_robot_tcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> target_at = frame_base2goal * frame_tcp2robot_tcp;

    // get configurations for collisions
    rw::invkin::ClosedFormIKSolverUR::Ptr closed_form_solver = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot, state));
    std::vector<rw::math::Q> solutions = closed_form_solver->solve(target_at, state);

    // check the configurations for a collision free solution
    rw::math::Q collision_free_solution;
    for (unsigned int i = 0; i < solutions.size(); i++) {
        // set the robot at the configuration
        robot->setQ(solutions[i], state);

        // check if it is in collision
        if (!detector->inCollision(state, NULL, true)) {
            collision_free_solution = solutions[i];
            break; // only need one
        }
    }

    std::cout << collision_free_solution << std::endl;
    return collision_free_solution;
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

/**
 * The functions is inspired from the exercise in lecture 6 of robotics.
 * @brief calculates the path of RRT
 * @param step_size     :   size of the steps in RRT
 * @param lua_path      :   path to the lua file
 * @return              :   0 if ok else -1
 */
std::vector<double> calculatePathRRT(const std::string luaPath,
                                     const rw::math::Q from,
                                     const rw::math::Vector3D<> position,
                                     const double stepSize=0.05) {
    std::vector<double> result;

    // open file to store path for robworks
    std::ofstream myfile;
    myfile.open(luaPath);

    // set the random seed
    rw::math::Math::seed();

    // load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);

    // find the tool frame
    rw::kinematics::Frame *toolFrame = wc->findFrame("Tool");
    if (toolFrame == NULL) {
        std::cerr << "Tool not found!" << std::endl;
        return result;
    }

    // find cylinder frame
    rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
    if (cylinderFrame == NULL) {
        std::cerr << "Cylinder frame not found!" << std::endl;
        return result;
    }

    // find device
    rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(DEVICE_NAME);
    if (device == NULL) {
        std::cerr << "Device: " << DEVICE_NAME << " not found!" << std::endl;
        return result;
    }

    //Get the state
    rw::kinematics::State state = wc->getDefaultState();

    // move cylinder to position
    rw::math::Transform3D<> pose(position);
    cylinderFrame->moveTo(pose, state);

    //These Q's contains the start and end configurations
//    rw::math::Rotation3D<> rot = cylinder_frame->getTransform(state).R();
//    rw::math::Vector3D<> pos = cylinder_frame->getTransform(state).P();
//    rw::math::Transform3D<> trans(pos, rot);
//    std::cout << "From RPY: "
//              << rw::math::RPY<>(trans.R())[0] * rw::math::Rad2Deg
//              << " " << rw::math::RPY<>(trans.R())[1] * rw::math::Rad2Deg
//              << " " << rw::math::RPY<>(trans.R())[2] * rw::math::Rad2Deg
//              << std::endl;
//    std::cout << "From position: " << pos << std::endl;
//    rw::math::Q from = getQ(device, wc, state, trans, cylinder_frame);
//    std::cout << from << std::endl;

//    pos(0) = 0.3; pos(1) = -0.5; pos(2) = 0.15;
//    trans = rw::math::Transform3D<>(pos, rot);
//    rw::math::Q to = getQ(device, wc, state, trans, cylinder_frame);
//    std::cout << to << std::endl;

    // configurations to grap the cylinder from the top
//    rw::math::Q from(6, 2.5, -2.099, -1.593, -0.991, 1.571, 0.0); // cylinder (-0.25, 0.474, 0.15)
//    rw::math::Q from(6, 2.185, -1.795, -1.987, -0.915, 1.571, 0.0); // cylinder (0.0, 0.474, 0.15)
//    rw::math::Q from(6, 1.693, -1.728, -2.068, -0.932, 1.571, 0.0); // cylinder (0.25, 0.474, 0.15)
    rw::math::Q to (6, -1.154, -1.798, -1.993, -0.934, 1.571, 0.0); // goal (0.3, -0.5, 0.15)

    //Set Q to the initial state and grip the bottle frame
    device->setQ(from, state);
    rw::kinematics::Kinematics::gripFrame(cylinderFrame, toolFrame, state);

    rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector, device, state);

    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),
                                                                                          constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint,
                                                                                                   sampler,
                                                                                                   metric,
                                                                                                   stepSize,
                                                                                                   rwlibs::pathplanners::RRTPlanner::RRTConnect);
    if (!checkCollisions(device, state, detector, from)) {
        std::cout << "Collision from!" << std::endl;
        return result;
    }
    if (!checkCollisions(device, state, detector, to)) {
        std::cout << "Collision to!" << std::endl;
        return result;
    }

    //Creates the functions for the LUA script and initializes the position and state of the robot
    myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n"
              <<"state = wc:getDefaultState()"
              <<"\ndevice = wc:findDevice(\"UR-6-85-5-A\")"
              <<"\ngripper = wc:findFrame(\"Tool\")"
              <<"\nbottle = wc:findFrame(\"Cylinder\")\n"
              <<"table = wc:findFrame(\"Table\")\n\n"
              <<"function setQ(q)\n"
              <<"qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
              <<"device:setQ(qq,state)\n"
              <<"rws.getRobWorkStudio():setState(state)\n"
              <<"rw.sleep(0.1)\n"
              <<"end\n\n"
              <<"function attach(obj, tool)\n"
              <<"rw.gripFrame(obj, tool, state)\n"
              <<"rws.getRobWorkStudio():setState(state)\n"
              <<"rw.sleep(0.1)\n"
              <<"end\n\n";

    //cout << "Planning from " << from << " to " << to << endl;
    rw::trajectory::QPath path;
    rw::common::Timer t;
    t.resetAndResume();

    // use the planner to find a trajectory between the configurations
    planner->query(from, to, path);
    planner->make(constraint);

    // check if the time extends the max time
    t.pause();
    if (t.getTime() >= MAXTIME) {
        std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
        return result;
    }

    // appends the path to the LUA script. This file can be "played" in RobWorkStudio.
    double distance = 0;
    for (unsigned int i = 0; i < path.size(); i++) {
        if(i == 1) { myfile << "attach(bottle, gripper)\n"; }
        if(i >= 1) {
            double tmp = 0.0;
            tmp += std::pow((path.at(i)(0) - path.at(i-1)(0)), 2);
            tmp += std::pow((path.at(i)(1) - path.at(i-1)(1)), 2);
            tmp += std::pow((path.at(i)(2) - path.at(i-1)(2)), 2);
            tmp += std::pow((path.at(i)(3) - path.at(i-1)(3)), 2);
            tmp += std::pow((path.at(i)(4) - path.at(i-1)(4)), 2);
            tmp += std::pow((path.at(i)(5) - path.at(i-1)(5)), 2);
            distance += std::sqrt(tmp);
        }
        //cout << path.at(i)(0) << endl;
        myfile << "setQ({"
               << path.at(i)(0) << " , "
               << path.at(i)(1) << " , "
               << path.at(i)(2) << " , "
               << path.at(i)(3) << " , "
               << path.at(i)(4) << " , "
               << path.at(i)(5) << "})"
               << "\n";
    }
    myfile.close();

    unsigned int nodes = path.size();
    result.push_back(stepSize);
    result.push_back(t.getTime());
    result.push_back(distance);
    result.push_back(nodes);
    return result;
}

rw::trajectory::QPath getRRTPath(const rw::models::WorkCell::Ptr wc, const rw::models::SerialDevice::Ptr device,
                                 const rw::kinematics::State state, const rw::math::Q from, const rw::math::Q to,
                                 const double stepSize=0.6) {
    rw::trajectory::QPath result;

    // set the random seed
    rw::math::Math::seed();

    rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector, device, state);

    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),
                                                                                          constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint,
                                                                                                   sampler,
                                                                                                   metric,
                                                                                                   stepSize,
                                                                                                   rwlibs::pathplanners::RRTPlanner::RRTConnect);
    // use the planner to find a trajectory between the configurations
    planner->query(from, to, result);
    planner->make(constraint);

    return result;
}

std::vector<Q> getConfigurationsFromPath(rw::trajectory::QPath path) {
    std::vector<Q> result;
    for (std::size_t i = 0; i < path.size(); i++) {
        Q q = path.at(i);
        result.push_back(q);
    }
    return result;
}

std::map<int, rw::math::Q> q_interpolation_nb(std::vector<rw::math::Q> points, std::vector<double> times) {

    double time_step = 0.1;
    double time = 0;
    double t = 0;
    int time_index = 0;

    // Linear interpolation between the points:
    std::map<int, Q> interpolation;

    for (size_t j = 0; j < times.size(); j++) {

        // Calculate the number of steps for the next loop.
        int interval = int(std::round((time+times[j])-time));

        for (int i = 0; i <= interval/time_step; i++) {
            interpolation.insert(std::pair<int, rw::math::Q>(time_index, points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j])));
            t += time_step;
            time_index++;
        }

        time_index--;
        time += times[j];
        t = time;
    }

    return interpolation;
}

/*
 * MAIN ENTRY POINT
 */
int main(int argc, char* argv[]) {
    std::cout << "\nProgram started\n" << std::endl;

    std::cout << "Input is " << argv[1] << std::endl;
    const std::string input = argv[1];

    if (input == "analysis") {
        std::cout << "Starting analysis of RRT-Connect.." << std::endl;
        std::vector<std::string> paths {
            "../../cylinder_0.0/",
            "../../cylinder_0.25/",
            "../../cylinder_-0.25/"
        };
        std::vector<rw::math::Q> froms {
            rw::math::Q(6, 2.185, -1.795, -1.987, -0.915, 1.571, 0.0),  // cylinder (0.0, 0.474, 0.15)
            rw::math::Q(6, 1.693, -1.728, -2.068, -0.932, 1.571, 0.0),  // cylinder (0.25, 0.474, 0.15)
            rw::math::Q(6, 2.5, -2.099, -1.593, -0.991, 1.571, 0.0)     // cylinder (-0.25, 0.474, 0.15)
        };
        std::vector<rw::math::Vector3D<>> positions {
            rw::math::Vector3D<>(0.0, 0.474, 0.15),
            rw::math::Vector3D<>(0.25, 0.474, 0.15),
            rw::math::Vector3D<>(-0.25, 0.474, 0.15)
        };
        for (unsigned int j = 0; j < paths.size(); j++) {
            std::cout << "Writing to file: " << paths[j] << std::endl;
            std::cout << "Using configurations: " << froms[j] << std::endl;
            std::vector<std::vector<double>> datas;
            for (double stepSize = 0.05; stepSize < MAX_STEP_SIZE + 0.05; stepSize += 0.05) {
                std::string luaPath = paths[j] + "path_" + convertDoub2Str(stepSize) + ".lua";
                std::cout << "Writing to lua file: " << luaPath << std::endl;
                for (unsigned int i = 0; i < MAX_ITERATIONS; i++) {
                    std::vector<double> result = calculatePathRRT(luaPath, froms[j], positions[j], stepSize);
                    if (result.size() == 0) {
                        std::cout << "Terminate the program!" << std::endl;
                        return 0;
                    }
                    // print info
                    std::cout << "Step size: " << result[0] << "\t" <<
                                 "Time: " << result[1] << "\t" <<
                                 "Distance: " << result[2] << "\t" <<
                                 "Nodes: " << result[3] << "\t" <<
                                 "Iteration: " << i+1 << "/" << MAX_ITERATIONS << std::endl;
                    datas.push_back(result);
                }
            }

            // write data to file
            std::string path = paths[j] + "data.txt";
            std::cout << "Writing data to file: " << path << std::endl;
            std::ofstream my_file;
            my_file.open(path);
            for (std::vector<double> data : datas) {
                std::string str = std::to_string(data[0]) + " "
                                + std::to_string(data[1]) + " "
                                + std::to_string(data[2]) + " "
                                + std::to_string(data[3]) + "\n";
                my_file << str;
            }
            my_file.close();
        }
    }
    else if (input == "trajectory") {
        std::cout << "Creating RRT path with step size 0.6.." << std::endl;

        // load workcell
        rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);

        // find the tool frame
        rw::kinematics::Frame *toolFrame = wc->findFrame("Tool");
        if (toolFrame == NULL) {
            std::cerr << "Tool not found!" << std::endl;
            return -1;
        }

        // find cylinder frame
        rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
        if (cylinderFrame == NULL) {
            std::cerr << "Cylinder frame not found!" << std::endl;
            return -1;
        }

        // find device
        rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(DEVICE_NAME);
        if (device == NULL) {
            std::cerr << "Device: " << DEVICE_NAME << " not found!" << std::endl;
            return -1;
        }

        // get tcp frame
        Frame *tcpFrame = wc->findFrame(DEVICE_NAME + ".TCP");
        if (tcpFrame == NULL) {
            std::cerr << "TCP frame not found!" << std::endl;
            return -1;
        }

        // get table frame
        Frame *tableFrame = wc->findFrame("Table");
        if (tableFrame == NULL) {
            std::cerr << "Table frame not found!" << std::endl;
            return -1;
        }

        // get the default state
        rw::kinematics::State state = wc->getDefaultState();

        // from and to
        Q to(6, 1.55, -1.798, -2.007, -0.909, 1.571, 0.0);
        Q from(6, -1.079, -2.083, -1.555, -1.075, 1.571, 0.0);

        //Set Q to the initial state and grip the bottle frame
        device->setQ(from, state);
        rw::kinematics::Kinematics::gripFrame(cylinderFrame, toolFrame, state);

        // calculate rrt path
        rw::trajectory::QPath path = getRRTPath(wc, device, state, from, to);

        // create linear interpolation between configurations
        std::vector<Q> qs = getConfigurationsFromPath(path);
        std::cout << "Number of configurations --> " << qs.size() << std::endl;
        std::vector<double> times;
        for (std::size_t i = 0; i < qs.size()-1; i++) {
            // position 1
            device->setQ(qs[i],state);
            Vector pos1 = device->baseTend(state).P();
            // position 2
            device->setQ(qs[i+1], state);
            Vector pos2 = device->baseTend(state).P();
            // difference
            rw::math::EuclideanMetric<Vector> metric;
            double diff = metric.distance(pos1, pos2);
            double time = std::round(diff / VELOCITY);
            time = ((int)time == 0) ? 1 : time;
            std::cout << "\tTime " << i << " --> " << time << std::endl;
            times.push_back(time);
        }
        std::map<int, Q> interpolation = q_interpolation_nb(qs, times);
        std::cout << "Interpolaton size --> " << interpolation.size() << std::endl;

        // calculate forward kinematic for each configuration
        std::cout << "\nCalculating forward kinematics.." << std::endl;
        const std::string dataFile = "../../forward_kinematics.txt";
        std::ofstream file;
        file.open(dataFile);
        rw::trajectory::TimedStatePath statePathQ;
        for (std::map<int,Q>::iterator i = interpolation.begin(); i != interpolation.end(); i++) {
            // get info
            double time = i->first/10.0;
            Q q = i->second;
            device->setQ(q, state);
            Pose baseTtool = device->baseTend(state);
            Vector pos = baseTtool.P();
            Rotation rot = baseTtool.R();
            std::cout << "\nConfiguration " << q << std::endl;
            std::cout << "Position --> " << pos << std::endl;
            std::cout << "Rotation --> " << rot << std::endl;
            std::cout << "Time --> " << time << std::endl;

            // write to file
            file << time << " "
                 << pos(0) << " " << pos(1) << " " << pos(2) << " "
                 << rot.getRow(0)[0] << " " << rot.getRow(0)[1] << " " << rot.getRow(0)[2] << " "
                 << rot.getRow(1)[0] << " " << rot.getRow(1)[1] << " " << rot.getRow(1)[2] << " "
                 << rot.getRow(2)[0] << " " << rot.getRow(2)[1] << " " << rot.getRow(2)[2] << "\n";

            // create rwplay file
            statePathQ.push_back(rw::trajectory::TimedState(time, state));
        }
        file.close();
        rw::loaders::PathLoader::storeTimedStatePath(*wc, statePathQ, "../../trajectory.rwplay");
    }

    std::cout << "\nProgram ended\n" << std::endl;
	return 0;
}
