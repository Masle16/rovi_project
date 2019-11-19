#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// using namespace std;
// using namespace rw::common;
// using namespace rw::math;
// using namespace rw::kinematics;
// using namespace rw::loaders;
// using namespace rw::models;
// using namespace rw::pathplanning;
// using namespace rw::proximity;
// using namespace rw::trajectory;
// using namespace rwlibs::pathplanners;
// using namespace rwlibs::proximitystrategies;

#define MAXTIME 60.
//#define ESTEPSIZE 0.005
#define WC_FILE "../../workcell/Scene.wc.xml"
#define DEVICE_NAME "UR-6-85-5-A"
#define MAX_STEP_SIZE 2.0
#define MAX_ITERATIONS 50

void showUsages() {
    std::cerr << "-path_to_lua_file" << std::endl;
    std::cerr << "/path/to/file.lua" << std::endl;
}

std::string convertDoub2Str(const double &input) {
    std::ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(2);
    streamObj << input;
    return streamObj.str();
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

int main(int argc, char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

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

    std::cout << "\nProgram ended\n" << std::endl;
	return 0;
}
