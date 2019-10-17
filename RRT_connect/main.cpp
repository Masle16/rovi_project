#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
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
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
    return true;
}

/**
 * The functions is inspired from the exercise in lecture 6 of robotics.
 * @brief calculate_path_rrt
 * @param wcFile
 * @param deviceName
 * @param stepSize
 * @return
 */
int calculate_path_rrt(const std::string wcFile, const std::string deviceName, const double stepSize=0.05) {
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;

    // open file to store path for robworks
    std::ofstream myfile;
    myfile.open("path.lua");

    // set the random seed
    rw::math::Math::seed();

    // load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);

    // find the tool frame
    rw::kinematics::Frame *tool_frame = wc->findFrame("Tool");
    if (tool_fream == NULL) {
        std::cerr << "Tool not found!" << std::endl;
        return -1;
    }

    // find bottle frame
    rw::kinematics::Frame *bottle_frame = wc->findFrame("Bottle");
    if (bottle_frame == NULL) {
        std::cerr << "Bottle frame not found!" << std::endl;
        return -1;
    }

    // find device
    rw::models::Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return -1;
    }

    //Get the state
    rw::kinematics::State state = wc->getDefaultState();

    //These Q's contains the start and end configurations
    rw::math::Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
    rw::math::Q to(6,1.571, 0.006, 0.030, 0.153, 0.762, 4.490);

    //Set Q to the initial state and grip the bottle frame
    device->setQ(from, state);
    rw::kinematics::Kinematics::gripFrame(bottle_frame, tool_frame, state);

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
        return -1;
    }
    if (!checkCollisions(device, state, detector, to)) {
        std::cout << "Collision to!" << std::endl;
        return -1;
    }

    //Creates the functions for the LUA script and initializes the position and state of the robot
    myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n"
              <<"state = wc:getDefaultState()"
              <<"\ndevice = wc:findDevice(\"KukaKr16\")"
              <<"\ngripper = wc:findFrame(\"Tool\")"
              <<"\nbottle = wc:findFrame(\"Bottle\")\n"
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
        return -1;
    }

    // appends the path to the LUA script. This file can be "played" in RobWorkStudio.
    double distance = 0;
    for (unsigned int i = 0; i< path.size(); i++)
    {
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

    std::cout << stepSize << " " << t.getTime() << " " << distance << std::endl;

    return 0;
}

int main(int argc, char** argv) {
    std::cout << "Program started" << std::endl;
    const string wcFile = "../lab6/Kr16WallWorkCell/Scene.wc.xml";
    const string deviceName = "KukaKr16";
    for (unsigned int i = 0; i < 10; i++) {
        double stepSize = std::pow(0.5, i);
        int result = calculate_path_rrt(wcFile, deviceName, stepSize);
        if (result != 0) {
            std::cout << "Terminate the program!" << std::endl;
            return 0;
        }
    }

	return 0;
}
