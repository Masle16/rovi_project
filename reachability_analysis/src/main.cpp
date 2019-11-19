#include <iostream>
#include <fstream>
#include <string>

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

#define WC_FILE "../../workcell/Scene.wc.xml"

/**
 * @brief showUsages
 */
void showUsages() {
    std::cerr << "-cylinder_x -cylinder_y -cylinder_z -position_stepsize -data_file" << std::endl;
    std::cerr << "0.0 0.56 0.15 0.1 /path/to/data/file.txt" << std::endl;
}

/**
 * @brief save2file: saves the position of the robot base with the corresponding collision free solutions
 * @param x: the x position on the table
 * @param y: the y position on the table
 * @param solutions: the number of collision free solutions at (x, y)
 */
void save2file(const std::vector<float> &x,
               const std::vector<float> &y,
               const std::vector<unsigned int> &solutions,
               const std::string &file_path) {
    // check if vectors have the same length
    if (x.size() != y.size() && y.size() != solutions.size()) { return; }

    // write to file
    //std::string file_path = "../../data_cylinder_top.txt";
    std::ofstream my_file;
    my_file.open(file_path);
    for (unsigned int i = 0; i < x.size(); i++) {
        std::string x_str = std::to_string(x[i]), y_str = std::to_string(y[i]), s_str = std::to_string(solutions[i]);
        std::string str = x_str + " " + y_str + " " + " " + s_str + "\n";
        my_file << str;
    }
    my_file.close();
}

/**
 * The function was given in lecture 5 of robotics in the exercise.
 * @brief gets all the configurations of the solved inverse kinematics
 * @param nameGoal  :   name of the object to reach
 * @param nameTcp   :   the tool center point of the tool
 * @param robot     :   the device
 * @param wc        :   the workcell
 * @param state     :   the state of the workcell
 * @return all the configurations of the solved inverse kinematics
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

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot, state));
    return closedFormSovler->solve(targetAt, state);
}

/**
 * This function takes inspiration from the exercise from lecture 5 in robotics.
 * @brief getCollisionFreeSolutions: Get the collisions free solution for the robot to grap the object
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
    // create detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    std::vector<rw::math::Q> collisionFreeSolutions;

    // get collision free solutions for every degree around the roll axis
    for (double roll_angle = 0; roll_angle < 360.0; roll_angle += 1.0) {
        // rotate object
        rw::math::RPY<> rot(roll_angle*rw::math::Deg2Rad, 0, 0);
        rw::math::Vector3D<> pos = object->getTransform(state).P();
        rw::math::Transform3D<> trans(pos, rot);
        object->moveTo(trans, state);

        // get configurations for the GraspTarget == GraspTCP
        std::vector<rw::math::Q> solutions = getConfigurations(target, "GraspTCP", device, wc, state);

        // check all the configurations for collisions
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
//    if (collisionFreeSolutions.size() > 0) {
//        TimedStatePath tStatePath;
//        double time=0;
//        for (unsigned int i = 0; i < collisionFreeSolutions.size(); i++) {
//            device->setQ(collisionFreeSolutions[i], state);
//            tStatePath.push_back(TimedState(time, state));
//            time+=0.01;
//        }
//        // Store at rwplay file
//        rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
//    }

    return collisionFreeSolutions;
}

/**
 * MAIN FUNCTION
 */
int main(int argc, char *argv[]) {
    rw::math::Vector3D<> cylinder_pos;
    double stepsize;
    std::string file_path;

    if (argc < 5) {
        showUsages();
        return 0;
    }
    else if ((std::string(argv[0]) == "-h") || (std::string(argv[0]) == "--help")) {
        showUsages();
        return 0;
    }
    else {
        cylinder_pos = rw::math::Vector3D<>(std::stod(argv[1]),
                                            std::stod(argv[2]),
                                            std::stod(argv[3]));
        std::cout << "Cylinder position: " << cylinder_pos << std::endl;

        stepsize = std::stod(argv[4]);
        std::cout << "Robot base stepsize: " << stepsize << std::endl;

        file_path = argv[5];
        std::cout << "File path: " << file_path << std::endl;
    }


    // load workcell;
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    if (NULL == wc) {
        RW_THROW("COULD NOT LOAD WORKCELL ... check path!");
        return -1;
    }

    // find device
    const std::string device_name = "UR-6-85-5-A";
    rw::models::SerialDevice::Ptr robot_ur5 = wc->findDevice<rw::models::SerialDevice>(device_name);
    if (NULL == robot_ur5) {
        RW_THROW("Could not load " + device_name + " ... check model");
        return -1;
    }

    // find ur5 robot base frame
    const std::string base_name = "URReference";
    rw::kinematics::MovableFrame::Ptr base_frame = wc->findFrame<rw::kinematics::MovableFrame>(base_name);
    if (NULL == base_frame) {
        RW_THROW("Could not load " + base_name + " ... check model");
        return -1;
    }

    // find cylinder
    const std::string cylinder_name = "Cylinder";
    rw::kinematics::MovableFrame::Ptr cylinder_frame = wc->findFrame<rw::kinematics::MovableFrame>(cylinder_name);
    if (NULL == cylinder_frame) {
        RW_THROW("Could not find movable frame " + cylinder_name + " ... check model");
        return -1;
    }

    // get start state and default rotation
    rw::kinematics::State state = wc->getDefaultState();
    rw::math::Rotation3D<> base_rot = base_frame->getTransform(state).R();
    rw::math::Rotation3D<> cylinder_rot = cylinder_frame->getTransform(state).R();

    // generate position for reachability analysis
    std::vector<rw::math::Vector3D<>> base_positions;
    for (float y = 0.2; y >= -0.5; y -= stepsize) { // y goes from 0.2 to -0.5
        for (float x = -0.3; x <= 0.3; x += stepsize) { // x goes from -0.3 to 0.3
            // check for goal position
            if (y < -0.3 && x > 0.1) { continue; }
            rw::math::Vector3D<> pos(x, y, 0.0);
            base_positions.push_back(pos);
        }
    }
    std::cout << "Number of positions to analyse => " << base_positions.size() << std::endl;

//    // create vector with cylinder positions
//    std::vector<rw::math::Vector3D<>> cylinder_positions = {
//        rw::math::Vector3D<>(-0.36, 0.56, 0.15),
//        rw::math::Vector3D<>(0.0, 0.56, 0.15),
//        rw::math::Vector3D<>(0.36, 0.56, 0.15),
//        rw::math::Vector3D<>(0.3, -0.5, 0.15)
//    };

    // check for every base position the collision free solution to each cylinder position
    std::vector<unsigned int> number_of_solutions;
    std::vector<float> x_positions, y_positions;
    std::string rwplay = "";
    for (unsigned int i = 0; i < base_positions.size(); i++) {
        // move base frame
        rw::math::Transform3D<> base_trans(base_positions[i], base_rot);
        base_frame->moveTo(base_trans, state);

        // get collision free solutions
        unsigned int solutions = 0;
//        for (unsigned int j = 0; j < cylinder_positions.size(); j++) {
//            // generate rwplay file
//            const std::string folder = "../rwplays_cylinder_top/";
//            rwplay = folder + "_position" + std::to_string(i) + "_cylinder" + std::to_string(j+1) + ".rwplay";

//            // move cylinder
//            rw::math::Transform3D<> cylinder_trans(cylinder_positions[j], cylinder_rot);
//            cylinder_frame->moveTo(cylinder_trans, state);

//            // get collision free solutions
//            std::vector<rw::math::Q> collision_free_solutions = getCollisionFreeSolutions(wc, robot_ur5, cylinder_frame, "GraspTarget", rwplay, state);

//            // store total number of solutions
//            solutions += collision_free_solutions.size();
//        }

        // generate rwplay file
//        const std::string folder = "../rwplays_cylinder_top/";
//        rwplay = folder + "_position" + std::to_string(i) + "_cylinder_-0.36_0.56" + ".rwplay";

        // move cylinder
        rw::math::Transform3D<> cylinder_trans(cylinder_pos, cylinder_rot);
        cylinder_frame->moveTo(cylinder_trans, state);

        // get collision free solutions
        std::vector<rw::math::Q> collision_free_solutions = getCollisionFreeSolutions(wc, robot_ur5, cylinder_frame, "GraspTarget", rwplay, state);

        // store total number of solutions
        solutions += collision_free_solutions.size();

        // save data
        x_positions.push_back(base_positions[i](0));
        y_positions.push_back(base_positions[i](1));
        number_of_solutions.push_back(solutions);

        // show process
        if (i % 10 == 0) { std::cout << i << " / " << base_positions.size() << std::endl; }
    }
    std::cout << base_positions.size() << " / " << base_positions.size() << std::endl;

    // save all data to file_path
    save2file(x_positions, y_positions, number_of_solutions, file_path);

    return 0;
}
