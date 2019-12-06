#include <iostream>
#include "p2p_interpolation.h"
#include <rw/rw.hpp>
#include <vector>

int main() {

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/bjarke/Documents/ROVI_project/rovi_project/Project_WorkCell/Scene.wc.xml");
    if (wc.isNull()) {
        std::cout << "Error loading workcell." << std::endl;
        return -1;
    }

    const std::string deviceName = "UR-6-85-5-A";
    const std::string baseName = "URReference";

    rw::models::SerialDevice::Ptr robot_ur5 = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if (robot_ur5.isNull()) {
        std::cout << "Error finding device." << std::endl;
        return -1;
    }

    rw::kinematics::State state = wc->getDefaultState();



    // Test Q points.
    rw::math::Q p1(6, 2.259, -1.861, -1.936, -0.918, 1.573, 0.689);
    rw::math::Q p2(6, 2.259, -1.669, -1.549, -1.497, 1.573, 0.689);
    rw::math::Q p3(6, -0.619, -1.656, 1.056, 0.614, 1.235, 1.635);
    rw::math::Q p4(6, 0.329, -1.137, 1.467, -0.451, 0.504, 1.717);
    rw::math::Q p5(6, -0.825, -1.824, -1.564, -1.356, 1.542, 2.387);
    rw::math::Q p6(6, -0.825, -1.937, -1.775, -0.971, 1.541, 0.785);

    // Test tool space points.
    rw::math::Vector3D<double> xyz1(-25, 47.5, 18.5);
    rw::math::Vector3D<double> xyz2(-25, 47.5, 41);
    rw::math::Vector3D<double> xyz3(-37.5, 10, 65);
    rw::math::Vector3D<double> xyz4(-51, -36.5, 27);
    rw::math::Vector3D<double> xyz5(31, -50, 33);
    rw::math::Vector3D<double> xyz6(31, -50, 20.5);
    rw::math::RPY<double> rpy1(0, 0, 3.138);
    rw::math::RPY<double> rpy2(0, 0, 3.138);
    rw::math::RPY<double> rpy3(1.5, -1.5, 4.5);
    rw::math::RPY<double> rpy4(2, -1.5, -0.6);
    rw::math::RPY<double> rpy5(3.1, 0, -3.1);
    rw::math::RPY<double> rpy6(3.1, 0, -3.1);

    std::vector<rw::math::Q> trajectory = {p1, p2, p3, p4, p5, p6};
    std::vector<rw::math::Vector3D<double>> trajectory_xyz = {xyz1, xyz2, xyz3, xyz4, xyz5, xyz6};

    std::vector<double> times = {4, 3, 2, 3, 4};

    P2P_interpolator interpolator;

    std::map<int, rw::math::Q> path = interpolator.q_interpolation(trajectory, times);

    rw::trajectory::TimedStatePath statePath;


    /***** Interpolate Q's *****/

    /*robot_ur5->setQ(path.find(int(0))->second, state);
    rw::kinematics::Frame *frame_goal = wc->findFrame("GraspTarget");
    rw::kinematics::Frame *frame_tcp = wc->findFrame("GraspTCP");
    rw::kinematics::Kinematics::gripFrame(frame_goal, frame_tcp, state); // Code for gripping the object
    */

    for (size_t i = 0; i < path.size(); i++) {
        robot_ur5->setQ(path.find(int(i))->second, state);
        std::cout << double(path.find(int(i))->first)/10 << ", " << path.find(int(i))->second << std::endl;
        statePath.push_back(rw::trajectory::TimedState(double(path.find(int(i))->first)/10, state));
    }



    rw::math::Rotation3D<double> rot1 = rpy1.toRotation3D();
    rw::math::Quaternion<double> quat1(rot1);
    rw::math::Rotation3D<double> rot2 = rpy2.toRotation3D();
    rw::math::Quaternion<double> quat2(rot2);
    rw::math::Rotation3D<double> rot3 = rpy3.toRotation3D();
    rw::math::Quaternion<double> quat3(rot3);
    rw::math::Rotation3D<double> rot4 = rpy4.toRotation3D();
    rw::math::Quaternion<double> quat4(rot4);
    rw::math::Rotation3D<double> rot5 = rpy5.toRotation3D();
    rw::math::Quaternion<double> quat5(rot5);
    rw::math::Rotation3D<double> rot6 = rpy6.toRotation3D();
    rw::math::Quaternion<double> quat6(rot6);

    std::vector<rw::math::Quaternion<double>> quat_rotations = {quat1, quat2, quat3, quat4, quat5, quat6};

    std::map<int, rw::math::Vector3D<double>> xyz_path = interpolator.xyz_interpolation(trajectory_xyz, times);
    std::map<int, rw::math::Quaternion<double>> quat_int = interpolator.quat_interpolation(quat_rotations, times);

    std::map<int, rw::math::Quaternion<double>>::iterator quat_it;
    for (quat_it = quat_int.begin(); quat_it != quat_int.end(); quat_it++) {
        std::cout << double(quat_it->first)/10 << ", " << quat_it->second << std::endl;
    }

    // Go through the positions and rotations and do inverse kinematics.

    rw::invkin::ClosedFormIKSolverUR::Ptr ikSolver = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot_ur5, state));
    rw::trajectory::TimedStatePath statePath_ts;


    rw::math::Q last_best = p1;
    for (int i = 0; i < int(xyz_path.size()); i++) {
        rw::math::Transform3D<double> trans(xyz_path.find(i)->second/100., quat_int.find(i)->second.toRotation3D());
        //std::cout << trans << std::endl;
        std::vector<rw::math::Q> ik_result = ikSolver->solve(trans, state);
        //std::cout << ik_result.size() << std::endl;

        size_t best_result = 0;
        rw::math::Q q_best;
        rw::math::Q q_dif;
        q_best = last_best-ik_result[0];
        for (size_t j = 1; j < ik_result.size(); j++) {
            q_dif = last_best-ik_result[j];
            if (q_dif.norm1() < q_best.norm1()) {
                q_best = q_dif;
                best_result = j;
            }
        }
        last_best = ik_result[best_result];
        std::cout << last_best << std::endl;

        robot_ur5->setQ(ik_result[best_result], state);
        statePath_ts.push_back(rw::trajectory::TimedState(double(xyz_path.find(i)->first)/10., state));
    }


    /***** Interpolate xyz *****/

    // Transform into quaternions from rotation3d. Then interpolate and do inverse kinematics.
/*
    // Inverse kinematics:

    const std::string robot_name = robot_ur5->getName();
    const std::string base_name = robot_name + ".Base";
    const std::string tcp_name = robot_name + ".TCP";
    rw::kinematics::Frame *frame_base = wc->findFrame(base_name);
    rw::kinematics::Frame *frame_tcp = wc->findFrame(tcp_name);

    rw::math::Rotation3D<double> rot1 = rpy1.toRotation3D();
    rw::math::Quaternion<double> quat1(rot1);
    rw::math::Transform3D<double> trans1(xyz1, rot1);
    rw::math::Transform3D<double> trans2(xyz2, rpy2.toRotation3D());

    //rw::math::Transform3D<> frame_base2goal = frame_base->fTf(trans1, state);
    rw::math::Transform3D<> test_t(frame_tcp->getTransform(state).P(), frame_tcp->getTransform(state).R());


    rw::invkin::JacobianIKSolver::Ptr ikSolver = rw::common::ownedPtr(new rw::invkin::JacobianIKSolver(robot_ur5, state));
    std::vector<rw::math::Q> ik_result = ikSolver->solve(test_t, state);

    std::cout << test_t << ", " << ik_result.size() << std::endl;


*/


/*
    // Test of inverse kinematics
    // create name of frames
    const std::string robot_name = robot_ur5->getName();
    const std::string robot_base_name = robot_name + ".Base";
    const std::string robot_tcp_name = robot_name + ".TCP";

    // find frames
    rw::kinematics::Frame *frame_goal = wc->findFrame("GraspTarget");
    rw::kinematics::Frame *frame_tcp = wc->findFrame("GraspTCP");
    rw::kinematics::Frame *frame_robot_base = wc->findFrame(robot_base_name);
    rw::kinematics::Frame *frame_robot_tcp = wc->findFrame(robot_tcp_name);

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

    rw::math::Transform3D<double> trans2(xyz2/100, rpy2.toRotation3D());

    // get configurations for collisions
    rw::invkin::ClosedFormIKSolverUR::Ptr closed_form_solver = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot_ur5, state));
    std::vector<rw::math::Q> solutions = closed_form_solver->solve(trans2, state);

    std::cout << trans2 << std::endl << target_at << ", " << solutions.size() << std::endl;
*/

    rw::loaders::PathLoader::storeTimedStatePath(*wc,statePath,"rw_play.rwplay");
    rw::loaders::PathLoader::storeTimedStatePath(*wc,statePath_ts,"rw_play_ts.rwplay");

    return 0;
}
