#include <iostream>
#include "p2p_interpolation.h"
#include <rw/rw.hpp>
#include <vector>
#include <string>
#include <fstream>

typedef rw::math::Quaternion<double> QUAT;

int main() {

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/bjarke/Documents/ROVI_project/rovi_project/p2p_interpolation/Project_WorkCell/Scene.wc.xml");
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


    /***** Test Trajectory *****/

    // Home
    rw::math::Q joint_config_home = {6, 1.571, -1.572, -1.572, -1.572, 1.571, 0};
    rw::math::Vector3D<double> xyz_home(10.75, 48.5, 44);
    rw::math::RPY<double> rpy_home(0, 0, 3.138);

    // Above pick (1)
    rw::math::Q joint_config_ap1 = {6, 2.265, -1.663, -1.607, -1.447, 1.573, 0.695};
    rw::math::Vector3D<double> xyz_ap1(-25.149, 47.093, 39);
    rw::math::RPY<double> rpy_ap1(0, 0, 3.138);

    // On pick (1)
    rw::math::Q joint_config_op1 = {6, 2.265, -1.845, -1.943, -0.928, 1.573, 0.695};
    rw::math::Vector3D<double> xyz_op1(-25, 47, 19);
    rw::math::RPY<double> rpy_op1(0, 0, 3.138);

    // Intermediate (1)
    rw::math::Q joint_config_intermediate1 = {6, 1.082, -1.128, -1.854, -1.917, 0.367, 0.650};
    rw::math::Vector3D<double> xyz_intermediate1(30, 17, 50);
    rw::math::RPY<double> rpy_intermediate1(-1.5, -1, 4);

    // Intermediate (2)
    rw::math::Q joint_config_intermediate2 = {6, -0.239, -1.911, -0.651, -1.546, 1.514, -0.918};
    rw::math::Vector3D<double> xyz_intermediate2(55, -25, 70);
    rw::math::RPY<double> rpy_intermediate2(-1, -0.5, 3.5);

    // Above place (1)
    rw::math::Q joint_config_above_place1 = {6, -0.875, -1.755, -1.504, -1.452, 1.568, -2.446};
    rw::math::Vector3D<double> xyz_above_place1(27.5, -50, 39);
    rw::math::RPY<double> rpy_above_place1(0, 0, 3.138);

    // On place (1)
    rw::math::Q joint_config_on_place1 = {6, -0.875, -1.922, -1.834, -0.953, 1.568, -2.446};
    rw::math::Vector3D<double> xyz_on_place1(27.5, -50, 19);
    rw::math::RPY<double> rpy_on_place1(0, 0, 3.138);

    /***************************/

    /***** Paths *****/

    std::vector<rw::math::Vector3D<double>> xyz_home_to_pick = {xyz_home, xyz_ap1, xyz_op1};
    std::vector<QUAT> quat_home_to_pick = {QUAT(rpy_home.toRotation3D()), QUAT(rpy_ap1.toRotation3D()), QUAT(rpy_op1.toRotation3D())};
    std::vector<double> times_home_to_pick = {2, 4};

    std::map<int, rw::math::Vector3D<double>> xyz_path_home_to_pick;
    std::map<int, QUAT> quat_path_home_to_pick;
    std::vector<rw::math::Q> inv_kin_home_to_pick;


    std::vector<rw::math::Vector3D<double>> xyz_pick_to_place = {xyz_op1, xyz_ap1, xyz_intermediate1, xyz_intermediate2, xyz_above_place1, xyz_on_place1};
    std::vector<QUAT> quat_pick_to_place = {QUAT(rpy_op1.toRotation3D()), QUAT(rpy_ap1.toRotation3D()), QUAT(rpy_intermediate1.toRotation3D()), QUAT(rpy_intermediate2.toRotation3D()), QUAT(rpy_above_place1.toRotation3D()), QUAT(rpy_on_place1.toRotation3D())};
    std::vector<rw::math::Q> q_pick_to_place = {joint_config_op1, joint_config_ap1, joint_config_intermediate1, joint_config_intermediate2, joint_config_above_place1, joint_config_on_place1};
    std::vector<double> times_pick_to_place = {4, 3, 2, 3, 4};

    std::map<int, rw::math::Vector3D<double>> xyz_path_pick_to_place;
    std::map<int, QUAT> quat_path_pick_to_place;
    std::vector<rw::math::Q> inv_kin_pick_to_place;


    std::vector<rw::math::Vector3D<double>> xyz_place_to_home = {xyz_on_place1, xyz_above_place1, xyz_intermediate2, xyz_home};
    std::vector<QUAT> quat_place_to_home = {QUAT(rpy_on_place1.toRotation3D()), QUAT(rpy_above_place1.toRotation3D()), QUAT(rpy_intermediate2.toRotation3D()), QUAT(rpy_home.toRotation3D())};
    std::vector<double> times_place_to_home = {4, 3, 4};

    std::map<int, rw::math::Vector3D<double>> xyz_path_place_to_home;
    std::map<int, QUAT> quat_path_place_to_home;
    std::vector<rw::math::Q> inv_kin_place_to_home;


    /*****************/

    P2P_interpolator interpolator;

    xyz_path_pick_to_place = interpolator.xyz_interpolation(xyz_pick_to_place, times_pick_to_place);
    quat_path_pick_to_place = interpolator.quat_interpolation(quat_pick_to_place, times_pick_to_place);


    rw::trajectory::TimedStatePath statePath_pick_to_place = interpolator.inverse_kin(robot_ur5, state, joint_config_op1, xyz_path_pick_to_place, quat_path_pick_to_place);
    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath_pick_to_place, "pick_to_place_ts.rwplay");

    // Joint space

    std::ofstream file_output;
    std::string jointSpace_filnemae = "jointspace_trajectory.txt";
    std::map<int, rw::math::Q> path = interpolator.q_interpolation(q_pick_to_place, times_pick_to_place);

    rw::trajectory::TimedStatePath statePath_pick_to_place_Q;

    rw::math::Transform3D<double> FK;
    file_output.open(jointSpace_filnemae);
    for (size_t i = 0; i < path.size(); i++) {
        robot_ur5->setQ(path.find(int(i))->second, state);
        //std::cout << double(path.find(int(i))->first)/10 << ", " << path.find(int(i))->second << std::endl;
        statePath_pick_to_place_Q.push_back(rw::trajectory::TimedState(double(path.find(int(i))->first)/10, state));
        FK = robot_ur5->baseTend(state);
        file_output << double(path.find(int(i))->first)/10. << "," << FK.P()[0] << "," << FK.P()[1] << "," << FK.P()[2] <<
                       "," << FK.R().getRow(0)[0] << "," << FK.R().getRow(0)[1] << "," << FK.R().getRow(0)[2] <<
                       "," << FK.R().getRow(1)[0] << "," << FK.R().getRow(1)[1] << "," << FK.R().getRow(1)[2] <<
                       "," << FK.R().getRow(2)[0] << "," << FK.R().getRow(2)[1] << "," << FK.R().getRow(2)[2] << std::endl;
    }
    file_output.close();

    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath_pick_to_place_Q, "pick_to_place_q.rwplay");



    /*
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
    rw::math::Vector3D<double> xyz3(32, 40, 50);
    rw::math::Vector3D<double> xyz4(32, -45, 20);
    rw::math::Vector3D<double> xyz5(31, -50, 33);
    rw::math::Vector3D<double> xyz6(31, -50, 20.5);
    rw::math::RPY<double> rpy1(0, 0, 3.138);
    rw::math::RPY<double> rpy2(0, 0, 3.138);
    rw::math::RPY<double> rpy3(-1, 0.5, 3.138);
    rw::math::RPY<double> rpy4(0, 0, 3.138);
    rw::math::RPY<double> rpy5(0, 0, 3.138);
    rw::math::RPY<double> rpy6(0, 0, 3.138);

    std::vector<rw::math::Q> trajectory = {p1, p2, p3, p4, p5, p6};
    std::vector<rw::math::Vector3D<double>> trajectory_xyz = {xyz1, xyz2, xyz3, xyz4, xyz5, xyz6};

    std::vector<double> times = {4, 3, 2, 3, 4};



    std::map<int, rw::math::Q> path = interpolator.q_interpolation(trajectory, times);

    rw::trajectory::TimedStatePath statePath;


    // Interpolate Q's

    robot_ur5->setQ(path.find(int(0))->second, state);
    rw::kinematics::Frame *frame_goal = wc->findFrame("Cylinder");
    rw::kinematics::Frame *frame_tcp = wc->findFrame("GraspTCP");
    rw::kinematics::Kinematics::gripFrame(frame_goal, frame_tcp, state); // Code for gripping the object


    //for (size_t i = 0; i < path.size(); i++) {
    //    robot_ur5->setQ(path.find(int(i))->second, state);
    //    std::cout << double(path.find(int(i))->first)/10 << ", " << path.find(int(i))->second << std::endl;
    //    statePath.push_back(rw::trajectory::TimedState(double(path.find(int(i))->first)/10, state));
    //}



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

    std::map<int, rw::math::Vector3D<double>> path_xyz = interpolator.xyz_interpolation(trajectory_xyz, times);
    std::map<int, QUAT> rot_path = interpolator.quat_interpolation(quat_rotations, times);



    statePath = interpolator.inverse_kin(robot_ur5, state, p1, path_xyz, rot_path);


    rw::loaders::PathLoader::storeTimedStatePath(*wc,statePath,"rw_play.rwplay");
*/
    return 0;
}
