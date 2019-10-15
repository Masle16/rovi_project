#include <iostream>
#include <rw/rw.hpp>
#include <rw/invkin.hpp>

#include "ReachabilityAnalysis.hpp"

int main() {
    // load workcell
    const std::string wc_file = "/home/mathi/Documents/rovi/rovi_project/Project_WorkCell/Scene.wc.xml";
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_file);
    if (NULL == wc) {
        RW_THROW("COULD NOT LOAD " + wc_file + " ... check path!");
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
    std::vector<rw::kinematics::MovableFrame::Ptr> cylinder_frames;
    for (unsigned int i = 1; i <= 3; i++) {
        const std::string cylinder_name = "Cylinder" + std::to_string(i);
        rw::kinematics::MovableFrame::Ptr cylinder_frame = wc->findFrame<rw::kinematics::MovableFrame>(cylinder_name);
        if (NULL == cylinder_frame) {
            RW_THROW("Could not find movable frame " + cylinder_name + " ... check model");
            return -1;
        }
        cylinder_frames.push_back(cylinder_frame);
    }

    // get start state and default rotation
    rw::kinematics::State state = wc->getDefaultState();
    rw::math::Rotation3D<> rot = base_frame->getTransform(state).R();

    // generate position for reachability analysis
    std::vector<rw::math::Vector3D<>> positions;
    for (float y = 0.2; y >= -0.5; y -= 0.1) { // y goes from 0.2 to -0.5
        for (float x = -0.3; x <= 0.3; x += 0.1) { // x goes from -0.3 to 0.3
            // check for goal position
            if (y < -0.3 && x > 0.1) { continue; }
            rw::math::Vector3D<> pos(x, y, 0.0);
            positions.push_back(pos);
        }
    }

    for (unsigned int i = 0; i < positions.size(); i++) {
        // move base frame
        rw::math::Transform3D<> trans(positions[i], rot);
        base_frame->moveTo(trans, state);

        // get collision free solutions
        for (unsigned int j = 0; j < cylinder_frames.size(); j++) {
            // generate rwplay file
            const std::string folder = "/home/mathi/Documents/rovi/rovi_project/robwork_plays/reachability_analysis_play/";
            const std::string rwplay = folder + "_position" + std::to_string(i) + "_cylinder" + std::to_string(j+1) + ".rwplay";

            // selected grasp target
            const std::string target = "GraspTarget" + std::to_string(j + 1);

            // get collision free solutions
            std::vector<rw::math::Q> collision_free_solutions = getCollisionFreeSolutions(wc, robot_ur5, cylinder_frames[j], target, rwplay, state);
            std::cout << "x: " << positions[i](0) << " y: " << positions[i](1) << " cylinder: " << j << " => " << collision_free_solutions.size() << std::endl;
        }
    }

    return 0;
}
