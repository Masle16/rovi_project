#include <iostream>
#include "rw/rw.hpp"

#define WC_FILE "/home/mathi/Documents/rovi_project/Project_WorkCell/Scene.wc.xml"

int main() {
    const std::string device_name = "UR-6-85-5-A";
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    if (wc.isNull()) { RW_THROW("Workcell could not be loaded"); }
    rw::models::Device::Ptr device = wc->findDevice(device_name);
    if (device.isNull()) { RW_THROW("Device could not be found"); }
    // Compute base to TCP frame
    rw::kinematics::State state = wc->getDefaultState();
    std::cout << "\nDefault q vector => " << device->getQ(state) << std::endl;
    //device->setQ(q, state);
    rw::kinematics::Frame* rw_tcp_frame = wc->findFrame(device_name + ".TCP");
    if (rw_tcp_frame == nullptr) { RW_THROW("TCP frame not found."); }
    rw::math::Transform3D<> base_T_tcp = device->baseTframe(rw_tcp_frame, state);
    std::cout << base_T_tcp << std::endl;

    return 0;
}
