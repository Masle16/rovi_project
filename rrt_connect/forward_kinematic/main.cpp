/* RoVi Final Project
 * Forward kinematic
 */

/* cylinder 0.0 with step size 0.6
    setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
    attach(bottle, gripper)
    setQ({2.21387 , -1.66701 , -1.49836 , -1.13278 , 1.75142 , 0.155048})
    setQ({2.19337 , -1.66365 , -1.49944 , -1.12476 , 1.74289 , 0.15751})
    setQ({1.68081 , -1.57967 , -1.52644 , -0.924375 , 1.52945 , 0.21907})
    setQ({1.16825 , -1.49568 , -1.55343 , -0.723989 , 1.31602 , 0.280629})
    setQ({0.655687 , -1.41169 , -1.58042 , -0.523602 , 1.10259 , 0.342188})
    setQ({0.143126 , -1.3277 , -1.60742 , -0.323216 , 0.889156 , 0.403748})
    setQ({-0.17961 , -1.56809 , -1.65184 , -0.617809 , 0.659415 , 0.166015})
    setQ({-0.588964 , -1.70914 , -1.49383 , -0.733771 , 0.92632 , 0.416784})
    setQ({-0.970898 , -1.78748 , -1.79866 , -0.846118 , 1.24639 , 0.418926})
    setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
*/

/*
 * Includes
 */
#include <iostream>
#include <fstream>
#include <rw/rw.hpp>

/*
 * Defines
 */

/*
 * Typedefs
 */
typedef rw::math::Transform3D<> Pose;
typedef rw::math::Rotation3D<> Rotation;
typedef rw::math::Vector3D<> Vector;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Q Q;
typedef rw::kinematics::Frame Frame;

/*
 * Functions
 */
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

/*
 * Main entry point
 */
int main(int argv, char** argc) {
   std::cout << "\nProgram started\n" << std::endl;

   // workcell and device
   const std::string wc_file = "../../workcell/Scene.wc.xml";
   const std::string device_file = "UR-6-85-5-A";
   // create configurations
   std::vector<Q> qs = {
       Q(6, 2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0),
       Q(6, 2.21387 , -1.66701 , -1.49836 , -1.13278 , 1.75142 , 0.155048),
       Q(6, 2.19337 , -1.66365 , -1.49944 , -1.12476 , 1.74289 , 0.15751),
       Q(6, 1.68081 , -1.57967 , -1.52644 , -0.924375 , 1.52945 , 0.21907),
       Q(6, 1.16825 , -1.49568 , -1.55343 , -0.723989 , 1.31602 , 0.280629),
       Q(6, 0.655687 , -1.41169 , -1.58042 , -0.523602 , 1.10259 , 0.342188),
       Q(6, 0.143126 , -1.3277 , -1.60742 , -0.323216 , 0.889156 , 0.403748),
       Q(6, -0.17961 , -1.56809 , -1.65184 , -0.617809 , 0.659415 , 0.166015),
       Q(6, -0.588964 , -1.70914 , -1.49383 , -0.733771 , 0.92632 , 0.416784),
       Q(6, -0.970898 , -1.78748 , -1.79866 , -0.846118 , 1.24639 , 0.418926),
       Q(6, -1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0)
   };
   // load workcell
   rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_file);
   if (wc == NULL) {
       std::cerr << wc_file << " not found!" << std::endl;
       return -1;
   }
   // find device
   rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(device_file);
   if (device == NULL) {
       std::cerr << device_file << " not found!" << std::endl;
       return -1;
   }
   // fine TCP frame
   Frame* tcpFrame = wc->findFrame(device_file + ".TCP");
   if (tcpFrame == NULL) {
       std::cerr << "TCP frame not found!" << std::endl;
       return 0;
   }
   // get the state
   rw::kinematics::State state = wc->getDefaultState();
   // calculate forward kinematic for each configuration
   std::cout << "Calculating forward kinemtic.." << std::endl;
   const std::string dataFile = "../forward_kinematics.txt";
   std::ofstream file;
   file.open(dataFile);
   std::cout << "Writing to file: " << dataFile << std::endl;
   for (unsigned int i = 0; i < qs.size(); i++) {
       std::cout << "\tConfiguration: " << qs[i] << std::endl;
       device->setQ(qs[i], state);
       // compute baseTtcp
       Pose baseTtool = device->baseTframe(tcpFrame, state);
       std::cout << "\tPosition: " << baseTtool.P() << std::endl;
       std::cout << "\tRotation: " << Rpy(baseTtool.R()) << std::endl;
       std::cout << std::endl;
       // write to file
       Vector pos = baseTtool.P();
       Rpy rpy = Rpy(baseTtool.R());
       file << "Pos: " << pos(0) << " " << pos(1) << " " << pos(2)
            << " RPY: " << rpy(0) << " " << rpy(1) << " " << rpy(2)
            << "\n";
   }
   file.close();

   std::cout << "\nProgram ended\n" << std::endl;
   return 0;
}
