#include <iostream>
#include <fstream>
#include <rw/rw.hpp>

#define WC_FILE "../../workcell/Scene.wc.xml"

typedef rw::math::Vector3D<float> Vec;
typedef rw::math::RPY<float> Rpy;
typedef rw::math::Transform3D<float> Pose;
typedef Eigen::Matrix4f Mat;

/** Returns transform
 * @brief getTransform : Gets the transform of a frame from the workcell
 * @param frameName : name of the wanted frame
 * @return : rw::math::Transform3D of the frame
 */
Mat getTransform(const std::string &frameName) {
    Pose result = rw::math::Transform3D<float>::identity();
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::Frame *objectFrame = wc->findFrame(frameName);
    if (objectFrame == NULL) {
        std::cerr << frameName << " not found!" << std::endl;
        return result.e();
    }
    rw::math::Transform3D<> pose = objectFrame->getTransform(state);
    for (size_t row = 0; row < 3; row++)
        for (size_t col = 0; col < 4; col++)
            result(row, col) = (float)pose(row, col);
    return result.e();
}

/** Transform3D --> Matrix4f
 * @brief transfrom2matrix4f : Converts a Transform3D to Matrix4f
 * @param objectT : rw::math::Transform3D
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f transfrom2matrix4f(rw::math::Transform3D<> objectT) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    rw::math::Vector3D<> position = objectT.P();
    rw::math::Rotation3D<> rotation = objectT.R();
    // convert to matrix4
    result(0,0) = (float)rotation(0,0); result(0,1) = (float)rotation(0,1); result(0,2) = (float)rotation(0,2); result(0,3) = (float)position(0);
    result(1,0) = (float)rotation(1,0); result(1,1) = (float)rotation(1,1); result(1,2) = (float)rotation(1,2); result(1,3) = (float)position(1);
    result(2,0) = (float)rotation(0,2); result(2,1) = (float)rotation(2,1); result(2,2) = (float)rotation(2,2); result(2,3) = (float)position(2);
    return result;
}

/** Matrix4f --> Transform3D
 * @brief matrix4f2transform : Converts a Matrix4f to Transform3D
 * @param matrix : Eigen::Matrix4f
 * @return : rw::math::Transform3D<>
 */
rw::math::Transform3D<> matrix4f2transform(const Eigen::Matrix4f matrix) {
    rw::math::Vector3D<> position;
    position(0) = matrix(0,3); position(1) = matrix(1,3); position(2) = matrix(2,3);
    rw::math::Rotation3D<> rotation;
    rotation(0,0) = matrix(0,0); rotation(0,1) = matrix(0,1); rotation(0,2) = matrix(0,2);
    rotation(1,0) = matrix(1,0); rotation(1,1) = matrix(1,1); rotation(1,2) = matrix(1,2);
    rotation(2,0) = matrix(2,0); rotation(2,1) = matrix(2,1); rotation(2,2) = matrix(2,2);
    return rw::math::Transform3D<>(position, rotation);
}

/**
 * @brief data2File
 * @param poses
 * @param times
 */
void data2File(const std::vector<Eigen::Matrix4f> &poses, const std::vector<double> &times) {
    std::cout << "Writing data to file data.txt" << std::endl;
    std::ofstream file;
    file.open("../../data/data.txt");
    for (size_t i = 0; i < poses.size(); i++) {
        file << times[i]      << " "
             << poses[i](0,0) << " " << poses[i](0,1) << " " << poses[i](0,2) << " " << poses[i](0,3) << " "
             << poses[i](1,0) << " " << poses[i](1,1) << " " << poses[i](1,2) << " " << poses[i](1,3) << " "
             << poses[i](2,0) << " " << poses[i](2,1) << " " << poses[i](2,2) << " " << poses[i](2,3) << " "
             << poses[i](3,0) << " " << poses[i](3,1) << " " << poses[i](3,2) << " " << poses[i](3,3) << "\n";
    }
    file.close();
}

std::vector<Mat> getRealPoses() {
    std::cout << "Loading real poses.." << std::endl;
    std::vector<Mat> result;
    std::vector<std::string> realPoses = {
        "-0.145188 0.443078 0.13275 119.655720 0 0",
        "-0.093862 0.436711 0.13275 118.604228 0 0",
        "0.122399 0.495443 0.13275 170.850844 0 0",
        "0.091337 0.498684 0.1327 290.151910 0 0",
        "0.264603 0.418769 0.13275 298.242230 0 0",
        "-0.235944 0.419935 0.13275 62.161712 0 0",
        "0.135748 0.504414 0.13275 40.789436 0 0",
        "-0.250770 0.419906 0.13275 303.808868 0 0",
        "-0.200208 0.488607 0.13275 184.357109 0 0",
        "0.108419 0.449887 0.13275 137.183094 0 0",
        "0.217942 0.460303 0.13275 224.500953 0 0",
        "-0.203439 0.422350 0.13275 30.890149 0 0",
        "0.205437 0.440262 0.13275 306.213153 0 0",
        "0.000940 0.390936 0.13275 258.436252 0 0",
        "0.011533 0.466524 0.13275 39.489767 0 0",
        "-0.083706 0.473673 0.13275 82.114563 0 0",
        "-0.215880 0.460187 0.13275 299.988160 0 0",
        "0.111775 0.417223 0.13275 15.156711 0 0",
        "0.286724 0.451241 0.13275 109.719710 0 0",
        "-0.007980 0.407581 0.13275 110.609545 0 0",
        "0.142925 0.431557 0.13275 134.673396 0 0",
        "-0.025873 0.417860 0.13275 338.794893 0 0",
        "0.260845 0.376162 0.13275 183.421458 0 0",
        "-0.143089 0.519847 0.13275 99.111381 0 0",
        "-0.190782 0.508894 0.13275 352.283274 0 0",
        "-0.272231 0.508522 0.13275 18.848865 0 0",
        "-0.289646 0.426160 0.13275 103.919571 0 0",
        "-0.247180 0.440277 0.13275 36.188349 0 0",
        "-0.213684 0.408139 0.13275 117.088340 0 0",
        "-0.241997 0.445336 0.13275 357.345558 0 0"
    };
    for (auto line : realPoses) {
        std::vector<float> nums;
        std::string str = "";
        for (auto x : line) {
            if (x == ' ') {
                float num = std::stof(str);
                nums.push_back(num);
                str = "";
            }
            else {
                str += x;
            }
        }
        Vec pos = Vec(nums[0], nums[1], nums[2]);
        Rpy rpy = Rpy(nums[3], nums[4], nums[5]);
        Pose pose = Pose(pos, rpy.toRotation3D());
        result.push_back(pose.e());
    }
    return result;
}

Mat loadTablePose() {
    // From scene.wc.xml: <RPY>0 0 0</RPY> <Pos>0 0 -0.1</Pos>
    Mat result = Mat::Identity();
    Vec pos = Vec(0, 0, -0.1);
    Rpy rpy = Rpy(0, 0, 0);
    Pose pose = Pose(pos, rpy.toRotation3D());
    result = pose.e();
    return result;
}

Mat loadScannerPose() {
    // From scene.wc.xml: <Pos>0 1.033 1.325</Pos> <RPY>0 0 -25</RPY>
    Mat result = Mat::Identity();
    Vec pos = Vec(0, 1.033, 1.325);
    Rpy rpy = Rpy(0, 0, -25);
    Pose pose = Pose(pos, rpy.toRotation3D());
    result = pose.e();
    return result;
}
