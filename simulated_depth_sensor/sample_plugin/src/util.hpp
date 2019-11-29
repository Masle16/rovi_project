#include <iostream>
#include <fstream>
#include <vector>

#include <rw/rw.hpp>

typedef Eigen::Matrix4f Mat;
typedef rw::math::Vector3D<float> Vec;
typedef rw::math::RPY<float> Rpy;
typedef rw::math::Transform3D<float> Pose;
typedef rw::kinematics::Frame Frame;

Mat transform2Matrix4f(const rw::math::Transform3D<> &pose) {
    Mat result = Mat::Identity();
    Pose temp;
    for (size_t row = 0; row < 3; row++)
        for (size_t col = 0; col < 4; col++)
            temp(row, col) = (float)pose(row, col);
    result = temp.e();
    return result;
}

void save2File(const std::string &filePath,
               const std::vector<float> &noises,
               const std::vector<double> &times,
               const std::vector<float> &diffAngle,
               const std::vector<float> &diffPos) {
    std::cout << "Writing data to file: " << filePath << std::endl;
    std::ofstream file;
    file.open(filePath);
    if (times.size() != diffAngle.size() || diffAngle.size() != diffPos.size()) {
        std::cerr << "Vectors do not have the same size!" << std::endl;
        return;
    }
    for (size_t i = 0; i < times.size(); i++) {
        file << noises[i]    << " "
             << times[i]     << " "
             << diffAngle[i] << " "
             << diffPos[i]   << "\n";
    }
    file.close();
}

std::pair<float, float> calcPerformance(const Mat &pose,
                                        rw::models::WorkCell::Ptr workcell,
                                        rw::models::Device::Ptr device,
                                        rw::kinematics::State &state) {
    std::cout << "Calculating performance.." << std::endl;
    std::pair<float, float> result = std::make_pair(0.0, 0.0);

    // get frames
    Mat duck, table, scanner;
    {
        Frame *duckFrame = workcell->findFrame("Duck");
        duck = transform2Matrix4f(duckFrame->getTransform(state));
        Frame *tableFrame = workcell->findFrame("Table");
        table = transform2Matrix4f(tableFrame->getTransform(state));
        Frame *scannerFrame = workcell->findFrame("Scanner25D");
        scanner = transform2Matrix4f(scannerFrame->getTransform(state));
    }

    // estimate pose
    Mat estimatedPose = table.inverse() * scanner * pose;
    return result;
}
