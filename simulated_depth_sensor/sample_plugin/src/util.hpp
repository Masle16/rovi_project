#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <rw/rw.hpp>

typedef Eigen::Matrix4f Mat;
typedef rw::math::Vector3D<> Vec;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Transform3D<> Pose;
typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::MovableFrame MovFrame;
typedef rw::math::Rotation3D<> Rotm;

Pose matrix2Transform(const Mat matrix) {
    Vec pos = Vec(matrix(0,3), matrix(1,3), matrix(2,3));
    Rotm rotm = Rotm(matrix(0,0), matrix(0,1), matrix(0,2),
                    matrix(1,0), matrix(1,1), matrix(1,2),
                    matrix(2,0), matrix(2,1), matrix(2,2));
    return Pose(pos, rotm);
}

void save2File(const std::string &filePath,
               const std::vector<double> &data,
               const std::vector<double> &times,
               const std::vector<double> &diffAngle,
               const std::vector<double> &diffPos) {
    std::cout << "Writing data to file: " << filePath << std::endl;

    // check for same vector size
    if ( (times.size() != diffAngle.size()) &&
         (diffAngle.size() != diffPos.size()) &&
         (diffPos.size() != data.size()) ) {
        std::cerr << "Vectors do not have the same size!" << std::endl;
        return;
    }

    // write to file
    std::ofstream file;
    file.open(filePath);
    for (std::size_t i = 0; i < times.size(); i++) {
        file << data[i]      << " "
             << times[i]     << " "
             << diffAngle[i] << " "
             << diffPos[i]   << "\n";
    }
    file.close();
}

std::pair<double, double> calcError(rw::models::WorkCell::Ptr workcell, rw::kinematics::State &state, const Pose &pose, const Pose &realPose) {
    std::cout << "Calculating performance.." << std::endl;
    std::pair<double, double> result = std::make_pair(0.0, 0.0);

    // get poses
    Pose table, scanner;
    {
        Frame *tableFrame = workcell->findFrame("Table");
        table = tableFrame->getTransform(state);
        MovFrame *scannerFrame = workcell->findFrame<MovFrame>("Scanner25D");
        scanner = scannerFrame->getTransform(state);
    }

    // show real pose
    std::cout << "\tReal pose -->"
              << "\n\t\tPosition: " << realPose.P()
              << "\n\t\tRotation: " << realPose.R() << std::endl;

    // estimate pose
    Pose tableInv = rw::math::inverse(table);
    Pose estimatedPose = tableInv * scanner * pose;
    estimatedPose(2,3) = 0.13275;
    estimatedPose(0,2) = 0; estimatedPose(1,2) = 0;
    estimatedPose(2,0) = 0; estimatedPose(2,1) = 0; estimatedPose(2,2) = 1;

    // show estimated pose
    std::cout << "\tEstimated pose -->"
              << "\n\t\tPosition: " << estimatedPose.P()
              << "\n\t\tRotation: " << estimatedPose.R() << std::endl;

    // difference in angle
    double diffAngle;
    {
        Rotm P = realPose.R(), Q = estimatedPose.R();
        std::cout << "\tReal pose rotation matrix --> " << P << std::endl;
        std::cout << "\tEstimated pose rotation matrix --> " << Q << std::endl;
        Rotm R = P * Q.inverse(); // The transpose of a rotation matrix is the same as the inverse
        std::cout << "\tP * Q' = R --> " << R << std::endl;

        double traceR = R(0,0) + R(1,1) + R(2,2);
        std::cout << "\tTrace of R --> "
                  << R(0,0) << " + "
                  << R(1,1) << " + "
                  << R(2,2) << " = "
                  << traceR << std::endl;
        diffAngle = acos((traceR-1.0)/2.0);
        diffAngle *= rw::math::Rad2Deg;
        std::cout << "\tAngle error: " << diffAngle << std::endl;
    }

    // difference in position
    double diffPos;
    {
        rw::math::EuclideanMetric<Vec> metric;
        diffPos = metric.distance(realPose.P(), estimatedPose.P());
        std::cout << "\tPosition error: " << diffPos << std::endl;
    }

    result = std::make_pair(diffAngle, diffPos);
    return result;
}

void moveFrameRandom(rw::models::WorkCell::Ptr &workcell, rw::kinematics::State &state, const std::string &frameName) {
    std::cout << "Moving " << frameName << " to random position.." << std::endl;

    // initialise seed
    rw::math::Math::seed();

    // get pose
    MovFrame *frame = workcell->findFrame<MovFrame>(frameName);
    Pose pose = frame->getTransform(state), newPose;
    {
        // get position and rotation
        Vec position = pose.P();
        Rpy rpy = Rpy(pose.R());

        // generate random pose
        position(0) = rw::math::Math::ran(-0.3, 0.3);
        position(1) = rw::math::Math::ran(0.37, 0.53);
        rpy(0) = rw::math::Math::ran(0.0, 359.0);

        // save new pose
        newPose = Pose(position, rpy.toRotation3D());
    }

    // move object
    frame->moveTo(newPose, state);

    std::cout << "\tNew position of " << frameName << " -->"
              << "\n\tPosition: " << frame->getTransform(state).P()
              << "\n\tRotation: " << frame->getTransform(state).R()
              << std::endl;
}

void moveFrame(rw::models::WorkCell::Ptr &wc, rw::kinematics::State &state, const std::string &frameName, const Pose &newPose) {
    std::cout << "Moving " << frameName << " to pose -->"
              << "\nPosition: " << newPose.P()
              << "\nRotation: " << newPose.R()
              << std::endl;

    // get frame
    MovFrame *frame = wc->findFrame<MovFrame>(frameName);

    // get current pose
    std::cout << "\tOld pose -->"
              << "\n\t\tPosition: " << frame->getTransform(state).P()
              << "\n\t\tRotation: " << frame->getTransform(state).R()
              << std::endl;

    // move object
    frame->moveTo(newPose, state);

    // get current pose
    std::cout << "\tNew pose -->"
              << "\n\t\tPosition: " << frame->getTransform(state).P()
              << "\n\t\tRotation: " << frame->getTransform(state).R()
              << std::endl;
}

std::vector<Pose> loadRandomPoses() {
    std::cout << "Loading random poses.." << std::endl;
    std::vector<Pose> result = {
        Pose( Vec(-0.145188, 0.443078, 0.13275), Rpy(119.655720, 0, 0).toRotation3D() ),
        Pose( Vec(-0.093862, 0.436711, 0.13275), Rpy(118.604228, 0, 0).toRotation3D() ),
        Pose( Vec(0.122399, 0.495443, 0.13275), Rpy(170.850844, 0, 0).toRotation3D() ),
        Pose( Vec(0.091337, 0.498684, 0.1327), Rpy(290.151910, 0, 0).toRotation3D() ),
        Pose( Vec(0.264603, 0.418769, 0.13275), Rpy(298.242230, 0, 0).toRotation3D() ),
        Pose( Vec(-0.235944, 0.419935, 0.13275), Rpy(62.161712, 0, 0).toRotation3D() ),
        Pose( Vec(0.135748, 0.504414, 0.13275), Rpy(10.789436, 0, 0).toRotation3D() ),
        Pose( Vec(-0.250770, 0.419906, 0.13275), Rpy(303.808868, 0, 0).toRotation3D() ),
        Pose( Vec(-0.200208, 0.488607, 0.13275), Rpy(184.357109, 0, 0).toRotation3D() ),
        Pose( Vec(0.108419, 0.449887, 0.13275), Rpy(137.183094, 0, 0).toRotation3D() ),
        Pose( Vec(0.217942, 0.460303, 0.13275), Rpy(224.500953, 0, 0).toRotation3D() ),
        Pose( Vec(-0.203439, 0.422350, 0.13275), Rpy(30.890149, 0, 0).toRotation3D() ),
        Pose( Vec(0.205437, 0.440262, 0.13275), Rpy(306.213153, 0, 0).toRotation3D() ),
        Pose( Vec(0.000940, 0.390936, 0.13275), Rpy(258.436252, 0, 0).toRotation3D() ),
        Pose( Vec(0.011533, 0.466524, 0.13275), Rpy(39.489767, 0, 0).toRotation3D() ),
        Pose( Vec(-0.083706, 0.473673, 0.13275), Rpy(82.114563, 0, 0).toRotation3D() ),
        Pose( Vec(-0.215880, 0.460187, 0.13275), Rpy(299.988160, 0, 0).toRotation3D() ),
        Pose( Vec(0.111775, 0.417223, 0.13275), Rpy(15.156711, 0, 0).toRotation3D() ),
        Pose( Vec(0.286724, 0.451241, 0.13275), Rpy(109.719710, 0, 0).toRotation3D() ),
        Pose( Vec(-0.007980, 0.407581, 0.13275), Rpy(110.609545, 0, 0).toRotation3D() ),
        Pose( Vec(0.142925, 0.431557, 0.13275), Rpy(134.673396, 0, 0).toRotation3D() ),
        Pose( Vec(-0.025873, 0.417860, 0.13275), Rpy(338.794893, 0, 0).toRotation3D() ),
        Pose( Vec(0.260845, 0.376162, 0.13275), Rpy(183.421458, 0, 0).toRotation3D() ),
        Pose( Vec(-0.143089, 0.519847, 0.13275), Rpy(99.111381, 0, 0).toRotation3D() ),
        Pose( Vec(-0.190782, 0.508894, 0.13275), Rpy(252.283274, 0, 0).toRotation3D() ),
        Pose( Vec(-0.272231, 0.508522, 0.13275), Rpy(18.848865, 0, 0).toRotation3D() ),
        Pose( Vec(-0.289646, 0.426160, 0.13275), Rpy(103.919571, 0, 0).toRotation3D() ),
        Pose( Vec(-0.247180, 0.440277, 0.13275), Rpy(36.188349, 0, 0).toRotation3D() ),
        Pose( Vec(-0.213684, 0.408139, 0.13275), Rpy(117.088340, 0, 0).toRotation3D() ),
        Pose( Vec(-0.241997, 0.445336, 0.13275), Rpy(357.345558, 0, 0).toRotation3D() )
    };
    return result;
}
