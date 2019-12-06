#include <iostream>
#include <fstream>
#include <vector>

#include <rw/rw.hpp>

typedef Eigen::Matrix4f Mat;
typedef rw::math::Vector3D<> Vec;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Transform3D<> Pose;
typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::MovableFrame movFrame;
typedef rw::math::Rotation3D<> Rotm;

Pose matrix2Transform(const Mat matrix) {
    Vec pos = Vec(matrix(0,3), matrix(1,3), matrix(2,3));
    Rotm rotm = Rotm(matrix(0,0), matrix(0,1), matrix(0,2),
                    matrix(1,0), matrix(1,1), matrix(1,2),
                    matrix(2,0), matrix(2,1), matrix(2,2));
    return Pose(pos, rotm);
}

void save2File(const std::string &filePath,
               const std::vector<float> &noises,
               const std::vector<double> &times,
               const std::vector<float> &diffAngle,
               const std::vector<Vec> &diffPos) {
    std::cout << "Writing data to file: " << filePath << std::endl;
    std::ofstream file;
    file.open(filePath);
    if (times.size() != diffAngle.size() || diffAngle.size() != diffPos.size()) {
        std::cerr << "Vectors do not have the same size!" << std::endl;
        return;
    }
    file << "noise time[ms] angle[Deg] x[m] y[m] z[m]\n";
    for (size_t i = 0; i < times.size(); i++) {
        file << noises[i]    << " "
             << times[i]     << " "
             << diffAngle[i] << " "
             << diffPos[i](0)<< " "
             << diffPos[i](1)<< " "
             << diffPos[i](2)<< "\n";
    }
    file.close();
}

std::pair<float, Vec> calcError(const Mat &input,
                                        rw::models::WorkCell::Ptr workcell,
                                        rw::kinematics::State &state) {
    std::cout << "Calculating performance.." << std::endl;
    std::pair<float, Vec> result = std::make_pair(0.0, Vec(0,0,0));

    // get poses
    Pose duck, table, scanner;
    {
        movFrame *duckFrame = workcell->findFrame<movFrame>("Duck");
        duck = duckFrame->getTransform(state);
        Frame *tableFrame = workcell->findFrame("Table");
        table = tableFrame->getTransform(state);
        movFrame *scannerFrame = workcell->findFrame<movFrame>("Scanner25D");
        scanner = scannerFrame->getTransform(state);
    }

    // show real pose
    std::cout << "\tReal pose->" << std::endl;
    std::cout << "\tP= " << duck.P() << std::endl;
    std::cout << "\tR= " << duck.R() << std::endl;

    // estimate pose
    Pose pose = matrix2Transform(input);
    Pose tableInv = rw::math::inverse(table);
    Pose estimatedPose = tableInv * scanner * pose;

    // show estimated pose
    std::cout << "\tEstimated pose->" << std::endl;
    std::cout << "\tP= " << estimatedPose.P() << std::endl;
    std::cout << "\tR= " << estimatedPose.R() << std::endl;

    // difference in angle
    float diffAngle = 0.0;
    {
        Rotm P = duck.R(), Q = estimatedPose.R();
        std::cout << "Duck rotation matrix --> " << P << std::endl;
        std::cout << "Estimate rotation matrix --> " << Q << std::endl;
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
    Vec diffPos;
    {
        diffPos = rw::math::Math::abs(duck.P() - estimatedPose.P());
        std::cout << "\tPosition error: " << diffPos << std::endl;
    }

    result = std::make_pair(diffAngle, diffPos);
    return result;
}

void moveFrame(rw::models::WorkCell::Ptr &workcell, rw::kinematics::State &state) {
    // initialise seed
    rw::math::Math::seed();

    // get pose
    //Frame *tableFrame = workcell->findFrame("Table");
    movFrame *duckFrame = workcell->findFrame<movFrame>("Duck");
    Pose duckPose = duckFrame->getTransform(state);
    Vec position = duckPose.P();
    Rpy rpy = Rpy(duckPose.R());

    // generate random pose
    position(0) = rw::math::Math::ran(-0.3, 0.3);
    position(1) = rw::math::Math::ran(0.37, 0.53);
    rpy(0) = rw::math::Math::ran(0.0, 359.0);
    Pose newPose = Pose(position, rpy.toRotation3D());

    // move object
    duckFrame->moveTo(newPose, state);
}
