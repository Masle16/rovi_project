/***********/
/* INCLUDE */
/***********/
#pragma once

#include <iostream>
#include <vector>
#include <array>

#include <Eigen/Core>

#include <rw/rw.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/***********/
/* DEFINES */
/***********/
#define WC_FILE "/home/mathi/Documents/rovi/rovi_project/sparse_stereo/workcell/Scene.wc.xml"
//#define WC_FILE "../../workcell/Scene.wc.xml"

/************/
/* TYPEDEFS */
/************/
typedef rw::kinematics::Frame Frame;
typedef rw::math::Transform3D<> Pose;
typedef rw::kinematics::State State;
typedef rw::math::RPY<> Rpy;

/***********/
/* STRUCTS */
/***********/
struct Camera {
    cv::Mat intrinsic;
    cv::Mat transformation;
    cv::Mat distortion;
    cv::Mat projection;
    cv::Mat translation;
    cv::Mat rotation;
    double image_width;
    double image_height;

    void printData() {
        std::cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
                << distortion << "\n" << transformation << "\n" << projection
                << std::endl;
    }
};

struct StereoPair {
    Camera camLeft;
    Camera camRight;
};

/*************/
/* FUNCTIONS */
/*************/
cv::Mat colorFiltering(const cv::Mat &input) {
    cv::Mat result, img = input.clone(), hsv, mask;
//    //Create trackbars in "Control" window
//    cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); //create a window called "Control"
//    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;
//    cv::createTrackbar("LowH", "Control", &lowH, 179); //Hue (0 - 179)
//    cv::createTrackbar("HighH", "Control", &highH, 179);
//    cv::createTrackbar("LowS", "Control", &lowS, 255); //Saturation (0 - 255)
//    cv::createTrackbar("HighS", "Control", &highS, 255);
//    cv::createTrackbar("LowV", "Control", &lowV, 255); //Value (0 - 255)
//    cv::createTrackbar("HighV", "Control", &highV, 255);
//    while (true) {
//        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
//        cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask); //Threshold the image
//        //morphological opening (remove small objects from the foreground)
//        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
//        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
//        //morphological closing (fill small holes in the foreground)
//        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
//        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
//        cv::bitwise_and(img, img, result, mask);
//        cv::imshow("Thresholded Image", mask); //show the thresholded image
//        cv::imshow("Original", img); //show the original image
//        cv::imshow("Output", result);
//        //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
//        if (cv::waitKey(0) == 27) { break; }
//    }
//    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
//    cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask);
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 0), cv::Scalar(20, 255, 255), mask);
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::bitwise_and(img, img, result, mask);
    return result;
}

cv::Mat constructProjectionMat(Eigen::Matrix<double, 3, 4> KA, Eigen::Matrix<double, 4, 4> H) {
    // convert eigen matrix to cv::mat
    cv::Mat _KA, _H;
    cv::eigen2cv(KA, _KA);
    cv::eigen2cv(H, _H);
    // construct projection matrix
    return _KA * _H;
}

std::array<cv::Mat, 2> splitPp(cv::Mat proj) {
    std::array<cv::Mat, 2> pp;
    pp[0] = proj(cv::Range(0,3), cv::Range(0,3));
    pp[1] = proj(cv::Range(0,3), cv::Range(3,4));
    return pp;
}

cv::Mat computeOpticalCenter(std::array<cv::Mat, 2> pp) {
    // compute homogeneous coordinates
    cv::Mat one = cv::Mat::ones(1, 1, CV_64F);
    cv::Mat C = -1.0 * pp[0].inv(cv::DECOMP_SVD) * pp[1];
    cv::vconcat(C, one, C);
    return C;
}

cv::Mat computeFundamentalMat(cv::Mat e, cv::Mat projRight, cv::Mat projLeft) {
    // create symmetric skew 'cross product matrix' from the right epipole
    cv::Mat erx = cv::Mat::zeros(3,3,CV_64F);
    erx.at<double>(0,1) = -e.at<double>(2);
    erx.at<double>(0,2) = e.at<double>(1);
    erx.at<double>(1,0) = e.at<double>(2);
    erx.at<double>(1,2) = -e.at<double>(0);
    erx.at<double>(2,0) = -e.at<double>(1);
    erx.at<double>(2,1) = e.at<double>(0);
    return erx * projRight * projLeft.inv(cv::DECOMP_SVD);
}

void drawEpipolarLine(cv::Mat &img, cv::Mat line, double width=640) {
    cv::Point p1, p2;
    double x = line.at<double>(0,0), y = line.at<double>(0,1), z = line.at<double>(0,2);
    p1.x = 0;
    p1.y = -z / y;
    p2.x = width;
    p2.y = -width * x / y - z / y;
    cv::line(img, p1, p2, cv::Scalar(0,255,0), 2, cv::LINE_AA);
}

std::array<cv::Mat, 2> computePluckerLine(cv::Mat M1, cv::Mat M2) {
    std::array<cv::Mat, 2> plucker;
    plucker[0] = M1.cross(M2) / cv::norm(M2);
    plucker[1] = M2 / cv::norm(M2);
    return plucker;
}

cv::Mat computePluckerIntersect(std::array<cv::Mat, 2> plucker1, std::array<cv::Mat, 2> plucker2) {
    cv::Mat mu1 = plucker1[0], mu2 = plucker2[0], v1 = plucker1[1], v2 = plucker2[1];
    // compute the point on the left line which is closeset to the right line, and vice versa
    double v1_v2xmu2 = v1.dot(v2.cross(mu2));
    double v1v2_v1_v2xmu1 = v1.dot(v2) * v1.dot(v2.cross(mu1));
    double pow_v1xv2 = pow(norm(v1.cross(v2)), 2);
    cv::Mat M1 = (v1_v2xmu2 - v1v2_v1_v2xmu1) / pow_v1xv2 * v1 + v1.cross(mu1);
    double v2_v1xmu1 = v2.dot(v1.cross(mu1));
    double v2v1_v2_v1xmu2 = v2.dot(v1) * v2.dot(v1.cross(mu2));
    double pow_v2xv1 = pow(norm(v2.cross(v1)), 2);
    cv::Mat M2 = (v2_v1xmu1 - v2v1_v2_v1xmu2) / pow_v2xv1 * v2 + v2.cross(mu2);
    return M1 + (M2 - M1) / 2;
}

void getProjectionMatrix(const std::string &frameName, cv::Mat &output, cv::Mat &camMat) {
    // load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    if (wc != NULL) {
        State state = wc->getDefaultState();
        Frame *cameraFrame = wc->findFrame(frameName);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss(camParam, std::istringstream::in);
                iss >> fovy >> width >> height;

                double fovyPixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0);

                Eigen::Matrix<double, 3, 4> KA;
                KA << fovyPixel, 0, width /2.0, 0,
                      0, fovyPixel, height / 2.0, 0,
                      0, 0, 1, 0;

                cv::eigen2cv(KA, camMat);

                Pose camPosOGL = cameraFrame->wTf(state);
                Pose openGL2Vis = Pose(Rpy(-M_PI, 0, M_PI).toRotation3D());
                Pose H = rw::math::inverse(camPosOGL * rw::math::inverse(openGL2Vis));

                output = constructProjectionMat(KA, H.e());
            }
        }
    }
}

cv::Mat point2Mat(cv::Point2f pt) {
    cv::Mat m(3,1,CV_64F);
    m.at<double>(0,0) = pt.x;
    m.at<double>(1,0) = pt.y;
    m.at<double>(2,0) = 1;
    return m;
}

cv::Point3f mat2Point3f(cv::Mat m) {
    cv::Point3f pnt;
    pnt.x = m.at<float>(0,0);
    pnt.y = m.at<float>(1,0);
    pnt.z = m.at<float>(2,0);
    return pnt;
}

cv::Mat triangulate(const cv::Point2f &ptLeft,
                    const cv::Point2f &ptRight,
                    const cv::Mat &fLeftRight,
                    const cv::Mat &centerLeft,
                    const cv::Mat &centerRight,
                    const std::array<cv::Mat, 2> &ppLeft,
                    const std::array<cv::Mat, 2> &ppRight) {
    cv::Mat mLeft = point2Mat(ptLeft);
    cv::Mat epipoleLineRight = fLeftRight * mLeft;
    std::cout << "Right epipolar line:\n" << epipoleLineRight << std::endl;

//    drawEpipolarLine(sceneRight, epipoleLineRight);
//    cv::imshow("Scene right", sceneRight);

    cv::Mat mRight = point2Mat(ptRight);

    // project point to infinity
    cv::Mat mInfLeft = ppLeft[0].inv(cv::DECOMP_SVD) * mLeft;
    cv::Mat mInfRight = ppRight[0].inv(cv::DECOMP_SVD) * mRight;

    std::array<cv::Mat, 2> pluckerLeft = computePluckerLine(centerLeft(cv::Range(0,3), cv::Range(0,1)), mInfLeft);
    std::array<cv::Mat, 2> pluckerRight = computePluckerLine(centerRight(cv::Range(0,3), cv::Range(0,1)), mInfRight);
    std::cout << "Plucker line parameters:" << std::endl;
    std::cout << "mu_l: " << pluckerLeft[0] << std::endl
              << "v_l: " << pluckerLeft[1] << std::endl << std::endl;
    std::cout << "mu_r: " << pluckerRight[0] << std::endl
              << "v_r: " << pluckerRight[1] << std::endl << std::endl;

    auto intersection = computePluckerIntersect(pluckerLeft, pluckerRight);
    std::cout << "Triangulated point:\n" << intersection << std::endl;

    return intersection;
}

cv::Mat triangulate2(const cv::Point2f &ptLeft,
                     const cv::Point2f &ptRight,
                     const cv::Mat &projLeft,
                     const cv::Mat &projRight) {
    cv::Mat mLeft = point2Mat(ptLeft);
    cv::Mat mRight = point2Mat(ptRight);

    // opencv triangulation
    cv::Mat pnts3D(1,1,CV_64FC4);
    cv::Mat cam0pnts(1,1,CV_64FC2);
    cv::Mat cam1pnts(1,1,CV_64FC2);

    cam0pnts.at<cv::Vec2d>(0)[0] = mLeft.at<double>(0,0);
    cam0pnts.at<cv::Vec2d>(0)[1] = mLeft.at<double>(1,0);
    cam1pnts.at<cv::Vec2d>(0)[0] = mRight.at<double>(0,0);
    cam1pnts.at<cv::Vec2d>(0)[1] = mRight.at<double>(1,0);

    cv::triangulatePoints(projLeft, projRight, cam0pnts, cam1pnts, pnts3D);
    std::cout << "OpenCV triangulation" << std::endl;
    std::cout << "Image points: " << cam0pnts << "\t" << cam1pnts << std::endl << std::endl;
    std::cout << "Triangulated point (normalized): " << std::endl << pnts3D / pnts3D.at<double>(3, 0) << std::endl << std::endl;

    return pnts3D / pnts3D.at<double>(3,0);
}
