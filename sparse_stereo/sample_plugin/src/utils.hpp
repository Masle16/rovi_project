#pragma once

/*
 * INCLUDES
 */
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
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

/*
 * DEFINES
 */
#define WC_FILE "sparse_stereo/workcell/Scene.wc.xml"

/*
 * TYPEDEFS
 */
typedef rw::kinematics::Frame Frame;
typedef rw::kinematics::MovableFrame MovFrame;

typedef rw::math::Transform3D<> Pose;
typedef rw::math::Vector3D<> Vec;
typedef rw::math::Rotation3D<> Rotation;
typedef rw::math::RPY<> Rpy;

typedef rw::kinematics::State State;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

/*
 * GLOBAL VARIABLES
 */
const std::string DEVICE_FILE = "UR-6-85-5-A";

/*
 * STRUCTS
 */

/*************/
/* FUNCTIONS */
/*************/

void add_gaussian_noise(cv::Mat &i1, cv::Mat &i2, const float sd1, const float sd2, const float sd3);

/**
 * Inspiration --> https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html?fbclid=IwAR0iNvPB5CqZsyx7cWDhiPAi8egmKFPp-NXYJWEJvw5xbdS3QI4QMRB7LD0
 * @brief colorFiltering
 * @param input
 * @return
 */
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
    //cv::inRange(hsv, cv::Scalar(0, 30, 0), cv::Scalar(20, 255, 255), mask);
    cv::inRange(hsv, cv::Scalar(0, 90, 125), cv::Scalar(179, 255, 255), mask);
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::bitwise_and(img, img, result, mask);
    return result;
}

/**
 * This functions is from lecture 2 in computer vision
 * @brief constructProjectionMat
 * @param KA
 * @param H
 * @return
 */
cv::Mat constructProjectionMat(Eigen::Matrix<double, 3, 4> KA, Eigen::Matrix<double, 4, 4> H) {
    // convert eigen matrix to cv::mat
    cv::Mat _KA, _H;
    cv::eigen2cv(KA, _KA);
    cv::eigen2cv(H, _H);
    // construct projection matrix
    return _KA * _H;
}

/**
 * This functions is from lecture 2 in computer vision
 * @brief splitPp
 * @param proj
 * @return
 */
std::array<cv::Mat, 2> splitPp(cv::Mat proj) {
    std::array<cv::Mat, 2> pp;
    pp[0] = proj(cv::Range(0,3), cv::Range(0,3));
    pp[1] = proj(cv::Range(0,3), cv::Range(3,4));
    return pp;
}

/**
 * This functions is from lecture 2 in computer vision
 * @brief computeOpticalCenter
 * @param pp
 * @return
 */
cv::Mat computeOpticalCenter(std::array<cv::Mat, 2> pp) {
    // compute homogeneous coordinates
    cv::Mat one = cv::Mat::ones(1, 1, CV_64F);
    cv::Mat C = -1.0 * pp[0].inv(cv::DECOMP_SVD) * pp[1];
    cv::vconcat(C, one, C);
    return C;
}

/**
 * This functions is from lecture 2 in computer vision
 * @brief computeFundamentalMat
 * @param e
 * @param projRight
 * @param projLeft
 * @return
 */
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

/**
 * This functions is from lecture 2 in computer vision
 * @brief drawEpipolarLine
 * @param img
 * @param line
 * @param width
 */
void drawEpipolarLine(cv::Mat &img, cv::Mat line, double width=640) {
    cv::Point p1, p2;
    double x = line.at<double>(0,0), y = line.at<double>(0,1), z = line.at<double>(0,2);
    p1.x = 0;
    p1.y = -z / y;
    p2.x = width;
    p2.y = -width * x / y - z / y;
    cv::line(img, p1, p2, cv::Scalar(0,255,0), 2, cv::LINE_AA);
}

/**
 * This functions is from lecture 2 in computer vision
 * @brief computePluckerLine
 * @param M1
 * @param M2
 * @return
 */
std::array<cv::Mat, 2> computePluckerLine(cv::Mat M1, cv::Mat M2) {
    std::array<cv::Mat, 2> plucker;
    plucker[0] = M1.cross(M2) / cv::norm(M2);
    plucker[1] = M2 / cv::norm(M2);
    return plucker;
}

/**
 * This functions is from lecture 2 in computer vision
 * @brief computePluckerIntersect
 * @param plucker1
 * @param plucker2
 * @return
 */
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

/**
 * This functions was given on BlackBoard
 * @brief getProjectionMatrix
 * @param frameName
 * @param output
 * @param camMat
 */
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

                // create camera matrix
                camMat = cv::Mat::zeros(3,3,CV_64FC1);
                camMat.at<double>(0,0) = fovyPixel;
                camMat.at<double>(1,1) = fovyPixel;
                camMat.at<double>(0,2) = width/2.0;
                camMat.at<double>(1,2) = height/2.0;
                camMat.at<double>(2,2) = 1;

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

/**
 * This functions is from lecture 2 in computer vision
 * @brief triangulate
 * @param ptLeft
 * @param ptRight
 * @param fLeftRight
 * @param centerLeft
 * @param centerRight
 * @param ppLeft
 * @param ppRight
 * @return
 */
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
    //std::cout << "OpenCV triangulation" << std::endl;
    //std::cout << "Image points: " << cam0pnts << "\t" << cam1pnts << std::endl << std::endl;
    //std::cout << "Triangulated point (normalized): " << std::endl << pnts3D / pnts3D.at<double>(3, 0) << std::endl << std::endl;

    return pnts3D; // / pnts3D.at<double>(3, 0);
}

/**
 * @brief stringToInt : converts a given string to an integer
 * @param text
 * @return
 */
int string2Int( const std::string &text) {
    std::istringstream ss(text);
    int result;
    return ss >> result ? result : 0;
}

/**
 * @brief float2string : converts a given float to a string
 * @param number
 * @return
 */
std::string float2String(float num) {
    std::ostringstream ss;
    ss << num;
    return ss.str();
}

std::string int2String(int num) {
    std::ostringstream ss;
    ss << num;
    return ss.str();
}

int ratioTest(std::vector<std::vector<cv::DMatch>> &matches, const float ratio=0.7f) {
    int removed = 0;

    // for all matches
    for (std::vector<std::vector<cv::DMatch>>::iterator matchIterator = matches.begin();
         matchIterator != matches.end(); matchIterator++) {

        // if 2 NN has been identified
        if (matchIterator->size() > 1) {
            // check distance ratio
            if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio) {
                matchIterator->clear();
                removed++;
            }
        }
        else {
            // does not have 2 neighbours
            matchIterator->clear();
            removed++;
        }
    }
    return removed;
}

void symmetryTest(const std::vector<std::vector<cv::DMatch>> &matches1,
                  const std::vector<std::vector<cv::DMatch>> &matches2,
                  std::vector<cv::DMatch> &symMatches) {
    // for all matches image 1 --> image 2
    for (std::vector<std::vector<cv::DMatch>>::const_iterator matchIterator1 = matches1.begin();
         matchIterator1 != matches1.end();
         matchIterator1++) {

        // ignore deleted matches
        if (matchIterator1->empty() || matchIterator1->size() < 2) { continue; }

        // for all matches image 2 --> image 1
        for (std::vector<std::vector<cv::DMatch>>::const_iterator matchIterator2 = matches2.begin();
             matchIterator2 != matches2.end();
             matchIterator2++) {

            // ignore deleted matches
            if (matchIterator2->empty() || matchIterator2->size() < 2) { continue; }

            // match symmetry test
            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx &&
                    (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {

                // add symmetry match
                symMatches.push_back(
                    cv::DMatch(
                        (*matchIterator1)[0].queryIdx,
                        (*matchIterator1)[0].trainIdx,
                        (*matchIterator1)[0].distance
                    )
                );
            }
        }
    }
}

cv::Mat setPMat(const cv::Mat &rMat, const cv::Mat &tMat) {
    cv::Mat pMat = cv::Mat::zeros(4,4,CV_64FC1);

    // Rotation-Translation Matrix Definition
    pMat.at<double>(0,0) = rMat.at<double>(0,0);
    pMat.at<double>(0,1) = rMat.at<double>(0,1);
    pMat.at<double>(0,2) = rMat.at<double>(0,2);
    pMat.at<double>(1,0) = rMat.at<double>(1,0);
    pMat.at<double>(1,1) = rMat.at<double>(1,1);
    pMat.at<double>(1,2) = rMat.at<double>(1,2);
    pMat.at<double>(2,0) = rMat.at<double>(2,0);
    pMat.at<double>(2,1) = rMat.at<double>(2,1);
    pMat.at<double>(2,2) = rMat.at<double>(2,2);
    pMat.at<double>(0,3) = tMat.at<double>(0);
    pMat.at<double>(1,3) = tMat.at<double>(1);
    pMat.at<double>(2,3) = tMat.at<double>(2);

    pMat.at<double>(3,0) = 0;
    pMat.at<double>(3,1) = 0;
    pMat.at<double>(3,2) = 0;
    pMat.at<double>(3,3) = 1;

    return pMat;
}

/**
 * --> https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
 * @brief euler2rot
 * @param euler
 * @return
 */
cv::Mat euler2rot(const cv::Mat &euler) {
    cv::Mat rotm(3,3,CV_64F);

    double bank = euler.at<double>(0);
    double attitude = euler.at<double>(1);
    double heading = euler.at<double>(2);

    double ch = cos(heading), sh = sin(heading);
    double ca = cos(attitude), sa = sin(attitude);
    double cb = cos(bank), sb = sin(bank);

    rotm.at<double>(0,0) = ch*ca;
    rotm.at<double>(0,1) = sh*sb-ch*sa*cb;
    rotm.at<double>(0,2) = ch*sa*sb+sh*cb;
    rotm.at<double>(1,0) = sa;
    rotm.at<double>(1,1) = ca*cb;
    rotm.at<double>(1,2) = -ca*sb;
    rotm.at<double>(2,0) = -sh*ca;
    rotm.at<double>(2,1) = sh*sa*cb+ch*sb;
    rotm.at<double>(2,2) = -sh*sa*sb+ch*cb;

    return rotm;
}

void findMatches(cv::Mat &src_l, cv::Mat &src_r, std::vector<cv::Point2f> &l2r, std::vector<cv::Point2f> &r2l) {
    // filter images
    cv::Mat _src_l = colorFiltering(src_l);
    cv::Mat _src_r = colorFiltering(src_r);

    cv::imshow("Filtered left", _src_l);
    cv::imshow("Filtered right", _src_r);

    // step 1: detect the keypoints using surf detector, compute the descriptors
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptor1, descriptor2;
    detector->detectAndCompute(_src_l, cv::noArray(), keypoints1, descriptor1);
    detector->detectAndCompute(_src_r, cv::noArray(), keypoints2, descriptor2);

    // step 2: matching descriptor vectors with a brute force matcher
    // since surf is floating-point descriptor NORM_L2 is used
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch>> matches1, matches2;

    // image 1 --> 2
    matcher->knnMatch(descriptor1, descriptor2, matches1, 2);
    // image 2 --> 1
    matcher->knnMatch(descriptor2, descriptor1, matches2, 2);

    // remove matches for which NN ratio is > than threshold
    // std::cout << "\nClean image 1 --> image 2 matches: " << ratioTest(matches1) << " (removed)."
    //           << "\nClean image 2 --> image 1 matches: " << ratioTest(matches2) << " (removed)."
    //           << std::endl;

    // remove non-symmetrical matches
    std::vector<cv::DMatch> goodMatches;
    symmetryTest(matches1, matches2, goodMatches);

    cv::Mat matchImg;
    cv::drawMatches(_src_l, keypoints1, _src_r, keypoints2, goodMatches, matchImg);
    cv::imshow("Matches", matchImg);

    // get point
    for (std::size_t i = 0; i < goodMatches.size(); i++) {
        int idx1 = goodMatches[i].trainIdx;
        int idx2 = goodMatches[i].queryIdx;
        l2r.push_back(keypoints1[idx1].pt);
        r2l.push_back(keypoints2[idx2].pt);
    }
}

void getInitial3dPnts(const cv::Mat &proj_l, const cv::Mat &proj_r, std::vector<cv::Mat> &pnts_3d, rw::math::Vector3D<> &v3d_out, float noise) {
    

    // get optical center
    std::array<cv::Mat, 2> pp_l = splitPp(proj_l);
    std::array<cv::Mat, 2> pp_r = splitPp(proj_r);
    cv::Mat c_l = computeOpticalCenter(pp_l);
    cv::Mat c_r = computeOpticalCenter(pp_r);
    //std::cout << "\nOptical center left:\n" << c_l
    //          << "\nOptical center Right:\n" << c_r << std::endl;


    // load images
    cv::Mat src_l = cv::imread("Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat src_r = cv::imread("Camera_Right.png", cv::IMREAD_COLOR);

    add_gaussian_noise(src_l, src_r, noise, noise, noise);


    cv::imshow("left",src_l);
    cv::imshow("right",src_r);
    //cv::waitKey(0);

    // Find features in the images:
    std::vector<cv::Point2f> l2r, r2l;
    findMatches(src_l, src_r, l2r, r2l);

    //std::cout << "Projection 1:\n" << proj_l << std::endl << "Projection 2:\n" << proj_r << std::endl;
    std::cout << "l2r: \n" << l2r.size() << "\nr2l: \n" << r2l.size() << std::endl;
    if (l2r.size() > 0 && r2l.size() > 0) {
        cv::Mat out_points;
        cv::triangulatePoints(proj_l,proj_r,l2r,r2l,out_points);

        //std::cout << out_points.cols << std::endl;
        // Scales with the fourth value.
        std::vector<rw::math::Vector3D<>> lc_points;
        for (int i = 0; i < out_points.cols; i++) {
            float scale = 1/out_points.at<float>(3,i);
            lc_points.push_back(rw::math::Vector3D<>(scale*out_points.at<float>(0,i), scale*out_points.at<float>(1,i), scale*out_points.at<float>(2,i)));
        }


        // Find the point with the shortest average dist to all other points
        double best_avg_dist = std::numeric_limits<double>::max();
        double avg_dist = 0;
        size_t best_point = 0;
        rw::math::EuclideanMetric<rw::math::Vector3D<>> metric;
        std::cout << "Før beregning af bedste punkt" << std::endl;
        for (size_t i = 0; i < lc_points.size(); i++) {
            for (size_t j = 0; j < lc_points.size(); j++) {
                // If i not equal j then calculate the distance between the points and sum them.
                if (i != j) {
                    avg_dist += metric.distance(lc_points[i], lc_points[j])/lc_points.size()-1;
                }
            }
            if (avg_dist < best_avg_dist) {
                // Update the best and save the i for indexing on lc_points.
                best_avg_dist = avg_dist;
                best_point = i;
            }
            avg_dist = 0;
        }
        std::cout << "Lige efter bedste punkt " << best_point << std::endl;
        v3d_out = lc_points[best_point];
    } else {
        v3d_out = rw::math::Vector3D<>(0, 0, 0);
        std::cout << "Failed at noise level: " << noise << std::endl;
    }

    // Find the outliers:
    // Run 100 times. ( Only works if points are 3 or more ).
    /*
    rw::math::Math::seed();
    rw::math::Vector3D<> best_center(500, 500, 500);
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < 100; i++) {
        // Find three random points:
        int r1 = rw::math::Math::ranI(0, int(lc_points.size()));
        int r2 = r1;
        int r3 = r1;
        while (r2 == r1) {
            r2 = rw::math::Math::ranI(0, int(lc_points.size()));
        }
        while (r3 == r1 || r3 == r2) {
            r3 = rw::math::Math::ranI(0, int(lc_points.size()));
        }

        rw::math::Vector3D<> p1 = lc_points[r1];
        rw::math::Vector3D<> p2 = lc_points[r2];
        rw::math::Vector3D<> p3 = lc_points[r3];

        rw::math::Vector3D<> center = rw::math::Vector3D<>((p1(0)+p2(0)+p3(0))/3, (p1(1)+p2(1)+p3(1))/3, (p1(2)+p2(2)+p3(2))/3);
        rw::math::EuclideanMetric<rw::math::Vector3D<>> metric;
        double dist = metric.distance(best_center, center);
        if (dist < best_dist) {
            best_center = center;
            best_dist = dist;
        }
    }
    

    v3d_out = best_center;
*/
    

    /*
    // get optical center
    std::array<cv::Mat, 2> pp_l = splitPp(proj_l);
    std::array<cv::Mat, 2> pp_r = splitPp(proj_r);
    cv::Mat c_l = computeOpticalCenter(pp_l);
    cv::Mat c_r = computeOpticalCenter(pp_r);
    std::cout << "\nOptical center left:\n" << c_l
              << "\nOptical center Right:\n" << c_r << std::endl;

    // get epipole
    cv::Mat epi_l = proj_l * c_r;
    cv::Mat epi_r = proj_r * c_l;
    std::cout << "\nLeft epipole:\n" << epi_l
              << "\nRight epipole:\n" << epi_r << std::endl;

    // get fundamental matrix left to right
    cv::Mat f_l2r = computeFundamentalMat(epi_r, proj_r, proj_l);
    std::cout << "\nFundamental matrix left to right:\n" << f_l2r << std::endl;

    // load images
    cv::Mat src_l = cv::imread("/home/mathi/Documents/rovi/rovi_project/sparse_stereo/Src_Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat src_r = cv::imread("/home/mathi/Documents/rovi/rovi_project/sparse_stereo/Src_Camera_Right.png", cv::IMREAD_COLOR);

    // find matches
    std::vector<cv::Point2f> l2r, r2l;
    findMatches(src_l, src_r, l2r, r2l);

    // triangulate
    for (std::size_t i = 0; i < l2r.size(); i++) {
        cv::Mat pnt3D = triangulate(l2r[i], r2l[i], f_l2r, c_l, c_r, pp_l, pp_r);
//        cv::Mat pnt3D = triangulate2(l2r[i], r2l[i], proj_l, proj_r);
        pnts_3d.push_back(pnt3D);

        std::cout << "\n3D point -->\n" << pnts_3d[i] << std::endl;
    }*/
}

void getNew3dPnts(const cv::Mat &proj_l, const cv::Mat &proj_r, std::vector<cv::Mat> &pnts_3d) {
    // get optical center
    std::array<cv::Mat, 2> pp_l = splitPp(proj_l);
    std::array<cv::Mat, 2> pp_r = splitPp(proj_r);
    cv::Mat c_l = computeOpticalCenter(pp_l);
    cv::Mat c_r = computeOpticalCenter(pp_r);
    std::cout << "\nOptical center left:\n" << c_l
              << "\nOptical center Right:\n" << c_r << std::endl;

    // get epipole
    cv::Mat epi_l = proj_l * c_r;
    cv::Mat epi_r = proj_r * c_l;
    std::cout << "\nLeft epipole:\n" << epi_l
              << "\nRight epipole:\n" << epi_r << std::endl;

    // get fundamental matrix left to right
    cv::Mat f_l2r = computeFundamentalMat(epi_r, proj_r, proj_l);
    std::cout << "\nFundamental matrix left to right:\n" << f_l2r << std::endl;

    // load images
    cv::Mat src_l = cv::imread("Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat src_r = cv::imread("Camera_Right.png", cv::IMREAD_COLOR);

    // find matches
    std::vector<cv::Point2f> l2r, r2l;
    findMatches(src_l, src_r, l2r, r2l);

    // triangulate
    for (std::size_t i = 0; i < l2r.size(); i++) {
        cv::Mat pnt3D = triangulate(l2r[i], r2l[i], f_l2r, c_l, c_r, pp_l, pp_r);
//        cv::Mat pnt3D = triangulate2(l2r[i], r2l[i], proj_l, proj_r);
        pnts_3d.push_back(pnt3D);

        std::cout << "\n3D point -->\n" << pnts_3d[i] << std::endl;
    }
}

void getInitialPose(Pose &pose) {
    // load workcell and state
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    rw::kinematics::State state = wc->getDefaultState();

    // get frames
//    Frame *duckFrame = wc->findFrame("Duck");
//    if (duckFrame == NULL) {
//        std::cerr << "Duck frame not found!" << std::endl;
//        return;
//    }

//    Frame *tableFrame = wc->findFrame("Table");
//    if (tableFrame == NULL) {
//        std::cerr << "Table frame not found!" << std::endl;
//        return;
//    }

    Frame *camLeftFrame = wc->findFrame("Camera_Left");
    if (camLeftFrame == NULL) {
        std::cerr << "Camera left not found!" << std::endl;
        return;
    }

//    Frame *camRightFrame = wc->findFrame("Camera_Right");
//    if (camRightFrame == NULL) {
//        std::cerr << "Camera right not found!" << std::endl;
//        return;
//    }

    // get pose
//    Pose duckPoseInv = rw::math::inverse(duckFrame->getTransform(state));
//    Pose tablePoseInv = rw::math::inverse(tableFrame->getTransform(state));
//    Pose camLeftPose = camLeftFrame->getTransform(state);
//    pose = duckPoseInv * tablePoseInv * camLeftPose;
    pose = camLeftFrame->getTransform(state);

    // print info
    std::cout << "\nInitial pose -->"
              << "\nPosition: " << pose.P()
              << "\nRotaion: " << pose.R() << std::endl;
}

void getCamerasInfo(cv::Mat &proj_l, cv::Mat &proj_r, cv::Mat &cam_mat_l, cv::Mat &cam_mat_r) {
    //std::cout << "\nLeft camera.." << std::endl;
    getProjectionMatrix("Camera_Left", proj_l, cam_mat_l);
    //std::cout << "\nLeft camera projection matrix -->\n" << proj_l << std::endl;
    //std::cout << "\nLeft camera matrix -->\n" << cam_mat_l << std::endl;

    //std::cout << "\nRight camera.." << std::endl;
    getProjectionMatrix("Camera_Right", proj_r, cam_mat_r);
    //std::cout << "\nRight camera projection matrix -->\n" << proj_r << std::endl;
    //std::cout << "\nRight camera matrix -->\n" << cam_mat_r << std::endl;
}

void moveFrameRandom(rw::models::WorkCell::Ptr &workcell, rw::kinematics::State &state, const std::string &frameName) {
    //std::cout << "\nMoving " << frameName << " to random position.." << std::endl;

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

    // std::cout << "\tNew position of " << frameName << " -->"
    //           << "\n\tPosition: " << frame->getTransform(state).P()
    //           << "\n\tRotation: " << frame->getTransform(state).R()
    //           << std::endl;
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
              << "\n\t\tRotation: " << rw::math::RPY<>(frame->getTransform(state).R())
              << std::endl;

    // move object
    frame->moveTo(newPose, state);

    // get current pose
    std::cout << "\tNew pose -->"
              << "\n\t\tPosition: " << frame->getTransform(state).P()
              << "\n\t\tRotation: " << Rpy(frame->getTransform(state).R())
              << std::endl;
}

std::vector<Pose> loadRandomPoses() {
    std::cout << "Loading random poses.." << std::endl;
    double ya = rw::math::Deg2Rad*90;
    double ra = rw::math::Deg2Rad*90;
    std::vector<Pose> result = {
        Pose( Vec(-0.145188, 0.443078, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.093862, 0.436711, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.122399, 0.495443, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.091337, 0.498684, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.264603, 0.418769, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.235944, 0.419935, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.135748, 0.504414, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.250770, 0.419906, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.200208, 0.488607, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.108419, 0.449887, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.217942, 0.460303, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.203439, 0.422350, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.205437, 0.440262, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.000940, 0.390936, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.011533, 0.466524, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.083706, 0.473673, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.215880, 0.460187, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.111775, 0.417223, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.286724, 0.451241, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.007980, 0.407581, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.142925, 0.431557, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.025873, 0.417860, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(0.260845, 0.376162, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.143089, 0.519847, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.190782, 0.508894, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.272231, 0.508522, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.289646, 0.426160, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.247180, 0.440277, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.213684, 0.408139, 0.13275), Rpy(ra, 0, ya).toRotation3D() ),
        Pose( Vec(-0.241997, 0.445336, 0.13275), Rpy(ra, 0, ya).toRotation3D() )
    };
    return result;
}

Pose matrix2Transform(const Eigen::Matrix4f matrix) {
    Vec pos = Vec(matrix(0,3), matrix(1,3), matrix(2,3));
    Rotation rotm = Rotation( matrix(0,0), matrix(0,1), matrix(0,2),
                              matrix(1,0), matrix(1,1), matrix(1,2),
                              matrix(2,0), matrix(2,1), matrix(2,2) );
    return Pose(pos, rotm);
}

Pose calcTransformationP2P(const std::vector<cv::Mat> &src_pnts_3d,
                           const std::vector<cv::Mat> &new_pnts_3d,
                           const Pose &src_pose) {

    Pose result;

    std::cout << "\nCalculating transformation based on the initial points, new points and the known pose.." << std::endl;

    // convert Point3d to point cloud
    // PointT pcl_default;
    PointCloudT::Ptr src_cloud(new PointCloudT);
    PointCloudT::Ptr new_cloud(new PointCloudT);

    cv::Mat cv_pnt;
    PointT pcl_pnt;
    for (std::size_t i = 0; i < src_pnts_3d.size(); i++) {
        // add point to src cloud
        cv_pnt = src_pnts_3d[i];
        pcl_pnt.x = cv_pnt.at<double>(0,0);
        pcl_pnt.y = cv_pnt.at<double>(1,0);
        pcl_pnt.z = cv_pnt.at<double>(2,0);
        src_cloud->push_back(pcl_pnt);

        // add point to new cloud
        cv_pnt = new_pnts_3d[i];
        pcl_pnt.x = cv_pnt.at<double>(0,0);
        pcl_pnt.y = cv_pnt.at<double>(1,0);
        pcl_pnt.z = cv_pnt.at<double>(2,0);
        new_cloud->push_back(pcl_pnt);

        std::cout << "\nCorresponding points -->"
                  << "\n" << src_cloud->points[i].x << " <--> " << new_cloud->points[i].x
                  << "\n" << src_cloud->points[i].y << " <--> " << new_cloud->points[i].y
                  << "\n" << src_cloud->points[i].z << " <--> " << new_cloud->points[i].z << std::endl;
    }

    src_cloud->width = src_cloud->points.size();
    src_cloud->height = 1;
    src_cloud->is_dense = true;

    new_cloud->width = new_cloud->points.size();
    new_cloud->height = 1;
    new_cloud->is_dense = true;

    // estimate transformation
    Eigen::Matrix4f T;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
    svd.estimateRigidTransformation(*src_cloud, *new_cloud, T);

    Pose pose = matrix2Transform(T);
    result = src_pose * pose;
    std::cout << "\nEstimated pose -->"
              << "\nPos: " << result.P()
              << "\nRot: " << result.R() << std::endl;

    return result;
}

std::pair<double, double> calcError(rw::models::WorkCell::Ptr workcell, rw::kinematics::State &state, const Pose &pose) {
    std::cout << "Calculating performance.." << std::endl;
    std::pair<double, double> result = std::make_pair(0.0, 0.0);

    // get poses
    Pose table, cam, realPose;
    {
        Frame *tableFrame = workcell->findFrame("Table");
        table = tableFrame->getTransform(state);
        MovFrame *camFrame = workcell->findFrame<MovFrame>("Camera_Left");
        cam = camFrame->getTransform(state);
        Frame *duckFrame = workcell->findFrame("Duck");
        realPose = duckFrame->getTransform(state);
    }

    // show real pose
    std::cout << "\tReal pose -->"
              << "\n\t\tPosition: " << realPose.P()
              << "\n\t\tRotation: " << realPose.R() << std::endl;

    // estimate pose
    //Pose tableInv = rw::math::inverse(table);
    //Pose estimatedPose = tableInv * cam * pose;
    Pose estimatedPose = cam * pose;
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
        Rotation P = realPose.R(), Q = estimatedPose.R();
        std::cout << "\tReal pose rotation matrix --> " << P << std::endl;
        std::cout << "\tEstimated pose rotation matrix --> " << Q << std::endl;
        Rotation R = P * Q.inverse(); // The transpose of a rotation matrix is the same as the inverse
        std::cout << "\tP * Q' = R --> " << R << std::endl;

        double traceR = R(0,0) + R(1,1) + R(2,2);
        std::cout << "\tTrace of R --> "
                  << R(0,0) << " + "
                  << R(1,1) << " + "
                  << R(2,2) << " = "
                  << traceR << std::endl;
        diffAngle = acos( (traceR-1.0) / 2.0 );
        std::cout << "\tAngle error: " << diffAngle << " [rad]" << std::endl;
        diffAngle = diffAngle * rw::math::Rad2Deg;
        std::cout << "\tAngle error: " << diffAngle << " [deg]" << std::endl;
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


void add_gaussian_noise(cv::Mat &i1, cv::Mat &i2, const float sd1, const float sd2, const float sd3) {
    std::cout << "Starting to add gaussian" << std::endl;
    cv::Mat mean = cv::Mat::zeros(cv::Size(1, i1.channels()), CV_32FC1);
    float sd_vals[] = {sd1, sd2, sd3};
    std::cout << "Mean: " << mean << std::endl;
    cv::Mat std_dev = cv::Mat(i1.channels(), 1, CV_32FC1, sd_vals);
    std::cout << "Std dev: " << std_dev << std::endl;
    cv::Mat noise = cv::Mat(i1.size(), i1.type());
    cv::randn(noise, mean, std_dev);
    i1 += noise;

    noise = cv::Mat(i2.size(), i2.type());
    cv::randn(noise, mean, std_dev);
    i2 += noise;
}
