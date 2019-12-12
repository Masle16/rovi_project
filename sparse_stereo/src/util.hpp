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

/*
 * DEFINES
 */
#define WC_FILE "../../workcell/Scene.wc.xml"

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
    //std::cout << "Projection matrix and parts: " << std::endl << proj << ", " << std::endl;// << pp[0] << ", " << std::endl << pp[1] << std::endl;
    //std::cout << "pp[0]: \n" << pp[0] << std::endl;
    //std::cout << "pp[1]: \n" << pp[1] << std::endl;
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
    std::cout << "\nClean image 1 --> image 2 matches: " << ratioTest(matches1) << " (removed)."
              << "\nClean image 2 --> image 1 matches: " << ratioTest(matches2) << " (removed)."
              << std::endl;

    // remove non-symmetrical matches
    std::vector<cv::DMatch> goodMatches;
    symmetryTest(matches1, matches2, goodMatches);

    for (size_t i = 0; i < goodMatches.size(); i++) {
        std::cout << "trainIdx: " << goodMatches[i].trainIdx << std::endl;
        std::cout << "queryIdx: " << goodMatches[i].queryIdx << std::endl;
    }
    std::vector<cv::DMatch> test = {goodMatches[0]};

    //cv::Mat matched_im;
    //cv::drawMatches(_src_l,keypoints1,_src_r,keypoints2,test,matched_im);
    //cv::imshow("Mathces: ", matched_im);

    // get point
    for (std::size_t i = 0; i < goodMatches.size(); i++) {
        int idx1 = goodMatches[i].trainIdx;
        int idx2 = goodMatches[i].queryIdx;
        l2r.push_back(keypoints1[idx1].pt);
        r2l.push_back(keypoints2[idx2].pt);
        std::cout << l2r[i] << ", " << r2l[i] << std::endl;
    }
}

void getInitial3dPnts(const cv::Mat &proj_l, const cv::Mat &proj_r, std::vector<cv::Mat> &pnts_3d) {
    // get optical center
    std::array<cv::Mat, 2> pp_l = splitPp(proj_l);
    std::array<cv::Mat, 2> pp_r = splitPp(proj_r);
    cv::Mat c_l = computeOpticalCenter(pp_l);
    cv::Mat c_r = computeOpticalCenter(pp_r);
    std::cout << "\nOptical center left:\n" << c_l
              << "\nOptical center Right:\n" << c_r << std::endl;


    // load images
    cv::Mat src_l = cv::imread("../../Src_Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat src_r = cv::imread("../../Src_Camera_Right.png", cv::IMREAD_COLOR);

    //cv::imshow("left",src_l);
    //cv::imshow("right",src_r);

    // Find features in the images:
    std::vector<cv::Point2f> l2r, r2l;
    findMatches(src_l, src_r, l2r, r2l);

    std::cout << "Projection 1:\n" << proj_l << std::endl << "Projection 2:\n" << proj_r << std::endl;
    cv::Mat out_points;
    cv::triangulatePoints(proj_l,proj_r,l2r,r2l,out_points);

    std::cout << out_points.cols << std::endl;
    std::vector<rw::math::Vector3D<>> lc_points;
    for (int i = 0; i < out_points.cols; i++) {
        float scale = 1/out_points.at<float>(3,i);
        lc_points.push_back(rw::math::Vector3D<>(scale*out_points.at<float>(0,i), scale*out_points.at<float>(1,i), scale*out_points.at<float>(2,i)));
    }

    std::cout << lc_points[0] << std::endl;

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    rw::kinematics::State state = wc->getDefaultState();
    const std::string deviceName = "UR-6-85-5-A";
    rw::models::SerialDevice::Ptr robot = wc->findDevice<rw::models::SerialDevice>(deviceName);

    rw::invkin::ClosedFormIKSolverUR ikinSolver(robot, state);


    rw::math::Transform3D<> desired_point(lc_points[0], rw::math::EAA<>(0,0,0).toRotation3D());

    std::vector<rw::math::Q> test_point = ikinSolver.solve(desired_point,state);

    std::cout << test_point.size() << ", " << test_point[0] << std::endl;
    robot->setQ(test_point[0], state);


    /*
    // get epipole
    cv::Mat epi_l = proj_l * c_r;
    cv::Mat epi_r = proj_r * c_l;
    std::cout << "\nLeft epipole:\n" << epi_l
              << "\nRight epipole:\n" << epi_r << std::endl;

    // get fundamental matrix left to right
    cv::Mat f_l2r = computeFundamentalMat(epi_r, proj_r, proj_l);
    std::cout << "\nFundamental matrix left to right:\n" << f_l2r << std::endl;

    // load images
    cv::Mat src_l = cv::imread("../../Src_Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat src_r = cv::imread("../../Src_Camera_Right.png", cv::IMREAD_COLOR);

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
    cv::Mat src_l = cv::imread("../../Src_Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat src_r = cv::imread("../../Src_Camera_Right.png", cv::IMREAD_COLOR);

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
    std::cout << "\nLeft camera.." << std::endl;
    getProjectionMatrix("Camera_Left", proj_l, cam_mat_l);
    std::cout << "\nLeft camera projection matrix -->\n" << proj_l << std::endl;
    std::cout << "\nLeft camera matrix -->\n" << cam_mat_l << std::endl;

    std::cout << "\nRight camera.." << std::endl;
    getProjectionMatrix("Camera_Right", proj_r, cam_mat_r);
    std::cout << "\nRight camera projection matrix -->\n" << proj_r << std::endl;
    std::cout << "\nRight camera matrix -->\n" << cam_mat_r << std::endl;
}

void moveFrameRandom(rw::models::WorkCell::Ptr &workcell, rw::kinematics::State &state, const std::string &frameName) {
    std::cout << "\nMoving " << frameName << " to random position.." << std::endl;

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
        Pose( Vec(-0.145188, 0.443078, 0.13275), Rpy(90.655720, 0, 0).toRotation3D() ),
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

Pose matrix2Transform(const Eigen::Matrix4f matrix) {
    Vec pos = Vec(matrix(0,3), matrix(1,3), matrix(2,3));
    Rotation rotm = Rotation( matrix(0,0), matrix(0,1), matrix(0,2),
                              matrix(1,0), matrix(1,1), matrix(1,2),
                              matrix(2,0), matrix(2,1), matrix(2,2) );
    return Pose(pos, rotm);
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
