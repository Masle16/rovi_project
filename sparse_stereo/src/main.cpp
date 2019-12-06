#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/traits.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "util.hpp"

int main(int argv, char** argc) {
    std::cout << "\nProgram started\n" << std::endl;

    /*****************/
    /* CAMERA MATRIX */
    /*****************/
    // get projection matrix
    cv::Mat projLeft, projRight, camMatLeft, camMatRight;
    std::cout << "Left camera.." << std::endl;
    getProjectionMatrix("Camera_Left", projLeft, camMatLeft);
    std::cout << "\nLeft camera projection matrix -->\n" << projLeft << std::endl;
    std::cout << "\nLeft camera matrix -->\n" << camMatRight << std::endl;

    std::cout << "Right camera.." << std::endl;
    getProjectionMatrix("Camera_Right", projRight, camMatRight);
    std::cout << "\nRight camera projection matrix -->\n" << projRight << std::endl;
    std::cout << "\nRight camera matrix -->\n" << camMatRight << std::endl;

    // get optical center
    std::array<cv::Mat, 2> ppLeft = splitPp(projLeft);
    std::array<cv::Mat, 2> ppRight = splitPp(projRight);
    cv::Mat centerLeft = computeOpticalCenter(ppLeft);
    cv::Mat centerRight = computeOpticalCenter(ppRight);
    std::cout << "\nOptical center left:\n" << centerLeft
              << "\nOptical center Right:\n" << centerRight << std::endl;

    // get epipole
    cv::Mat epipoleLeft = projLeft * centerRight;
    cv::Mat epipoleRight = projRight * centerLeft;
    std::cout << "\nLeft epipole:\n" << epipoleLeft
              << "\nRight epipole:\n" << epipoleRight << std::endl;

    // get fundamental matrix left to right
    cv::Mat fundamentalLeftRight = computeFundamentalMat(epipoleRight, projRight, projLeft);
    std::cout << "\nFundamental matrix left to right:\n" << fundamentalLeftRight << std::endl;

    /**************************/
    /* find matches in images */
    /**************************/
    // load images
    cv::Mat sceneLeft = cv::imread("../../Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat sceneRight = cv::imread("../../Camera_Right.png", cv::IMREAD_COLOR);

    // filter images
    sceneLeft = colorFiltering(sceneLeft);
    sceneRight = colorFiltering(sceneRight);

    // step 1: detect the keypoints using surf detector, compute the descriptors
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptor1, descriptor2;
    detector->detectAndCompute(sceneLeft, cv::noArray(), keypoints1, descriptor1);
    detector->detectAndCompute(sceneRight, cv::noArray(), keypoints2, descriptor2);

    // step 2: matching descriptor vectors with a brute force matcher
    // since surf is floating-point descriptor NORM_L2 is used
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptor1, descriptor2, matches, 2);

    // filter matches using the Lowe's ratio test
    const float ratioThresh = 0.7f;
    std::vector<cv::DMatch> goodMatches;
    for (std::size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < ratioThresh * matches[i][1].distance) {
            goodMatches.push_back(matches[i][0]);
        }
    }

    std::vector<cv::Point2f> matchedPoints1, matchedPoints2;
    for (std::size_t i = 0; i < goodMatches.size(); i++) {
        int idx1 = goodMatches[i].trainIdx;
        int idx2 = goodMatches[i].queryIdx;
        matchedPoints1.push_back(keypoints1[idx1].pt);
        matchedPoints2.push_back(keypoints2[idx2].pt);
    }

//    // draw matches
//    cv::Mat imgMatches;
//    cv::drawMatches(sceneLeft, keypoints1, sceneRight, keypoints2, goodMatches, imgMatches,
//                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

//    // show matches
//    cv::imshow("Matches", imgMatches);
//    cv::waitKey();

    /*****************/
    /* Triangulate z */
    /*****************/
    std::vector<cv::Point3d> pnt3Ds;
    for (std::size_t i = 0; i < matchedPoints1.size(); i++) {
        cv::Point2f pntLeft = matchedPoints1[i];
        cv::Point2f pntRight = matchedPoints2[i];
//        cv::Mat pnt3D = triangulate(pntLeft, pntRight, fundamentalLeftRight, centerLeft, centerRight, ppLeft, ppRight);
        cv::Mat pnt3D = triangulate2(pntLeft, pntRight, projLeft, projRight);
        pnt3Ds.push_back(mat2Point3f(pnt3D));
    }

    /*******************/
    /* POSE ESTIMATION */
    /*******************/
    std::cout << "\nNumber of image points: " << matchedPoints1.size()
              << "\nNumber of model points: " << pnt3Ds.size() << std::endl;

    cv::Mat distCoeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
    cv::Mat rotation, translation;
    cv::solvePnP(pnt3Ds, matchedPoints1, projLeft, distCoeffs, rotation, translation);
    std::cout << "Rotation -->\n" << rotation << std::endl;
    std::cout << "Position -->\n" << translation << std::endl;

    std::cout << "\nProgram ended\n" << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
