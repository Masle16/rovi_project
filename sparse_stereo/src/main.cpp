#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
//        cv::bitwise_and(img, img, result, mask=mask);
//        cv::imshow("Thresholded Image", mask); //show the thresholded image
//        cv::imshow("Original", img); //show the original image
//        cv::imshow("Output", result);
//        //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
//        if (cv::waitKey(0) == 27) { break; }
//    }
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0,0,0), cv::Scalar(197,30,255), mask);
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::bitwise_and(img, img, result, mask=mask);
    return result;
}

int main(int argv, char** argc) {
    std::cout << "\nProgram started\n" << std::endl;

    // load images
    cv::Mat object = cv::imread("../../object.png", cv::IMREAD_COLOR);
    cv::Mat sceneLeft = cv::imread("../../Camera_Left.png", cv::IMREAD_COLOR);
    cv::Mat sceneRight = cv::imread("../../Camera_Right.png", cv::IMREAD_COLOR);

    // filter images
    object = colorFiltering(object);
    sceneLeft = sceneLeft(cv::Rect(245, 203, 299, 98));
    sceneRight = sceneRight(cv::Rect(170, 203, 300, 98));
    cv::imshow("Object", object);
    cv::imshow("Left image", sceneLeft);
    cv::imshow("Right image", sceneRight);
    cv::waitKey();

    // step 1: detect the keypoints using surf detector, compute the descriptors
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;
    cv::Mat descriptor1, descriptor2, descriptor3;
    detector->detectAndCompute(object, cv::noArray(), keypoints1, descriptor1);
    detector->detectAndCompute(sceneLeft, cv::noArray(), keypoints2, descriptor2);
    detector->detectAndCompute(sceneRight, cv::noArray(), keypoints3, descriptor3);

    // step 2: matching descriptor vectors with a brute force matcher
    // since surf is floating-point descriptor NORM_L2 is used
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    std::vector<cv::DMatch> matchesLeft, matchesRight;
    matcher->match(descriptor1, descriptor2, matchesLeft);
    matcher->match(descriptor1, descriptor3, matchesRight);

    // draw matches
    cv::Mat leftImgMatches, rightImgMatches;
    cv::drawMatches(object, keypoints1, sceneLeft, keypoints2, matchesLeft, leftImgMatches);
    cv::drawMatches(object, keypoints1, sceneRight, keypoints3, matchesRight, rightImgMatches);

    // show matches
    cv::imshow("Left matches", leftImgMatches);
    cv::imshow("Right matches", rightImgMatches);
    cv::waitKey();

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
