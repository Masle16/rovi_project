#include <iostream>
#include <vector>

#include "util.hpp"

int main(int argc, char **argv) {
    
    // Read Image
    cv::Mat img = cv::imread("../../Camera_Right.png", cv::IMREAD_COLOR);

	// Images for visualization
	cv::Mat img_show, img_warp_show;
	
	// Set image corners and and visualize
    std::vector<cv::Point2f> corners_image;

    corners_image.push_back(cv::Point2f(184, 203));
    corners_image.push_back(cv::Point2f(170, 301));
    corners_image.push_back(cv::Point2f(456, 203));
    corners_image.push_back(cv::Point2f(469, 301));

    for (unsigned int i = 0; i <corners_image.size(); i++) {
        cv::circle(img, corners_image[i], 50, cv::Scalar(255,255,255), 10);
    }

	cv::pyrDown( img, img_show );
	cv::pyrDown( img_show, img_show );
	cv::imshow("Original", img_show );
    cv::waitKey();	
	
	// Compute the homography
	cv::Mat H = computeHomography( corners_image );
    std::cout << "H:\n" << H << std::endl;
    cv::Mat img_warp = warpImage( img, H );
	cv::pyrDown( img_warp, img_warp_show );
	cv::pyrDown( img_warp_show, img_warp_show );
    cv::imshow("Warp", img_warp_show);
    cv::waitKey();


    cv::waitKey();
}
