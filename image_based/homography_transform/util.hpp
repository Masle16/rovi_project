#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

cv::Mat computeHomography( std::vector<cv::Point2f> corners_image)
{
    std::vector<cv::Point2f> corners_plane;
    corners_plane.push_back(cv::Point2f(184, 203));
    corners_plane.push_back(cv::Point2f(170, 301));
    corners_plane.push_back(cv::Point2f(456, 203));
    corners_plane.push_back(cv::Point2f(469, 301));

    cv::Mat H = cv::findHomography(corners_image,corners_plane);

    return H;
}

cv::Mat warpImage( cv::Mat img, cv::Mat H )
{
    cv::Mat img_warp;
    cv::Size paper_size;
    paper_size = cv::Size(2970,2100);

    cv::warpPerspective(img, img_warp, H, paper_size);

    return img_warp;

}

void computeWidthHeight( std::vector<cv::Vec3d> corners_pcb_original, cv::Mat H, double &width, double &height )
{

    std::vector<cv::Vec2d> corners_pcb_warped;

    for (unsigned int i = 0; i <corners_pcb_original.size(); i++)
    {
        cv::Mat p0 = H * cv::Mat(corners_pcb_original[i]);
        corners_pcb_warped.push_back(
                cv::Vec2d(p0.at<double>(0, 0) / p0.at<double>(2, 0), p0.at<double>(1, 0) / p0.at<double>(2, 0)));
    }

    width = cv::norm(corners_pcb_warped[0] - corners_pcb_warped[1], cv::NORM_L2);
    height = cv::norm(corners_pcb_warped[0] - corners_pcb_warped[2], cv::NORM_L2);

}
