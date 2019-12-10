/*
 * INCLUDES
 */
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/traits.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <rw/rw.hpp>

#include "util.hpp"

/*
 * DEFINES
 */

/*
 * TYPEDEFS
 */

/*
 * GLOBAL VARIABLES
 */
const std::string DEVICE_NAME = "UR-6-85-5-A";

/*
 * FUNCTIONS
 */
void getCamerasInfo(cv::Mat &proj_l, cv::Mat &proj_r, cv::Mat &cam_mat_l, cv::Mat &cam_mat_r);
void findMatches(cv::Mat &src_l, cv::Mat &src_r, std::vector<cv::Point2f> &l2r, std::vector<cv::Point2f> &r2l, const bool showMatches=false);
void getPose(const rw::models::WorkCell::Ptr &wc, const State &state, Pose &pose);
void get3dPnts(const cv::Mat &proj_l, const cv::Mat &proj_r,std::vector<cv::Point3d> &pnts_3d);

/*
 * MAIN ENTRY POINT
 */
int main(int argv, char** argc) {
    std::cout << "\nProgram started\n" << std::endl;

    // get projection matrix
    cv::Mat proj_l, proj_r, cam_mat_l, cam_mat_r;
    getCamerasInfo(proj_l, proj_r, cam_mat_l, cam_mat_r);

    // get 3d points
    std::vector<cv::Point3d> src_pnts_3d;
    get3dPnts(proj_l, proj_3, src_pnts_3d);

    // get initial pose
    Pose src_pose;
    getPose(src_pose);

    std::cout << "\nProgram ended\n" << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
