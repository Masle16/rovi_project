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

/*
 * MAIN ENTRY POINT
 */
int main(int argv, char** argc) {
    std::cout << "\nProgram started\n" << std::endl;

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    rw::kinematics::State state = wc->getDefaultState();

    cv::Mat proj_l, proj_r, cam_mat_l, cam_mat_r;
    getCamerasInfo(proj_l, proj_r, cam_mat_l, cam_mat_r);



    // get initial points
    std::vector<cv::Mat> src_pnts_3d;
    getInitial3dPnts(proj_l, proj_r, src_pnts_3d);

    /*

    // get initial pose
    Pose src_pose;
    getInitialPose(src_pose);

    // get new points
    std::vector<cv::Mat> new_pnts_3d;
    getNew3dPnts(proj_l, proj_r, new_pnts_3d);

    */

    std::cout << "\nProgram ended\n" << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
