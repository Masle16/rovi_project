#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/stereo/stereo_matching.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <iostream>
#include <thread>

#define LEFT_IMG "../../Camera_Left.pcd"
#define RIGHT_IMG "../../Camera_Right.pcd"

using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

void savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        for (size_t j = 0; j < points.cols; j++) {
            // Check if points are too far away
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            if (fabs(xyz[2]) > max_z) { continue; }

            // If not save them in point cloud
            pclPoint p;
            p.x = xyz[0]; p.y = xyz[1]; p.z = xyz[2];
            p.b = bgr[0]; p.g = bgr[1]; p.r = bgr[2];
            dst->at(i,j) = p;
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}

cv::Mat defineQ(int img_width, int img_height)
{
    // Define Q matrix as per document
    cv::Mat Q = (cv::Mat_<double>(4, 4) << 1.0, 0, 0, -img_width/2.0,
                                           0, 1.0, 0, -img_height/2.0,
                                           0, 0, 0, 500.0,
                                           0, 0, -1.0/(-100.0), 0);
    return Q;
}

cv::Mat DisparityBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    // Compute disparity with cv::StereoBM
    cv::Mat result;
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(nDisparities, BlockSize);
    sbm->compute(imgL, imgR, result);
    return result;
}

cv::Mat DisparitySGBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    // Compute disparity with cv::StereoSGBM
    cv::Mat result;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, nDisparities, BlockSize);
    sgbm->compute(imgL, imgR, result);
    return result;
}

cv::Mat normDisparity(cv::Mat disp)
{
    // Normalize matrix to unsigned integers between 0-255 needed for visualization
    cv::Mat result;
    cv::normalize(disp, result, 0, 255, cv::NORM_MINMAX,  CV_8UC1);
    return result;
}

cv::Mat reproject3D(cv::Mat disp, cv::Mat Q)
{
    // Reproject disparity image to 3D
    cv::Mat result;
    cv::reprojectImageTo3D(disp, result, Q);
    return result;
}

int main(int argc, char** argv) {
    std::cout << "\nProgram Started\n" << std::endl;

    pcl::PointCloud<pcl::RGB>::Ptr leftCloud(new pcl::PointCloud<pcl::RGB>);
    pcl::PointCloud<pcl::RGB>::Ptr rightCloud(new pcl::PointCloud<pcl::RGB>);

    // Read files
    pcl::PCDReader pcd;
    if (pcd.read(LEFT_IMG, *leftCloud)) {
        std::cerr << "Could not read " << LEFT_IMG << std::endl;
        return -1;
    }
    if (pcd.read(RIGHT_IMG, *rightCloud)) {
        std::cerr << "Could not read " << RIGHT_IMG << std::endl;
        return -1;
    }

    if (!leftCloud->isOrganized() || !rightCloud->isOrganized() ||
            leftCloud->width != rightCloud->width || leftCloud->height != rightCloud->height) {
        std::cerr << "Stereo pair does not match --> check inputs.." << std::endl;
        return -1;
    }

    // choice between the two algorithms
    //pcl::AdaptiveCostSOStereoMatching stereo;
    pcl::BlockBasedStereoMatching stereo;
    stereo.setMaxDisparity(60);
    stereo.setXOffset(0);
    stereo.setRadius(5);
//    // only needed for AdaptiveCostSOStereoMatching
//    stereo.setSmoothWeak(20);
//    stereo.setSmoothStrong(100);
//    stereo.setGammaC(25);
//    sterep.setGammaS(10);
    stereo.setRatioFilter(20);
    stereo.setPeakFilter(0);
    stereo.setLeftRightCheck(true);
    stereo.setLeftRightCheckThreshold(1);
    stereo.setPreProcessing(true);
    stereo.compute(*leftCloud, *rightCloud);
    stereo.medianFilter(4);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    stereo.getPointCloud(318.112200, 224.334900, 368.534700, 0.8387445, cloud, leftCloud);

    pcl::PointCloud<pcl::RGB>::Ptr vMap(new pcl::PointCloud<pcl::RGB>);
    stereo.getVisualMap(vMap);
    pcl::visualization::ImageViewer imgViewer("Image View");
    imgViewer.addRGBImage<pcl::RGB>(vMap);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D View"));
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> intensity(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, intensity, "stereo");
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spin();
        imgViewer.spin();
    }

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
