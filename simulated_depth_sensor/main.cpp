#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>

/**
 * @brief voxelGrid
 * @param input_cloud
 * @param output_cloud
 */
void voxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01);
    voxel_grid.filter(*output_cloud);
}

/**
 * @brief outlierRemoval
 * @param input_cloud
 * @param output_clod
 */
void outlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;
    statistical_outlier_removal.setInputCloud(input_cloud);
    statistical_outlier_removal.setMeanK(200);
    statistical_outlier_removal.setStddevMulThresh(0.1);
    statistical_outlier_removal.filter(*output_cloud);
}

/**
 * @brief spatialFilter
 * @param input_cloud
 * @param output_cloud
 */
void spatialFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    pcl::PassThrough<pcl::PointXYZ> spatial_filter;
    spatial_filter.setInputCloud(input_cloud);
    spatial_filter.setFilterFieldName("z");
    spatial_filter.setFilterLimits(-1.5, -1.225);
    spatial_filter.filter(*output_cloud);
}

/**
 * @brief smoothing
 * @param input_cloud
 * @param output_cloud
 */
void smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    // create a kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

    // set parameters
    mls.setInputCloud(input_cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(5);

    // reconstruct
    mls.process(mls_points);
    pcl::copyPointCloud(mls_points, *output_cloud);
}

int main(int argc, char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // load the point cloud
    pcl::PCDReader reader;
    reader.read("../Scanner25D.pcd", *cloud);

    // display point cloud info
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::cout << "Max x: " << max_pt.x << std::endl;
    std::cout << "Max y: " << max_pt.y << std::endl;
    std::cout << "Max z: " << max_pt.z << std::endl;
    std::cout << "Min x: " << min_pt.x << std::endl;
    std::cout << "Min y: " << min_pt.y << std::endl;
    std::cout << "Min z: " << min_pt.z << std::endl;
    std::cerr << "PointCloud before filtering: "
              << cloud->width * cloud->height
              << " data points ("
              << pcl::getFieldsList(*cloud)
              << ")."
              << std::endl;

    // filter point cloud
    voxelGrid(cloud, cloud_filtered);
    //outlierRemoval(cloud_filtered, cloud_filtered);
    spatialFilter(cloud_filtered, cloud_filtered);
    //smoothing(cloud_filtered, cloud_filtered);

    // display filtered cloud info
    std::cerr << "PointCloud after filtering: "
              << cloud_filtered->width * cloud_filtered->height
              << " data points ("
              << pcl::getFieldsList(*cloud_filtered)
              << ")."
              << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("../cloud_filtered.pcd", *cloud_filtered, false);

    std::cout << "\nProgram ended\n" << std::endl;

    return 0;
}
