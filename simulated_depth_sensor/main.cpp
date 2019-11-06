//----------------------------------------------------------
// Includes
#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/registration.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/common/random.h>
//---------------------------------------------------------

//---------------------------------------------------------
// Defines
#define MAX_ITERATIONS 10000
#define LEAF_SIZE 0.01
#define MEAN 200
#define STD_DEV 0.1
#define FILTER_LIMIT_MIN -1.5
#define FILTER_LIMIT_MAX -1.225
#define SMOOTHING_RADIUS 5
#define SPIN_IMAGES_RADIUS 0.05
#define ALIGNMENT_THRESHOLD std::pow(0.05, 2)
#define SCENE_PATH "../Scanner25D.pcd"
#define OBJECT_PATH "../../point_clouds_of_objects/bottle.pcd"
//---------------------------------------------------------

//---------------------------------------------------------
// Functions
/**
 * @brief showPointClouds
 * @param scene
 * @param object
 * @param view_name
 */
void showPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                     const std::string &view_name) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(view_name));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_g(scene, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_r(object, 255, 0, 0);
    viewer->addPointCloud(scene, color_g, "Scene");
    viewer->addPointCloud(object, color_r, "Object");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Scene");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Object");
    viewer->initCameraParameters();
    viewer->setCameraClipDistances(0.244446, 0.824412);
    viewer->setCameraPosition(-0.0562254, -0.0666346, -0.529442, -0.00165773, -0.0633305, -0.167617, 0.982829, 0.108549, -0.149214);
    viewer->setPosition(662, 137);
    while (!viewer->wasStopped()) { viewer->spinOnce(100); }
}

/**
 * @brief voxelGrid
 * @param input_cloud
 * @param output_cloud
 */
void voxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
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
    statistical_outlier_removal.setMeanK(MEAN);
    statistical_outlier_removal.setStddevMulThresh(STD_DEV);
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
    spatial_filter.setFilterLimits(FILTER_LIMIT_MIN, FILTER_LIMIT_MAX);
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
    mls.setSearchRadius(SMOOTHING_RADIUS);

    // reconstruct
    mls.process(mls_points);
    pcl::copyPointCloud(mls_points, *output_cloud);
}

/**
 * This function is from exercise 7 in the computer vision course.
 *
 * @brief estimateSurfaceNormal : Compute the surface normals using k=10 nearest
 *  neighbors around each point
 * @param cloud
 * @return
 */
pcl::PointCloud<pcl::Normal>::Ptr estimateSurfaceNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // create normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    // creat KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normal_estimator.setSearchMethod(tree);
    // create result
    pcl::PointCloud<pcl::Normal>::Ptr result(new pcl::PointCloud<pcl::Normal>);
    // set search size and compute
    normal_estimator.setKSearch(10);
    normal_estimator.compute(*result);
    return result;
}

/**
 * This function is from exercise 7 in the computer vision course.
 *
 * @brief computeSpinImages : Compute local spin image features with r=5 surface
 *  description.
 * @param cloud
 * @param normals
 * @return
 */
pcl::PointCloud<pcl::Histogram<153>>::Ptr computeSpinImages(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                            pcl::PointCloud<pcl::Normal>::Ptr normals) {
    // setup spin image computation
    pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>> spin_image_descriptor(8, 0.5, 4);
    spin_image_descriptor.setInputCloud(cloud);
    spin_image_descriptor.setInputNormals(normals);
    // create KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    spin_image_descriptor.setSearchMethod(tree);
    // Calculate spin images
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spin_images(new pcl::PointCloud<pcl::Histogram<153>>());
    spin_image_descriptor.setRadiusSearch(SPIN_IMAGES_RADIUS);
    // compute the spin images
    spin_image_descriptor.compute(*spin_images);
    return spin_images;
}

/**
 * This functions is form exercise 7 in the computer vision course
 *
 * @brief calcL2Dist : compute the l2 distance between the two histograms
 * @param object : histogram of the object
 * @param scene : histogram of the scene
 * @return : l2 distance
 */
float computeL2Dist(const pcl::Histogram<153> &object,
                    const pcl::Histogram<153> &scene) {
    float result = 0.0;
    for (unsigned int i = 0; i < scene.descriptorSize(); i++) {
        result += std::sqrt(std::pow((scene.histogram[i] - object.histogram[i]), 2));
    }
    return result;
}

/**
 * This function is from exercise 7 in the computer vision course.
 *
 * @brief nearestMatchingFeature : find the nearest matching (k=1) scene feature
 *  for each object feature
 * @param scene : spin images of the scene point cloud
 * @param object : spin images of the object point cloud
 * @return :
 */
std::vector<int> nearestMatchingFeature(pcl::PointCloud<pcl::Histogram<153>>::Ptr scene,
                                        pcl::PointCloud<pcl::Histogram<153>>::Ptr object) {
    std::vector<int> result;
    for (unsigned int i = 0; i < object->points.size(); i++) {
        pcl::Histogram<153> object_histogram = object->points[i];
        float min_distance = std::numeric_limits<float>::max();
        int index = 0;
        for (unsigned int j = 0; j < scene->points.size(); j++) {
            pcl::Histogram<153> scene_histogram = scene->points[j];
            float distance = computeL2Dist(object_histogram, scene_histogram);
            if (distance < min_distance) {
                min_distance = distance;
                index = j;
            }
        }
        result.push_back(index);
    }
    return result;
}

/**
 * This function is from exercise 7 in the computer vision course.
 *
 * @brief findAlignment
 * @param scene
 * @param object
 * @param object2scene
 * @return
 */
pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 findAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
                                                                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                                                                                                    std::vector<int> object2scene) {
    pcl::common::UniformGenerator<int> uniform_generator(0, object2scene.size()-1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_transform(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimate_svd;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 best_transform;

    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(scene);

    object_points->width = 3;
    object_points->height = 1;
    object_points->is_dense = false;
    object_points->resize(object_points->height * object_points->width);
    scene_points->width = 3;
    scene_points->height = 1;
    scene_points->is_dense = false;
    scene_points->resize(scene_points->height * scene_points->width);

    //float threshold = std::pow(0.005, 2);
    int max_inliers = 0;
    unsigned int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_dist(k);
    auto time_start = std::chrono::high_resolution_clock::now();
    for (unsigned int i = 0; i < MAX_ITERATIONS; i++) {
        for (unsigned int j = 0; j < 3; j++) {
            int random_int = uniform_generator.run();
            object_points->points[j] = object->points[random_int];
            scene_points->points[j] = scene->points[object2scene[random_int]];
        }
        estimate_svd.estimateRigidTransformation(*object_points, *scene_points, transform);
        pcl::transformPointCloud(*object, *object_transform, transform);
        int inliers = 0;
        for (unsigned int j = 0; j < object_transform->points.size(); j++) {
            tree.nearestKSearch(object_transform->points[j], k, k_indices, k_squared_dist);
            if (k_squared_dist[0] < ALIGNMENT_THRESHOLD) { inliers++; }
        }
        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_transform = transform;
            std::cout << "New model => inlers = " << max_inliers << std::endl;
        }
        if (i % 1000 == 0) { std::cout << i << " / " << MAX_ITERATIONS << std::endl; }
    }
    auto time_end = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::seconds>(time_end - time_start);
    std::cout << "\nExecution time => " << time.count() << " s." << std::endl;
    return best_transform;
}
//---------------------------------------------------------


//---------------------------------------------------------
// Main
int main(int argc, char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

    //---------------------------
    // POINT CLOUD PREPROCESSING
    //---------------------------
    std::cout << "\nPoint cloud preprocessing\n" << std::endl;

    // load the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(SCENE_PATH, *scene);

    // display point cloud info
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*scene, min_pt, max_pt);
    std::cout << "Max x: " << max_pt.x << std::endl;
    std::cout << "Max y: " << max_pt.y << std::endl;
    std::cout << "Max z: " << max_pt.z << std::endl;
    std::cout << "Min x: " << min_pt.x << std::endl;
    std::cout << "Min y: " << min_pt.y << std::endl;
    std::cout << "Min z: " << min_pt.z << std::endl;
    std::cerr << "PointCloud before filtering: "
              << scene->width * scene->height
              << " data points ("
              << pcl::getFieldsList(*scene)
              << ")."
              << std::endl;

    // filter point cloud
    voxelGrid(scene, scene);
    //outlierRemoval(cloud_filtered, cloud_filtered);
    spatialFilter(scene, scene);
    //smoothing(cloud_filtered, cloud_filtered);

    // display filtered cloud info
    std::cerr << "PointCloud after filtering: "
              << scene->width * scene->height
              << " data points ("
              << pcl::getFieldsList(*scene)
              << ")."
              << std::endl;

    // save the new point cloud
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("../cloud_filtered.pcd", *scene, false);

    //---------------------------
    // POSE ESTIMATION 3D TO 3D
    //---------------------------
    std::cout << "\nPose estimation 3D to 3D\n" << std::endl;

    // load the generated object point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(OBJECT_PATH, *object);

    // compute surface normals
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals = estimateSurfaceNormal(scene);
    pcl::PointCloud<pcl::Normal>::Ptr object_normals = estimateSurfaceNormal(object);

    // compute spin images
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spin_images_scene = computeSpinImages(scene, scene_normals);
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spin_images_object = computeSpinImages(object, object_normals);

    // find the nearest match indices
    std::vector<int> object2scene = nearestMatchingFeature(spin_images_scene, spin_images_object);

    // find alignment
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = findAlignment(scene, object, object2scene);
    std::cout << "\nTransform =>\n" << transform << std::endl;

    // show the state of the scene and the object
    pcl::transformPointCloud(*object, *object, transform);
    showPointClouds(scene, object, "Finale state");

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
//---------------------------------------------------------
