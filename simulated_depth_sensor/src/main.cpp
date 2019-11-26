/* RoVi Final Project
 * Simulated Depth Sensor
 */

// INCLUDES
#include <iostream>
#include <chrono>
#include <random>
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
#include <rw/rw.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

//---------------------------------------------------------

// DEFINES
#define SCENE_PATH "../../scanner25D_point_clouds/Scanner25D_"
#define OBJECT_PATH "../../../point_clouds_of_objects/rubber_duck.pcd"
#define WC_FILE "../../workcell/Scene.wc.xml"
#define SCANNER_FRAME "Scanner25D"

//---------------------------------------------------------

// TYPEDEFS
typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudT;
typedef pcl::Histogram<153> HistT;

//---------------------------------------------------------

// GLOBAL VARIABLES
// voxel grid
float LEAF_SIZE = 1e-2;

// outlier removal
float MEAN = 200; // number of neighbors
float STD_DEV = 0.1; // distance between

// spatial filter
float X_MIN = -0.5, X_MAX = 0.5;
float Y_MIN = -0.3, Y_MAX = 0.1625;
float Z_MIN = -1.5, Z_MAX = 0;

// smoothing
unsigned int SMOOTHING_POLY_ORDER = 2;
float SMOOTHING_RADIUS = 0.03;

// cluster extraction
unsigned int CLUSTER_MIN_SIZE = 750; // 750
unsigned int CLUSTER_MAX_SIZE = 1500; // 950
float CLUSTER_RADIUS = 0.01;

// global alignment
//std::size_t GLOBAL_ITERATIONS = 20000;
std::size_t NUMBER_INLIERS = 240; // 215
float SPIN_IMG_RADIUS = 0.05; // see sin_imgs_radius_analysis.m
float GLOBAL_THRESH = 0.0001; //2.5e-5; // see thresh_analysis.m

// local alignment
std::size_t LOCAL_ITERATIONS = 1000;
float LOCAL_THRESH = 0.0001; //2.5e-5; // see thresh_analysis.m

// add guassian noise
float NOISE = 0.01; // see noise_analysis.m

//---------------------------------------------------------

// FUNCTIONS
/** Creates a view with two point clouds
 * @brief showPointClouds
 * @param scene
 * @param object
 * @param view_name
 */
void showPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const std::string &view_name) {
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
    viewer->close();
}

/** Performs voxel grid on input
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/voxel_grid.cpp
 * @brief voxelGrid
 * @param inputCloud
 * @param outputCloud
 * @param leafSize
 */
void voxelGrid(PointCloudT::Ptr inputCloud, PointCloudT::Ptr &outputCloud, const float leafSize = 0.01) {
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*outputCloud);
}

/** Statistical Outlier Removal
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/outlier_removal.cpp
 * @brief outlierRemoval : performs statistical outlier removal of input cloud
 * @param inputCloud : input point cloud
 * @param outputCloud : output point cloud
 */
void outlierRemoval(PointCloudT::Ptr inputCloud, PointCloudT::Ptr &outputCloud) {
    pcl::StatisticalOutlierRemoval<PointT> statsOutlierRemoval;
    statsOutlierRemoval.setInputCloud(inputCloud);
    statsOutlierRemoval.setMeanK(MEAN);
    statsOutlierRemoval.setStddevMulThresh(STD_DEV);
    statsOutlierRemoval.filter(*outputCloud);
}

/** Spatial filters of cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/passthrough_filter.cpp
 * @brief spatialFilter : Performs spatial filtering in x, y and z of the input cloud
 * @param inputCloud : input point cloud
 * @param outputCloud : output point cloud
 */
void spatialFilter(const PointCloudT::Ptr &inputCloud, PointCloudT::Ptr &outputCloud) {
    pcl::PassThrough<PointT> spatialFilter;
    spatialFilter.setInputCloud(inputCloud);
    // Z
    spatialFilter.setFilterFieldName("z");
    spatialFilter.setFilterLimits(Z_MIN, Z_MAX);
    spatialFilter.filter(*outputCloud);
    // X
    spatialFilter.setFilterFieldName("x");
    spatialFilter.setFilterLimits(X_MIN, X_MAX);
    spatialFilter.filter(*outputCloud);
    // Y
    spatialFilter.setFilterFieldName("y");
    spatialFilter.setFilterLimits(Y_MIN, Y_MAX);
    spatialFilter.filter(*outputCloud);
}

/** Smoothing of point cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/mls_smoothing.cpp
 * @brief smoothing : Performs smoothing of input cloud
 * @param input_cloud : input point cloud
 * @param output_cloud : output point cloud
 */
void smoothing(const PointCloudT::Ptr &input_cloud, PointCloudT::Ptr &output_cloud) {
    // create a kd-tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<PointT> mls_points;
    pcl::MovingLeastSquares<PointT, PointT> mls;
    // set parameters
    mls.setInputCloud(input_cloud);
    //mls.setPolynomialOrder(SMOOTHING_POLY_ORDER);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(SMOOTHING_RADIUS);
    // reconstruct
    mls.process(mls_points);
    pcl::copyPointCloud(mls_points, *output_cloud);
}

/** Remove planar point cloud clusters
 * Inspiration --> http://pointclouds.org/documentation/tutorials/planar_segmentation.php
 * @brief planarSegmentation : removes planer point cloud clusters from cloud
 * @param inputCloud : point cloud
 * @return point cloud
 */
PointCloudT::Ptr planarSegmentation(const PointCloudT::Ptr &cloud) {
    std::cout << "Planer segmentaion.." << std::endl;
    PointCloudT::Ptr result(new PointCloudT);
    pcl::copyPointCloud(*cloud, *result);
    int nPoints = (int)result->points.size();
    while (result->points.size() > (0.1 * nPoints)) {
        std::cout << "\tCloud points: " << result->points.size() << std::endl;
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.001);
        seg.setInputCloud(result);
        seg.segment(*inliers, *coeff);
        if (inliers->indices.size() == 0) {
            PCL_ERROR("Cloud not estimate a planar model for the dataset.");
            std::cout << std::endl;
            return result;
        }
    //    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    //    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
    //        std::cerr << "Indices: " << inliers->indices[i]
    //                  << "\tpoint: " << cloud->points[inliers->indices[i]].x << " "
    //                                 << cloud->points[inliers->indices[i]].y << " "
    //                                 << cloud->points[inliers->indices[i]].z << std::endl;
        // extract inliers
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(result);
        extract.setIndices(inliers);
        extract.setNegative(0.005f);
        extract.filter(*result);
    }
    return result;
}

/** Cluster extraction of cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/examples/segmentation/example_extract_clusters_normals.cpp
 * @brief euclideanCusterExtraction : Performs cluster extraction of cloud and remove to big and to small clusters
 * @param cloud : the scene from the Scanner25D
 * @return : a new scene with the cluster which muchly resembles the duck
 */
PointCloudT::Ptr euclideanCusterExtraction(const PointCloudT::Ptr &cloud) {
    std::cout << "Euclidean cluster extraction.." << std::endl;
    PointCloudT::Ptr result(new PointCloudT);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> euclideanCluster;
    euclideanCluster.setClusterTolerance(CLUSTER_RADIUS);
    euclideanCluster.setMinClusterSize(CLUSTER_MIN_SIZE);
    euclideanCluster.setMaxClusterSize(CLUSTER_MAX_SIZE);
    euclideanCluster.setInputCloud(cloud);
    euclideanCluster.extract(clusterIndices);
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++) {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            result->points.push_back(cloud->points[*pit]);
        }
        result->width = result->points.size();
        result->height = 1;
        result->is_dense = true;
        std::cout << "\tCluster information -->" << std::endl;
        std::cerr << "\t" << result->width * result->height
                  << " data points ("
                  << pcl::getFieldsList(*result)
                  << ")."
                  << std::endl;
    }
    return result;
////    pcl::ExtractIndices<PointT> extract;
////    extract.setInputCloud(cloud);
//    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++) {
//        PointCloudT::Ptr cloudCluster(new PointCloudT);
//        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
//            cloudCluster->points.push_back(cloud->points[*pit]);
//        }
//        cloudCluster->width = cloudCluster->points.size();
//        cloudCluster->height = 1;
//        cloudCluster->is_dense = true;
////        // show cluster
////        {
////            pcl::visualization::PCLVisualizer view("cluster");
////            view.addPointCloud<PointT>(cloudCluster, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloudCluster, 0, 255, 0),"cluster");
////            view.spin();
////        }
//        std::cout << "\tCluster information -->" << std::endl;
//        std::cerr << "\t" << cloudCluster->width * cloudCluster->height
//                  << " data points ("
//                  << pcl::getFieldsList(*cloudCluster)
//                  << ")."
//                  << std::endl;
//        if (cloudCluster->points.size() > MIN_CLUSTER_SIZE) { // < 925 && > 750
//            for (std::size_t i = 0; i < cloudCluster->points.size(); i++) {
//                PointT point = cloudCluster->points[i];
//                result->points.push_back(point);
//            }
//            result->width = result->points.size();
//            result->height = 1;
//            result->is_dense = true;
//        }
//    }
//    // return cloud if no cluster extraction was made
//    if (!result->empty()) { return result; }
//    else { return cloud; }
}

/** Calculates the squared distance
 * @brief distSqr : Calculates the square distance between query and target
 * @param query : HistT
 * @param target : HistT
 * @return : square distance between query and target
 */
float distSqr(const HistT &query, const HistT &target) {
    float result = 0.0;
    for (int i = 0; i < pcl::Histogram<153>::descriptorSize(); i++) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    return result;
}

/** Finds nearest feature
 * @brief nearestFeature : find nearest HistT in target of query --> updates index and sqrDist
 * @param query : input HistT
 * @param target : HistTs to search through
 * @param index : the index of the nearest feature
 * @param sqrDist : sqrDist to the nearest feature
 */
void nearestFeature(const HistT &query, const pcl::PointCloud<HistT> &target, int &index, float &sqrDist) {
    index = 0;
    sqrDist = distSqr(query, target[0]);
    for (size_t i = 0; i < target.size(); i++) {
        const float dist = distSqr(query, target[i]);
        if (dist < sqrDist) {
            index = i;
            sqrDist = dist;
        }
    }
}

/** Estimates surface normals
 * This function is from exercise 6 in the computer vision course.
 * @brief estimateSurfaceNormal : Compute the surface normals using k=10 nearest neighbors around each point
 * @param cloud : input point cloud
 * @return point cloud with estimated surface normals
 */
PointCloudT::Ptr estimateSurfaceNormal(PointCloudT::Ptr cloud) {
    // create normal estimation
    pcl::NormalEstimation<PointT, PointT> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    // creat KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    normal_estimator.setSearchMethod(tree);
    // create result
    PointCloudT::Ptr result(new PointCloudT());
    // set search size and compute
    normal_estimator.setKSearch(10);
    normal_estimator.compute(*result);
    return result;
}

/** Computes spin images
 * This function is from exercise 6 in the computer vision course.
 * @brief computeSpinImages : Compute local spin image features surface description.
 * @param cloud : point cloud
 * @param normals : surface normals of the point cloud
 * @return histograms
 */
pcl::PointCloud<HistT>::Ptr computeSpinImages(PointCloudT::Ptr cloud, PointCloudT::Ptr normals) {
    // setup spin image computation
    pcl::SpinImageEstimation<PointT, PointT, HistT> spin_image_descriptor(8, 0.5, 4);
    spin_image_descriptor.setInputCloud(cloud);
    spin_image_descriptor.setInputNormals(normals);
    // create KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    spin_image_descriptor.setSearchMethod(tree);
    // Calculate spin images
    pcl::PointCloud<HistT>::Ptr spin_images(new pcl::PointCloud<HistT>());
    spin_image_descriptor.setRadiusSearch(SPIN_IMG_RADIUS);
    // compute the spin images
    spin_image_descriptor.compute(*spin_images);
    return spin_images;
}

/** Compute L2 distance
 * This functions is form exercise 6 in the computer vision course
 * @brief calcL2Dist : compute the l2 distance between the two histograms
 * @param object : histogram of the object
 * @param scene : histogram of the scene
 * @return : l2 distance
 */
float computeL2Dist(const HistT &object, const HistT &scene) {
    float result = 0.0;
    for (int i = 0; i < scene.descriptorSize(); i++) {
        result += std::sqrt(std::pow((scene.histogram[i] - object.histogram[i]), 2));
    }
    return result;
}

/** Finds the nearest matching features
 * This function is from exercise 6 in the computer vision course.
 * @brief nearestMatchingFeature : find the nearest matching (k=1) scene feature for each object feature
 * @param scene : spin images of the scene point cloud
 * @param object : spin images of the object point cloud
 * @return indeces
 */
std::vector<int> nearestMatchingFeature(pcl::PointCloud<HistT>::Ptr scene, pcl::PointCloud<HistT>::Ptr object) {
    std::vector<int> result;
    for (size_t i = 0; i < object->points.size(); i++) {
        HistT object_histogram = object->points[i];
        float min_distance = std::numeric_limits<float>::max();
        int index = 0;
        for (size_t j = 0; j < scene->points.size(); j++) {
            HistT scene_histogram = scene->points[j];
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

/** Find global alignment
 * This function is from exercise 6 in the computer vision course.
 * @brief findAlignment
 * @param scene
 * @param object
 * @return : final transform
 */
Eigen::Matrix4f findGlobalAlignment(PointCloudT::Ptr scene, PointCloudT::Ptr object) {
    // compute surface normals
    std::cout << "\tcomputing surface normals.." << std::endl;
    {
        pcl::NormalEstimation<PointT, PointT> normalEsitmator;
        normalEsitmator.setKSearch(10);
        // compute for object
        normalEsitmator.setInputCloud(object);
        normalEsitmator.compute(*object);
        // compute for scene
        normalEsitmator.setInputCloud(scene);
        normalEsitmator.compute(*scene);
    }

    // compute spin images
    std::cout << "\tcomputing spin images.." << std::endl;
    pcl::PointCloud<HistT>::Ptr sceneFeatures(new pcl::PointCloud<HistT>());
    pcl::PointCloud<HistT>::Ptr objectFeatures(new pcl::PointCloud<HistT>());
    {
        pcl::SpinImageEstimation<PointT, PointT, HistT> spinEstimator(8, 0.5, 0);
        spinEstimator.setRadiusSearch(SPIN_IMG_RADIUS);
        // object
        spinEstimator.setInputCloud(object);
        spinEstimator.setInputNormals(object);
        spinEstimator.compute(*objectFeatures);
        // scene
        spinEstimator.setInputCloud(scene);
        spinEstimator.setInputNormals(scene);
        spinEstimator.compute(*sceneFeatures);
    }
    // display spin images features
    //std::cout << "\tFirst spin image feature of object-->\n" << objectFeatures->points[0] << std::endl;
    //std::cout << "\tFirst spin image feature of scene-->\n" << sceneFeatures->points[0] << std::endl;

    // find feature matches
    std::cout << "\tfinding feature matches.." << std::endl;
    pcl::Correspondences correspondences(objectFeatures->size());
    {
        for (size_t i = 0; i < objectFeatures->size(); i++) {
            correspondences[i].index_query = i;
            nearestFeature(objectFeatures->points[i], *sceneFeatures, correspondences[i].index_match, correspondences[i].distance);
        }
    }

//    // show matches
//    {
//        pcl::visualization::PCLVisualizer view("Scene before preprocessing");
//        view.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
//        view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
//        view.addCorrespondences<PointT>(object, scene, correspondences, 1);
//        view.spin();
//    }

    // create a kd-tree for scene
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(scene);

    // start RANSAC
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloudT::Ptr objectAligned(new PointCloudT());
    float penalty = FLT_MAX;
    {
        std::cout << "\tStarting RANSAC.." << std::endl;
        auto timeStart = std::chrono::high_resolution_clock::now();
        pcl::common::UniformGenerator<int> gen(0, correspondences.size()-1);
        //for (size_t i = 0; i < GLOBAL_ITERATIONS; i++) {
            //if ((i +1) % 250 == 0) { std::cout << "\t" << i+1 << " / " << GLOBAL_ITERATIONS << std::endl; }
        while (true) {
            // sample 3 random correspondeces
            std::vector<int> idxObject(3), idxScene(3);
            for (int j = 0; j < 3; j++) {
                const int idx = gen.run();
                idxObject[j] = correspondences[idx].index_query;
                idxScene[j] = correspondences[idx].index_match;
            }
            // esitmate transformation
            Eigen::Matrix4f T;
            pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
            svd.estimateRigidTransformation(*object, idxObject, *scene, idxScene, T);
            // apply pose
            pcl::transformPointCloud(*object, *objectAligned, T);
            // validate by computing inliers and RMSE
            std::vector<std::vector<int>> idx;
            std::vector<std::vector<float>> sqrDist;
            tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, idx, sqrDist);
            std::size_t inliers = 0;
            float rmse = 0.0;
            for (std::size_t j = 0; j < sqrDist.size(); j++) {
                if (sqrDist[j][0] <= GLOBAL_THRESH) {
                    inliers++;
                    rmse += sqrDist[j][0];
                }
            }
            rmse = std::sqrt(rmse / inliers);
            // evaluate penalty function
            const float outlierRate = 1.0f - float(inliers) / object->size();
            if (outlierRate < penalty) {
                std::cout << "\t--> Got a new model with " << inliers << " inliers!" << std::endl;
                penalty = outlierRate;
                result = T;
                // stopping criteria
                if (inliers >= NUMBER_INLIERS) { break; }

//                // show matches
//                {
//                    pcl::visualization::PCLVisualizer view("Global alignment new model");
//                    view.addPointCloud<PointT>(objectAligned,pcl::visualization::PointCloudColorHandlerCustom<PointT>(objectAligned,255,0,0),"object");
//                    view.addPointCloud<PointT>(scene,pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene,0,255,0),"scene");
//                    view.spin();
//                }
            }
        }
        // compute inliers and RMSE
        std::vector<std::vector<int>> idx;
        std::vector<std::vector<float>> sqrDist;
        tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, idx, sqrDist);
        std::size_t inliers = 0;
        float rmse = 0.0;
        for (std::size_t j = 0; j < sqrDist.size(); j++) {
            if (sqrDist[j][0] <= GLOBAL_THRESH) {
                inliers++;
                rmse += sqrDist[j][0];
            }
        }
        // print timing
        auto timeEnd = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::seconds>(timeEnd - timeStart);
        std::cout << "\tExecution time for global alignment --> " << time.count() << " s" << std::endl;
        // print pose
        std::cout << "\tGot the following pose:\n" << result << std::endl;
        std::cout << "\tInliers: " << inliers << std::endl;
        std::cout << "\tRMSE: " << rmse << std::endl;
        return result;
    }
}

/** Find local alignment
 * This function is from exercise 6 in the computer vision course.
 * @brief findLocalAlignment : implements ICP for bringing the models into accurate alignment
 * @param scene : scene point cloud
 * @param object : object point cloud
 * @return final transform
 */
Eigen::Matrix4f findLocalAlignment(const PointCloudT::Ptr &scene, const PointCloudT::Ptr &object) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(scene);
    PointCloudT::Ptr objectAligned(new PointCloudT(*object));
    std::cout << "\tStarting ICP.." << std::endl;
    auto timeStart = std::chrono::high_resolution_clock::now();
    {
        for (std::size_t i = 0; i < LOCAL_ITERATIONS; i++) {
            if ((i +1) % 250 == 0) { std::cout << "\t" << i+1 << " / " << LOCAL_ITERATIONS << std::endl; }
            // find the closest points
            std::vector<std::vector<int>> indexs;
            std::vector<std::vector<float>> sqrDists;
            tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, indexs, sqrDists);
            // Threshold and create indices for object/scene and compute RMSE
            std::vector<int> idxObject, idxScene;
            for (std::size_t j = 0; j < indexs.size(); j++) {
                if (sqrDists[j][0] <= LOCAL_THRESH) {
                    idxObject.push_back(j);
                    idxScene.push_back(indexs[j][0]);
                }
            }
            // estimate transformation
            Eigen::Matrix4f T;
            pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
            svd.estimateRigidTransformation(*objectAligned, idxObject, *scene, idxScene, T); // Kabsch algorithm
            // apply pose
            pcl::transformPointCloud(*objectAligned, *objectAligned, T);
//            // show matches
//            if ((i+1) % 1000 == 0) {
//                pcl::visualization::PCLVisualizer view("ICP object and mode");
//                view.addPointCloud<PointT>(objectAligned, pcl::visualization::PointCloudColorHandlerCustom<PointT>(objectAligned, 255, 0, 0),"object");
//                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
//                view.spin();
//            }
            // update result
            result = result * T;
        }
    }
    // compute inliers and RMSE
    std::vector<std::vector<int>> indexs;
    std::vector<std::vector<float>> sqrDists;
    tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, indexs, sqrDists);
    std::size_t inliers = 0;
    float rmse = 0.0;
    for (size_t i = 0; i < sqrDists.size(); i++) {
        if (sqrDists[i][0] <= LOCAL_THRESH) {
            ++inliers;
            rmse += sqrDists[i][0];
        }
    }
    rmse = std::sqrt(rmse / inliers);
    // print timing
    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::seconds>(timeEnd - timeStart);
    std::cout << "\n\tExecution time for local alignment --> " << time.count() << " s" << std::endl;
    // print pose
    std::cout << "\tGot the following pose:\n" << result << std::endl;
    std::cout << "\tInliers: " << inliers << std::endl;
    std::cout << "\tRMSE: " << rmse << std::endl;
    return result;
}

/** Compute Iterative Closest Point
 * from: https://github.com/Masle16/pcl/blob/master/tools/iterative_closest_point.cpp
 * @brief computeICP : Computes the ICP from target to source
 * @param target : target point cloud
 * @param source : source point cloud
 * @return : Eigen::Matrix4f transform to get target into source
 */
Eigen::Matrix4f computeICP(const PointCloudT::Ptr &target, const PointCloudT::Ptr &source) {
    PointCloudT::Ptr src = source, tgt = target;

    std::cerr << "\tComputing ICP.." << std::endl;

    pcl::registration::TransformationEstimationLM<PointT, PointT, float>::Ptr transEsti(new pcl::registration::TransformationEstimationLM<PointT, PointT, float>);
    pcl::registration::CorrespondenceEstimation<PointT, PointT, float>::Ptr corEsti(new pcl::registration::CorrespondenceEstimation<PointT, PointT, float>);
    corEsti->setInputSource(src);
    corEsti->setInputTarget(tgt);

    pcl::registration::CorrespondenceRejectorOneToOne::Ptr corRejOne2One(new pcl::registration::CorrespondenceRejectorOneToOne);

    pcl::registration::CorrespondenceRejectorMedianDistance::Ptr corRejMed(new pcl::registration::CorrespondenceRejectorMedianDistance);
    corRejMed->setInputSource<PointT>(src);
    corRejMed->setInputTarget<PointT>(tgt);

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr corRejSac(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>);
    corRejSac->setInputSource(src);
    corRejSac->setInputTarget(tgt);
    corRejSac->setInlierThreshold(0.005);
    corRejSac->setMaximumIterations(10000);

    pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr corRejVar(new pcl::registration::CorrespondenceRejectorVarTrimmed);
    corRejVar->setInputSource<PointT>(src);
    corRejVar->setInputTarget<PointT>(tgt);

    pcl::registration::CorrespondenceRejectorTrimmed::Ptr corRejTrim(new pcl::registration::CorrespondenceRejectorTrimmed);

    pcl::IterativeClosestPoint<PointT, PointT, float> icp;
    icp.setCorrespondenceEstimation(corEsti);
    icp.setTransformationEstimation(transEsti);
    icp.addCorrespondenceRejector(corRejOne2One);
    icp.addCorrespondenceRejector(corRejMed);
    icp.addCorrespondenceRejector(corRejSac);
    icp.addCorrespondenceRejector(corRejVar);
    icp.addCorrespondenceRejector(corRejTrim);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-5);
    PointCloudT output;
    icp.align(output);
    std::cerr << "\t" << icp.getFitnessScore() << std::endl;
    return icp.getFinalTransformation();
}

/** Returns transform
 * @brief getTransform : Gets the transform of a frame from the workcell
 * @param frameName : name of the wanted frame
 * @return : rw::math::Transform3D of the frame
 */
rw::math::Transform3D<> getTransform(const std::string &frameName) {
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::Frame *objectFrame = wc->findFrame(frameName);
    if (objectFrame == NULL) {
        std::cerr << frameName << " not found!" << std::endl;
        return rw::math::Transform3D<>();
    }
    return objectFrame->getTransform(state);
}

/** Transform3D --> Matrix4f
 * @brief transfrom2matrix4f : Converts a Transform3D to Matrix4f
 * @param objectT : rw::math::Transform3D
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f transfrom2matrix4f(rw::math::Transform3D<> objectT) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    rw::math::Vector3D<> position = objectT.P();
    rw::math::Rotation3D<> rotation = objectT.R();
    // convert to matrix4
    result(0,0) = (float)rotation(0,0); result(0,1) = (float)rotation(0,1); result(0,2) = (float)rotation(0,2); result(0,3) = (float)position(0);
    result(1,0) = (float)rotation(1,0); result(1,1) = (float)rotation(1,1); result(1,2) = (float)rotation(1,2); result(1,3) = (float)position(1);
    result(2,0) = (float)rotation(0,2); result(2,1) = (float)rotation(2,1); result(2,2) = (float)rotation(2,2); result(2,3) = (float)position(2);
    return result;
}

/** Matrix4f --> Transform3D
 * @brief matrix4f2transform : Converts a Matrix4f to Transform3D
 * @param matrix : Eigen::Matrix4f
 * @return : rw::math::Transform3D<>
 */
rw::math::Transform3D<> matrix4f2transform(const Eigen::Matrix4f matrix) {
    rw::math::Vector3D<> position;
    position(0) = matrix(0,3); position(1) = matrix(1,3); position(2) = matrix(2,3);
    rw::math::Rotation3D<> rotation;
    rotation(0,0) = matrix(0,0); rotation(0,1) = matrix(0,1); rotation(0,2) = matrix(0,2);
    rotation(1,0) = matrix(1,0); rotation(1,1) = matrix(1,1); rotation(1,2) = matrix(1,2);
    rotation(2,0) = matrix(2,0); rotation(2,1) = matrix(2,1); rotation(2,2) = matrix(2,2);
    return rw::math::Transform3D<>(position, rotation);
}

/** Saves data to ../../data/noise.txt
 * @brief noiseData2File : saves global poses, local poses, executiom times and noise
 * @param globalPoses : poses after global alignment
 * @param localPoses : poses after local alignment
 * @param times : the full execution time of the process
 * @param noises : noises applied to the input cloud
 */
void noiseData2File(const std::vector<rw::math::Transform3D<>>& globalPoses,
                    const std::vector<rw::math::Transform3D<>>& localPoses,
                    const std::vector<std::chrono::seconds> &times,
                    const std::vector<float> &noises) {
    std::cout << "Writing data to file noise.txt" << std::endl;
    std::ofstream file;

    // pose estimation
    file.open("../../data/noise.txt");
    for (std::size_t i = 0; i < times.size(); i++) {
        // Global
        rw::math::Vector3D<> posGlobal = globalPoses[i].P();
        rw::math::RPY<> rpyGlobal = rw::math::RPY<>(globalPoses[i].R());
        // Local
        rw::math::Vector3D<> posLocal = localPoses[i].P();
        rw::math::RPY<> rpyLocal = rw::math::RPY<>(localPoses[i].R());
        file << noises[i]       << " " // noise
             << times[i].count()<< " " // time
             << posGlobal(0)    << " " << posGlobal(1)  << " " << posGlobal(2)  << " "   // pos global
             << rpyGlobal(0)    << " " << rpyGlobal(1)  << " " << rpyGlobal(2)  << " "   // rpy global
             << posLocal(0)     << " " << posLocal(1)   << " " << posLocal(2)   << " "   // pos local
             << rpyLocal(0)     << " " << rpyLocal(1)   << " " << rpyLocal(2)   << "\n"; // rpy local
    }
    file.close();
}

/** Saves transforms to ../../data/transforms.txt
 * @brief writeTransforms2File : saves table transform, world transform and scanner transform to file
 * @param tableT : table transform
 * @param worldT : world transform
 * @param scannerT : scanner transform
 */
void writeTransforms2File(const rw::math::Transform3D<>& tableT,
                          const rw::math::Transform3D<>& worldT,
                          const rw::math::Transform3D<>& scannerT) {
    rw::math::Vector3D<> tablePos = tableT.P(), worldPos = worldT.P(), scannerPos = scannerT.P();
    rw::math::RPY<> tableRPY = rw::math::RPY<>(tableT.R()), worldRPY = rw::math::RPY<>(worldT.R()), scannerRPY = rw::math::RPY<>(scannerT.R());
    std::ofstream file;
    file.open("../../data/transforms.txt");
    file << tablePos(0)     << " " << tablePos(1)   << " " << tablePos(2)   << " "
         << tableRPY(0)     << " " << tableRPY(1)   << " " << tableRPY(2)   << "\n"
         << worldPos(0)     << " " << worldPos(1)   << " " << worldPos(2)   << " "
         << worldRPY(0)     << " " << worldRPY(1)   << " " << worldRPY(2)   << "\n"
         << scannerPos(0)   << " " << scannerPos(1) << " " << scannerPos(2) << " "
         << scannerRPY(0)   << " " << scannerRPY(1) << " " << scannerRPY(2) << "\n";
    file.close();
}

/** Saves two point cloud in one pcd file
 * @brief saveSceneWithObject : merged two point cloud and saves a pcd file
 * @param object : point cloud
 * @param scene : point cloud
 * @param fileName : path and name of the decired save location
 */
void saveSceneWithObject(const PointCloudT::Ptr &object, const PointCloudT::Ptr &scene, const std::string &fileName) {
    std::cout << "Saving view to --> " << fileName << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

    // load object --> cloud
    std::uint8_t r = 255, g = 0, b = 0; // Red
    std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    for (std::size_t i = 0; i < object->points.size(); i++) {
        pcl::PointXYZRGB point;
        point.x = object->points[i].x;
        point.y = object->points[i].y;
        point.z = object->points[i].z;
        point.rgb = *reinterpret_cast<float*>(&rgb);
        result->points.push_back(point);
    }

    // load scene --> cloud
    r = 0, g = 255, b = 0; // Green
    rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    for (std::size_t i = 0; i < scene->points.size(); i++) {
        pcl::PointXYZRGB point;
        point.x = scene->points[i].x;
        point.y = scene->points[i].y;
        point.z = scene->points[i].z;
        point.rgb = *reinterpret_cast<float*>(&rgb);
        result->points.push_back(point);
    }

    result->width = result->points.size();
    result->height = 1;
    result->is_dense = true;

    // save pcd  file
    pcl::io::savePCDFile(fileName, *result);
}

/** Returns input cloud with gaussian noise
 * From: https://github.com/Masle16/pcl/blob/master/tools/add_gaussian_noise.cpp
 * @brief addGaussianNoise : copies input cloud and applies gaussian noise to new cloud
 * @param input : the cloud to copy
 * @param stdDev : standard deviation of the gaussian distribution
 * @return : a copy of input with noise applied
 */
PointCloudT::Ptr addGaussianNoise(const PointCloudT::Ptr &input, const float stdDev = 1e-4f) {
    std::cout << "Adding Gaussian noise with mean 0.0 and standard deviation " << stdDev << std::endl;
    PointCloudT::Ptr result(new PointCloudT);
    result->points.resize(input->points.size());
    result->header = input->header;
    result->width = input->width;
    result->height = input->height;

    std::random_device ranDevice;
    std::mt19937 ranNumGen(ranDevice());
    std::normal_distribution<float> normDist(0.0f, stdDev);

    for (std::size_t i = 0; i < input->points.size(); i++) {
        result->points[i].x = input->points[i].x + normDist(ranNumGen);
        result->points[i].y = input->points[i].y + normDist(ranNumGen);
        result->points[i].z = input->points[i].z + normDist(ranNumGen);
    }

    return result;
}

/** Saves data to ../../data/thresh.txt
 * @brief threshData2File : saves global poses, local poses, alignment threshold and executiom times
 * @param globalPoses : poses after global alignment
 * @param localPoses : poses after local alignment
 * @param threshs : alignment threshold for global and local alignment
 * @param times : execution time for the full process
 */
void threshData2File(const std::vector<rw::math::Transform3D<>> &globalPoses,
                     const std::vector<rw::math::Transform3D<>> &localPoses,
                     const std::vector<float> &threshs,
                     const std::vector<std::chrono::seconds> &times) {
    std::cout << "Writing data to file thresh.txt " << std::endl;
    std::ofstream file;

    // pose estimation
    file.open("../../data/thresh.txt");
    for (std::size_t i = 0; i < times.size(); i++) {
        // Global
        rw::math::Vector3D<> posGlobal = globalPoses[i].P();
        rw::math::RPY<> rpyGlobal = rw::math::RPY<>(globalPoses[i].R());
        // Local
        rw::math::Vector3D<> posLocal = localPoses[i].P();
        rw::math::RPY<> rpyLocal = rw::math::RPY<>(localPoses[i].R());
        file << threshs[i]       << " " // thresh
             << times[i].count() << " " // time
             << posGlobal(0)     << " " << posGlobal(1) << " " << posGlobal(2) << " "   // pos global
             << rpyGlobal(0)     << " " << rpyGlobal(1) << " " << rpyGlobal(2) << " "   // rpy global
             << posLocal(0)      << " " << posLocal(1)  << " " << posLocal(2)  << " "   // pos local
             << rpyLocal(0)      << " " << rpyLocal(1)  << " " << rpyLocal(2)  << "\n"; // rpy local
    }
    file.close();
}

/** Saves data to ../../data/spin_imgs_radis.txt
 * @brief radiusData2File : saves global poses, local poses, spin images radis and execution time
 * @param globalPoses : poses after global alignment
 * @param localPoses : poses after local alignment
 * @param radis : radius of which the spin images are created
 * @param times : execution time for the full process
 */
void radiusData2File(const std::vector<rw::math::Transform3D<>> &globalPoses,
                     const std::vector<rw::math::Transform3D<>> &localPoses,
                     const std::vector<float> &radis,
                     const std::vector<std::chrono::seconds> &times) {
    std::cout << "Writing data to file spin_imgs_radis.txt " << std::endl;
    std::ofstream file;

    // pose estimation
    file.open("../../data/spin_imgs_radis.txt");
    for (std::size_t i = 0; i < times.size(); i++) {
        // Global
        rw::math::Vector3D<> posGlobal = globalPoses[i].P();
        rw::math::RPY<> rpyGlobal = rw::math::RPY<>(globalPoses[i].R());
        // Local
        rw::math::Vector3D<> posLocal = localPoses[i].P();
        rw::math::RPY<> rpyLocal = rw::math::RPY<>(localPoses[i].R());
        file << radis[i]         << " " // radis
             << times[i].count() << " " // time
             << posGlobal(0)     << " " << posGlobal(1) << " " << posGlobal(2) << " "   // pos global
             << rpyGlobal(0)     << " " << rpyGlobal(1) << " " << rpyGlobal(2) << " "   // rpy global
             << posLocal(0)      << " " << posLocal(1)  << " " << posLocal(2)  << " "   // pos local
             << rpyLocal(0)      << " " << rpyLocal(1)  << " " << rpyLocal(2)  << "\n"; // rpy local
    }
    file.close();
}

/** Saves data to ../../data/iterations.txt
 * @brief iterationData2File : saves global poses, local poses, number of iterations and execution times
 * @param globalPoses : poses after global alignment
 * @param localPoses : poses after local alignment
 * @param iterations : number of iterations for both global anad local alignment
 * @param times : execution times for the full process
 */
void iterationData2File(const std::vector<rw::math::Transform3D<>> &globalPoses,
                        const std::vector<rw::math::Transform3D<>> &localPoses,
                        const std::vector<std::size_t> &iterations,
                        const std::vector<std::chrono::seconds> &times) {
    std::cout << "Writing data to file iterations.txt " << std::endl;
    std::ofstream file;

    // pose estimation
    file.open("../../data/iterations.txt");
    for (std::size_t i = 0; i < times.size(); i++) {
        // Global
        rw::math::Vector3D<> posGlobal = globalPoses[i].P();
        rw::math::RPY<> rpyGlobal = rw::math::RPY<>(globalPoses[i].R());
        // Local
        rw::math::Vector3D<> posLocal = localPoses[i].P();
        rw::math::RPY<> rpyLocal = rw::math::RPY<>(localPoses[i].R());
        file << iterations[i]    << " " // iterations
             << times[i].count() << " " // time
             << posGlobal(0)     << " " << posGlobal(1) << " " << posGlobal(2) << " "   // pos global
             << rpyGlobal(0)     << " " << rpyGlobal(1) << " " << rpyGlobal(2) << " "   // rpy global
             << posLocal(0)      << " " << posLocal(1)  << " " << posLocal(2)  << " "   // pos local
             << rpyLocal(0)      << " " << rpyLocal(1)  << " " << rpyLocal(2)  << "\n"; // rpy local
    }
    file.close();
}

/** Saves data ../../data/data.txt
 * @brief data2File : saves global poses, local poses and execution times to data.txt
 * @param globalPoses : poses after global alignment
 * @param localPoses : poses after local alignment
 * @param times : execution time of the full process (filtering, global alignment and local alignment)
 */
void data2File(const std::vector<rw::math::Transform3D<>> &globalPoses,
               const std::vector<rw::math::Transform3D<>> &localPoses,
               const std::vector<std::chrono::seconds> &times) {
    std::cout << "Writing data to file data.txt " << std::endl;
    std::ofstream file;

    // pose estimation
    file.open("../../data/data.txt");
    for (std::size_t i = 0; i < times.size(); i++) {
        // Global
        rw::math::Vector3D<> posGlobal = globalPoses[i].P();
        rw::math::RPY<> rpyGlobal = rw::math::RPY<>(globalPoses[i].R());
        // Local
        rw::math::Vector3D<> posLocal = localPoses[i].P();
        rw::math::RPY<> rpyLocal = rw::math::RPY<>(localPoses[i].R());
        file << times[i].count() << " " // time
             << posGlobal(0)     << " " << posGlobal(1) << " " << posGlobal(2) << " "   // pos global
             << rpyGlobal(0)     << " " << rpyGlobal(1) << " " << rpyGlobal(2) << " "   // rpy global
             << posLocal(0)      << " " << posLocal(1)  << " " << posLocal(2)  << " "   // pos local
             << rpyLocal(0)      << " " << rpyLocal(1)  << " " << rpyLocal(2)  << "\n"; // rpy local
    }
    file.close();
}

//---------------------------------------------------------

/** Main entry point
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

    // data to save for every analysis
    std::vector<rw::math::Transform3D<>> globalPoses, localPoses;
    std::vector<std::chrono::seconds> times;

    //---------------------------
    // SPIN IMAGES RADIUS
    //---------------------------
//    std::vector<float> radis { 0.1, 0.05, 0.01, 0.005, 0.001 };
//    std::vector<float> radiusData;
//    for (std::size_t idx = 0; idx < radis.size(); idx++) {
//        SPIN_IMG_RADIUS = radis[idx];
//        std::cout << "\n------------------------------------" << std::endl;
//        std::cout << "SPIN IMAGES RADIUS: " << SPIN_IMG_RADIUS << std::endl;
//        std::cout << "------------------------------------\n" << std::endl;

    //---------------------------
    // ALIGNMENT THRESH ANALYSIS
    //---------------------------
//    std::vector<float> threshs = { 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8 }; // thresh.txt
//    std::vector<float> threshData;
//    for (std::size_t idx = 0; idx < threshs.size(); idx++) {
//        GLOBAL_THRESH = threshs[idx];
//        LOCAL_THRESH = threshs[idx];
//        std::cout << "\n-------------------------------------" << std::endl;
//        std::cout << "Analysing for thresh --> " << GLOBAL_THRESH << " " << LOCAL_THRESH << std::endl;
//        std::cout << "-------------------------------------\n" << std::endl;

    //---------------------------
    // MAX ITERATIONS ANALYSIS
    //---------------------------
//    std::vector<std::size_t> iterations { 1000, 2500, 5000, 7500, 10000 };
//    std::vector<std::size_t> iterationData;
//    for (std::size_t idx = 0; idx < iterations.size(); idx++) {
//        MAX_ITERATIONS = iterations[idx];
//        std::cout << "\n------------------------------------" << std::endl;
//        std::cout << "MAX ITERATIONS ANALYSIS: " << MAX_ITERATIONS << std::endl;
//        std::cout << "------------------------------------\n" << std::endl;

    //---------------------------
    // NOISE ANALYSIS
    //---------------------------
//    std::vector<float> noises { 1e-5, 1e-4, 1e-3, 1e-2, 1e-1 };
//    std::vector<float> noiseData;
//    for (std::size_t idx = 0; idx < noises.size(); idx++) {
//        NOISE = noises[idx]
//        std::cout << "\n------------------------------------" << std::endl;
//        std::cout << "NOISE ANALYSIS: " << NOISE << std::endl;
//        std::cout << "------------------------------------\n" << std::endl;

        for (std::size_t i = 0; i < 30; i++) {

            // time start of method
            auto timeStart = std::chrono::high_resolution_clock::now();

            // load the point cloud
            PointCloudT::Ptr scene(new PointCloudT), origin(new PointCloudT);
            const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
            //const std::string path = "../../scanner25D_point_clouds/Scanner25D_25.pcd";
            pcl::io::loadPCDFile(path, *scene);

            std::cout << "Processing file: " << path << " number " << i << " / 30" << std::endl;

            // add noise to Scanner25D point cloud
            scene = addGaussianNoise(scene, NOISE); // noise analysis
            // show noise
            {
                pcl::visualization::PCLVisualizer view("Initial scene");
                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
                view.spin();
            }

            //---------------------------
            // POINT CLOUD PREPROCESSING
            //---------------------------
            std::cout << "Point cloud preprocessing" << std::endl;

            // display point cloud info
            std::cout << "Point cloud before preprocessing => " << std::endl;
            PointT min_pt, max_pt;
            pcl::getMinMax3D(*scene, min_pt, max_pt);
            std::cout << "Max x: " << max_pt.x << std::endl;
            std::cout << "Max y: " << max_pt.y << std::endl;
            std::cout << "Max z: " << max_pt.z << std::endl;
            std::cout << "Min x: " << min_pt.x << std::endl;
            std::cout << "Min y: " << min_pt.y << std::endl;
            std::cout << "Min z: " << min_pt.z << std::endl;
            std::cerr << scene->width * scene->height
                      << " data points ("
                      << pcl::getFieldsList(*scene)
                      << ")."
                      << std::endl;

            spatialFilter(scene, scene);
            std::cerr << "PointCloud after spatial filter: "
                      << scene->width * scene->height
                      << " data points ("
                      << pcl::getFieldsList(*scene)
                      << ")."
                      << std::endl;
            {
                pcl::visualization::PCLVisualizer view("Scene after spatial filter");
                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
                view.spin();
            }

//            // filter point cloud
//            voxelGrid(scene, scene, 0.01);
//            std::cerr << "PointCloud after voxel grid: "
//                      << scene->width * scene->height
//                      << " data points ("
//                      << pcl::getFieldsList(*scene)
//                      << ")."
//                      << std::endl;

            smoothing(scene, scene);
            std::cerr << "PointCloud after smoothing: "
                      << scene->width * scene->height
                      << " data points ("
                      << pcl::getFieldsList(*scene)
                      << ")."
                      << std::endl;
            {
                pcl::visualization::PCLVisualizer view("Scene after smoothing");
                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
                view.spin();
            }

            // remove plans in the scene
            scene = planarSegmentation(scene);
            std::cerr << "PointCloud after planar segmentation: "
                      << scene->width * scene->height
                      << " data points ("
                      << pcl::getFieldsList(*scene)
                      << ")."
                      << std::endl;
            {
                pcl::visualization::PCLVisualizer view("Scene after planer segmentation");
                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
                view.spin();
            }

            outlierRemoval(scene, scene);
            std::cerr << "PointCloud after outlier removal filter: "
                      << scene->width * scene->height
                      << " data points ("
                      << pcl::getFieldsList(*scene)
                      << ")."
                      << std::endl;
            {
                pcl::visualization::PCLVisualizer view("Scene after outlier removal");
                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
                view.spin();
            }

            scene = euclideanCusterExtraction(scene);
            std::cerr << "PointCloud after cluster extration: "
                      << scene->width * scene->height
                      << " data points ("
                      << pcl::getFieldsList(*scene)
                      << ")."
                      << std::endl;
            {
                pcl::visualization::PCLVisualizer view("Scene after cluster extraction");
                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
                view.spin();
            }

            // load the generated object point cloud
            PointCloudT::Ptr object(new PointCloudT);
            pcl::io::loadPCDFile(OBJECT_PATH, *object);

            // display point cloud info
            std::cout << "Object point cloud before preprocessing => " << std::endl;
            PointT minPt, maxPt;
            pcl::getMinMax3D(*object, minPt, maxPt);
            std::cout << "Max x: " << maxPt.x << std::endl;
            std::cout << "Max y: " << maxPt.y << std::endl;
            std::cout << "Max z: " << maxPt.z << std::endl;
            std::cout << "Min x: " << minPt.x << std::endl;
            std::cout << "Min y: " << minPt.y << std::endl;
            std::cout << "Min z: " << minPt.z << std::endl;
            std::cerr << object->width * object->height
                      << " data points ("
                      << pcl::getFieldsList(*object)
                      << ")."
                      << std::endl;

            // filter object point cloud
            voxelGrid(object, object, 0.01); // 0.005
            std::cerr << "object point cloud after voxel grid: "
                      << object->width * object->height
                      << " data points ("
                      << pcl::getFieldsList(*object)
                      << ")."
                      << std::endl;

//            // show intial state of scene and object
//            {
//                pcl::visualization::PCLVisualizer view("Scene before preprocessing");
//                view.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
//                view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
//                view.spin();
//            }

            //---------------------------
            // POSE ESTIMATION 3D TO 3D
            //---------------------------
            std::cout << "Pose estimation 3D to 3D" << std::endl;

            //---------------------------
            // GLOBAL ALIGNMENT
            //---------------------------
            std::cout << "Global alignment.." << std::endl;

            // find alignment
            Eigen::Matrix4f poseGlobal = findGlobalAlignment(scene, object);
            pcl::transformPointCloud(*object, *object, poseGlobal);

//    //        // show the state of the scene and the object
//    //        {
//    //            pcl::visualization::PCLVisualizer view("After global alignment");
//    //            view.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
//    //            view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
//    //            view.spin();
//    //        }

            //-----------------
            // LOCAL ALIGNMENT
            //-----------------
            std::cout << "Local alignment.." << std::endl;
            Eigen::Matrix4f poseLocal = findLocalAlignment(scene, object);
            //Eigen::Matrix4f poseLocal = computeICP(object, scene);
            pcl::transformPointCloud(*object, *object, poseLocal);

            // get time of method
            auto timeEnd = std::chrono::high_resolution_clock::now();
            std::chrono::seconds time = std::chrono::duration_cast<std::chrono::seconds>(timeEnd - timeStart);
            std::cout << "Execution time: " << time.count() << std::endl;

            std::cout << "Found pose -->\n" << poseGlobal * poseLocal << std::endl;

            // store data in vectors
            globalPoses.push_back(matrix4f2transform(poseGlobal));
            localPoses.push_back(matrix4f2transform(poseLocal));
            times.push_back(time);
            //noises.push_back(NOISE); // noise analysis
            //threshData.push_back(GLOBAL_THRESH); // thresh analysis
            //iterationData.push_back(MAX_ITERATIONS); // iteration analysis
            //radiusData.push_back(SPIN_IMG_RADIUS); // spin images radius analysis

//            // show the state of the scene and the object
//            {
//                pcl::visualization::PCLVisualizer view("After alignment in scene");
//                view.addPointCloud<PointT>(object,pcl::visualization::PointCloudColorHandlerCustom<PointT>(object,255,0,0),"object");
//                view.addPointCloud<PointT>(scene,pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene,0,255,0),"scene");
//                view.spin();
//            }

            // show the state of the scene and the object
            {
                PointCloudT::Ptr origin(new PointCloudT);
                const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
                pcl::io::loadPCDFile(path, *origin);
                spatialFilter(origin, origin);
                pcl::visualization::PCLVisualizer view("After alignment in origin scene");
                view.addPointCloud<PointT>(object,pcl::visualization::PointCloudColorHandlerCustom<PointT>(object,255,0,0),"object");
                view.addPointCloud<PointT>(origin,pcl::visualization::PointCloudColorHandlerCustom<PointT>(origin,0,255,0),"origin");
                view.spin();
            }

            // save pcd file of scene
            //const std::string fileName = "../../data/point_cloud_estimations/Scanner25D_" + std::to_string(i) + "_" + std::to_string(noise) + ".pcd";
            //saveSceneWithObject(object, origin, fileName);
        //}
    }

    // save data
    data2File(globalPoses, localPoses, times);
    //noiseData2File(globalPoses, localPoses, times, noises); // noise analysis
    //threshData2File(globalPoses, localPoses, threshData, times); // thresh analysis
    //iterationData2File(globalPoses, localPoses, iterationData, times); // iterations analysis
    //radiusData2File(globalPoses, localPoses, radiusData, times); // spin images radius analysis
    writeTransforms2File(getTransform("Table"), getTransform("WORLD"), getTransform("Scanner25D"));

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
//---------------------------------------------------------
