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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>

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
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::FPFHEstimationOMP<PointT, PointT, FeatureT> FeatureEstimationT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

//---------------------------------------------------------

// GLOBAL VARIABLES
// voxel grid
float LEAF_SIZE = 1e-2f;

// outlier removal
float MEAN = 200.0f; // number of neighbors
float STD_DEV = 0.1f; // distance between

// spatial filter
float X_MIN = -0.5f, X_MAX = 0.5f;
float Y_MIN = -0.3f, Y_MAX = 0.1625f;
float Z_MIN = -1.5f, Z_MAX = 0.0f;

// smoothing
unsigned int SMOOTHING_POLY_ORDER = 4;
float SMOOTHING_RADIUS = 0.01f; // 0.05

// cluster extraction
unsigned int CLUSTER_MIN_SIZE = 750; // 750
unsigned int CLUSTER_MAX_SIZE = 1500; // 950
float CLUSTER_RADIUS = 0.01f; // 0.01

// global alignment
std::size_t MAX_ITERATIONS = 50000;
std::size_t NUMBER_INLIERS = 260; // 240
float SPIN_IMG_RADIUS = 0.05f; // 0.05 // see sin_imgs_radius_analysis.m
float GLOBAL_THRESH = 0.0001f; //2.5e-5; // see thresh_analysis.m

// local alignment
std::size_t LOCAL_ITERATIONS = 100;
float LOCAL_THRESH = 0.0001f; //2.5e-5; // see thresh_analysis.m

// add guassian noise
float NOISE = 0.001f; // see noise_analysis.m

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
    std::cout << "Voxel grid.." << std::endl;
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*outputCloud);
    std::cerr << "\tPointCloud after voxel grid: " << outputCloud->width * outputCloud->height << std::endl;
}

/** Statistical Outlier Removal
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/outlier_removal.cpp
 * @brief outlierRemoval : performs statistical outlier removal of input cloud
 * @param inputCloud : input point cloud
 * @param outputCloud : output point cloud
 */
void outlierRemoval(PointCloudT::Ptr inputCloud, PointCloudT::Ptr &outputCloud) {
    std::cout << "Outlier removal.." << std::endl;
    pcl::StatisticalOutlierRemoval<PointT> statsOutlierRemoval;
    statsOutlierRemoval.setInputCloud(inputCloud);
    statsOutlierRemoval.setMeanK(MEAN);
    statsOutlierRemoval.setStddevMulThresh(STD_DEV);
    statsOutlierRemoval.filter(*outputCloud);
    std::cerr << "\tPointCloud after outlier removal: " << outputCloud->width * outputCloud->height << std::endl;
}

/** Spatial filters of cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/passthrough_filter.cpp
 * @brief spatialFilter : Performs spatial filtering in x, y and z of the input cloud
 * @param inputCloud : input point cloud
 * @param outputCloud : output point cloud
 */
void spatialFilter(const PointCloudT::Ptr &inputCloud, PointCloudT::Ptr &outputCloud) {
    std::cout << "Spatial filter.." << std::endl;
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
    std::cerr << "\tPointCloud after spatial filter: " << outputCloud->width * outputCloud->height << std::endl;
}

/** Smoothing of point cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/mls_smoothing.cpp
 * @brief smoothing : Performs smoothing of input cloud
 * @param input_cloud : input point cloud
 * @param output_cloud : output point cloud
 */
void smoothing(const PointCloudT::Ptr &input_cloud, PointCloudT::Ptr &output_cloud) {
    std::cout << "Smoothing.." << std::endl;
    // create a kd-tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<PointT> mls_points;
    pcl::MovingLeastSquares<PointT, PointT> mls;
    // set parameters
    mls.setInputCloud(input_cloud);
    mls.setPolynomialOrder(SMOOTHING_POLY_ORDER);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(SMOOTHING_RADIUS);
    // reconstruct
    mls.process(mls_points);
    pcl::copyPointCloud(mls_points, *output_cloud);
    std::cerr << "\tPointCloud after smoothing: " << output_cloud->width * output_cloud->height << std::endl;
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
            std::cout << "Cloud not estimate a planar model for the dataset!" << std::endl;
            return result;
        }
        // extract inliers
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(result);
        extract.setIndices(inliers);
        extract.setNegative(0.005f);
        extract.filter(*result);
    }
    std::cerr << "\tPointCloud after planar segmentation: " << result->width * result->height << std::endl;
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
        std::cout << "\tCluster size: " << result->width * result->height << std::endl;
    }
    return result;
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
Eigen::Matrix4f findGlobalAlignment(PointCloudT::Ptr scene, PointCloudT::Ptr object, unsigned int inliersThresh = NUMBER_INLIERS) {
    std::cout << "Global alignment.." << std::endl;
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
        unsigned int i = 0;
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
                std::cout << "\t\t--> Got a new model with " << inliers << " inliers!" << std::endl;
                penalty = outlierRate;
                result = T;
                // stop if enough inliers are found
//                if (inliers >= inliersThresh) {
//                    std::cout << "\t\tFound enough inliers" << std::endl;
//                    break;
//                }
            }
            // stop if max iterations reached
            if (i > MAX_ITERATIONS) {
                std::cout << "\t\tMax iterations reached" << std::endl;
                break;
            }
            i++;
            if (i%1000 == 0) { std::cout << "\t\t" << i << " / " << MAX_ITERATIONS << std::endl; }
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
    std::cout << "Local alignment.." << std::endl;
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(scene);
    PointCloudT::Ptr objectAligned(new PointCloudT(*object));
    std::cout << "\tStarting ICP.." << std::endl;
    {
        pcl::ScopeTime t("\tLocal alignment");
        for (std::size_t i = 0; i < LOCAL_ITERATIONS; i++) {
            if ((i +1) % 250 == 0) { std::cout << "\t\t" << i+1 << " / " << LOCAL_ITERATIONS << std::endl; }
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
    // print pose
    std::cout << "\tGot the following pose-->" << std::endl;
    std::cout << "\t\t"   << result(0,0) << " " << result(0,1) << " " << result(0,2) << "\n"
              << "\tR =\t"<< result(1,0) << " " << result(1,1) << " " << result(1,2) << "\n"
              << "\t\t"   << result(2,0) << " " << result(2,1) << " " << result(2,2) << std::endl;
    std::cout << "\tP =\t"<< result(0,3) << " " << result(1,3) << " " << result(2,3) << std::endl;
    std::cout << "\tInliers: " << inliers << std::endl;
    std::cout << "\tRMSE: " << rmse << std::endl;
    return result;
}

/**
 * Inspiration --> http://pointclouds.org/documentation/tutorials/alignment_prerejective.php?fbclid=IwAR1GlmwhDLB4sJwawsdhV_Osas8oDqZebw4fxtcO8T7b1TMeOpurmk1bWoM
 * @brief computeGlobalPose
 * @param scene
 * @param object
 * @return
 */
Eigen::Matrix4f computeGlobalPose(PointCloudT::Ptr &scene, PointCloudT::Ptr &object) {
    std::cout << "Computing global pose.." << std::endl;
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    PointCloudT::Ptr objectAligned(new PointCloudT);
    FeatureCloudT::Ptr objectFeatures(new FeatureCloudT);
    FeatureCloudT::Ptr sceneFeatures(new FeatureCloudT);
    // downsample
    std::cout << "\tDownsampling.." << std::endl;
    pcl::VoxelGrid<PointT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object);
    std::cout << "\tObject after downsampling --> " << object->width * object->height << std::endl;
    grid.setInputCloud(scene);
    grid.filter(*scene);
    std::cout << "\tScene after downsampling --> " << scene->width * scene->height << std::endl;
    // Estimate normals for scene
    std::cout << "\tEstimating scene normals.." << std::endl;
    pcl::NormalEstimationOMP<PointT, PointT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(scene);
    nest.compute(*scene);
    // Estimate features
    std::cout << "\tEstimating features.." << std::endl;
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*objectFeatures);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*sceneFeatures);
    // perform alignment
    std::cout << "\tStarting alignment.." << std::endl;
    pcl::SampleConsensusPrerejective<PointT, PointT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(objectFeatures);
    align.setInputTarget(scene);
    align.setTargetFeatures(sceneFeatures);
    align.setMaximumIterations(50000); // number of RANSAC iterations
    align.setNumberOfSamples(3); // number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(5); // number of nearest features to use
    align.setSimilarityThreshold(0.9f); // polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(0.01); // inlier threshold
    align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("\tGlobal alignment");
        align.align(*objectAligned);
    }
    if (align.hasConverged()) {
        // print results
        result = align.getFinalTransformation();
        std::cout << "\tFinal transformation -->" << std::endl;
        std::cout << "\t\t"   << result(0,0) << " " << result(0,1) << " " << result(0,2) << "\n"
                  << "\tR =\t"<< result(1,0) << " " << result(1,1) << " " << result(1,2) << "\n"
                  << "\t\t"   << result(2,0) << " " << result(2,1) << " " << result(2,2) << std::endl;
        std::cout << "\tP =\t"<< result(0,3) << " " << result(1,3) << " " << result(2,3) << std::endl;
        std::cout << "\tInliers: " << align.getInliers().size() << " / " << object->size() << std::endl;
//        // show alignment
//        {
//            pcl::visualization::PCLVisualizer view("Global alignment");
//            view.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
//            view.addPointCloud(objectAligned, ColorHandlerT(objectAligned, 255.0, 0.0, 0.0), "objectAligned");
//            view.spin();
//        }
        return result;
    }
    else {
        // alignment failed
        std::cout << "\tGlobal alignment failed!" << std::endl;
        return result;
    }
}

/** Compute Iterative Closest Point
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/iterative_closest_point.cpp
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
               const std::vector<double> &times) {
    std::cout << "Writing data to file data.txt " << std::endl;
    std::ofstream file;
    file.open("../../data/data.txt");
    for (std::size_t i = 0; i < times.size(); i++) {
        rw::math::Vector3D<> posGlobal = globalPoses[i].P();
        rw::math::RPY<> rpyGlobal = rw::math::RPY<>(globalPoses[i].R());
        rw::math::Vector3D<> posLocal = localPoses[i].P();
        rw::math::RPY<> rpyLocal = rw::math::RPY<>(localPoses[i].R());
        file << times[i]     << " " // time
             << posGlobal(0) << " " << posGlobal(1) << " " << posGlobal(2) << " "   // pos global
             << rpyGlobal(0) << " " << rpyGlobal(1) << " " << rpyGlobal(2) << " "   // rpy global
             << posLocal(0)  << " " << posLocal(1)  << " " << posLocal(2)  << " "   // pos local
             << rpyLocal(0)  << " " << rpyLocal(1)  << " " << rpyLocal(2)  << "\n"; // rpy local
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
    std::vector<double> times;
        for (std::size_t i = 0; i < 30; i++) {
            // load the point cloud
            PointCloudT::Ptr scene(new PointCloudT);
            const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
            pcl::io::loadPCDFile(path, *scene);
            std::cout << "Processing file: " << path << " number " << i << " / 30" << std::endl;
            // load the generated object point cloud
            PointCloudT::Ptr object(new PointCloudT);
            pcl::io::loadPCDFile(OBJECT_PATH, *object);
            //add noise to Scanner25D point cloud
            scene = addGaussianNoise(scene, NOISE); // noise analysis
            {
                pcl::ScopeTime t("Execution time");
                //==========================
                // point cloud preprocessing
                //==========================
                std::cout << "Point cloud preprocessing" << std::endl;
                spatialFilter(scene, scene);
                smoothing(scene, scene);
                scene = planarSegmentation(scene);
//                outlierRemoval(scene, scene);
//                scene = euclideanCusterExtraction(scene);
                //=========================
                // pose estimation 3D to 3D
                //=========================
                std::cout << "Pose estimation 3D to 3D" << std::endl;
                // global alignment
                Eigen::Matrix4f poseGlobal = computeGlobalPose(scene, object);
                pcl::transformPointCloud(*object, *object, poseGlobal);
                // local alignment
                Eigen::Matrix4f poseLocal = findLocalAlignment(scene, object);
                pcl::transformPointCloud(*object, *object, poseLocal);
                // save data
                times.push_back(t.getTime());
                globalPoses.push_back(matrix4f2transform(poseGlobal));
                localPoses.push_back(matrix4f2transform(poseLocal));
                // print esitmate pose
                Eigen::Matrix4f result = poseGlobal * poseLocal;
                std::cout << "Estimate transformation -->" << std::endl;
                std::cout << "\t"   << result(0,0) << " " << result(0,1) << " " << result(0,2) << "\n"
                          << "R =\t"<< result(1,0) << " " << result(1,1) << " " << result(1,2) << "\n"
                          << "\t"   << result(2,0) << " " << result(2,1) << " " << result(2,2) << std::endl;
                std::cout << "P =\t"<< result(0,3) << " " << result(1,3) << " " << result(2,3) << std::endl;
            }
//            // show the state of the scene and the object
//            {
//                PointCloudT::Ptr origin(new PointCloudT);
//                const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
//                pcl::io::loadPCDFile(path, *origin);
//                spatialFilter(origin, origin);
//                pcl::visualization::PCLVisualizer view("After alignment in origin scene");
//                view.addPointCloud<PointT>(object, ColorHandlerT(object,255,0,0), "object");
//                view.addPointCloud<PointT>(origin, ColorHandlerT(origin,0,255,0), "origin");
//                view.spin();
//            }
    }
    // save data
    data2File(globalPoses, localPoses, times);
    writeTransforms2File(getTransform("Table"), getTransform("WORLD"), getTransform("Scanner25D"));

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
//---------------------------------------------------------
