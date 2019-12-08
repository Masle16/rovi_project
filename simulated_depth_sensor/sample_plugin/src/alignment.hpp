#pragma once

// INCLUDE
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

// DEFINES
#define OBJECT_PATH "/home/mathi/Documents/rovi/rovi_project/point_clouds_of_objects/rubber_duck.pcd"

// TYPEDEFS
typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudT;
typedef pcl::Histogram<153> HistT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::FPFHEstimationOMP<PointT, PointT, FeatureT> FeatureEstimationT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

// FUNCTIONS

/** Performs voxel grid on input
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/voxel_grid.cpp
 * @brief voxelGrid
 * @param inputCloud
 * @param outputCloud
 * @param leafSize
 */
void voxelGrid(PointCloudT::Ptr inputCloud, PointCloudT::Ptr &outputCloud, const float leafSize=0.005f) {
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
void outlierRemoval(const PointCloudT::Ptr inputCloud,
                    PointCloudT::Ptr &outputCloud,
                    const float mean=10.0f,
                    const float stdDev=1.0f) {
    std::cout << "Outlier removal.." << std::endl;
    pcl::StatisticalOutlierRemoval<PointT> statsOutlierRemoval;
    statsOutlierRemoval.setInputCloud(inputCloud);
    statsOutlierRemoval.setMeanK(mean);
    statsOutlierRemoval.setStddevMulThresh(stdDev);
    statsOutlierRemoval.filter(*outputCloud);
    std::cerr << "\tPointCloud after outlier removal: " << outputCloud->width * outputCloud->height << std::endl;
}

/** Spatial filters of cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/passthrough_filter.cpp
 * @brief spatialFilter : Performs spatial filtering in x, y and z of the input cloud
 * @param inputCloud : input point cloud
 * @param outputCloud : output point cloud
 */
void spatialFilter(const PointCloudT::Ptr &inputCloud,
                   PointCloudT::Ptr &outputCloud,
                   const float zMin=-1.5f,
                   const float zMax=0.0f,
                   const float xMin=-0.5f,
                   const float xMax=0.5f,
                   const float yMin=-0.3f,
                   const float yMax=0.1625f) {
    std::cout << "Spatial filter.." << std::endl;
    pcl::PassThrough<PointT> spatialFilter;
    spatialFilter.setInputCloud(inputCloud);
    // Z
    spatialFilter.setFilterFieldName("z");
    spatialFilter.setFilterLimits(zMin, zMax);
    spatialFilter.filter(*outputCloud);
    // X
    spatialFilter.setFilterFieldName("x");
    spatialFilter.setFilterLimits(xMin, xMax);
    spatialFilter.filter(*outputCloud);
    // Y
    spatialFilter.setFilterFieldName("y");
    spatialFilter.setFilterLimits(yMin, yMax);
    spatialFilter.filter(*outputCloud);
    std::cerr << "\tPointCloud after spatial filter: " << outputCloud->width * outputCloud->height << std::endl;
}

/** Smoothing of point cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/tools/mls_smoothing.cpp
 * @brief smoothing : Performs smoothing of input cloud
 * @param input_cloud : input point cloud
 * @param output_cloud : output point cloud
 */
void smoothing(const PointCloudT::Ptr &input_cloud,
               PointCloudT::Ptr &output_cloud,
               const int polyOrder=2,
               const float radius=0.01f) {
    std::cout << "Smoothing.." << std::endl;
    // create a kd-tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<PointT> mls_points;
    pcl::MovingLeastSquares<PointT, PointT> mls;
    // set parameters
    mls.setInputCloud(input_cloud);
    mls.setPolynomialOrder(polyOrder);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
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
void planarSegmentation(const PointCloudT::Ptr &cloud,
                        PointCloudT::Ptr &output,
                        const float removeFraction=0.3f,
                        const float distance=0.01f) {
    std::cout << "Planer segmentaion.." << std::endl;
    int nPoints = (int)output->points.size();
    while (output->points.size() > (removeFraction * nPoints)) {
        std::cout << "\tCloud points: " << output->points.size() << std::endl;
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance);
        seg.setInputCloud(output);
        seg.segment(*inliers, *coeff);
        if (inliers->indices.size() == 0) {
            std::cout << "Cloud not estimate a planar model for the dataset!" << std::endl;
            return;
        }
        // extract inliers
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(output);
        extract.setIndices(inliers);
        extract.setNegative(0.005f);
        extract.filter(*output);
    }
    std::cerr << "\tPointCloud after planar segmentation: " << output->width * output->height << std::endl;
}

/** Cluster extraction of cloud
 * Inspiration --> https://github.com/Masle16/pcl/blob/master/examples/segmentation/example_extract_clusters_normals.cpp
 * @brief euclideanCusterExtraction : Performs cluster extraction of cloud and remove to big and to small clusters
 * @param cloud : the scene from the Scanner25D
 * @return : a new scene with the cluster which muchly resembles the duck
 */
PointCloudT::Ptr euclideanCusterExtraction(const PointCloudT::Ptr &cloud,
                                           const float radius=0.01f,
                                           const int minSize=750,
                                           const int maxSize=1500) {
    std::cout << "Euclidean cluster extraction.." << std::endl;
    PointCloudT::Ptr result(new PointCloudT(*cloud));
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> euclideanCluster;
    euclideanCluster.setClusterTolerance(radius);
    euclideanCluster.setMinClusterSize(minSize);
    euclideanCluster.setMaxClusterSize(maxSize);
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
pcl::PointCloud<HistT>::Ptr computeSpinImages(PointCloudT::Ptr cloud,
                                              PointCloudT::Ptr normals,
                                              const float radius=0.05) {
    // setup spin image computation
    pcl::SpinImageEstimation<PointT, PointT, HistT> spin_image_descriptor(8, 0.5, 4);
    spin_image_descriptor.setInputCloud(cloud);
    spin_image_descriptor.setInputNormals(normals);
    // create KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    spin_image_descriptor.setSearchMethod(tree);
    // Calculate spin images
    pcl::PointCloud<HistT>::Ptr spin_images(new pcl::PointCloud<HistT>());
    spin_image_descriptor.setRadiusSearch(radius);
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
Eigen::Matrix4f findGlobalAlignment(PointCloudT::Ptr scene,
                                    PointCloudT::Ptr object,
                                    const float spinImgRadius=0.05f,
                                    const unsigned int inliersThresh=0.0001f,
                                    const std::size_t maxIterations=50000) {
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
        spinEstimator.setRadiusSearch(spinImgRadius);
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
                if (sqrDist[j][0] <= inliersThresh) {
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
            if (i > maxIterations) {
                std::cout << "\t\tMax iterations reached" << std::endl;
                break;
            }
            i++;
            if (i%1000 == 0) { std::cout << "\t\t" << i << " / " << maxIterations << std::endl; }
        }
        // compute inliers and RMSE
        std::vector<std::vector<int>> idx;
        std::vector<std::vector<float>> sqrDist;
        tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, idx, sqrDist);
        std::size_t inliers = 0;
        float rmse = 0.0;
        for (std::size_t j = 0; j < sqrDist.size(); j++) {
            if (sqrDist[j][0] <= inliersThresh) {
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
Eigen::Matrix4f findLocalAlignment(const PointCloudT::Ptr &scene,
                                   const PointCloudT::Ptr &object,
                                   const std::size_t maxIterations=500,
                                   const float inlierThreshold=0.0001f) {
    std::cout << "Local alignment.." << std::endl;
    Eigen::Matrix4f result;
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(scene);
    PointCloudT::Ptr objectAligned(new PointCloudT(*object));
    std::cout << "\tStarting ICP.." << std::endl;
    {
        pcl::ScopeTime t("\tLocal alignment");
        for (std::size_t i = 0; i < maxIterations; i++) {
            if ((i+1) % 250 == 0) { std::cout << "\t\t" << i+1 << " / " << maxIterations << std::endl; }
            // find the closest points
            std::vector<std::vector<int>> indexs;
            std::vector<std::vector<float>> sqrDists;
            tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, indexs, sqrDists);
            // Threshold and create indices for object/scene and compute RMSE
            std::vector<int> idxObject, idxScene;
            for (std::size_t j = 0; j < indexs.size(); j++) {
                if (sqrDists[j][0] <= inlierThreshold) {
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
            // update result
            if (i == 0) { result = T; }
            else { result = T * result; }
        }
    }
    // compute inliers and RMSE
    std::vector<std::vector<int>> indexs;
    std::vector<std::vector<float>> sqrDists;
    tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, indexs, sqrDists);
    std::size_t inliers = 0;
    float rmse = 0.0;
    for (size_t i = 0; i < sqrDists.size(); i++) {
        if (sqrDists[i][0] <= inlierThreshold) {
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
Eigen::Matrix4f computeGlobalPose(const PointCloudT::Ptr &scene,
                                  const PointCloudT::Ptr &object,
                                  const float normalEstimationRadiusSearch=0.01,
                                  const float featureRadiusSearch=0.01,
                                  const int maxIterations=80000,
                                  const int numOfSamples2GeneratePose=3,
                                  const int numOfNearestFeatures=3,
                                  const float similarityThreshold=0.9f,
                                  const float inlierThreshold=1.5*0.005f,
                                  const float inlierFraction=0.25f) {
    std::cout << "Computing global pose.." << std::endl;
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    // point clouds
    PointCloudT::Ptr objectAligned(new PointCloudT);
    FeatureCloudT::Ptr objectFeatures(new FeatureCloudT);
    FeatureCloudT::Ptr sceneFeatures(new FeatureCloudT);
    // Estimate normals for scene
    std::cout << "\tEstimating scene normals.." << std::endl;
    pcl::NormalEstimationOMP<PointT, PointT> nest;
    nest.setRadiusSearch(normalEstimationRadiusSearch);
    nest.setInputCloud(scene);
    nest.compute(*scene);
    // Estimate features
    std::cout << "\tEstimating features.." << std::endl;
    FeatureEstimationT fest;
    fest.setRadiusSearch(featureRadiusSearch);
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
    align.setMaximumIterations(maxIterations); // number of RANSAC iterations
    align.setNumberOfSamples(numOfSamples2GeneratePose); // number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(numOfNearestFeatures); // number of nearest features to use
    align.setSimilarityThreshold(similarityThreshold); // polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(inlierThreshold); // inlier threshold
    align.setInlierFraction(inlierFraction); // Required inlier fraction for accepting a pose hypothesis
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
Eigen::Matrix4f computeICP(const PointCloudT::Ptr &target,
                           const PointCloudT::Ptr &source,
                           const int maxIterations=1000,
                           const float inlierThresh=0.001f) {
    PointCloudT::Ptr src = source, tgt = target;

    std::cerr << "\tComputing ICP.." << std::endl;

    pcl::registration::TransformationEstimationLM<PointT, PointT, float>::Ptr transEsti(new pcl::registration::TransformationEstimationLM<PointT, PointT, float>);
    pcl::registration::CorrespondenceEstimation<PointT, PointT, float>::Ptr corEsti(new pcl::registration::CorrespondenceEstimation<PointT, PointT, float>);
    corEsti->setInputSource(src);
    corEsti->setInputTarget(tgt);

    pcl::registration::CorrespondenceRejectorOneToOne::Ptr corRejOne2One(new pcl::registration::CorrespondenceRejectorOneToOne);

//    pcl::registration::CorrespondenceRejectorMedianDistance::Ptr corRejMed(new pcl::registration::CorrespondenceRejectorMedianDistance);
//    corRejMed->setInputSource<PointT>(src);
//    corRejMed->setInputTarget<PointT>(tgt);

//    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr corRejSac(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>);
//    corRejSac->setInputSource(src);
//    corRejSac->setInputTarget(tgt);
//    corRejSac->setInlierThreshold(0.005);
//    corRejSac->setMaximumIterations(10000);

//    pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr corRejVar(new pcl::registration::CorrespondenceRejectorVarTrimmed);
//    corRejVar->setInputSource<PointT>(src);
//    corRejVar->setInputTarget<PointT>(tgt);

//    pcl::registration::CorrespondenceRejectorTrimmed::Ptr corRejTrim(new pcl::registration::CorrespondenceRejectorTrimmed);

    pcl::IterativeClosestPoint<PointT, PointT, float> icp;
    icp.setCorrespondenceEstimation(corEsti);
    icp.setTransformationEstimation(transEsti);
    icp.addCorrespondenceRejector(corRejOne2One);
//    icp.addCorrespondenceRejector(corRejMed);
//    icp.addCorrespondenceRejector(corRejSac);
//    icp.addCorrespondenceRejector(corRejVar);
//    icp.addCorrespondenceRejector(corRejTrim);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaximumIterations(maxIterations);
    icp.setTransformationEpsilon(inlierThresh);
    PointCloudT output;
    icp.align(output);
    std::cerr << "\t" << icp.getFitnessScore() << std::endl;
    return icp.getFinalTransformation();
}

/** Returns input cloud with gaussian noise
 * From: https://github.com/Masle16/pcl/blob/master/tools/add_gaussian_noise.cpp
 * @brief addGaussianNoise : copies input cloud and applies gaussian noise to new cloud
 * @param input : the cloud to copy
 * @param stdDev : standard deviation of the gaussian distribution
 * @return : a copy of input with noise applied
 */
PointCloudT::Ptr addGaussianNoise(const PointCloudT::Ptr &input, const float stdDev=0.0f) {
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

/** Saves two point cloud in one pcd file
 * @brief saveSceneWithObject : merged two point cloud and saves a pcd file
 * @param object : point cloud
 * @param scene : point cloud
 * @param fileName : path and name of the decired save location
 */
void saveSceneWithObject(const std::string &fileName, const Eigen::Matrix4f &pose) {
    std::cout << "Saving view to --> " << fileName << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

    // load object
    PointCloudT::Ptr object(new PointCloudT);
    pcl::io::loadPCDFile(OBJECT_PATH, *object);
    voxelGrid(object, object, 0.005f);

    // transform object into scene
    pcl::transformPointCloud(*object, *object, pose);

    // load scene
    PointCloudT::Ptr scene(new PointCloudT);
    pcl::io::loadPCDFile("Scanner25D.pcd", *scene);
    spatialFilter(scene, scene);

    // load object --> result
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

    // load scene --> result
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

Eigen::Matrix4f alignment(const float noise=0.0) {
    Eigen::Matrix4f result;

    // load the scene
    PointCloudT::Ptr scene(new PointCloudT);
    pcl::io::loadPCDFile("Scanner25D.pcd", *scene);

    // load the object
    PointCloudT::Ptr object(new PointCloudT);
    pcl::io::loadPCDFile(OBJECT_PATH, *object);
    voxelGrid(object, object, 0.005f);

    // add noise
    scene = addGaussianNoise(scene, noise);

    // calculate pose
    Eigen::Matrix4f poseGlobal, poseLocal;
    {
        pcl::ScopeTime t("Execution time");

        // point cloud filtering
        spatialFilter(scene, scene);
        voxelGrid(scene, scene, 0.005f);
        smoothing(scene, scene);
        planarSegmentation(scene, scene);
        outlierRemoval(scene, scene);
        //scene = euclideanCusterExtraction(scene);

        // pose estimation 3D to 3D
        poseGlobal = computeGlobalPose(scene, object);
        pcl::transformPointCloud(*object, *object, poseGlobal);
        poseLocal = findLocalAlignment(scene, object);
        pcl::transformPointCloud(*object, *object, poseLocal);
    }
    result = poseLocal * poseGlobal;

//    // show the alignment in origin scene
//    {
//        PointCloudT::Ptr _scene(new PointCloudT);
//        PointCloudT::Ptr _object(new PointCloudT);

//        pcl::io::loadPCDFile("Scanner25D.pcd", *_scene);
//        spatialFilter(_scene, _scene);
//        voxelGrid(scene, scene);

//        pcl::io::loadPCDFile(OBJECT_PATH, *_object);
//        voxelGrid(_object, _object, 0.005f);
//        pcl::transformPointCloud(*_object, *_object, result);

//        pcl::visualization::PCLVisualizer view("After alignment");
//        view.addPointCloud<PointT>(_object, ColorHandlerT(_object, 255, 0 , 0), "Object");
//        view.addPointCloud<PointT>(_scene, ColorHandlerT(_scene, 0, 255, 0), "Origin");
//        view.spin();
//    }

    return result;
}
