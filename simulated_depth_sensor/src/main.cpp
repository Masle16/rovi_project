/* RoVi Final Project
 * Simulated Depth Sensor
 */

// INCLUDES
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
#include <rw/rw.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

//---------------------------------------------------------

// DEFINES
#define MAX_ITERATIONS 5000
#define LEAF_SIZE 0.001
#define MEAN 200
#define STD_DEV 0.1
#define FILTER_LIMIT_MIN -1.5
#define FILTER_LIMIT_MAX 0// -1.15
#define SMOOTHING_RADIUS 5
#define SPIN_IMAGES_RADIUS 0.05
#define ALIGNMENT_THRESHOLD 0.000025
#define THRESH_SQR 0.000025
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

// FUNCTIONS
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
    viewer->close();
}

/**
 * @brief voxelGrid
 * @param input_cloud
 * @param output_cloud
 */
void voxelGrid(PointCloudT::Ptr inputCloud,
               PointCloudT::Ptr &outputCloud,
               const float leafSize = 0.01) {
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*outputCloud);
}

/**
 * @brief outlierRemoval
 * @param input_cloud
 * @param output_clod
 */
void outlierRemoval(PointCloudT::Ptr inputCloud,
                    PointCloudT::Ptr &outputCloud) {
    pcl::StatisticalOutlierRemoval<PointT> statsOutlierRemoval;
    statsOutlierRemoval.setInputCloud(inputCloud);
    statsOutlierRemoval.setMeanK(MEAN);
    statsOutlierRemoval.setStddevMulThresh(STD_DEV);
    statsOutlierRemoval.filter(*outputCloud);
}

/**
 * @brief spatialFilter
 * @param input_cloud
 * @param output_cloud
 */
void spatialFilter(PointCloudT::Ptr inputCloud,
                   PointCloudT::Ptr &outputCloud) {
    pcl::PassThrough<PointT> spatialFilter;
    spatialFilter.setInputCloud(inputCloud);
    spatialFilter.setFilterFieldName("z");
    spatialFilter.setFilterLimits(FILTER_LIMIT_MIN, FILTER_LIMIT_MAX);
    spatialFilter.filter(*outputCloud);
}

/**
 * @brief smoothing
 * @param input_cloud
 * @param output_cloud
 */
void smoothing(PointCloudT::Ptr input_cloud,
               PointCloudT::Ptr &output_cloud) {
    // create a kd-tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<PointT> mls_points;
    pcl::MovingLeastSquares<PointT, PointT> mls;

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
 * @brief planarSegmentation
 * @param inputCloud
 * @param outputCloud
 */
PointCloudT::Ptr planarSegmentation(PointCloudT::Ptr &cloud) {
    PointCloudT::Ptr result(new PointCloudT());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.size() == 0) {
        PCL_ERROR("Cloud not estimate a planar model for the dataset.");
        return cloud;
    }
//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
//        std::cerr << "Indices: " << inliers->indices[i]
//                  << "\tpoint: " << cloud->points[inliers->indices[i]].x << " "
//                                 << cloud->points[inliers->indices[i]].y << " "
//                                 << cloud->points[inliers->indices[i]].z << std::endl;
    // extract inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(0.005f);
    extract.filter(*result);
    return result;
}

/**
 * @brief euclideanCusterExtraction :
 * @param cloud : the scene from the Scanner25D
 * @return : a new scene with the cluster which muchly resembles the duck
 */
PointCloudT::Ptr euclideanCusterExtraction(PointCloudT::Ptr &cloud) {
    //PointCloudT::Ptr result(new PointCloudT);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> euclideanCluster;
    euclideanCluster.setClusterTolerance(0.04);
    euclideanCluster.setMinClusterSize(0);
    euclideanCluster.setMaxClusterSize(25000);
    euclideanCluster.setInputCloud(cloud);
    euclideanCluster.extract(clusterIndices);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++) {
        PointCloudT::Ptr cloudCluster(new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloudCluster->points.push_back(cloud->points[*pit]);
        }
       cloudCluster->width = cloudCluster->points.size();
       cloudCluster->height = 1;
       cloudCluster->is_dense = true;
//        // show cluster
//        {
//            pcl::visualization::PCLVisualizer view("cluster");
//            view.addPointCloud<PointT>(cloudCluster, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloudCluster, 0, 255, 0),"cluster");
//            view.spin();
//        }
        std::cout << "\tCluster information -->" << std::endl;
        std::cerr << "\t" << cloudCluster->width * cloudCluster->height
                  << " data points ("
                  << pcl::getFieldsList(*cloudCluster)
                  << ")."
                  << std::endl;
        if (cloudCluster->points.size() < 925 && cloudCluster->points.size() > 750) {
            cloud = cloudCluster;
            break;
        }
    }
    return cloud;
}

/**
 * @brief distSqr
 * @param query
 * @param target
 * @return
 */
float distSqr(const HistT &query, const HistT &target) {
    float result = 0.0;
    for (int i = 0; i < pcl::Histogram<153>::descriptorSize(); i++) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    return result;
}

/**
 * @brief nearestFeature
 * @param query
 * @param target
 * @param index
 * @param sqrDist
 */
void nearestFeature(const HistT &query,
                    const pcl::PointCloud<HistT> &target,
                    int &index,
                    float &sqrDist) {
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

/**
 * This function is from exercise 6 in the computer vision course.
 *
 * @brief estimateSurfaceNormal : Compute the surface normals using k=10 nearest
 *  neighbors around each point
 * @param cloud
 * @return
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

/**
 * This function is from exercise 6 in the computer vision course.
 *
 * @brief computeSpinImages : Compute local spin image features with r=5 surface
 *  description.
 * @param cloud
 * @param normals
 * @return
 */
pcl::PointCloud<HistT>::Ptr computeSpinImages(PointCloudT::Ptr cloud,
                                              PointCloudT::Ptr normals) {
    // setup spin image computation
    pcl::SpinImageEstimation<PointT, PointT, HistT> spin_image_descriptor(8, 0.5, 4);
    spin_image_descriptor.setInputCloud(cloud);
    spin_image_descriptor.setInputNormals(normals);
    // create KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    spin_image_descriptor.setSearchMethod(tree);
    // Calculate spin images
    pcl::PointCloud<HistT>::Ptr spin_images(new pcl::PointCloud<HistT>());
    spin_image_descriptor.setRadiusSearch(SPIN_IMAGES_RADIUS);
    // compute the spin images
    spin_image_descriptor.compute(*spin_images);
    return spin_images;
}

/**
 * This functions is form exercise 6 in the computer vision course
 *
 * @brief calcL2Dist : compute the l2 distance between the two histograms
 * @param object : histogram of the object
 * @param scene : histogram of the scene
 * @return : l2 distance
 */
float computeL2Dist(const HistT &object,
                    const HistT &scene) {
    float result = 0.0;
    for (int i = 0; i < scene.descriptorSize(); i++) {
        result += std::sqrt(std::pow((scene.histogram[i] - object.histogram[i]), 2));
    }
    return result;
}

/**
 * This function is from exercise 6 in the computer vision course.
 *
 * @brief nearestMatchingFeature : find the nearest matching (k=1) scene feature
 *  for each object feature
 * @param scene : spin images of the scene point cloud
 * @param object : spin images of the object point cloud
 * @return :
 */
std::vector<int> nearestMatchingFeature(pcl::PointCloud<HistT>::Ptr scene,
                                        pcl::PointCloud<HistT>::Ptr object) {
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

/**
 * This function is from exercise 6 in the computer vision course.
 *
 * @brief findAlignment
 * @param scene
 * @param object
 * @param object2scene
 * @param &output
 * @return
 */
Eigen::Matrix4f findGlobalAlignment(PointCloudT::Ptr scene,
                                    PointCloudT::Ptr object) {
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
        spinEstimator.setRadiusSearch(0.05);
        // object
        spinEstimator.setInputCloud(object);
        spinEstimator.setInputNormals(object);
        spinEstimator.compute(*objectFeatures);
        // scene
        spinEstimator.setInputCloud(scene);
        spinEstimator.setInputNormals(scene);
        spinEstimator.compute(*sceneFeatures);
    }

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
        for (size_t i = 0; i < MAX_ITERATIONS; i++) {
            if ((i +1) % 250 == 0) { std::cout << "\t" << i+1 << std::endl; }
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
            size_t inliers = 0;
            float rmse = 0.0;
            for (size_t j = 0; j < sqrDist.size(); j++) {
                if (sqrDist[j][0] <= THRESH_SQR) {
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
            }
        }
        // compute inliers and RMSE
        std::vector<std::vector<int>> idx;
        std::vector<std::vector<float>> sqrDist;
        tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, idx, sqrDist);
        size_t inliers = 0;
        float rmse = 0.0;
        for (size_t j = 0; j < sqrDist.size(); j++) {
            if (sqrDist[j][0] <= THRESH_SQR) {
                inliers++;
                rmse += sqrDist[j][0];
            }
        }
        // print timing
        auto timeEnd = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::seconds>(timeEnd - timeStart);
        std::cout << "\tExecution time for local alignment --> " << time.count() << " s" << std::endl;
        // print pose
        std::cout << "\tGot the following pose:\n" << result << std::endl;
        std::cout << "\tInliers: " << inliers << std::endl;
        std::cout << "\tRMSE: " << rmse << std::endl;
        return result;
    }
}

/**
 * This function is from exercise 6 in the computer vision course.
 *
 * @brief findLocalAlignment : implements ICP for bringing the models
 *  into accurate alignment
 * @param scene : scene point cloud
 * @param object : object point cloud
 * @param &output : estimated transformation
 */
Eigen::Matrix4f findLocalAlignment(PointCloudT::Ptr scene,
                                   PointCloudT::Ptr object) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(scene);
    PointCloudT::Ptr objectAligned(new PointCloudT(*object));
    std::cout << "\tStarting ICP.." << std::endl;
    auto timeStart = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < 50; i++) {
        // find the closest points
        std::vector<std::vector<int>> indexs;
        std::vector<std::vector<float>> sqrDists;
        tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, indexs, sqrDists);
        //
        std::vector<int> idxObject, idxScene;
        for (size_t j = 0; j < indexs.size(); j++) {
            if (sqrDists[j][0] <= THRESH_SQR) {
                idxObject.push_back(j);
                idxScene.push_back(indexs[j][0]);
            }
        }
        // estimate transformation
        Eigen::Matrix4f T;
        pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
        svd.estimateRigidTransformation(*objectAligned, idxObject, *scene, idxScene, T);
        // apply pose
        pcl::transformPointCloud(*objectAligned, *objectAligned, T);
        // update result
        result = T * result;
    }
    // compute inliers and RMSE
    std::vector<std::vector<int>> indexs;
    std::vector<std::vector<float>> sqrDists;
    tree.nearestKSearch(*objectAligned, std::vector<int>(), 1, indexs, sqrDists);
    size_t inliers = 0;
    float rmse = 0.0;
    for (size_t i = 0; i < sqrDists.size(); i++) {
        if (sqrDists[i][0] <= THRESH_SQR) {
            ++inliers;
            rmse += sqrDists[i][0];
        }
    }
    rmse = std::sqrt(rmse / inliers);
    // print timing
    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::seconds>(timeEnd - timeStart);
    std::cout << "\nExecution time for local alignment --> " << time.count() << " s" << std::endl;
    // print pose
    std::cout << "Got the following pose:\n" << result << std::endl;
    std::cout << "Inliers: " << inliers << std::endl;
    std::cout << "RMSE: " << rmse << std::endl;
    return result;
}

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

/**
 * @brief convertTransform3D2Matrix4
 * @param frame : frame name in the Scene.wx.xml file
 * @return : return the
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

/**
 * @brief matrix4f2transform
 * @param matrix
 * @return
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

/**
 * @brief writeMatrix4f2txt
 * @param matrix
 */
void writeData2File(const std::vector<rw::math::Transform3D<>>& globalPoses,
                    const std::vector<rw::math::Transform3D<>>& localPoses,
                    const std::vector<std::chrono::seconds> &times) {
    std::cout << "Writing data to files.." << std::endl;
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
        file << 0               << " " // noise
             << times[i].count()<< " " // time
             << posGlobal(0)    << " " << posGlobal(1)  << " " << posGlobal(2)  << " "   // pos global
             << rpyGlobal(0)    << " " << rpyGlobal(1)  << " " << rpyGlobal(2)  << " "   // rpy global
             << posLocal(0)     << " " << posLocal(1)   << " " << posLocal(2)   << " "   // pos local
             << rpyLocal(0)     << " " << rpyLocal(1)   << " " << rpyLocal(2)   << "\n"; // rpy local
    }
    file.close();
}

/**
 * @brief writeTransforms2File
 * @param tableT
 * @param worldT
 * @param scannerT
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

/**
 * @brief saveSceneWithObject
 * @param object
 * @param path2scene
 */
void saveSceneWithObject(const std::vector<Eigen::Matrix4f> Ts,
                         const PointCloudT scene,
                         const std::string& fileName) {
    // load scene
    PointCloudT object;
    pcl::io::loadPCDFile(OBJECT_PATH, object);
    for (unsigned int i = 0; i < Ts.size(); i++)
        pcl::transformPointCloud(object, object, Ts[i]);
    // merge point clouds
    PointCloudT newCloud;
    newCloud += scene;
    newCloud += object;
    pcl::io::savePCDFile(fileName, newCloud);
}

//---------------------------------------------------------

/*
 * MAIN ENTRY POINT
 */
int main(int argc, char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

    //---------------------------
    // POINT CLOUD PREPROCESSING
    //---------------------------
    std::cout << "Point cloud preprocessing" << std::endl;

    std::vector<rw::math::Transform3D<>> globalPoses, localPoses;
    std::vector<std::chrono::seconds> times;
    for (std::size_t i = 0; i < 30; i++) {
        // time start of method
        auto timeStart = std::chrono::high_resolution_clock::now();

        // load the point cloud
        PointCloudT::Ptr scene(new PointCloudT);
        const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(path, *scene);
        std::cout << "Processing file: " << path << " number " << i << " / 30" << std::endl;

    //    // show point cloud
    //    {
    //        pcl::visualization::PCLVisualizer view("Scene before preprocessing");
    //        view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
    //        view.spin();
    //    }

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

        // filter point cloud
        voxelGrid(scene, scene, 0.005);
        std::cerr << "PointCloud after voxel grid: "
                  << scene->width * scene->height
                  << " data points ("
                  << pcl::getFieldsList(*scene)
                  << ")."
                  << std::endl;

    //    outlierRemoval(scene, scene);
    //    std::cerr << "PointCloud after outlier removal filter: "
    //              << scene->width * scene->height
    //              << " data points ("
    //              << pcl::getFieldsList(*scene)
    //              << ")."
    //              << std::endl;

        spatialFilter(scene, scene);
        std::cerr << "PointCloud after spatial filter: "
                  << scene->width * scene->height
                  << " data points ("
                  << pcl::getFieldsList(*scene)
                  << ")."
                  << std::endl;

    //    smoothing(scene, scene);
    //    std::cerr << "PointCloud after spatial smoothing: "
    //              << scene->width * scene->height
    //              << " data points ("
    //              << pcl::getFieldsList(*scene)
    //              << ")."
    //              << std::endl;

        // remove plans in the scene
        int nPoints = (int)scene->points.size();
        while (scene->points.size() > (0.15 * nPoints)) {
            scene = planarSegmentation(scene);
        }
        std::cerr << "PointCloud after planar segmentation: "
                  << scene->width * scene->height
                  << " data points ("
                  << pcl::getFieldsList(*scene)
                  << ")."
                  << std::endl;


        scene = euclideanCusterExtraction(scene);
        std::cerr << "PointCloud after cluster extration: "
                  << scene->width * scene->height
                  << " data points ("
                  << pcl::getFieldsList(*scene)
                  << ")."
                  << std::endl;

        // save the new point cloud
        //pcl::io::savePCDFile("../cloud_filtered.pcd", *scene);

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
        voxelGrid(object, object, 0.005);
        std::cerr << "object point cloud after voxel grid: "
                  << object->width * object->height
                  << " data points ("
                  << pcl::getFieldsList(*object)
                  << ")."
                  << std::endl;

    //    // show intial state of scene and object
    //    {
    //        pcl::visualization::PCLVisualizer view("Scene before preprocessing");
    //        view.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
    //        view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
    //        view.spin();
    //    }

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
    //    // show the state of the scene and the object
    //    {
    //        pcl::visualization::PCLVisualizer view("After global alignment");
    //        view.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
    //        view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
    //        view.spin();
    //    }

        //-----------------
        // LOCAL ALIGNMENT
        //-----------------
        std::cout << "Local alignment.." << std::endl;
        Eigen::Matrix4f poseLocal = findLocalAlignment(scene, object);
        pcl::transformPointCloud(*object, *object, poseLocal);
        std::cout << "Found transform after local alignment -->\n" << poseGlobal * poseLocal << std::endl;
//        // show the state of the scene and the object
//        {
//            pcl::visualization::PCLVisualizer view("After local alignment");
//            view.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
//            view.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 255, 0),"scene");
//            view.spin();
//        }

        // get time of method
        auto timeEnd = std::chrono::high_resolution_clock::now();
        std::chrono::seconds time = std::chrono::duration_cast<std::chrono::seconds>(timeEnd - timeStart);
        std::cout << "Execution time: " << time.count() << std::endl;

//        rw::math::Transform3D<> poseEstimation = matrix4f2transform(poseGlobal) * matrix4f2transform(poseLocal);
//        std::cout << "Transform found -->\n" << poseEstimation << std::endl;

        // store data in vectors
        globalPoses.push_back(matrix4f2transform(poseGlobal));
        localPoses.push_back(matrix4f2transform(poseLocal));
        times.push_back(time);

//        // save pcd file of scene
//        const std::string fileName = "../../data/point_cloud_estimations/Scanner25D_" + std::to_string(i) + ".pcd";
//        PointCloudT initialScene;
//        pcl::io::loadPCDFile(path, initialScene);
//        std::vector<Eigen::Matrix4f> Ts = {
//            poseGlobal,
//            poseLocal
//        };
//        saveSceneWithObject(Ts, initialScene, fileName);
    }

    // save data
    writeData2File(globalPoses, localPoses, times);
    writeTransforms2File(getTransform("Table"), getTransform("WORLD"), getTransform("Scanner25D"));

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
//---------------------------------------------------------
