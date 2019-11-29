/* RoVi Final Project
 * Simulated Depth Sensor
 */

#include "alignment.hpp"
#include "util.hpp"

#define SCENE_PATH "../../scanner25D_point_clouds/Scanner25D_"
#define OBJECT_PATH "../../../point_clouds_of_objects/rubber_duck.pcd"

float NOISE = 0.0f;

/** Main entry point
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

    // data to save for every analysis
    std::vector<Eigen::Matrix4f> poses;
    std::vector<double> times;
    for (std::size_t i = 0; i < 30; i++) {
        const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
//        const std::string path = "../../scanner25D_point_clouds/Scanner25D_0.pcd";
        std::cout << "Processing file: " << path << " number " << i << " / 30" << std::endl;

        // load the point cloud
        PointCloudT::Ptr scene(new PointCloudT);
        pcl::io::loadPCDFile(path, *scene);

        // load the generated object point cloud
        PointCloudT::Ptr object(new PointCloudT);
        pcl::io::loadPCDFile(OBJECT_PATH, *object);
        voxelGrid(object, object, 0.005f);
        //add noise to Scanner25D point cloud
        scene = addGaussianNoise(scene, NOISE); // noise analysis

        Mat poseGlobal, poseLocal;
        std::cout << "Performing alignment.." << std::endl;
        {
            pcl::ScopeTime t("Execution time");
            //==========================
            // point cloud preprocessing
            //==========================
            spatialFilter(scene, scene);
            //smoothing(scene, scene);
            scene = planarSegmentation(scene);
            //outlierRemoval(scene, scene, 10.0f, 1.0f);
            voxelGrid(scene, scene, 0.005f);
//          scene = euclideanCusterExtraction(scene);
            //=========================
            // pose estimation 3D to 3D
            //=========================
            poseGlobal = computeGlobalPose(scene, object);
            pcl::transformPointCloud(*object, *object, poseGlobal);
            poseLocal = findLocalAlignment(scene, object);
            pcl::transformPointCloud(*object, *object, poseLocal);
            times.push_back(t.getTime());
        }
        // print esitmate pose
        Eigen::Matrix4f result = poseLocal * poseGlobal;
        poses.push_back(result);
        std::cout << "Estimated transformation -->" << std::endl;
        std::cout << "\t"   << result(0,0) << " " << result(0,1) << " " << result(0,2) << "\n"
                  << "R =\t"<< result(1,0) << " " << result(1,1) << " " << result(1,2) << "\n"
                  << "\t"   << result(2,0) << " " << result(2,1) << " " << result(2,2) << std::endl;
        std::cout << "P =\t"<< result(0,3) << " " << result(1,3) << " " << result(2,3) << std::endl;
        // show the state of the scene and the object
        {
            PointCloudT::Ptr origin(new PointCloudT);
            const std::string path = "../../scanner25D_point_clouds/Scanner25D_" + std::to_string(i) + ".pcd";
            //const std::string path = "../../scanner25D_point_clouds/Scanner25D_0.pcd";
            pcl::io::loadPCDFile(path, *origin);
            spatialFilter(origin, origin);
            pcl::visualization::PCLVisualizer view("After alignment in origin scene");
            view.addPointCloud<PointT>(object, ColorHandlerT(object,255,0,0), "object");
            view.addPointCloud<PointT>(origin, ColorHandlerT(origin,0,255,0), "origin");
            view.spin();
        }
    }

    //==============
    // EVALUATING
    //==============
    std::cout << "\nEvaluating method -->\n" << std::endl;
    std::vector<float> diffPosition, diffAngle;
    Mat tablePose = loadTablePose();
    std::cout << "Table pose inverse -->\n" << tablePose.inverse() << "\n" << std::endl;
    Mat scannerPose = loadScannerPose();
    std::cout << "Scanner pose -->\n" << scannerPose << "\n" << std::endl;
    std::vector<Mat> realPoses = getRealPoses();
    for (std::size_t i = 0; i < poses.size(); i++) {
        std::cout << "Real pose -->\n" << realPoses[i] << "\n" << std::endl;
        Mat estimatedPose = tablePose.inverse() * scannerPose * poses[i];
        std::cout << "Estimated pose -->\n" << estimatedPose << "\n" << std::endl;

    }

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
//---------------------------------------------------------
