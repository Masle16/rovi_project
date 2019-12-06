/* Image based method
 *
 */

//--------------------------------------
// INCLUDES
#include <eigen3/Eigen/Eigen>
#include <covis/covis.h>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/linemod.hpp>

//--------------------------------------

// DEFINES

//--------------------------------------
// TYPDEDEFS

//--------------------------------------
// FUNCTIONS

/**
 * @brief createLinemodDetector
 * @return
 */
cv::linemod::Detector createLinemodDetector() {
    std::vector<cv::Ptr<cv::linemod::Modality>> modalities = {
//        cv::linemod::Modality::create("DepthNormal")
        cv::linemod::Modality::create("ColorGradient")
    };
    std::vector<int> pyramids = { 4, 2, 1 };
    cv::linemod::Detector detector(modalities, pyramids);
    return detector;
}

/**
 * @brief isBorder
 * @param edge
 * @param color
 * @return
 */
inline bool isBorder(cv::Mat& edge, cv::Vec3b color) {
    bool result = true;
    cv::Mat image = edge.clone().reshape(0,1);
    for (int i = 0; i < image.cols; i++) {
        result &= (color == image.at<cv::Vec3b>(0,i));
    }
    return result;
}

/**
 * @brief autocrop
 * @param src
 * @return
 */
inline cv::Rect autocrop(cv::Mat& src) {
    COVIS_ASSERT(src.type() == CV_8UC3);
    cv::Rect window(0, 0, src.cols, src.rows);

    std::vector<cv::Rect> edges;
    edges.push_back(cv::Rect(0, 0, src.cols, 1));
    edges.push_back(cv::Rect(src.cols-2, 0, 1, src.rows));
    edges.push_back(cv::Rect(0, src.rows-2, src.cols, 1));
    edges.push_back(cv::Rect(0, 0, 1, src.rows));

    cv::Mat edge;
    int nBorder = 0;
    cv::Vec3b color = src.at<cv::Vec3b>(0,0);
    for (size_t i = 0; i < edges.size(); i++) {
        edge = src(edges[i]);
        nBorder += isBorder(edge, color);
    }

    if (nBorder < 4) { return window; }

    bool next;
    do {
      edge = src(cv::Rect(window.x, window.height-2, window.width, 1));
      if( (next = isBorder(edge, color)) )
        window.height--;
    } while (next && window.height > 0);

    do {
      edge = src(cv::Rect(window.width-2, window.y, 1, window.height));
      if( (next = isBorder(edge, color)) )
        window.width--;
    } while (next && window.width > 0);

    do {
      edge = src(cv::Rect(window.x, window.y, window.width, 1));
      if( (next = isBorder(edge, color)) )
        window.y++, window.height--;
    } while (next && window.y <= src.rows);

    do {
      edge = src(cv::Rect(window.x, window.y, 1, window.height));
      if( (next = isBorder(edge, color)) )
        window.x++, window.width--;
    } while (next && window.x <= src.cols);

    return window;
}


//--------------------------------------
// MAIN ENTRY POINT
int main(int argc, const char** argv) {
    std::cout << "\nProgram started\n" << std::endl;

    // setup program options
    covis::core::ProgramOptions po;
    po.addPositional("template", "folder containing template image(s) and pose(s)");
    po.addPositional("image", "test image(s)");
    po.addOption("threshold", 't', 50, "if positive, accept all detections up to this threshold");
    if (!po.parse(argc, argv)) { return 1; }

    // get the values from the program options
    const std::vector<std::string> imgPath = po.getVector("image");
    const std::string tempsPath = po.getValue("template");
    const float threshold = po.getValue<float>("threshold");

    // training data to be loaded for the 2D matcher
    std::vector<cv::Mat> temps;
    std::vector<Eigen::Matrix4f> poses;
    std::vector<cv::Point> offset;
    int index = 0;
    cv::linemod::Detector detector = createLinemodDetector();
    while(true) {
        // get rgb template
        char tempFile[1024];
        std::sprintf(tempFile, "/template%04i.png", index);
        cv::Mat tempImg = cv::imread(tempsPath + std::string(tempFile),
                                      cv::IMREAD_UNCHANGED);
        if (tempImg.empty()) { break; }
        temps.push_back(tempImg.clone());

        cv::Mat mask;
        cv::Rect window = autocrop(tempImg);
        window.height = window.height + 4;
        window.width = window.width + 4;
        window.x = window.x - 2;
        window.y = window.y - 2;
        tempImg = tempImg(window);
        offset.push_back(cv::Point(window.width, window.height));
        cv::inRange(tempImg, cv::Scalar(0,0,244), cv::Scalar(1,1,255), mask);
        mask = 255 - mask;

        // show template and mask
//        cv::imshow("template", tempImg);
//        cv::imshow("mask", mask);
//        cv::waitKey(0);

        std::vector<cv::Mat> sources;
        sources.push_back(tempImg.clone());
        std::sprintf(tempFile, "%04i", index);

        // insert templates into the detector
        detector.addTemplate(sources, std::string(tempFile), mask);
        index++;
    }

    std::cout << "Number of templates: " << temps.size() << std::endl;
    for (int imgIdx = 0; imgIdx < int(imgPath.size()); imgIdx++) {
        cv::Mat img = cv::imread(imgPath[imgIdx], cv::IMREAD_UNCHANGED);
        COVIS_ASSERT_MSG(!img.empty(), "Cannot read image " << po.getValue("image"));

        cv::Mat _img = img.clone();
        std::vector<cv::Mat> sources;
        sources.push_back(_img);
        std::vector<cv::linemod::Match> matches;
        detector.match(sources, threshold, matches);
        std::cout << "Number of matches: " << matches.size() << std::endl;
        int i = 0;
        cv::imshow("template", temps[atoi(matches[i].class_id.c_str())]);
        int x = matches[i].x+(offset[i].x/2), y = matches[i].y+(offset[i].y/2);
        cv::circle(img, cv::Point(x,y), 8, cv::Scalar(0, 255, 0), -1);

        char poseFile[1024];
        std::sprintf(poseFile, "template%04i_pose.txt", atoi(matches[i].class_id.c_str()));
        Eigen::Matrix4f matrix;
        covis::util::loadEigen(tempsPath + std::string(poseFile), matrix);
        std::cout << matrix << std::endl;

        cv::imshow("Image", img);
        cv::waitKey(0);
    }

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
