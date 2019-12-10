#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mesh.hpp"

static cv::Point3f cross(cv::Point3f v1, cv::Point3f v2) {
    cv::Point3f pnt;
    pnt.x = v1.y * v2.z - v1.z * v2.y;
    pnt.y = v1.z * v2.x - v1.x * v2.z;
    pnt.z = v1.x * v2.y - v1.y * v2.x;
    return pnt;
}

static double dot(cv::Point3f v1, cv::Point3f v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

static cv::Point3f sub(cv::Point3f v1, cv::Point3f v2) {
    cv::Point3f pnt;
    pnt.x = v1.x - v2.x;
    pnt.y = v1.y - v2.y;
    pnt.z = v1.z - v2.z;
    return pnt;
}

static cv::Point3f getNearest3DPoint(std::vector<cv::Point3f> &pntsList, cv::Point3f origin) {
    cv::Point3f pnt1 = pntsList[0];
    cv::Point3f pnt2 = pntsList[1];
    double d1 = std::sqrt(std::pow(pnt1.x-origin.x,2)+std::pow(pnt1.y-origin.y,2)+std::pow(pnt1.z-origin.z,2));
    double d2 = std::sqrt(std::pow(pnt2.x-origin.x,2)+std::pow(pnt2.y-origin.y,2)+std::pow(pnt2.z-origin.z,2));
    if (d1 < d2) { return pnt1; }
    else { return pnt2; }
}

class PnPProblem {
public:

    explicit PnPProblem() {
        _aMat = cv::Mat::zeros(3,3,CV_64FC1);
        _rMat = cv::Mat::zeros(3,3,CV_64FC1);
        _tMat = cv::Mat::zeros(3,1,CV_64FC1);
        _pMat = cv::Mat::zeros(3,4,CV_64FC1);
    }
    virtual ~PnPProblem() {}

    bool backproject2DPoint(const Mesh *mesh, const cv::Point2f &pnt2D, cv::Point3f &pnt3D);
    bool intersectMollerTrumbore(Ray &R, Triangle &T, double *out);
    std::vector<cv::Point2f> verifyPoints(Mesh *mesh);
    cv::Point2f backproject3DPoint(const cv::Point3f &pnt3D);
    bool estimatePose(const std::vector<cv::Point3f> &listPnts3D, const std::vector<cv::Point2f> &listPnts2D, int flags);
    void estimatePoseRANSAC(const std::vector<cv::Point3f> &listPnts3D, const std::vector<cv::Point2f> &listPnts2D,
                            int flags, cv::Mat &inliers, int iterationsCnt, float reprojectionError, double confidence);

    cv::Mat getAMat() const { return _aMat; }
    cv::Mat getRMat() const { return _rMat; }
    cv::Mat getTMat() const { return _tMat; }
    cv::Mat getPMat() const { return _pMat; }

    void setPMat(const cv::Mat &rMat, const cv::Mat &tMat) {
        // Rotation-Translation Matrix Definition
        _pMat.at<double>(0,0) = _rMat.at<double>(0,0);
        _pMat.at<double>(0,1) = _rMat.at<double>(0,1);
        _pMat.at<double>(0,2) = _rMat.at<double>(0,2);
        _pMat.at<double>(1,0) = _rMat.at<double>(1,0);
        _pMat.at<double>(1,1) = _rMat.at<double>(1,1);
        _pMat.at<double>(1,2) = _rMat.at<double>(1,2);
        _pMat.at<double>(2,0) = _rMat.at<double>(2,0);
        _pMat.at<double>(2,1) = _rMat.at<double>(2,1);
        _pMat.at<double>(2,2) = _rMat.at<double>(2,2);
        _pMat.at<double>(0,3) = _tMat.at<double>(0);
        _pMat.at<double>(1,3) = _tMat.at<double>(1);
        _pMat.at<double>(2,3) = _tMat.at<double>(2);
    }

private:
    cv::Mat _aMat; // calibration matrix
    cv::Mat _rMat; // rotation matrix
    cv::Mat _tMat; // tranlation matrix
    cv::Mat _pMat; // projection matrix
}
