#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>

#include "csv_reader.hpp"

/********************/
/*  TRIANGLE CLASS  */
/********************/
class Triangle {
public:

    explicit Triangle(const cv::Point3d &V0, const cv::Point3f &V1, const cv::Point3f &V2) : _v0(V0), _v1(V1), _v2(V2) {}
    virtual ~Triangle() {}

    cv::Point3f getV0() const { return _v0; }
    cv::Point3f getV1() const { return _v1; }
    cv::Point3f getV2() const { return _v2; }

private:
    // the three vertices that defines the triangle
    cv::Point3f _v0, _v1, _v2;
};

/****************/
/*  RAY CLASS   */
/****************/
class Ray {
public:

    explicit Ray(const cv::Point3f &P0, const cv::Point3f &P1) : _p0(P0), _p1(P1) {}
    virtual ~Ray();

    cv::Point3f getP0() { return _p0; }
    cv::Point3f getP1() { return _p1; }

private:
    // the two points that defines the ray
    cv::Point3f _p0, _p1;
};

/************************/
/*  OBJECT MESH CLASS   */
/************************/
class Mesh {
public:

    Mesh() : _numVertices(0), _numTriangles(0), _listVertex(0), _listTriangles(0) {}
    virtual ~Mesh();

    std::vector<std::vector<int>> getTrianglesList() const { return _listTriangles; }
    cv::Point3f getVertex(int pos) const { return _listVertex[pos]; }
    int getNumberVertices() const { return _numVertices; }

    void load(const std::string &pathFile) {
        // create the reader
        CsvReader csvReader(pathFile);

        // clear previous data
        _listVertex.clear();
        _listTriangles.clear();

        // read from .ply file
        csvReader.readPLY(_listVertex, _listTriangles);

        // update mesh attributes
        _numVertices = (int)_listVertex.size();
        _numTriangles = (int)_listTriangles.size();
    }

private:
    // the current number of vertices in the mesh
    int _numVertices;
    // the current number of triangles in the mesh
    int _numTriangles;
    // the list of triangles of the mesh
    std::vector<cv::Point3f> _listVertex;
    // the list of triangles of the mesh
    std::vector<std::vector<int>> _listTriangles;
};
