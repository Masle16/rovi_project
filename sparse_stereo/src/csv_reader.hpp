#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>

#include "util.hpp"

class CsvReader {
public:

    CsvReader(const std::string &path, char seperator=' ') {
        _file.open(path.c_str(), std::ifstream::in);
        _seperator = seperator;
    }

    void readPLY(std::vector<cv::Point3f> &listVertex, std::vector<std::vector<int>> &listTriangles) {
        std::string line, tmpStr, n;
        int numVertex = 0, numTriangles = 0, cnt = 0;
        bool endHeader = false, endVertex = false;

        // read the .ply file
        while (std::getline(_file, line)) {
            std::stringstream lineStream(line);

            // read header
            if (!endHeader) {
                std::getline(lineStream, tmpStr, _seperator);
                if (tmpStr == "element") {
                    std::getline(lineStream, tmpStr, _seperator);
                    std::getline(lineStream, n);

                    if (tmpStr == "vertex") { numVertex = string2Int(n); }
                    if (tmpStr == "face") { numTriangles = string2Int(n); }
                }
                if (tmpStr == "end_header") { endHeader = true; }
            }
            // read file content
            else if (endHeader) {
                // read vertex and add into 'listVertex'
                if (!endVertex && cnt < numVertex) {
                    std::string x, y, z;
                    std::getline(lineStream, x, _seperator);
                    std::getline(lineStream, y, _seperator);
                    std::getline(lineStream, z, _seperator);

                    cv::Point3f pnt;
                    pnt.x = (float)string2Int(x);
                    pnt.y = (float)string2int(y);
                    pnt.z = (float)string2Int(z);
                    listVertex.push_back(pnt);

                    cnt++;
                    if (cnt == numVertex) {
                        cnt = 0;
                        endVertex = !endVertex;
                    }
                }
            }
            // read faces and add into 'listTriangles'
            else if (endVertex && cnt < numTriangles) {
                string numPtsPerFace, id0, id1, id2;
                std::getline(lineStream, numPtsPerFace, _seperator);
                std::getline(lineStream, id0, _seperator);
                std::getline(lineStream, id1, _seperator);
                std::getline(lineStream, id2);

                std::vector<int> triangle(3);
                triangle[0] = string2Int(id0);
                triangle[1] = string2Int(id1);
                triangle[2] = string2Int(id2);

                cnt++;
            }
        }
    }

private:
    // the current stream file for the reader
    std::ifstream _file;
    // the seperator char between words for each line
    char _seperator;
};
