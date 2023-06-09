#pragma once

#ifndef MAP_FILE_ANALYSIS_H
#define MAP_FILE_ANALYSIS_H

#include <cstdint>
#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

// 函数声明：读取地图文件并转化为01矩阵
std::vector<std::vector<uint8_t>> readMapFile(const char* filename);

// 函数声明：将01矩阵转化为二值图像并打印
void printBinaryMatrix(const std::vector<std::vector<uint8_t>>& binaryMatrix);

#endif  // MAP_FILE_ANALYSIS_H

