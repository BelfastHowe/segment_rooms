#pragma once

#ifndef MAP_FILE_ANALYSIS_H
#define MAP_FILE_ANALYSIS_H

#include <cstdint>
#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

// 函数声明：读取地图文件并转化为01矩阵,高置信度
std::vector<std::vector<uint8_t>> readMapFile(const char* filename);

//读取地图文件并转化为01矩阵,高置信度+低置信度
std::vector<std::vector<uint8_t>> readMapFile_0x81(const char* filename);

// 函数声明：将01矩阵转化为二值图像并打印
void printBinaryMatrix(const std::vector<std::vector<uint8_t>>& binaryMatrix);

//png读取
std::vector<std::vector<int>> ConvertImageToMatrix(const std::string& imagePath);

#endif  // MAP_FILE_ANALYSIS_H

