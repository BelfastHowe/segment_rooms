#pragma once

#ifndef GENERATE_CONNECTED_REGION_H
#define GENERATE_CONNECTED_REGION_H


#include <random>
#include <algorithm>
#include <segment_rooms.h>
#include <iomanip>

//私有打印
int imwrite_mdy_private(cv::InputArray input, const std::string file_name);

//随机地图生成函数
std::vector<std::vector<int>> generate_connected_region(int length, int width, int desired_area);

//打印二值图像
void printBinaryImage(const std::vector<std::vector<int>>& matrix, int scale, const char* windowName);


#endif // !GENERATE_CONNECTED_REGION_H
