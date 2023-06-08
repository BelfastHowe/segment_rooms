#pragma once

#ifndef SEGMENT_ROOMS_H
#define SEGMENT_ROOMS_H

#include <mapFileAnalysis.h>


#include <string>
#include <array>
#include <cmath>
#include <stack>
#include <utility>
#include <iterator>


//多连通区域转化为单连通区域
//cv::Mat extract_filled_image(const cv::Mat& connected_region);
std::vector<std::vector<int>> extract_filled_image(const std::vector<std::vector<int>>& connected_region);

//全部外轮廓提取函数
//cv::Mat extract_edges(const cv::Mat& filled_image);
std::vector<std::vector<int>> extract_edges(const std::vector<std::vector<int>>& filled_image);



class Room {
private:
    int room_id;//房间编号
    std::vector<std::pair<int, int>> pixels;//房间内的像素点
    std::vector<std::pair<int, std::pair<std::pair<int, int>, std::pair<int, int>>>> connected_rooms;//房间关于门的连通信息
    std::vector<std::pair<int, int>> outline_pixels;//外轮廓像素点列表

public:
    Room(int room_id);//构造函数

    void add_pixel(std::pair<int, int> pixel);//添加像素点

    int get_pixel_count() const;//获取像素点数量

    std::string to_string() const;//输出房间信息

    int get_room_id() const;//获取房间编号

    const std::vector<std::pair<int, int>>& get_pixels() const;//获取房间内的像素点

    void add_connected_room(int room_id, const std::pair<std::pair<int, int>, std::pair<int, int>>& door);//添加房间关于门的连通信息

    void print_connected_rooms() const;//输出房间关于门的连通信息

    void calculate_outline(const std::vector<std::vector<int>>& matrix);//计算并保存外轮廓像素点

    const std::vector<std::pair<int, int>>& get_outline_pixels() const;//获取外轮廓像素点列表
};


//将uint8_t矩阵转化为int矩阵
std::vector<std::vector<int>> ConvertMatrixToInt(const std::vector<std::vector<uint8_t>>& uint8_matrix);

//判断像素点是否有效
bool is_valid_pixel(int x, int y, int rows, int cols);

//找出所有的门像素点
std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2);

//房间分割函数
std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//自定义膨胀函数
std::vector<std::vector<int>> customize_dilate(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//自定义腐蚀函数
std::vector<std::vector<int>> customize_erode(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//自定义开运算函数
std::vector<std::vector<int>> customize_opening(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//自定义闭运算函数
std::vector<std::vector<int>> customize_closing(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//将像素点列表转化为01矩阵
std::vector<std::vector<int>> pixels_to_matrix(const std::vector<std::pair<int, int>>& pixels, int height, int width);

//房间连通性判断函数
//void find_connected_rooms(const std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);
void find_connected_rooms(std::vector<Room>& rooms, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//凹角膨胀函数
std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms);

//成品地图绘制
void draw_map(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& expanded_rooms, std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//骨架细化算法具体实现
void thinningIteration(std::vector<std::vector<int>>& img, int iter);

//Zhange-Suen骨架化算法
void zhangSuenThinning(std::vector<std::vector<int>>& img);


#endif // !SEGMENT_ROOMS_H

