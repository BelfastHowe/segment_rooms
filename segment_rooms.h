#pragma once

#ifndef SEGMENT_ROOMS_H
#define SEGMENT_ROOMS_H

#include <mapFileAnalysis.h>

#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <stack>

class Room 
{
    private:
        int room_id;//房间编号
        std::vector<std::pair<int, int>> pixels;//储存房间内的像素点列表


    public:
        Room(int room_id);
        void add_pixel(std::pair<int, int> pixel);//添加像素点
        int get_pixel_count();//获取像素点数量
        std::string to_string();//输出房间信息

        int get_room_id() const //获取房间编号
        {
            return room_id;
        }

        const std::vector<std::pair<int, int>>& get_pixels() const//获取像素点列表
        {
            return pixels;
	    }
};

//判断像素点是否有效
bool is_valid_pixel(int x, int y, int rows, int cols);

//找出所有的门像素点
std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2);

//房间分割函数
std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//多连通区域转化为单连通区域
cv::Mat extract_filled_image(const cv::Mat& connected_region);

//全部外轮廓提取函数
cv::Mat extract_edges(const cv::Mat& filled_image);

#endif // !SEGMENT_ROOMS_H

