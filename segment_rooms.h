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
        int room_id;//������
        std::vector<std::pair<int, int>> pixels;//���淿���ڵ����ص��б�


    public:
        Room(int room_id);
        void add_pixel(std::pair<int, int> pixel);//������ص�
        int get_pixel_count();//��ȡ���ص�����
        std::string to_string();//���������Ϣ

        int get_room_id() const //��ȡ������
        {
            return room_id;
        }

        const std::vector<std::pair<int, int>>& get_pixels() const//��ȡ���ص��б�
        {
            return pixels;
	    }
};

//�ж����ص��Ƿ���Ч
bool is_valid_pixel(int x, int y, int rows, int cols);

//�ҳ����е������ص�
std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2);

//����ָ��
std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//����ͨ����ת��Ϊ����ͨ����
cv::Mat extract_filled_image(const cv::Mat& connected_region);

//ȫ����������ȡ����
cv::Mat extract_edges(const cv::Mat& filled_image);

#endif // !SEGMENT_ROOMS_H

