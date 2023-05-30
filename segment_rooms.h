#pragma once

#ifndef SEGMENT_ROOMS_H
#define SEGMENT_ROOMS_H

#include <mapFileAnalysis.h>

#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <stack>
#include <utility>

class Room {
private:
    int room_id;//������
    std::vector<std::pair<int, int>> pixels;//�����ڵ����ص�
    std::vector<std::pair<int, std::pair<std::pair<int, int>, std::pair<int, int>>>> connected_rooms;//��������ŵ���ͨ��Ϣ

public:
    Room(int room_id);//���캯��

    void add_pixel(std::pair<int, int> pixel);//������ص�

    int get_pixel_count() const;//��ȡ���ص�����

    std::string to_string() const;//���������Ϣ

    int get_room_id() const;//��ȡ������

    const std::vector<std::pair<int, int>>& get_pixels() const;//��ȡ�����ڵ����ص�

    void add_connected_room(int room_id, const std::pair<std::pair<int, int>, std::pair<int, int>>& door);//��ӷ�������ŵ���ͨ��Ϣ

    void print_connected_rooms() const;//�����������ŵ���ͨ��Ϣ
};


//�ж����ص��Ƿ���Ч
bool is_valid_pixel(int x, int y, int rows, int cols);

//�ҳ����е������ص�
std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2);

//����ָ��
std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//����ͨ����ת��Ϊ����ͨ����
//cv::Mat extract_filled_image(const cv::Mat& connected_region);
std::vector<std::vector<int>> extract_filled_image(const std::vector<std::vector<int>>& connected_region);

//ȫ����������ȡ����
//cv::Mat extract_edges(const cv::Mat& filled_image);
std::vector<std::vector<int>> extract_edges(const std::vector<std::vector<int>>& filled_image);

//�Զ������ͺ���
std::vector<std::vector<int>> customize_dilate(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//�Զ��帯ʴ����
std::vector<std::vector<int>> customize_erode(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//�Զ��忪���㺯��
std::vector<std::vector<int>> customize_opening(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//�Զ�������㺯��
std::vector<std::vector<int>> customize_closing(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel);

//�����ص��б�ת��Ϊ01����
std::vector<std::vector<int>> pixels_to_matrix(const std::vector<std::pair<int, int>>& pixels, int height, int width);

//������ͨ���жϺ���
void find_connected_rooms(const std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

#endif // !SEGMENT_ROOMS_H

