#pragma once

#ifndef GENERATE_CONNECTED_REGION_H
#define GENERATE_CONNECTED_REGION_H


#include <random>
#include <algorithm>
#include <segment_rooms.h>


//�����ͼ���ɺ���
std::vector<std::vector<int>> generate_connected_region(int length, int width, int desired_area);

//��ӡ��ֵͼ��
void printBinaryImage(const std::vector<std::vector<int>>& matrix, int scale);


#endif // !GENERATE_CONNECTED_REGION_H
