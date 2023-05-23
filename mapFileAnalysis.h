#pragma once

#ifndef MAP_FILE_ANALYSIS_H
#define MAP_FILE_ANALYSIS_H

#include <cstdint>
#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

// ������������ȡ��ͼ�ļ���ת��Ϊ01����
std::vector<std::vector<uint8_t>> readMapFile(const char* filename);

// ������������01����ת��Ϊ��ֵͼ�񲢴�ӡ
void printBinaryMatrix(const std::vector<std::vector<uint8_t>>& binaryMatrix);

#endif  // MAP_FILE_ANALYSIS_H

