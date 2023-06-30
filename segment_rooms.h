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

//using namespace std;


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


struct Line //各种线的结构体
{
    enum Direction { HORIZONTAL, VERTICAL, NONLINEAR, INTERSECTION };//水平线，垂直线，非线性，交点

    int id; //线的编号
    Direction direction; //线的方向或者非线性
    std::vector<std::pair<int, int>> points; //线上的点
    std::pair<int, int> startPoint; //线的起点
    std::pair<int, int> endPoint; //线的终点

    //带有默认值的Line结构体构造函数
    Line(int id = 0, Direction direction = NONLINEAR, std::vector<std::pair<int, int>> points = {}, std::pair<int, int> startPoint = std::make_pair(-1, -1), std::pair<int, int> endPoint = std::make_pair(-1, -1))
        : id(id), direction(direction), points(points), startPoint(startPoint), endPoint(endPoint) {}
};

struct Node //优化不规则线条时需要用到的节点路径结构体
{
    std::pair<int, int> pos; // 当前位置
    std::vector<std::pair<int, int>> path; // 从起点到当前位置的路径
    int turns; // 沿途转弯的次数

    // 节点构造函数
    Node(const std::pair<int, int>& pos, const std::vector<std::pair<int, int>>& path, int turns)
        : pos(pos), path(path), turns(turns) {}

    // 优先级队列的比较运算符
    bool operator<(const Node& rhs) const 
    {
        return turns > rhs.turns; // 注意：这将使最小的turns获得最高的优先权
    }
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
void draw_map(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms, std::vector<Room>& expanded_rooms, std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//骨架细化算法具体实现
void thinningIteration(std::vector<std::vector<int>>& img, int iter);

//Zhange-Suen骨架化算法
void zhangSuenThinning(std::vector<std::vector<int>>& img);

//去除骨架中的单像素分支
void removeBranches(std::vector<std::vector<int>>& img);

//提取交点并置为背景
std::vector<Line> extractIntersections(std::vector<std::vector<int>>& img);

//提取正交线段，并将提取过的像素点置为0
void extractOrthogonalLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines);

//提取非线性线条，并将提取过的像素点置为0
void findNonLinearLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines);

//获取方向
int getDirection(std::pair<int, int> a, std::pair<int, int> b);

//寻找mask限制下两端点之间的最少转折线段
std::vector<std::pair<int, int>> getLeastTurnPath(const std::pair<int, int>& start, const std::pair<int, int>& end, const std::vector<std::vector<int>>& mask);

//户型图轮廓的拆分、找直与重组
std::vector<std::vector<int>> floor_plan_outline_Orthogonalization(std::vector<std::vector<int>>& img, const std::vector<std::vector<int>>& segmented_matrix);

//户型图轮廓的八连通连接处补全
void completion_link(std::vector<std::vector<int>>& floor_plan_matrix);

//判断单个像素点与单个门位置关系的函数
std::pair<double, bool> distanceToSegment(const std::pair<int, int>& point, const std::pair<int, int>& segA, const std::pair<int, int>& segB);

//规整房间生成中，判断像素点是否应该被腐蚀
bool should_not_be_eroded(const std::pair<int, int>& point, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors, double threshold);

//腐蚀后规整房间的id标明、扩散
void tidyRoomDFS(int& x, int& y, int& h, int& w, std::vector<std::vector<int>>& tidy_room, int& id);

//户型图轮廓腐蚀版，规整房间生成集成函数
std::vector<std::vector<int>> tidy_room_erode(std::vector<std::vector<int>>& segmented_matrix,
                                              std::vector<std::vector<int>>& floor_plan,
                                              const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//原房间矩阵膨胀版，规整房间生成函数
std::vector<std::vector<int>> tidy_room_dilate(std::vector<std::vector<int>>& room_matrix, std::vector<std::vector<int>>& floor_plan_matrix, int rounds);

//单连通区域的多边形拟合
void polygon_fitting(std::vector<std::vector<int>>& room_matrix, double epsilon);

//规整房间的拟合生成
std::vector<std::vector<int>> tidy_room_approx(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms);

//去除二值图像边缘的突出部，对宽度小于阈值的正交方向上的部分予以删除
void delete_jut(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int uthreshold, int vthreshold);

//填补二值图像边缘的凹陷，对宽度小于阈值的正交方向上的空缺予以填补
void fill_hollow(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int uthreshold, int vthreshold);

//自定义均值滤波，二值图像
void customize_blur(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int windowSize, int threshold);

//全房间二值图像处理函数
void tidy_room_binary(std::vector<std::vector<int>>& src, std::vector<Room>& rooms);


/*
@brief 规整房间生成，条件形态学变换
*/
void tidy_room_Conditional_Morphological_Transformation(std::vector<std::vector<int>>& src,
                                                        std::vector<std::vector<int>>& mask,
                                                        std::vector<Room>& rooms,
                                                        const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//条件腐蚀，只腐蚀经典锯齿边上的单个像素凸起
void tidy_room_Conditional_Erosion_Transformation(std::vector<std::vector<int>>& src,
                                                  std::vector<std::vector<int>>& dst,
                                                  const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//条件膨胀，主要是填补单排像素凸出
void tidy_room_Conditional_Dilation_Transformation(std::vector<std::vector<int>>& src,
                                                   std::vector<std::vector<int>>& dst,
                                                   std::vector<std::vector<int>>& mask);

#endif // !SEGMENT_ROOMS_H

