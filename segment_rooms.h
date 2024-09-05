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
#include <map>

//using namespace std;

using p64 = std::pair<int, int>;
using MatrixInt = std::vector<std::vector<int>>;

template<typename T>
using Matrix = std::vector<std::vector<T>>;



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
    std::vector<std::pair<int, p64>> connection_info;//新连通信息

    std::vector<std::pair<int, int>> outline_pixels;//外轮廓像素点列表
    std::vector<std::vector<p64>> obstacle_pixels;//障碍物列表

public:
    Room(int room_id = 0);//构造函数

    void add_pixel(std::pair<int, int> pixel);//添加像素点

    int get_pixel_count() const;//获取像素点数量

    void clear_pixels();// 清空房间内的像素点

    std::string to_string() const;//输出房间信息

    int get_room_id() const;//获取房间编号

    const std::vector<std::pair<int, int>>& get_pixels() const;//获取房间内的像素点

    void add_connected_room(int room_id, const std::pair<std::pair<int, int>, std::pair<int, int>>& door);//添加房间关于门的连通信息

    void print_connected_rooms() const;//输出房间关于门的连通信息

    void calculate_outline(const std::vector<std::vector<int>>& matrix);//计算并保存外轮廓像素点

    const std::vector<std::pair<int, int>>& get_outline_pixels() const;//获取外轮廓像素点列表

    void add_connection_info(int room_id, p64 door_id);//添加连通信息

    const std::vector<std::pair<int, p64>>& get_connection_info() const;//获取连通信息

    void delete_connection_info(int id);//删除连通信息（输入房间id来删除）

    void clear_connection_info();//清除连通信息

    const std::vector<std::vector<p64>>& get_obstacle_pixels() const;//获取障碍物列表
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

struct Node_rs //优化不规则线条时需要用到的节点路径结构体
{
    std::pair<int, int> pos; // 当前位置
    std::vector<std::pair<int, int>> path; // 从起点到当前位置的路径
    int turns; // 沿途转弯的次数

    // 节点构造函数
    Node_rs(const std::pair<int, int>& pos, const std::vector<std::pair<int, int>>& path, int turns)
        : pos(pos), path(path), turns(turns) {}

    // 优先级队列的比较运算符
    bool operator<(const Node_rs& rhs) const
    {
        if (turns == rhs.turns)
        {
            // 当转弯次数相等时，路径较短的节点有更高的优先级
            return path.size() > rhs.path.size();
        }
        else
        {
            // 否则，转弯次数少的节点有更高的优先级
            return turns > rhs.turns;
        }
    }
};

struct Door  //门框结构体
{
    std::pair<int, int> startPoint;
    std::pair<int, int> endPoint;
    std::vector<std::pair<int, int>> path;

    // 带有默认值的构造函数
    Door(std::pair<int, int> p1 = { -1, -1 }, std::pair<int, int> p2 = { -1, -1 }, std::vector<std::pair<int, int>> l = {})
        : startPoint(p1), endPoint(p2), path(l)
    {}
};



//将uint8_t矩阵转化为int矩阵
std::vector<std::vector<int>> ConvertMatrixToInt(const std::vector<std::vector<uint8_t>>& uint8_matrix);

//判断像素点是否有效
bool is_valid_pixel(int x, int y, int rows, int cols);

//找出所有的门像素点
std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2);


//四连通光栅化直线绘制
std::vector<p64> bresenham4(int x0, int y0, int x1, int y1);

//门框的规则化
int door_regularization(const Matrix<int>& current_map, std::vector<std::pair<p64, p64>>& src_doors);

//带有粘连处理的门框规则化
int door_regularization_skid(Matrix<int>& current_map, std::vector<std::pair<p64, p64>>& src_doors);

//门框字典构造
std::map<p64, Door> doorVector2Map(std::vector<std::pair<p64, p64>>& doors);

//门框干涉组件，深度优先搜索
void doorDFS(MatrixInt& matrix, std::vector<std::vector<bool>>& visited, int a, int b, int target, Door& door);

//门框干涉组件，干涉后门框字典重构
std::map<p64, Door> findDoorFrames(MatrixInt& matrix);

//门框干涉，复合id门框字典重构
void door_frame_interaction(MatrixInt& src, std::map<p64, Door>& doorMap);

//去除无关的门
int filter_unassociated_doors(std::map<int, Room>& rooms, std::map<p64, Door>& doorMap);

//多边形拟合所用的观测函数
int show_cv_points(const Matrix<cv::Point>& pointSets, int h, int w, const std::string& windowName);


//房间分割函数
std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);
std::pair<MatrixInt, std::map<int, Room>> segment_rooms(MatrixInt& src, std::map<p64, Door>& doorMap, MatrixInt& bgmask);

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
void find_connected_rooms(std::map<int, Room>& rooms, const std::map<p64, Door>& doorMap);

//凹角膨胀函数
std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms);
std::pair<Matrix<int>, std::map<int, Room>> expand_rooms(const Matrix<int>& segmented_matrix, const std::map<int, Room>& rooms);

//简化的户型图生成函数，简单膨胀
std::pair<Matrix<int>, std::map<int, Room>> expand_rooms_simple(const Matrix<int>& segmented_matrix, const std::map<int, Room>& rooms);

//凹角膨胀，使用队列思路
std::pair<Matrix<int>, std::map<int, Room>> expand_rooms_queue(const Matrix<int>& segmented_matrix, const std::map<int, Room>& rooms);

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
bool should_not_be_eroded(const p64& point, const std::map<p64, Door>& doorMap, double threshold);

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

void tidy_room_Conditional_Morphological_Transformation(Matrix<int>& src,
    Matrix<int>& mask,
    std::map<int, Room>& rooms,
    const std::map<p64, Door>& doorMap);


//条件腐蚀，只腐蚀经典锯齿边上的单个像素凸起
void tidy_room_Conditional_Erosion_Transformation(std::vector<std::vector<int>>& src,
                                                  std::vector<std::vector<int>>& dst,
                                                  const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//条件膨胀，主要是填补单排像素凸出
void tidy_room_Conditional_Dilation_Transformation(std::vector<std::vector<int>>& src,
                                                   std::vector<std::vector<int>>& dst,
                                                   std::vector<std::vector<int>>& mask);

/*
@brief 户型图总体优化模块函数
*/
void floor_plan_optimizer(std::vector<std::vector<int>>& expanded_matrix,
    std::vector<std::vector<int>>& tidy_room,
    std::vector<Room>& expanded_rooms,
    const std::vector<std::vector<int>>& segmented_matrix,
    std::vector<Room>& rooms,
    const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

int floor_plan_optimizer(Matrix<int>& expanded_matrix,
    Matrix<int>& tidy_room,
    std::map<int, Room>& expanded_rooms,
    const Matrix<int>& segmented_matrix,
    std::map<int, Room>& rooms,
    const std::map<p64, Door>& doorMap);

//简单的正交多边形近似函数，主要用于确定户型图变形边界
std::vector<std::vector<int>> orthogonal_polygon_fitting(const std::vector<std::vector<int>>& floor_plan_matrix);

//点在直线上还是直线下的相对位置判断函数，在上输出true
bool isAboveLine(const cv::Point& point_to_check, const cv::Point& line_point_1, const cv::Point& line_point_2);

//单连通域重心计算函数
cv::Point find_centroid(const cv::Mat& mat);

//用优化后的户型图轮廓矩阵反向更新expanded_rooms列表
void expanded_room_renew(std::vector<Room>& expanded_rooms, const std::vector<std::vector<int>>& segmented_matrix, const std::vector<std::vector<int>>& floor_plan_optimization_matrix);
int expanded_room_renew(std::map<int, Room>& expanded_rooms, const Matrix<int>& segmented_matrix, const Matrix<int>& floor_plan_optimization_matrix);

//户型图轮廓的阈值对齐
std::vector<std::vector<int>> floor_plan_alignment(const std::vector<Room>& expanded_rooms, const std::vector<std::vector<int>>& floor_plan_optimization_matrix);
Matrix<int> floor_plan_alignment(const std::map<int, Room>& expanded_rooms, const Matrix<int>& floor_plan_optimization_matrix);

//转折点列表转内部的单连通填充矩阵
std::vector<std::vector<int>> cvPoint_to_matrix(std::vector<cv::Point>& cnt, size_t h, size_t w);

//新的最终图像绘制
void draw_final_map(std::vector<std::vector<int>>& segmented_matrix,
    std::vector<std::vector<int>>& expanded_matrix,
    std::vector<std::vector<int>>& tidy_room,
    std::vector<Room>& expanded_rooms,
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

void draw_final_map(Matrix<int>& segmented_matrix,
    Matrix<int>& expanded_matrix,
    Matrix<int>& tidy_room,
    std::map<int, Room>& expanded_rooms,
    std::map<p64, Door>& doorMap);

//宽度为1的封闭轮廓的逆时针顺序转折点提取
std::vector<cv::Point> findTurnPoints(std::vector<std::vector<int>>& outline_matrix);

//双房间手动合并
int room_merge(int room1, int room2, std::map<int, Room>& rooms, std::map<int, Room>& expanded_rooms, std::map<p64, Door>& doorMap, Matrix<int>& segmented_matrix, Matrix<int>& expanded_matrix);

//地图更新
int map_renew(Matrix<int>& new_map,
    p64 offset_xy,
    Matrix<int>& segmented_matrix,
    std::map<int, Room>& rooms,
    //std::map<int,Room>&expanded_rooms,
    std::vector<std::pair<p64, p64>>& doors,
    std::vector<std::pair<p64, p64>>& new_doors);

//正确的后分割id分配
int post_segment(Matrix<int>& segmented_matrix, std::map<int, Room>& rooms, std::map<p64, Door>& doorMap, const Matrix<int>& bgmask);

//去除孤岛
void keepLargesComponent(Matrix<int>& image);

//填补面积小于阈值的二值图像孔洞
int fill_small_holes(Matrix<int>& src, int threshold);

//改变画布大小
int resize_matrix(Matrix<int>& src);

//预处理输入数据
Matrix<int> map_pre_optimization(const char* filename, std::vector<std::pair<p64, p64>>& doors);




#endif // !SEGMENT_ROOMS_H

