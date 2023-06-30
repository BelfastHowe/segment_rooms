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


//����ͨ����ת��Ϊ����ͨ����
//cv::Mat extract_filled_image(const cv::Mat& connected_region);
std::vector<std::vector<int>> extract_filled_image(const std::vector<std::vector<int>>& connected_region);

//ȫ����������ȡ����
//cv::Mat extract_edges(const cv::Mat& filled_image);
std::vector<std::vector<int>> extract_edges(const std::vector<std::vector<int>>& filled_image);



class Room {
private:
    int room_id;//������
    std::vector<std::pair<int, int>> pixels;//�����ڵ����ص�
    std::vector<std::pair<int, std::pair<std::pair<int, int>, std::pair<int, int>>>> connected_rooms;//��������ŵ���ͨ��Ϣ
    std::vector<std::pair<int, int>> outline_pixels;//���������ص��б�

public:
    Room(int room_id);//���캯��

    void add_pixel(std::pair<int, int> pixel);//������ص�

    int get_pixel_count() const;//��ȡ���ص�����

    std::string to_string() const;//���������Ϣ

    int get_room_id() const;//��ȡ������

    const std::vector<std::pair<int, int>>& get_pixels() const;//��ȡ�����ڵ����ص�

    void add_connected_room(int room_id, const std::pair<std::pair<int, int>, std::pair<int, int>>& door);//��ӷ�������ŵ���ͨ��Ϣ

    void print_connected_rooms() const;//�����������ŵ���ͨ��Ϣ

    void calculate_outline(const std::vector<std::vector<int>>& matrix);//���㲢�������������ص�

    const std::vector<std::pair<int, int>>& get_outline_pixels() const;//��ȡ���������ص��б�
};


struct Line //�����ߵĽṹ��
{
    enum Direction { HORIZONTAL, VERTICAL, NONLINEAR, INTERSECTION };//ˮƽ�ߣ���ֱ�ߣ������ԣ�����

    int id; //�ߵı��
    Direction direction; //�ߵķ�����߷�����
    std::vector<std::pair<int, int>> points; //���ϵĵ�
    std::pair<int, int> startPoint; //�ߵ����
    std::pair<int, int> endPoint; //�ߵ��յ�

    //����Ĭ��ֵ��Line�ṹ�幹�캯��
    Line(int id = 0, Direction direction = NONLINEAR, std::vector<std::pair<int, int>> points = {}, std::pair<int, int> startPoint = std::make_pair(-1, -1), std::pair<int, int> endPoint = std::make_pair(-1, -1))
        : id(id), direction(direction), points(points), startPoint(startPoint), endPoint(endPoint) {}
};

struct Node //�Ż�����������ʱ��Ҫ�õ��Ľڵ�·���ṹ��
{
    std::pair<int, int> pos; // ��ǰλ��
    std::vector<std::pair<int, int>> path; // ����㵽��ǰλ�õ�·��
    int turns; // ��;ת��Ĵ���

    // �ڵ㹹�캯��
    Node(const std::pair<int, int>& pos, const std::vector<std::pair<int, int>>& path, int turns)
        : pos(pos), path(path), turns(turns) {}

    // ���ȼ����еıȽ������
    bool operator<(const Node& rhs) const 
    {
        return turns > rhs.turns; // ע�⣺�⽫ʹ��С��turns�����ߵ�����Ȩ
    }
};



//��uint8_t����ת��Ϊint����
std::vector<std::vector<int>> ConvertMatrixToInt(const std::vector<std::vector<uint8_t>>& uint8_matrix);

//�ж����ص��Ƿ���Ч
bool is_valid_pixel(int x, int y, int rows, int cols);

//�ҳ����е������ص�
std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2);

//����ָ��
std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

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
//void find_connected_rooms(const std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);
void find_connected_rooms(std::vector<Room>& rooms, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//�������ͺ���
std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms);

//��Ʒ��ͼ����
void draw_map(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms, std::vector<Room>& expanded_rooms, std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels);

//�Ǽ�ϸ���㷨����ʵ��
void thinningIteration(std::vector<std::vector<int>>& img, int iter);

//Zhange-Suen�Ǽܻ��㷨
void zhangSuenThinning(std::vector<std::vector<int>>& img);

//ȥ���Ǽ��еĵ����ط�֧
void removeBranches(std::vector<std::vector<int>>& img);

//��ȡ���㲢��Ϊ����
std::vector<Line> extractIntersections(std::vector<std::vector<int>>& img);

//��ȡ�����߶Σ�������ȡ�������ص���Ϊ0
void extractOrthogonalLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines);

//��ȡ������������������ȡ�������ص���Ϊ0
void findNonLinearLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines);

//��ȡ����
int getDirection(std::pair<int, int> a, std::pair<int, int> b);

//Ѱ��mask���������˵�֮�������ת���߶�
std::vector<std::pair<int, int>> getLeastTurnPath(const std::pair<int, int>& start, const std::pair<int, int>& end, const std::vector<std::vector<int>>& mask);

//����ͼ�����Ĳ�֡���ֱ������
std::vector<std::vector<int>> floor_plan_outline_Orthogonalization(std::vector<std::vector<int>>& img, const std::vector<std::vector<int>>& segmented_matrix);

//����ͼ�����İ���ͨ���Ӵ���ȫ
void completion_link(std::vector<std::vector<int>>& floor_plan_matrix);

//�жϵ������ص��뵥����λ�ù�ϵ�ĺ���
std::pair<double, bool> distanceToSegment(const std::pair<int, int>& point, const std::pair<int, int>& segA, const std::pair<int, int>& segB);

//�������������У��ж����ص��Ƿ�Ӧ�ñ���ʴ
bool should_not_be_eroded(const std::pair<int, int>& point, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors, double threshold);

//��ʴ����������id��������ɢ
void tidyRoomDFS(int& x, int& y, int& h, int& w, std::vector<std::vector<int>>& tidy_room, int& id);

//����ͼ������ʴ�棬�����������ɼ��ɺ���
std::vector<std::vector<int>> tidy_room_erode(std::vector<std::vector<int>>& segmented_matrix,
                                              std::vector<std::vector<int>>& floor_plan,
                                              const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//ԭ����������Ͱ棬�����������ɺ���
std::vector<std::vector<int>> tidy_room_dilate(std::vector<std::vector<int>>& room_matrix, std::vector<std::vector<int>>& floor_plan_matrix, int rounds);

//����ͨ����Ķ�������
void polygon_fitting(std::vector<std::vector<int>>& room_matrix, double epsilon);

//����������������
std::vector<std::vector<int>> tidy_room_approx(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms);

//ȥ����ֵͼ���Ե��ͻ�������Կ��С����ֵ�����������ϵĲ�������ɾ��
void delete_jut(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int uthreshold, int vthreshold);

//���ֵͼ���Ե�İ��ݣ��Կ��С����ֵ�����������ϵĿ�ȱ�����
void fill_hollow(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int uthreshold, int vthreshold);

//�Զ����ֵ�˲�����ֵͼ��
void customize_blur(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int windowSize, int threshold);

//ȫ�����ֵͼ������
void tidy_room_binary(std::vector<std::vector<int>>& src, std::vector<Room>& rooms);


/*
@brief �����������ɣ�������̬ѧ�任
*/
void tidy_room_Conditional_Morphological_Transformation(std::vector<std::vector<int>>& src,
                                                        std::vector<std::vector<int>>& mask,
                                                        std::vector<Room>& rooms,
                                                        const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//������ʴ��ֻ��ʴ�����ݱ��ϵĵ�������͹��
void tidy_room_Conditional_Erosion_Transformation(std::vector<std::vector<int>>& src,
                                                  std::vector<std::vector<int>>& dst,
                                                  const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels);

//�������ͣ���Ҫ�����������͹��
void tidy_room_Conditional_Dilation_Transformation(std::vector<std::vector<int>>& src,
                                                   std::vector<std::vector<int>>& dst,
                                                   std::vector<std::vector<int>>& mask);

#endif // !SEGMENT_ROOMS_H

