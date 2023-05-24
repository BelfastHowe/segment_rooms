#include <segment_rooms.h>


Room::Room(int room_id) 
{
    this->room_id = room_id;
}

void Room::add_pixel(std::pair<int, int> pixel) 
{
    this->pixels.push_back(pixel);
}

int Room::get_pixel_count() 
{
    return this->pixels.size();
}

std::string Room::to_string() 
{
    return "Room " + std::to_string(this->room_id) + ": " + std::to_string(this->get_pixel_count()) + " pixels";
}

bool is_valid_pixel(int x, int y, int rows, int cols) 
{
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}

std::vector<std::pair<int, int>> bresenham_line(int x1, int y1, int x2, int y2) 
{
    std::vector<std::pair<int, int>> points;
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) 
    {
        points.push_back(std::make_pair(x1, y1));
        if (x1 == x2 && y1 == y2) 
        {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) 
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) 
        {
            err += dx;
            y1 += sy;
        }
    }

    return points;
}

std::pair<std::vector<std::vector<int>>, std::vector<Room>> segment_rooms(const std::vector<std::vector<int>>& matrix, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels) 
{
    int rows = matrix.size();
    int cols = matrix[0].size();

    std::vector<std::vector<int>> segmented_matrix = matrix;//创建副本以保持原始矩阵不变
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));//创建与矩阵大小相同的访问标记矩阵
    std::vector<Room> rooms;//创建房间列表
    int room_id = 1;//房间的初始标识符

    std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };//定义DFS算法四个方向的偏移量

    std::function<void(int, int, Room&)> dfs = [&](int x, int y, Room& room) 
    {
        std::stack<std::pair<int, int>> stack;
        stack.push(std::make_pair(x, y));//将当前像素点压入栈中

        while (!stack.empty()) 
        {
            std::pair<int, int> current = stack.top();
            stack.pop();//将当前像素点弹出栈

            int current_x = current.first;
            int current_y = current.second;
            room.add_pixel(current);//将当前像素点添加到房间中

            visited[current_x][current_y] = true;//将当前像素点标记为已访问

            for (const auto& direction : directions) 
            {
                int next_x = current_x + direction.first;
                int next_y = current_y + direction.second;

                if (is_valid_pixel(next_x, next_y, rows, cols) && segmented_matrix[next_x][next_y] == 1 && !visited[next_x][next_y]) 
                {
                    stack.push(std::make_pair(next_x, next_y));//将下一个点压入栈中
                }
            }
        }
    };

    // 通过门像素进行分割
    for (const auto& door : door_pixels) 
    {
        std::vector<std::pair<int, int>> door_segment = bresenham_line(door.first.first, door.first.second, door.second.first, door.second.second);
        for (const auto& point : door_segment) 
        {
            int x = point.first;
            int y = point.second;
            if (is_valid_pixel(x, y, rows, cols)) 
            {
                segmented_matrix[x][y] = 0;//将门像素所在的点标记为0
            }
        }
    }

    // 遍历矩阵中的每个像素，进行房间搜索和标记
    for (int i = 0; i < rows; i++) 
    {
        for (int j = 0; j < cols; j++) 
        {
            if (segmented_matrix[i][j] == 1 && !visited[i][j]) 
            {
                Room room(room_id);//创建一个新的房间对象
                dfs(i, j, room);//开始深度优先搜索，将属于同一个房间的像素添加到房间对象中
                rooms.push_back(room);//将房间对象添加到房间列表中
                room_id++;
            }
        }
    }

    return std::make_pair(segmented_matrix, rooms);//返回分割后的矩阵和房间对象列表
}


cv::Mat extract_filled_image(const cv::Mat& connected_region) 
{
    // 生成示例连通区域的二值图像
    cv::Mat matrix = connected_region.clone();

    // 查找外边缘轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(matrix, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 创建空白图像
    cv::Mat filled_image = cv::Mat::zeros(matrix.size(), CV_8UC1);

    // 绘制外轮廓
    cv::drawContours(filled_image, contours, -1, cv::Scalar(255), cv::FILLED);

    return filled_image;
}

cv::Mat extract_edges(const cv::Mat& filled_image) 
{
    // 创建01矩阵
    cv::Mat matrix = filled_image.clone();

    // 在矩阵外围补一圈0
    cv::Mat padded_matrix;
    cv::copyMakeBorder(matrix, padded_matrix, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);

    // 定义3x3的全1卷积核
    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);

    // 对每个像素进行卷积并判断是否为边缘
    cv::Mat edges = cv::Mat::zeros(matrix.size(), CV_8U);
    for (int i = 1; i < padded_matrix.rows - 1; i++) 
    {
        for (int j = 1; j < padded_matrix.cols - 1; j++) 
        {
            if (padded_matrix.at<uchar>(i, j) == 0 && cv::sum(padded_matrix(cv::Rect(j - 1, i - 1, 3, 3)) & kernel)[0] > 0) 
            {
                edges.at<uchar>(i - 1, j - 1) = 1;
            }
        }
    }

    return edges;
}


int main() 
{
    std::vector<std::vector<int>> matrix = 
    {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0, 0, 0, 1, 1, 0},
        {0, 1, 1, 1, 0, 0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0, 0, 0, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 1, 1, 1, 1, 0, 0},
        {0, 1, 1, 0, 1, 1, 1, 1, 0, 0},
        {0, 1, 1, 0, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> door_pixels = 
    {
        {{3, 0}, {3, 2}},
        {{5, 0}, {5, 2}},
        {{3, 6}, {5, 6}},
        {{5, 3}, {5, 5}}
    };

    std::vector<std::vector<int>> segmented_matrix;
    std::vector<Room> rooms;

    std::tie(segmented_matrix, rooms) = segment_rooms(matrix, door_pixels);

    // 将房间单独打印到原来的地图背景
    for (const Room& room : rooms) 
    {
        int room_id = room.get_room_id();
        for (const std::pair<int, int>& pixel : room.get_pixels()) 
        {
            int x = pixel.first;
            int y = pixel.second;
            segmented_matrix[x][y] = room_id;
        }
    }

    // 输出分割后的矩阵
    std::cout << "Segmented Matrix:\n";
    for (const std::vector<int>& row : segmented_matrix) 
    {
        for (int pixel : row) 
        {
            std::cout << pixel << " ";
        }
        std::cout << "\n";
    }

    return 0;
}




