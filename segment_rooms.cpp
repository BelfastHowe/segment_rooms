#include <segment_rooms.h>
#include <generate_connected_region.h>
#include <sstream>

Room::Room(int room_id) : room_id(room_id) {}

void Room::add_pixel(std::pair<int, int> pixel) 
{
    pixels.push_back(pixel);
}

int Room::get_pixel_count() const 
{
    return pixels.size();
}

std::string Room::to_string() const 
{
    std::stringstream ss;
    ss << "Room ID: " << room_id << ", Pixel Count: " << get_pixel_count();
    return ss.str();
}

int Room::get_room_id() const 
{
    return room_id;
}

const std::vector<std::pair<int, int>>& Room::get_pixels() const 
{
    return pixels;
}

void Room::add_connected_room(int room_id, const std::pair<std::pair<int, int>, std::pair<int, int>>& door) 
{
    connected_rooms.push_back(std::make_pair(room_id, door));
}

void Room::print_connected_rooms() const 
{
    for (const auto& connected_room : connected_rooms) 
    {
        std::cout << "Room " << room_id << " is connected to Room " << connected_room.first << " through door ("
            << connected_room.second.first.first << ", " << connected_room.second.first.second << ") to ("
            << connected_room.second.second.first << ", " << connected_room.second.second.second << ")" << std::endl;
    }
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


/*  以cv::Mat作为输入输出参数的版本
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
*/

std::vector<std::vector<int>> extract_filled_image(const std::vector<std::vector<int>>& connected_region) 
{
    // 生成示例连通区域的二值图像
    cv::Mat matrix(connected_region.size(), connected_region[0].size(), CV_8UC1);
    for (size_t i = 0; i < connected_region.size(); i++) 
    {
        for (size_t j = 0; j < connected_region[i].size(); j++) 
        {
            matrix.at<uchar>(i, j) = connected_region[i][j];
        }
    }

    // 查找外边缘轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(matrix, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 创建空白图像
    cv::Mat filled_image(matrix.size(), CV_8UC1, cv::Scalar(0));

    // 绘制外轮廓
    cv::drawContours(filled_image, contours, -1, cv::Scalar(1), cv::FILLED);

    // 将结果转换为二维数组
    std::vector<std::vector<int>> filled_image_arr(filled_image.rows, std::vector<int>(filled_image.cols));
    for (int i = 0; i < filled_image.rows; i++) 
    {
        for (int j = 0; j < filled_image.cols; j++) 
        {
            filled_image_arr[i][j] = static_cast<int>(filled_image.at<uchar>(i, j));
        }
    }

    return filled_image_arr;
}


/*  以cv::Mat结构作为输入输出的版本
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
*/

std::vector<std::vector<int>> extract_edges(const std::vector<std::vector<int>>& filled_image) 
{
    // 创建01矩阵
    std::vector<std::vector<int>> matrix = filled_image;

    // 在矩阵外围补一圈0
    std::vector<std::vector<int>> padded_matrix(matrix.size() + 2, std::vector<int>(matrix[0].size() + 2, 0));
    for (int i = 0; i < matrix.size(); i++) 
    {
        for (int j = 0; j < matrix[i].size(); j++) 
        {
            padded_matrix[i + 1][j + 1] = matrix[i][j];
        }
    }

    // 定义3x3的全1卷积核
    std::vector<std::vector<int>> kernel(3, std::vector<int>(3, 1));

    // 对每个像素进行卷积并判断是否为边缘
    std::vector<std::vector<int>> edges(matrix.size(), std::vector<int>(matrix[0].size(), 0));
    for (int i = 1; i < padded_matrix.size() - 1; i++) 
    {
        for (int j = 1; j < padded_matrix[i].size() - 1; j++) 
        {
            if (padded_matrix[i][j] == 0 && (padded_matrix[i - 1][j - 1] + padded_matrix[i - 1][j] + padded_matrix[i - 1][j + 1] +
                padded_matrix[i][j - 1] + padded_matrix[i][j + 1] + padded_matrix[i + 1][j - 1] + padded_matrix[i + 1][j] + padded_matrix[i + 1][j + 1]) > 0) 
            {
                edges[i - 1][j - 1] = 1;
            }
        }
    }

    return edges;
}


std::vector<std::vector<int>> customize_dilate(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel) 
{
    int rows = binaryMatrix.size();
    int cols = binaryMatrix[0].size();
    int kernelRows = kernel.size();
    int kernelCols = kernel[0].size();
    int padRows = kernelRows / 2;
    int padCols = kernelCols / 2;

    std::vector<std::vector<int>> paddedMatrix(rows + 2 * padRows, std::vector<int>(cols + 2 * padCols, 0));
    for (int i = 0; i < rows; i++) 
    {
        for (int j = 0; j < cols; j++) 
        {
            paddedMatrix[i + padRows][j + padCols] = binaryMatrix[i][j];
        }
    }

    std::vector<std::vector<int>> dilatedMatrix(rows, std::vector<int>(cols, 0));
    for (int i = padRows; i < rows + padRows; i++) 
    {
        for (int j = padCols; j < cols + padCols; j++) 
        {
            bool dilate = false;
            for (int k = 0; k < kernelRows; k++) 
            {
                for (int l = 0; l < kernelCols; l++) 
                {
                    if (paddedMatrix[i - padRows + k][j - padCols + l] && kernel[k][l]) 
                    {
                        dilate = true;
                        break;
                    }
                }
                if (dilate) 
                {
                    break;
                }
            }
            if (dilate) 
            {
                dilatedMatrix[i - padRows][j - padCols] = 1;
            }
        }
    }

    return dilatedMatrix;
}

std::vector<std::vector<int>> customize_erode(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel) 
{
    int rows = binaryMatrix.size();
    int cols = binaryMatrix[0].size();
    int kernelRows = kernel.size();
    int kernelCols = kernel[0].size();
    int padRows = kernelRows / 2;
    int padCols = kernelCols / 2;

    std::vector<std::vector<int>> paddedMatrix(rows + 2 * padRows, std::vector<int>(cols + 2 * padCols, 1));
    for (int i = 0; i < rows; i++) 
    {
        for (int j = 0; j < cols; j++) 
        {
            paddedMatrix[i + padRows][j + padCols] = binaryMatrix[i][j];
        }
    }

    std::vector<std::vector<int>> erodedMatrix(rows, std::vector<int>(cols, 1));
    for (int i = padRows; i < rows + padRows; i++) 
    {
        for (int j = padCols; j < cols + padCols; j++) 
        {
            bool erode = true;
            for (int k = 0; k < kernelRows; k++) 
            {
                for (int l = 0; l < kernelCols; l++) 
                {
                    if (!paddedMatrix[i - padRows + k][j - padCols + l] && kernel[k][l]) 
                    {
                        erode = false;
                        break;
                    }
                }
                if (!erode) 
                {
                    break;
                }
            }
            if (erode) 
            {
                erodedMatrix[i - padRows][j - padCols] = 0;
            }
        }
    }

    return erodedMatrix;
}

std::vector<std::vector<int>> customize_opening(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel) 
{
    int rows = binaryMatrix.size();
    int cols = binaryMatrix[0].size();
    int kernelRows = kernel.size();
    int kernelCols = kernel[0].size();
    int padRows = kernelRows / 2;
    int padCols = kernelCols / 2;

    std::vector<std::vector<int>> paddedMatrix(rows + 2 * padRows, std::vector<int>(cols + 2 * padCols, 0));

    // 将原始矩阵复制到中间部分
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            paddedMatrix[i + padRows][j + padCols] = binaryMatrix[i][j];
        }
    }

    std::vector<std::vector<int>> openedMatrix(rows, std::vector<int>(cols, 0));

    // 腐蚀操作
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            bool isEroded = true;
            for (int m = -padRows; m <= padRows; ++m) 
            {
                for (int n = -padCols; n <= padCols; ++n) 
                {
                    int row = i + m + padRows;
                    int col = j + n + padCols;
                    if (paddedMatrix[row][col] != kernel[m + padRows][n + padCols]) 
                    {
                        isEroded = false;
                        break;
                    }
                }
                if (!isEroded) 
                {
                    break;
                }
            }
            openedMatrix[i][j] = (isEroded) ? 1 : 0;
        }
    }

    // 膨胀操作
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            if (binaryMatrix[i][j] == 1) 
            {
                for (int m = -padRows; m <= padRows; ++m) 
                {
                    for (int n = -padCols; n <= padCols; ++n) 
                    {
                        int row = i + m + padRows;
                        int col = j + n + padCols;
                        paddedMatrix[row][col] = 1;
                    }
                }
            }
        }
    }

    return openedMatrix;
}

std::vector<std::vector<int>> customize_closing(const std::vector<std::vector<int>>& binaryMatrix, const std::vector<std::vector<int>>& kernel) 
{
    int rows = binaryMatrix.size();
    int cols = binaryMatrix[0].size();
    int kernelRows = kernel.size();
    int kernelCols = kernel[0].size();
    int padRows = kernelRows / 2;
    int padCols = kernelCols / 2;

    std::vector<std::vector<int>> paddedMatrix(rows + 2 * padRows, std::vector<int>(cols + 2 * padCols, 0));

    // 将原始矩阵复制到中间部分
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            paddedMatrix[i + padRows][j + padCols] = binaryMatrix[i][j];
        }
    }

    std::vector<std::vector<int>> closedMatrix(rows, std::vector<int>(cols, 0));

    // 膨胀操作
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            if (binaryMatrix[i][j] == 1) 
            {
                for (int m = -padRows; m <= padRows; ++m)
                {
                    for (int n = -padCols; n <= padCols; ++n) 
                    {
                        int row = i + m + padRows;
                        int col = j + n + padCols;
                        paddedMatrix[row][col] = 1;
                    }
                }
            }
        }
    }

    // 腐蚀操作
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            bool isEroded = true;
            for (int m = -padRows; m <= padRows; ++m) 
            {
                for (int n = -padCols; n <= padCols; ++n) 
                {
                    int row = i + m + padRows;
                    int col = j + n + padCols;
                    if (paddedMatrix[row][col] != kernel[m + padRows][n + padCols]) 
                    {
                        isEroded = false;
                        break;
                    }
                }
                if (!isEroded) 
                {
                    break;
                }
            }
            closedMatrix[i][j] = (isEroded) ? 1 : 0;
        }
    }

    return closedMatrix;
}

std::vector<std::vector<int>> pixels_to_matrix(const std::vector<std::pair<int, int>>& pixels, int height, int width)
{
    std::vector<std::vector<int>> matrix(height, std::vector<int>(width, 0));
    for (const auto& pixel : pixels)
    {
        matrix[pixel.first][pixel.second] = 1;
    }
    return matrix;
}

void find_connected_rooms(const std::vector<std::vector<int>>& segmented_matrix,
    std::vector<Room>& rooms,
    const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels) 
{
    int rows = segmented_matrix.size();
    int cols = segmented_matrix[0].size();

    //对于每一对房间
    for (size_t i = 0; i < rooms.size(); i++) 
    {
        for (size_t j = i + 1; j < rooms.size(); j++) 
        {
            Room& room1 = rooms[i];
            Room& room2 = rooms[j];

            //为每个房间提取填充图像并提取边缘
            auto room1_filled_image = extract_filled_image(pixels_to_matrix(room1.get_pixels(), rows, cols));
            auto room2_filled_image = extract_filled_image(pixels_to_matrix(room2.get_pixels(), rows, cols));

            auto room1_edges = extract_edges(room1_filled_image);
            auto room2_edges = extract_edges(room2_filled_image);

            //检查每个门是否连接两个房间
            for (const auto& door : door_pixels) 
            {
                auto door_segment = bresenham_line(door.first.first, door.first.second, door.second.first, door.second.second);

                bool room1_connected = false;
                bool room2_connected = false;

                //检查门的每个像素是否连接到房间的边缘
                for (const auto& pixel : door_segment) 
                {
                    if (room1_edges[pixel.first][pixel.second] == 1)
                        room1_connected = true;

                    if (room2_edges[pixel.first][pixel.second] == 1)
                        room2_connected = true;
                }

                //如果两个房间都与门相连，则添加到连接的房间列表
                if (room1_connected && room2_connected) 
                {
                    room1.add_connected_room(room2.get_room_id(), door);
                    room2.add_connected_room(room1.get_room_id(), door);
                }
            }
        }
    }
}



/*
int main() 
{
    int length, width, desired_area;
    length = 500;
    width = 500;
    desired_area = 100000;

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


    std::vector<std::vector<int>> result = generate_connected_region(length, width, desired_area);

    printBinaryImage(result, 1, "result");

    std::vector<std::vector<int>> kernel =
    {
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1}
    };

  
    std::vector<std::vector<int>> result1 = customize_closing(result, kernel);

    std::vector<std::vector<int>> result2 = extract_edges(extract_filled_image(result1));
    
    printBinaryImage(result1, 1, "result1");

    printBinaryImage(result2, 1, "result2");

    cv::waitKey(0);

    return 0;
}
*/



void test_find_connected_rooms() {
    std::vector<std::vector<int>> matrix = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0, 0, 0, 1, 1, 0},
        {0, 1, 1, 1, 0, 0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0, 0, 0, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 1, 1, 1, 1, 0, 0},
        {0, 1, 1, 0, 1, 1, 1, 1, 0, 0},
        {0, 1, 1, 0, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };

    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> door_pixels = {
        {{3, 0}, {3, 2}},
        {{5, 0}, {5, 2}},
        {{3, 6}, {5, 6}},
        {{5, 3}, {5, 5}}
    };

    auto result = segment_rooms(matrix, door_pixels);
    auto& segmented_matrix = result.first;
    auto& rooms = result.second;


    find_connected_rooms(segmented_matrix, rooms, door_pixels);

    // Replace 1s in the segmented matrix with room id
    for (Room& room : rooms) {
        for (const auto& pixel : room.get_pixels()) {
            int x = pixel.first;
            int y = pixel.second;
            segmented_matrix[x][y] = room.get_room_id();
        }
    }

    // Print the segmented matrix
    std::cout << "Segmented Matrix:\n";
    for (const auto& row : segmented_matrix) {
        for (int val : row) {
            std::cout << val << ' ';
        }
        std::cout << '\n';
    }

    // Print the connected rooms
    for (Room& room : rooms) {
        room.print_connected_rooms();
    }
}

int main() {
    test_find_connected_rooms();
    return 0;
}


