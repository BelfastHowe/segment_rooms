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

void Room::calculate_outline(const std::vector<std::vector<int>>& matrix)
{
    // 初始化空房间矩阵
    std::vector<std::vector<int>> room_matrix(matrix.size(), std::vector<int>(matrix[0].size(), 0));

    // 填充房间矩阵
    for (const auto& pixel : pixels)
    {
        int x = pixel.first;
        int y = pixel.second;
        room_matrix[x][y] = 1;
    }

    // 计算轮廓
    std::vector<std::vector<int>> room_filled = extract_filled_image(room_matrix);
    std::vector<std::vector<int>> room_outline = extract_edges(room_filled);

    // 寻找初始点
    std::pair<int, int> start_pixel = { -1, -1 };
    for (int i = 0; i < room_outline.size(); ++i)
    {
        for (int j = 0; j < room_outline[i].size(); ++j)
        {
            if (room_outline[i][j] == 1)
            {
                start_pixel = { i, j };
                room_outline[i][j] = 0;  // mark as visited
                break;
            }
        }
        if (start_pixel.first != -1) break;
    }

    // 如果没有找到轮廓，清空outline_pixels并返回
    if (start_pixel.first == -1)
    {
        outline_pixels.clear();
        return;
    }

    // 存储初始点并开始深度优先搜索
    outline_pixels = { start_pixel };
    std::stack<std::pair<int, int>> stack;
    stack.push(start_pixel);

    // 四连通方向：左，上，右，下
    std::vector<std::pair<int, int>> directions = { {0, -1}, {-1, 0}, {0, 1}, {1, 0} };

    while (!stack.empty())
    {
        int x = stack.top().first, y = stack.top().second;
        stack.pop();

        for (const auto& d : directions)
        {
            int nx = x + d.first, ny = y + d.second;
            if (nx >= 0 && nx < room_outline.size() && ny >= 0 && ny < room_outline[0].size() && room_outline[nx][ny] == 1)
            {
                outline_pixels.push_back({ nx, ny });
                room_outline[nx][ny] = 0;  // mark as visited
                stack.push({ nx, ny });
                break;  // 只压入一个邻居到栈中
            }
        }
    }
}

const std::vector<std::pair<int, int>>& Room::get_outline_pixels() const
{
    return outline_pixels;
}



std::vector<std::vector<int>> ConvertMatrixToInt(const std::vector<std::vector<uint8_t>>& uint8_matrix) 
{
    std::vector<std::vector<int>> int_matrix;

    for (const auto& uint8_row : uint8_matrix) 
    {
        std::vector<int> int_row;
        for (uint8_t value : uint8_row)
        {
            int_row.push_back(static_cast<int>(value));
        }
        int_matrix.push_back(int_row);
    }

    return int_matrix;
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


/*
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
*/

void find_connected_rooms(std::vector<Room>& rooms, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels)
{
    //对于每一对房间
    for (size_t i = 0; i < rooms.size(); i++)
    {
        for (size_t j = i + 1; j < rooms.size(); j++)
        {
            Room& room1 = rooms[i];
            Room& room2 = rooms[j];

            // 获取每个房间的轮廓像素点
            const auto& room1_outline_pixels = room1.get_outline_pixels();
            const auto& room2_outline_pixels = room2.get_outline_pixels();

            //检查每个门是否连接两个房间
            for (const auto& door : door_pixels)
            {
                auto door_segment = bresenham_line(door.first.first, door.first.second, door.second.first, door.second.second);

                bool room1_connected = false;
                bool room2_connected = false;

                //检查门的每个像素是否连接到房间的轮廓
                for (const auto& pixel : door_segment)
                {
                    if (std::find(room1_outline_pixels.begin(), room1_outline_pixels.end(), pixel) != room1_outline_pixels.end())
                        room1_connected = true;

                    if (std::find(room2_outline_pixels.begin(), room2_outline_pixels.end(), pixel) != room2_outline_pixels.end())
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
std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms) 
{
    // 创建一个新的房间列表作为副本
    std::vector<Room> expanded_rooms = rooms;

    // 创建一个新的矩阵副本
    std::vector<std::vector<int>> expanded_matrix = segmented_matrix;

    // 获取矩阵的大小
    int height = segmented_matrix.size();
    int width = segmented_matrix[0].size();

    // 设置一个标志，用来表示是否还有像素可以进行膨胀
    bool expansion_occurred = true;
    //std::stack<std::pair<int, int>> expansion;

    // 只要还有像素可以膨胀，就继续循环
    while (expansion_occurred) 
    {
        // 在开始新一轮的循环时，首先将标志设置为False
        expansion_occurred = false;

        // 遍历矩阵
        for (int i = 0; i < height; i++) 
        {
            for (int j = 0; j < width; j++) 
            {
                // 当像素点不为0时，即该点位于某个房间内
                if (expanded_matrix[i][j] != 0) 
                {
                    int room_id = expanded_matrix[i][j];

                    // 获取周围8个点的坐标
                    std::vector<std::pair<int, int>> neighbor_coords;
                    for (int dx = -1; dx <= 1; dx++) 
                    {
                        for (int dy = -1; dy <= 1; dy++) 
                        {
                            int nx = i + dx;
                            int ny = j + dy;
                            if (0 <= nx && nx < height && 0 <= ny && ny < width) 
                            {
                                neighbor_coords.push_back({ nx, ny });
                            }
                        }
                    }

                    // 获取周围8个点的像素值
                    std::vector<int> neighbor_pixels;
                    for (const auto& coord : neighbor_coords) 
                    {
                        neighbor_pixels.push_back(expanded_matrix[coord.first][coord.second]);
                    }

                    // 判断周围8个点是否都等于房间号或0
                    if (std::all_of(neighbor_pixels.begin(), neighbor_pixels.end(), [room_id](int pixel) { return pixel == room_id || pixel == 0; })) 
                    {
                        // 计算周围8个点中0的数量
                        int num_zeros = std::count(neighbor_pixels.begin(), neighbor_pixels.end(), 0);

                        // 如果只有一个或两个0，则将该像素点置为房间号，并记录到副本中
                        if (num_zeros == 1 || num_zeros == 2)
                        {
                            auto zero_it = neighbor_pixels.begin();
                            while ((zero_it = std::find(zero_it, neighbor_pixels.end(), 0)) != neighbor_pixels.end())
                            {
                                int zero_index = std::distance(neighbor_pixels.begin(), zero_it);
                                std::pair<int, int> zero_coord = neighbor_coords[zero_index];
                                expanded_matrix[zero_coord.first][zero_coord.second] = room_id;
                                expanded_rooms[room_id - 1].add_pixel(zero_coord);

                                // 因为有像素点被膨胀了，所以将标志设置为True
                                expansion_occurred = true;

                                zero_it++;
                            }
                        }
                    }
                }
            }
        }
    }

    // 计算扩展后的房间轮廓
    for (Room& room : expanded_rooms) 
    {
        room.calculate_outline(expanded_matrix);
    }

    return { expanded_matrix, expanded_rooms };
}
*/

std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms)
{
    // 创建一个新的房间列表作为副本
    std::vector<Room> expanded_rooms = rooms;

    // 创建一个新的矩阵副本
    std::vector<std::vector<int>> expanded_matrix = segmented_matrix;

    // 获取矩阵的大小
    int height = segmented_matrix.size();
    int width = segmented_matrix[0].size();

    std::stack<std::pair<std::pair<int, int>, int>> expansion;
    bool expansion_occurred = true;

    while (expansion_occurred)
    {
        //标志位置0
        expansion_occurred = false;

        // 遍历矩阵
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                // 当像素点不为0时，即该点位于某个房间内
                if (expanded_matrix[i][j] != 0)
                {
                    int room_id = expanded_matrix[i][j];

                    // 获取周围8个点的坐标
                    std::vector<std::pair<int, int>> neighbor_coords;
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            int nx = i + dx;
                            int ny = j + dy;
                            if (0 <= nx && nx < height && 0 <= ny && ny < width)
                            {
                                neighbor_coords.push_back({ nx, ny });
                            }
                        }
                    }

                    // 获取周围8个点的像素值
                    std::vector<int> neighbor_pixels;
                    for (const auto& coord : neighbor_coords)
                    {
                        neighbor_pixels.push_back(expanded_matrix[coord.first][coord.second]);
                    }

                    // 判断周围8个点是否都等于房间号或0
                    if (std::all_of(neighbor_pixels.begin(), neighbor_pixels.end(), [room_id](int pixel) { return pixel == room_id || pixel == 0; }))
                    {
                        // 计算周围8个点中0的数量
                        int num_zeros = std::count(neighbor_pixels.begin(), neighbor_pixels.end(), 0);

                        // 如果只有一个或两个0，则将该像素点置为房间号，并记录到副本中
                        if (num_zeros == 1 || num_zeros == 2)
                        {
                            auto zero_it = neighbor_pixels.begin();
                            while ((zero_it = std::find(zero_it, neighbor_pixels.end(), 0)) != neighbor_pixels.end())
                            {
                                int zero_index = std::distance(neighbor_pixels.begin(), zero_it);
                                std::pair<int, int> zero_coord = neighbor_coords[zero_index];
                                //expanded_matrix[zero_coord.first][zero_coord.second] = room_id;
                                //expanded_rooms[room_id - 1].add_pixel(zero_coord);

                                expansion.push(std::make_pair(zero_coord, room_id));

                                // 因为有像素点被膨胀了，所以将标志设置为True
                                expansion_occurred = true;

                                zero_it++;
                            }
                        }
                    }
                }
            }
        }

        while (!expansion.empty())
        {
            std::pair<std::pair<int, int>, int> pending = expansion.top();
            expansion.pop();

            std::pair<int, int> p = pending.first;
            int id = pending.second;

            if (expanded_matrix[p.first][p.second] == 0)
            {
                expanded_matrix[p.first][p.second] = id;
                expanded_rooms[id - 1].add_pixel(p);
            }

            //expanded_matrix[p.first][p.second] = id;
            //expanded_rooms[id - 1].add_pixel(p);
        }
    }

    // 计算扩展后的房间轮廓
    for (Room& room : expanded_rooms)
    {
        room.calculate_outline(expanded_matrix);
    }

    return { expanded_matrix, expanded_rooms };

}


void draw_map(std::vector<std::vector<int>>& segmented_matrix,
              std::vector<Room>& rooms, 
              std::vector<Room>& expanded_rooms,
              std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels) 
{
    int h = segmented_matrix.size();
    int w = segmented_matrix[0].size();

    // 创建一个RGB画布，白色背景
    cv::Mat final_map(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    // 随机生成RGB颜色
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < expanded_rooms.size(); i++) 
    {
        colors.push_back(cv::Vec3b(dis(gen), dis(gen), dis(gen)));
    }
    /*
    // 染色每个房间
    for (int x = 0; x < h; x++) 
    {
        for (int y = 0; y < w; y++) 
        {
            if (segmented_matrix[x][y] != 0) 
            {
                final_map.at<cv::Vec3b>(x, y) = colors[segmented_matrix[x][y] - 1];
            }
        }
    }

    // 输出中间图像
    cv::imshow("Colored Rooms", final_map);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\Colored Rooms.jpg", final_map);
    //cv::waitKey(0);
    */

    // 绘制扩展后的房间轮廓
    std::vector<std::vector<int>> floor_plan_matrix(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> floor_plan_optimization_matrix(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> floor_plan_optimization_matrix1(h, std::vector<int>(w, 0));

    for (Room& room : expanded_rooms) 
    {
        for (auto& pixel : room.get_outline_pixels()) 
        {
            //final_map.at<cv::Vec3b>(pixel.first, pixel.second) = cv::Vec3b(0, 0, 0);  // 使用黑色绘制轮廓线
            int u = pixel.first;
            int v = pixel.second;
            floor_plan_matrix[u][v] = 1;
        }
    }

    printBinaryImage(floor_plan_matrix, 2, "floor_plan_matrix");

    cv::Mat floor_plan_img(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat floor_plan_optimization_img(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            if (floor_plan_matrix[x][y] != 0)
            {

                floor_plan_img.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    cv::imshow("floor_plan_img", floor_plan_img);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\floor_plan_img.jpg", floor_plan_img);

    floor_plan_optimization_matrix = floor_plan_matrix;

    for (int d = 0; d < h; d++)
    {
        for (int f = 0; f < w; f++)
        {
            if (floor_plan_optimization_matrix[d][f] == 1 && segmented_matrix[d][f] != 0)
            {
                floor_plan_optimization_matrix[d][f] = 0;
            }
        }
    }

    zhangSuenThinning(floor_plan_optimization_matrix);

    printBinaryImage(floor_plan_optimization_matrix, 2, "floor_plan_optimization_matrix0");

    removeBranches(floor_plan_optimization_matrix);

    printBinaryImage(floor_plan_optimization_matrix, 2, "floor_plan_optimization_matrix");
    //cv::waitKey(0);

    floor_plan_optimization_matrix1 = floor_plan_outline_Orthogonalization(floor_plan_optimization_matrix, segmented_matrix);

    completion_link(floor_plan_optimization_matrix1);//四连通连接处补全




    //std::vector<std::vector<int>> tidy_room = tidy_room_erode(segmented_matrix, floor_plan_optimization_matrix1, door_pixels);

    //std::vector<std::vector<int>> tidy_room = tidy_room_approx(segmented_matrix, rooms);

    //std::vector<std::vector<int>> tidy_room_cache(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> tidy_room(h, std::vector<int>(w, 0));
    //tidy_room_binary(tidy_room, rooms);

    tidy_room_Conditional_Morphological_Transformation(tidy_room, floor_plan_optimization_matrix1, rooms, door_pixels);
    //tidy_room_binary(tidy_room, rooms);
    //tidy_room = segmented_matrix;





    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            if (tidy_room[x][y] != 0)
            {
                final_map.at<cv::Vec3b>(x, y) = colors[tidy_room[x][y] - 1];
            }
        }
    }

    // 输出中间图像
    cv::imshow("Colored Rooms", final_map);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\Colored Rooms.jpg", final_map);

    
    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            if (floor_plan_optimization_matrix1[x][y] != 0)
            {
                floor_plan_optimization_img.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    cv::imshow("floor_plan_optimization_img", floor_plan_optimization_img);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\floor_plan_optimization_img.jpg", floor_plan_optimization_img);


    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            if (floor_plan_optimization_matrix1[x][y] != 0)
            {
                final_map.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
            }
        }
    }


    // 绘制门的线段
    for (auto& door : door_pixels) 
    {
        cv::line(final_map, cv::Point(door.first.second, door.first.first), cv::Point(door.second.second, door.second.first), cv::Scalar(0, 0, 255), 2);  // 红色
    }

    // 显示最终地图
    cv::imshow("Final Map", final_map);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\final_map.jpg", final_map);

    cv::waitKey(0);
}

void thinningIteration(std::vector<std::vector<int>>& img, int iter)
{
    int h = img.size();
    int w = img[0].size();

    // Mark pixels for removal
    std::vector<std::vector<bool>> marker(h, std::vector<bool>(w, false));

    for (int i = 1; i < h - 1; i++)
    {
        for (int j = 1; j < w - 1; j++)
        {
            if (img[i][j] != 0)
            {
                // Get 3x3 neighbourhood
                int p[9];
                p[0] = img[i][j];     // P1
                p[1] = img[i - 1][j];   // P2
                p[2] = img[i - 1][j + 1]; // P3
                p[3] = img[i][j + 1];   // P4
                p[4] = img[i + 1][j + 1]; // P5
                p[5] = img[i + 1][j];   // P6
                p[6] = img[i + 1][j - 1]; // P7
                p[7] = img[i][j - 1];   // P8
                p[8] = img[i - 1][j - 1]; // P9

                int np = 0; // Number of non-zero neighbours
                for (int k = 1; k <= 8; k++)
                {
                    np += (p[k] != 0);
                }

                int sp = 0; // Number of 0-1 transitions
                for (int k = 1; k < 8; k++)
                {
                    sp += (p[k] == 0 && p[k + 1] != 0);
                }
                sp += (p[8] == 0 && p[1] != 0);

                // Condition A: 2 <= np <= 6
                bool condA = np >= 2 && np <= 6;

                // Condition B: sp == 1
                bool condB = sp == 1;

                // Condition C and D depend on the iteration (odd or even)
                bool condC, condD;
                if (iter == 0)
                {
                    condC = (p[1] * p[3] * p[5]) == 0;
                    condD = (p[3] * p[5] * p[7]) == 0;
                }
                else
                {
                    condC = (p[1] * p[3] * p[7]) == 0;
                    condD = (p[1] * p[5] * p[7]) == 0;
                }

                // If A,B,C and D are all true mark the pixel for deletion
                if (condA && condB && condC && condD)
                {
                    marker[i][j] = true;
                }
            }
        }
    }

    // Now delete the marked pixels
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (marker[i][j])
            {
                img[i][j] = 0;
            }
        }
    }
}

void zhangSuenThinning(std::vector<std::vector<int>>& img)
{
    int h = img.size();
    int w = img[0].size();
    std::vector<std::vector<int>> newImg;

    do
    {
        newImg = img;
        thinningIteration(img, 0);
        thinningIteration(img, 1);
    } while (img != newImg);
}

void removeBranches(std::vector<std::vector<int>>& img)
{
    int height = img.size();
    int width = img[0].size();

    for (int i = 1; i < height - 1; i++)
    {
        for (int j = 1; j < width - 1; j++)
        {
            if (img[i][j] == 1)
            {
                // Get 3x3 neighbourhood
                int p[9];
                p[0] = img[i][j];        // P1
                p[1] = img[i - 1][j];      // P2
                p[2] = img[i - 1][j + 1];  // P3
                p[3] = img[i][j + 1];      // P4
                p[4] = img[i + 1][j + 1];  // P5
                p[5] = img[i + 1][j];      // P6
                p[6] = img[i + 1][j - 1];  // P7
                p[7] = img[i][j - 1];      // P8
                p[8] = img[i - 1][j - 1];  // P9

                int count = 0;
                for (int k = 0; k < 9; k++)
                {
                    count += p[k];
                }

                if (count == 3)
                {
                    int transitions = 0;
                    for (int k = 1; k < 8; k++)
                    {
                        transitions += (p[k] == 0 && p[k + 1] == 1);
                    }
                    transitions += (p[8] == 0 && p[1] == 1);

                    if (transitions == 1)
                    {
                        img[i][j] = 0;
                    }
                }
            }
        }
    }
}


std::vector<Line> extractIntersections(std::vector<std::vector<int>>& img) 
{
    std::vector<Line> lines;

    // offsets for 4-connectivity
    std::vector<std::pair<int, int>> offsets = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

    // 遍历矩阵
    for (int i = 0; i < img.size(); ++i) 
    {
        for (int j = 0; j < img[i].size(); ++j) 
        {
            // 检测到前景
            if (img[i][j] != 0) 
            {
                int connectedPoints = 0;
                // 计算四连通点中前景的数量
                for (const auto& offset : offsets) 
                {
                    int ni = i + offset.first, nj = j + offset.second;
                    if (ni >= 0 && ni < img.size() && nj >= 0 && nj < img[i].size() && img[ni][nj] != 0) 
                    {
                        connectedPoints++;
                    }
                }
                // 如果它是连接点
                if (connectedPoints > 2) 
                {
                    std::pair<int, int> intersectionPoint = { i, j };
                    lines.push_back(Line{ static_cast<int>(lines.size()), Line::INTERSECTION, {intersectionPoint}, intersectionPoint, intersectionPoint });
                    img[i][j] = 0;  // 将这个点置为背景
                }
            }
        }
    }

    return lines;
}

void extractOrthogonalLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines)
{
    int threshold = 15;//设置线段长度阈值


    //第一步，提取水平线
    for (int i = 0; i < img.size(); ++i) 
    {
        for (int j = 0; j < img[i].size(); ) 
        {
            if (img[i][j] == 1) 
            {
                int start = j;
                while (j < img[i].size() && img[i][j] == 1) 
                {
                    ++j;
                }
                int end = j - 1;

                // 只考虑长于阈值的线段
                if (end - start + 1 > threshold) 
                {
                    Line line;
                    line.id = lines.size();
                    line.direction = Line::HORIZONTAL;
                    line.startPoint = { i, start + 1 };
                    line.endPoint = { i, end - 1 };
                    for (int k = start + 1; k < end; ++k)
                    {
                        line.points.push_back({ i, k });
                        img[i][k] = 0;  // 重置为背景
                    }
                    lines.push_back(line);
                }
            }
            else 
            {
                ++j;
            }
        }
    }

    //第二步，提取垂直线
    for (int j = 0; j < img[0].size(); ++j) 
    {
        for (int i = 0; i < img.size(); ) 
        {
            if (img[i][j] == 1) 
            {
                int start = i;
                while (i < img.size() && img[i][j] == 1) 
                {
                    ++i;
                }
                int end = i - 1;

                // 只考虑长于阈值的线段
                if (end - start + 1 > threshold) 
                {
                    Line line;
                    line.id = lines.size();
                    line.direction = Line::VERTICAL;
                    line.startPoint = { start + 1, j };
                    line.endPoint = { end - 1, j };
                    for (int k = start + 1; k < end; ++k)
                    {
                        line.points.push_back({ k, j });
                        img[k][j] = 0;  // 重置为背景
                    }
                    lines.push_back(line);
                }
            }
            else 
            {
                ++i;
            }
        }
    }
}

void findNonLinearLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines)
{
    int height = img.size();
    int width = img[0].size();
    int lineId = lines.size();  //确定line的id

    std::vector<std::pair<int, int>> offsets = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };  //四连通偏移量

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (img[i][j] != 0) {  // 找到一条线的一个点
                Line line;
                line.id = lineId++;
                line.direction = Line::NONLINEAR;

                std::stack<std::pair<int, int>> dfsStack;
                dfsStack.push({ i, j });

                img[i][j] = 0;

                while (!dfsStack.empty())
                {
                    std::pair<int, int> cur = dfsStack.top();
                    dfsStack.pop();

                    line.points.push_back(cur);

                    for (const auto& offset : offsets)
                    {
                        std::pair<int, int> next = { cur.first + offset.first, cur.second + offset.second };
                        if (next.first >= 0 && next.first < height && next.second >= 0 && next.second < width && img[next.first][next.second] != 0)
                        {
                            dfsStack.push(next);
                            img[next.first][next.second] = 0;  //提取过的像素置为背景
                        }
                    }
                }

                for (auto& point : line.points)
                {
                    int connected = 0;
                    for (auto& p : line.points)
                    {
                        if (std::abs(p.first - point.first) + std::abs(p.second - point.second) == 1) //曼哈顿距离为1
                        {
                            connected++;
                        }
                    }

                    // 根据connected的值，确定线段的起点和终点
                    switch (connected)
                    {
                    case 0:  //如果它是一个孤立的像素
                        line.startPoint = point;
                        line.endPoint = point;
                        break;
                    case 1:  //如果它是一个端点
                        if (line.startPoint.first == -1 && line.startPoint.second == -1)
                        {
                            line.startPoint = point;
                        }
                        else if (line.endPoint.first == -1 && line.endPoint.second == -1)
                        {
                            line.endPoint = point;
                        }
                        else
                        {
                            std::cerr << "Found more than two endpoints in a non-linear." << std::endl;
                            throw std::runtime_error("Invalid line found.");
                        }
                        break;
                    case 2://如果该点在线的中间，就继续
                        break;
                    default:  //如果该点有多于两个邻居，就报错
                        std::cerr << "Found a point with " << connected << " neighbors in a non-linear." << std::endl;
                        throw std::runtime_error("Invalid line found.");
                    }
                }

                lines.push_back(line);  // Add the line to lines
            }
        }
    }
}

int getDirection(std::pair<int, int> a, std::pair<int, int> b) 
{
    if (a.first == b.first) 
    {
        if (b.second > a.second) return 1;  // right
        else return 3;  // left
    }
    else 
    {
        if (b.first > a.first) return 2;  // down
        else return 4;  // up
    }
    return 0;  // same point
}

std::vector<std::pair<int, int>> getLeastTurnPath(const std::pair<int, int>& start, const std::pair<int, int>& end, const std::vector<std::vector<int>>& mask) 
{
    // 四连通偏移量
    std::vector<std::pair<int, int>> offsets = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

    std::priority_queue<Node> pq;
    pq.emplace(start, std::vector<std::pair<int, int>>{start}, 0); // 将起始节点添加到队列中

    while (!pq.empty()) {
        Node curNode = pq.top();
        pq.pop();

        std::pair<int, int> curPos = curNode.pos;
        if (curPos == end) 
        { // 我们已经找到了一条通往终点的道路
            return curNode.path; // 返回这个路径
        }

        for (const auto& offset : offsets) 
        {
            std::pair<int, int> newPos = { curPos.first + offset.first, curPos.second + offset.second };

            // 边界检查和障碍物检查
            if (newPos.first < 0 || newPos.first >= mask.size() || newPos.second < 0 || newPos.second >= mask[0].size() || mask[newPos.first][newPos.second] == 0)
                continue;

            // 计算转折数
            int newTurns = curNode.turns;
            if (curNode.path.size() > 1 && getDirection(curNode.path[curNode.path.size() - 2], curPos) != getDirection(curPos, newPos))
            {
                newTurns++;
            }

            // 创建一个新的节点并将其添加到队列中
            std::vector<std::pair<int, int>> newPath = curNode.path;
            newPath.push_back(newPos);
            pq.emplace(newPos, newPath, newTurns);
        }
    }

    // 如果没有从起点到终点的路径
    return std::vector<std::pair<int, int>>{};
}

std::vector<std::vector<int>> floor_plan_outline_Orthogonalization(std::vector<std::vector<int>>& img, const std::vector<std::vector<int>>& segmented_matrix)
{
    std::vector<std::vector<int>> mask(segmented_matrix.size(), std::vector<int>(segmented_matrix[0].size(), 1));
    std::vector<std::vector<int>> output(img.size(), std::vector<int>(img[0].size(), 0));

    for (int i = 0; i < segmented_matrix.size(); ++i)
    {
        for (int j = 0; j < segmented_matrix[0].size(); ++j)
        {
            if (segmented_matrix[i][j] != 0) mask[i][j] = 0;
        }
    }

    //第1步：提取交叉点
    std::vector<Line> lines = extractIntersections(img);

    //printBinaryImage(img, 2, "img1");
    //cv::waitKey(0);

    // 第2步：提取正交线
    extractOrthogonalLines(img, lines);

    //printBinaryImage(img, 2, "img2");
    //cv::waitKey(0);

    // 第3步：寻找非线性线条
    findNonLinearLines(img, lines);

    printBinaryImage(img, 2, "img3");
    //cv::waitKey(0);

    //第4步：调整非线性线条，使其具有最小的转折点
    //如果可能的话，用最小转弯的路径来代替非线性的线路
    
    for (auto& line : lines)
    {
        if (line.direction == Line::NONLINEAR)
        {
            //检查该行的所有像素是否都在掩膜中
            bool allInMask = std::all_of(line.points.begin(), line.points.end(), [&mask](const std::pair<int, int>& p) 
                {
                    return mask[p.first][p.second] == 1; 
                });

            //如果所有的像素都在mask前景中，则尝试替换线条
            if (allInMask)
            {
                auto newPath = getLeastTurnPath(line.startPoint, line.endPoint, mask);
                if (!newPath.empty()) line.points = std::move(newPath);
            }
        }
    }
    

    // 第五步：将所有线条输出到一个新的图像上
    for (const auto& line : lines)
    {
        for (const auto& p : line.points)
        {
            output[p.first][p.second] = 1;
        }
    }


    return output;
}

void completion_link(std::vector<std::vector<int>>& floor_plan_matrix)
{
    int height = floor_plan_matrix.size();
    int width = floor_plan_matrix[0].size();

    for (int i = 1; i < height - 1; i++)
    {
        for (int j = 1; j < width - 1; j++)
        {
            if (floor_plan_matrix[i][j] == 1)
            {
                // Get 3x3 neighbourhood
                int p[9];
                p[0] = floor_plan_matrix[i][j];        // P1
                p[1] = floor_plan_matrix[i - 1][j];      // P2
                p[2] = floor_plan_matrix[i - 1][j + 1];  // P3
                p[3] = floor_plan_matrix[i][j + 1];      // P4
                p[4] = floor_plan_matrix[i + 1][j + 1];  // P5
                p[5] = floor_plan_matrix[i + 1][j];      // P6
                p[6] = floor_plan_matrix[i + 1][j - 1];  // P7
                p[7] = floor_plan_matrix[i][j - 1];      // P8
                p[8] = floor_plan_matrix[i - 1][j - 1];  // P9

                int four_connected_count = p[1] + p[3] + p[5] + p[7];

                int eight_transition_count = 0;
                for (int k = 1; k < 8; k++)
                {
                    eight_transition_count += (p[k] == 0 && p[k + 1] == 1);
                }
                eight_transition_count += (p[8] == 0 && p[1] == 1);

                switch (eight_transition_count - four_connected_count)
                {
                case 0:
                    break;
                case 1:
                    if (p[2] == 1 && p[1] == 0 && p[3] == 0)
                    {
                        if (p[5] == 1) floor_plan_matrix[i - 1][j] = 1;
                        else floor_plan_matrix[i][j + 1] = 1;
                    }
                    else if (p[4] == 1 && p[3] == 0 && p[5] == 0)
                    {
                        if (p[7] == 1) floor_plan_matrix[i][j + 1] = 1;
                        else floor_plan_matrix[i + 1][j] = 1;
                    }
                    else if (p[6] == 1 && p[5] == 0 && p[7] == 0)
                    {
                        if (p[3] == 1) floor_plan_matrix[i][j - 1] = 1;
                        else floor_plan_matrix[i + 1][j] = 1;
                    }
                    else if (p[8] == 1 && p[7] == 0 && p[1] == 0)
                    {
                        if (p[3] == 1) floor_plan_matrix[i][j - 1] = 1;
                        else floor_plan_matrix[i - 1][j] = 1;
                    }
                    else
                    {
                        std::cerr << "case=1时找不到孤立角点" << std::endl;
                        throw std::runtime_error("Invalid line found.");
                    }
                    break;
                case  2:
                    if (p[2] == 1 && p[4] == 1 && p[1] == 0 && p[3] == 0 && p[5] == 0) floor_plan_matrix[i][j + 1] = 1;
                    else if (p[4] == 1 && p[6] == 1 && p[3] == 0 && p[5] == 0 && p[7] == 0) floor_plan_matrix[i + 1][j] = 1;
                    else if (p[6] == 1 && p[8] == 1 && p[5] == 0 && p[7] == 0 && p[1] == 0) floor_plan_matrix[i][j - 1] = 1;
                    else if (p[8] == 1 && p[2] == 1 && p[7] == 0 && p[1] == 0 && p[3] == 0) floor_plan_matrix[i - 1][j] = 1;
                    else
                    {
                        std::cerr << "case=2时找不到符合条件的角点对" << std::endl;
                        throw std::runtime_error("Invalid line found.");
                    }
                    break;
                default:
                    std::cerr << "void completion_link error" << std::endl;
                    throw std::runtime_error("Invalid line found.");
                    break;
                }
            }
        }
    }
}



std::pair<double, bool> distanceToSegment(const std::pair<int, int>& point, const std::pair<int, int>& segA, const std::pair<int, int>& segB) 
{
    std::pair<int, int> vecAB = { segB.first - segA.first, segB.second - segA.second };
    std::pair<int, int> vecAP = { point.first - segA.first, point.second - segA.second };
    std::pair<int, int> vecBP = { point.first - segB.first, point.second - segB.second };

    int dotProduct = vecAB.first * vecAP.first + vecAB.second * vecAP.second;
    int dotProduct2 = vecAB.first * vecBP.first + vecAB.second * vecBP.second;

    bool isProjectionInside = false;

    if (dotProduct * dotProduct2 < 0) 
    {
        isProjectionInside = true;
        int crossProduct = vecAB.first * vecAP.second - vecAB.second * vecAP.first;
        double lengthAB = sqrt(vecAB.first * vecAB.first + vecAB.second * vecAB.second);
        return { std::abs(crossProduct / lengthAB), isProjectionInside };
    }

    return { std::min(sqrt(vecAP.first * vecAP.first + vecAP.second * vecAP.second),
                    sqrt(vecBP.first * vecBP.first + vecBP.second * vecBP.second)), isProjectionInside };
}

bool should_not_be_eroded(const std::pair<int, int>& point, const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors, double threshold)
{
    for (const auto& door : doors)
    {
        std::pair<double, bool> result = distanceToSegment(point, door.first, door.second);
        double distance = result.first;
        bool isProjectionInside = result.second;
        if (distance < threshold && isProjectionInside)
        {
            return true;
        }
    }
    return false;
}


void tidyRoomDFS(int& x, int& y,int& h, int& w, std::vector<std::vector<int>>& tidy_room, int& id)
{
    std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
    std::stack<std::pair<int, int>> stack;

    stack.push(std::make_pair(x, y));

    while (!stack.empty())
    {
        std::pair<int, int> p = stack.top();
        stack.pop();

        int px = p.first;
        int py = p.second;

        for (const auto& dir : directions)
        {
            int nx = px + dir.first;
            int ny = py + dir.second;

            if (is_valid_pixel(nx, ny, h, w) && tidy_room[nx][ny] == 1)
            {
                stack.push(std::make_pair(nx, ny));
                tidy_room[nx][ny] = id;
            }
        }
    }

}

std::vector<std::vector<int>> tidy_room_erode(std::vector<std::vector<int>>& segmented_matrix, 
                                              std::vector<std::vector<int>>& floor_plan,
                                              const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels)
{
    int erode_times = 3;//腐蚀几次
    double threshold = 10;//门框保护的宽度

    int h = floor_plan.size();
    int w = floor_plan[0].size();

    std::vector<std::vector<int>> tidy_room(h, std::vector<int>(w, 0));

    //户型图轮廓前景背景互换
    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            tidy_room[x][y] = floor_plan[x][y] == 0 ? 1 : 0;
        }
    }

    std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };//四连通

    std::stack<std::pair<int, int>> background_stack;
    background_stack.push(std::make_pair(0, 0));
    tidy_room[0][0] = 0;

    //将最外圈设为背景
    while (!background_stack.empty())
    {
        std::pair<int, int> background_pixel = background_stack.top();
        background_stack.pop();

        int bx = background_pixel.first;
        int by = background_pixel.second;

        for (const auto& direction : directions)
        {
            int nx = bx + direction.first;
            int ny = by + direction.second;

            if (is_valid_pixel(nx, ny, h, w) && tidy_room[nx][ny] == 1)
            {
                background_stack.push(std::make_pair(nx, ny));
                tidy_room[nx][ny] = 0;
            }
        }
    }

    //printBinaryImage(tidy_room, 2, "tidyroom_matrix");

    //条件腐蚀
    for (int times = 0; times < erode_times; ++times)
    {
        std::vector<std::vector<int>> tidy_room_cache = tidy_room;

        for (int i = 1; i < h - 1; ++i)
        {
            for (int j = 1; j < w - 1; ++j)
            {
                bool isEroded = false;
                for (int m = -1; m <= 1; ++m)
                {
                    for (int n = -1; n <= 1; ++n)
                    {
                        if (m == 0 && n == 0) continue;
                        if (tidy_room_cache[i + m][j + n] != 1)
                        {
                            isEroded = true;
                            break;
                        }
                    }
                    if (isEroded) break;
                }
                if (isEroded && !should_not_be_eroded(std::make_pair(i, j), doors_pixels, threshold))
                {
                    tidy_room[i][j]=0;
                }
            }
        }
    }

    //房间id继承
    int room_id = 1;
    bool hasChanged = true;

    while (hasChanged)
    {
        hasChanged = false;
        for (int u = 0; u < h; u++)
        {
            for (int v = 0; v < w; v++)
            {
                if (segmented_matrix[u][v] == room_id && tidy_room[u][v] == 1)
                {
                    tidy_room[u][v] = room_id;
                    hasChanged = true;
                    break;
                }
            }
            if (hasChanged) break;
        }
        room_id++;
    }

    //房间id扩散
    for (int id = 2; id < room_id - 1; id++)
    {
        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                if (tidy_room[i][j] == id)
                {
                    tidyRoomDFS(i, j, h, w, tidy_room, id);
                }
            }
        }
    }

    return tidy_room;
}


std::vector<std::vector<int>> tidy_room_dilate(std::vector<std::vector<int>>& room_matrix, std::vector<std::vector<int>>& floor_plan_matrix, int rounds) 
{
    const int dx[8] = { -1, 0, 1, 0, -1, -1, 1, 1 };
    const int dy[8] = { 0, 1, 0, -1, -1, 1, -1, 1 };

    std::vector<std::vector<int>> tidy_room_matrix = room_matrix;

    int m = tidy_room_matrix.size();
    int n = tidy_room_matrix[0].size();


    for (int round = 0; round < rounds; round++)
    {
        //记录需要腐蚀的点的队列
        std::queue<std::pair<int, int>> points_to_erode;
        for (int u = 0; u < m; u++)
        {
            for (int v = 0; v < n; v++)
            {
                if (tidy_room_matrix[u][v] > 0)
                {
                    for (int k = 0; k < 8; k++)
                    {
                        int nx = u + dx[k];
                        int ny = v + dy[k];
                        if (is_valid_pixel(nx, ny, m, n) && tidy_room_matrix[nx][ny] == 0)
                        {
                            points_to_erode.push(std::make_pair(u, v));
                            break;
                        }
                    }
                }
            }
        }

        //对队列中的点进行腐蚀
        while (!points_to_erode.empty())
        {
            std::pair<int, int> point = points_to_erode.front();
            points_to_erode.pop();

            tidy_room_matrix[point.first][point.second] = 0;
        }
    }

    for (int round = 0; round < rounds; round++) 
    {
        // 使用一个队列来存储每轮需要扩展的点
        std::queue<std::pair<int, int>> points_to_expand;
        for (int i = 0; i < m; i++) 
        {
            for (int j = 0; j < n; j++) 
            {
                if (tidy_room_matrix[i][j] > 0 && floor_plan_matrix[i][j] == 0)
                {
                    for (int k = 0; k < 8; k++)
                    {
                        int nx = i + dx[k];
                        int ny = j + dy[k];
                        if (is_valid_pixel(nx, ny, m, n) && tidy_room_matrix[nx][ny] == 0)
                        {
                            points_to_expand.push({ i, j });
                            break;
                        }
                    } 
                }
            }
        }

        // 对需要扩展的点进行膨胀
        while (!points_to_expand.empty()) 
        {
            std::pair<int, int> point = points_to_expand.front();
            points_to_expand.pop();

            for (int i = 0; i < 8; i++)
            {
                int new_x = point.first + dx[i];
                int new_y = point.second + dy[i];

                // 判断新点是否在矩阵范围内，以及是否可以被膨胀
                if (new_x >= 0 && new_x < m && new_y >= 0 && new_y < n && floor_plan_matrix[new_x][new_y] != 1 && tidy_room_matrix[new_x][new_y] == 0)
                {
                    tidy_room_matrix[new_x][new_y] = tidy_room_matrix[point.first][point.second];
                }
            }
        }
    }

    return tidy_room_matrix;
}


void polygon_fitting(std::vector<std::vector<int>>& room_matrix, double epsilon)
{
    //将输入的矩阵转化为opencv的Mat格式
    cv::Mat binary_image(room_matrix.size(), room_matrix[0].size(), CV_8U);
    for (size_t i = 0; i < room_matrix.size(); i++)
    {
        for (size_t j = 0; j < room_matrix[0].size(); j++)
        {
            binary_image.at<uchar>(i, j) = room_matrix[i][j] * 255;
        }
    }

    std::vector<std::vector<cv::Point>> contours;

    //使用findContours找出外部轮廓
    cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approx_contour;

    // 对找到的轮廓进行多边形拟合
    cv::approxPolyDP(contours[0], approx_contour, epsilon, true);

    // 创建一个空白图像，用于绘制拟合后的图像
    cv::Mat result = cv::Mat::zeros(binary_image.size(), CV_8U);

    // 使用drawContours绘制拟合后的图像
    cv::drawContours(result, std::vector<std::vector<cv::Point>>{approx_contour}, -1, cv::Scalar(255), cv::FILLED);

    // 将原矩阵全部置0
    for (auto& row : room_matrix)
    {
        fill(row.begin(), row.end(), 0);
    }

    // 将结果转换回二值矩阵，并直接修改原矩阵
    for (int i = 0; i < result.rows; i++)
    {
        for (int j = 0; j < result.cols; j++)
        {
            room_matrix[i][j] = static_cast<int>(result.at<uchar>(i, j) / 255);
        }
    }
}

std::vector<std::vector<int>> tidy_room_approx(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& rooms)
{
    int h = segmented_matrix.size();
    int w = segmented_matrix[0].size();

    std::vector<std::vector<int>> tidy_room_matrix(h, std::vector<int>(w, 0));

    for (auto& room : rooms)
    {
        int room_id = room.get_room_id();
        std::vector<std::vector<int>> single_room_matrix(h, std::vector<int>(w, 0));

        for (const auto& pixel : room.get_pixels())
        {
            single_room_matrix[pixel.first][pixel.second] = 1;
        }

        polygon_fitting(single_room_matrix, 0);

        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                if (single_room_matrix[i][j] != 0)
                {
                    tidy_room_matrix[i][j] = room_id;
                }
            }
        }

    }

    return tidy_room_matrix;
}


void delete_jut(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int uthreshold, int vthreshold)
{
    int height = src.size();
    int width = src[0].size();
    int k;
    dst = src;

    for (int i = 0; i < height - 1; i++)
    {
        for (int j = 0; j < width - 1; j++)
        {
            //行消除
            if (dst[i][j] == 0 && dst[i][j + 1] == 1)
            {
                if (j + uthreshold >= width)
                {
                    for (int k = j + 1; k < width; k++)
                    {
                        dst[i][k] = 0;
                    }
                }
                else
                {
                    for (k = j + 2; k <= j + uthreshold; k++)
                    {
                        if (dst[i][k] == 0) break;
                    }
                    if (dst[i][k] == 0)
                    {
                        for (int h = j + 1; h < k; h++)
                            dst[i][h] = 0;
                    }
                }
            }
            //列消除
            if (dst[i][j] == 0 && dst[i + 1][j] == 1)
            {
                if (i + vthreshold >= height)
                {
                    for (int k = i + 1; k < height; k++)
                        dst[k][j] = 0;
                }
                else
                {
                    for (k = i + 2; k <= i + vthreshold; k++)
                    {
                        if (dst[k][j] == 0) break;
                    }
                    if (dst[k][j] == 0)
                    {
                        for (int h = i + 1; h < k; h++)
                            dst[h][j] = 0;
                    }
                }
            }
        }
    }

}

void fill_hollow(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int uthreshold, int vthreshold)
{
    int height = src.size();
    int width = src[0].size();
    int k;
    dst = src;

    for (int i = 0; i < height - 1; i++)
    {
        for (int j = 0; j < width - 1; j++)
        {
            //行填补
            if (dst[i][j] == 1 && dst[i][j + 1] == 0)
            {
                if (j + uthreshold >= width)
                {
                    for (int k = j + 1; k < width; k++)
                    {
                        dst[i][k] = dst[i][k];
                    }
                }
                else
                {
                    for (k = j + 2; k <= j + uthreshold; k++)
                    {
                        if (dst[i][k] == 1) break;
                    }
                    if (dst[i][k] == 1)
                    {
                        for (int h = j + 1; h < k; h++)
                            dst[i][h] = 1;
                    }
                }
            }
            //列填补
            if (dst[i][j] == 1 && dst[i + 1][j] == 0)
            {
                if (i + vthreshold >= height)
                {
                    for (int k = i + 1; k < height; k++)
                        dst[k][j] = dst[k][j];
                }
                else
                {
                    for (k = i + 2; k <= i + vthreshold; k++)
                    {
                        if (dst[k][j] == 1) break;
                    }
                    if (dst[k][j] == 1)
                    {
                        for (int h = i + 1; h < k; h++)
                            dst[h][j] = 1;
                    }
                }
            }
        }
    }
}

void customize_blur(std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst, int windowSize, int threshold)
{
    int height = src.size();
    int width = src[0].size();
    int padSize = windowSize / 2;

    // Padding
    std::vector<std::vector<int>> paddedSrc(height + 2 * padSize, std::vector<int>(width + 2 * padSize, 0));
    for (int i = 0; i < height; ++i) 
    {
        for (int j = 0; j < width; ++j) 
        {
            paddedSrc[i + padSize][j + padSize] = src[i][j];
        }
    }

    dst = src; // copy original matrix to destination

    for (int i = padSize; i < height + padSize; ++i)
    {
        for (int j = padSize; j < width + padSize; ++j)
        {
            int sum = 0;

            // Create window and compute the sum
            for (int k = i - padSize; k <= i + padSize; ++k)
            {
                for (int l = j - padSize; l <= j + padSize; ++l)
                {
                    sum += paddedSrc[k][l];
                }
            }

            // 根据阈值设置新值
            dst[i - padSize][j - padSize] = sum > threshold ? 1 : 0;
        }
    }
}

void tidy_room_binary(std::vector<std::vector<int>>& src, std::vector<Room>& rooms)
{
    int h = src.size();
    int w = src[0].size();

    for (auto& row : src)
    {
        for (auto& element : row)
        {
            element = 0;
        }
    }

    for (auto& room : rooms)
    {
        int room_id = room.get_room_id();
        std::vector<std::vector<int>> cache1(h, std::vector<int>(w, 0));
        std::vector<std::vector<int>> cache2(h, std::vector<int>(w, 0));

        for (const auto& pixel : room.get_pixels())
        {
            cache1[pixel.first][pixel.second] = 1;
        }

        //customize_blur(cache1, cache2, 5, 12);

        cache2 = cache1;

        do
        {
            cache1 = cache2;
            delete_jut(cache1, cache2, 6, 6);
        } while (cache2 != cache1);

        do
        {
            cache1 = cache2;
            fill_hollow(cache1, cache2, 10, 10);
        } while (cache2 != cache1);

        for (auto& row : cache1)
        {
            for (auto& element : row)
            {
                element = 0;
            }
        }

        //customize_blur(cache2, cache1, 5, 12);

        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                if (cache2[i][j] == 1) src[i][j] = room_id;
            }
        }
    }


}


void tidy_room_Conditional_Morphological_Transformation(std::vector<std::vector<int>>& src,
                                                        std::vector<std::vector<int>>& mask,
                                                        std::vector<Room>& rooms,
                                                        const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels)
{
    int h = src.size();
    int w = src[0].size();

    for (auto& row : src)
    {
        for (auto& element : row)
        {
            element = 0;
        }
    }

    for (auto& room : rooms)
    {
        int room_id = room.get_room_id();
        std::vector<std::vector<int>> cache1(h, std::vector<int>(w, 0));
        std::vector<std::vector<int>> cache2(h, std::vector<int>(w, 0));

        for (const auto& pixel : room.get_pixels())
        {
            cache1[pixel.first][pixel.second] = 1;
        }
        cache2 = cache1;

        /*
        do
        {
            cache1 = cache2;
            tidy_room_Conditional_Erosion_Transformation(cache1, cache2, doors_pixels);
        } while (cache1 != cache2);
        */

        do
        {
            /* code */
            cache1 = cache2;
            tidy_room_Conditional_Dilation_Transformation(cache1, cache2, mask);
        } while (cache1 != cache2);


        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                if (cache2[i][j] == 1)
                {
                    src[i][j] = room_id;
                }
            }
        }

    }
}

void tidy_room_Conditional_Erosion_Transformation(std::vector<std::vector<int>>& src,
                                                  std::vector<std::vector<int>>& dst,
                                                  const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& doors_pixels)
{
    int threshold = 3;

    int h = src.size();
    int w = src[0].size();

    dst = src;

    bool erode_flag = true;
    std::queue<std::pair<int, int>> erode_points;

    while (erode_flag)
    {
        erode_flag = false;

        for (int i = 1; i < h - 1; i++)
        {
            for (int j = 1; j < w - 1; j++)
            {
                if (dst[i][j] == 1 && !should_not_be_eroded(std::make_pair(i, j), doors_pixels, threshold))
                {
                    //创建3*3邻域列表
                    int p[9];
                    p[0] = dst[i][j];          // P1
                    p[1] = dst[i - 1][j];      // P2
                    p[2] = dst[i - 1][j + 1];  // P3
                    p[3] = dst[i][j + 1];      // P4
                    p[4] = dst[i + 1][j + 1];  // P5
                    p[5] = dst[i + 1][j];      // P6
                    p[6] = dst[i + 1][j - 1];  // P7
                    p[7] = dst[i][j - 1];      // P8
                    p[8] = dst[i - 1][j - 1];  // P9

                    int eight_transition_count = 0;
                    for (int k = 1; k < 8; k++)
                    {
                        eight_transition_count += (p[k] == 0 && p[k + 1] == 1);
                    }
                    eight_transition_count += (p[8] == 0 && p[1] == 1);

                    int four_connected_count = p[1] + p[3] + p[5] + p[7];

                    int eight_connected_count = four_connected_count + p[2] + p[4] + p[6] + p[8];

                    if (eight_transition_count == 1 && eight_connected_count == 5 && four_connected_count == 2)
                    {
                        erode_points.push(std::make_pair(i, j));
                        erode_flag = true;
                    }


                }
            }
        }

        while (!erode_points.empty())
        {
            std::pair<int, int> p = erode_points.front();
            erode_points.pop();

            dst[p.first][p.second] = 0;
        }
    }

}

void tidy_room_Conditional_Dilation_Transformation(std::vector<std::vector<int>>& src,
                                                   std::vector<std::vector<int>>& dst,
                                                   std::vector<std::vector<int>>& mask)
{
    int h = src.size();
    int w = src[0].size();

    dst = src;

    bool dilate_flag = true;
    std::queue<std::pair<int, int>> dilate_points;

    while (dilate_flag)
    {
        dilate_flag = false;

        for (int i = 1; i < h - 1; i++)
        {
            for (int j = 1; j < w - 1; j++)
            {
                if (dst[i][j] == 0 && mask[i][j] == 0)
                {
                    //创建3*3邻域列表
                    int p[9];
                    p[0] = dst[i][j];          // P1
                    p[1] = dst[i - 1][j];      // P2
                    p[2] = dst[i - 1][j + 1];  // P3
                    p[3] = dst[i][j + 1];      // P4
                    p[4] = dst[i + 1][j + 1];  // P5
                    p[5] = dst[i + 1][j];      // P6
                    p[6] = dst[i + 1][j - 1];  // P7
                    p[7] = dst[i][j - 1];      // P8
                    p[8] = dst[i - 1][j - 1];  // P9

                    int eight_transition_count = 0;
                    for (int k = 1; k < 8; k++)
                    {
                        eight_transition_count += (p[k] == 0 && p[k + 1] == 1);
                    }
                    eight_transition_count += (p[8] == 0 && p[1] == 1);

                    int eight_connected_count = p[1] + p[2] + p[3] + p[4] + p[5] + p[6] + p[7] + p[8];

                    int four_connected_count = p[1] + p[3] + p[5] + p[7];

                    if (eight_transition_count == 1 && eight_connected_count == 4)
                    {
                        dilate_points.push(std::make_pair(i, j));
                        dilate_flag = true;
                    }

                    if (eight_transition_count == 1 && eight_connected_count == 3 && four_connected_count == 2)
                    {
                        dilate_points.push(std::make_pair(i, j));
                        dilate_flag = true;
                    }
                }
            }
        }

        while (!dilate_points.empty())
        {
            std::pair<int, int> p = dilate_points.front();
            dilate_points.pop();

            dst[p.first][p.second] = 1;
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

    for (Room& room : rooms)
    {
        room.calculate_outline(segmented_matrix);
    }

    find_connected_rooms(rooms, door_pixels);

    // Replace 1s in the segmented matrix with room id
    for (Room& room : rooms) {
        for (const auto& pixel : room.get_pixels())
        {
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

void test_final_map()
{
    const char* filename = "D:\\files\\mapfile\\dataset_occ\\seg_ori_20230519_012513_24.debug";

    // 读取地图文件并转化为01矩阵
    std::vector<std::vector<uint8_t>> binaryMatrix = readMapFile(filename);

    std::vector<std::vector<int>> origin_map = ConvertMatrixToInt(binaryMatrix);

    // 将01矩阵转化为二值图像并打印
    printBinaryImage(origin_map, 2, "origin_map");

    cv::Mat cv_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    std::vector<std::vector<int>> kernel;
    kernel.reserve(cv_kernel.rows);
    for (int i = 0; i < cv_kernel.rows; ++i) 
    {
        kernel.push_back(std::vector<int>());

        kernel[i].reserve(cv_kernel.cols);
        for (int j = 0; j < cv_kernel.cols; ++j) 
        {
            kernel[i].push_back(cv_kernel.at<uint8_t>(i, j));
        }
    }

    std::vector<std::vector<int>> optimization_map = customize_closing(extract_filled_image(origin_map), kernel);
    printBinaryImage(optimization_map, 2, "optimization_map");

    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> door_pixels =
    {
        {{216, 41}, {216,73}},
        {{111, 161}, {140, 161}},
        {{110, 175}, {110, 190}},
        {{110, 244}, {110, 258}},
        {{136, 244}, {136, 259}},
        {{115, 263}, {130, 263}},
        {{83, 315}, {83, 332}},
        {{178, 324}, {178, 338}},
        {{111, 42}, {213, 147}}
    };

    auto result = segment_rooms(optimization_map, door_pixels);
    auto& segmented_matrix = result.first;
    auto& rooms = result.second;

    for (Room& room : rooms)
    {
        room.calculate_outline(segmented_matrix);
    }

    find_connected_rooms(rooms, door_pixels);

    // Replace 1s in the segmented matrix with room id
    for (Room& room : rooms) 
    {
        for (const auto& pixel : room.get_pixels()) 
        {
            int x = pixel.first;
            int y = pixel.second;
            segmented_matrix[x][y] = room.get_room_id();
        }
    }


    // Print the connected rooms
    for (Room& room : rooms) 
    {
        room.print_connected_rooms();
    }

    auto expanded = expand_rooms(segmented_matrix, rooms);
    auto& expanded_matrix = expanded.first;
    auto& expanded_rooms = expanded.second;

    draw_map(segmented_matrix, rooms, expanded_rooms, door_pixels);

}


int main() {
    test_final_map();
    return 0;
}


