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
    // ��ʼ���շ������
    std::vector<std::vector<int>> room_matrix(matrix.size(), std::vector<int>(matrix[0].size(), 0));

    // ��䷿�����
    for (const auto& pixel : pixels)
    {
        int x = pixel.first;
        int y = pixel.second;
        room_matrix[x][y] = 1;
    }

    // ��������
    std::vector<std::vector<int>> room_filled = extract_filled_image(room_matrix);
    std::vector<std::vector<int>> room_outline = extract_edges(room_filled);

    // Ѱ�ҳ�ʼ��
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

    // ���û���ҵ����������outline_pixels������
    if (start_pixel.first == -1)
    {
        outline_pixels.clear();
        return;
    }

    // �洢��ʼ�㲢��ʼ�����������
    outline_pixels = { start_pixel };
    std::stack<std::pair<int, int>> stack;
    stack.push(start_pixel);

    // ����ͨ�������ϣ��ң���
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
                break;  // ֻѹ��һ���ھӵ�ջ��
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

    std::vector<std::vector<int>> segmented_matrix = matrix;//���������Ա���ԭʼ���󲻱�
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));//����������С��ͬ�ķ��ʱ�Ǿ���
    std::vector<Room> rooms;//���������б�
    int room_id = 1;//����ĳ�ʼ��ʶ��

    std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };//����DFS�㷨�ĸ������ƫ����

    std::function<void(int, int, Room&)> dfs = [&](int x, int y, Room& room) 
    {
        std::stack<std::pair<int, int>> stack;
        stack.push(std::make_pair(x, y));//����ǰ���ص�ѹ��ջ��

        while (!stack.empty()) 
        {
            std::pair<int, int> current = stack.top();
            stack.pop();//����ǰ���ص㵯��ջ

            int current_x = current.first;
            int current_y = current.second;
            room.add_pixel(current);//����ǰ���ص���ӵ�������

            visited[current_x][current_y] = true;//����ǰ���ص���Ϊ�ѷ���

            for (const auto& direction : directions) 
            {
                int next_x = current_x + direction.first;
                int next_y = current_y + direction.second;

                if (is_valid_pixel(next_x, next_y, rows, cols) && segmented_matrix[next_x][next_y] == 1 && !visited[next_x][next_y]) 
                {
                    stack.push(std::make_pair(next_x, next_y));//����һ����ѹ��ջ��
                }
            }
        }
    };

    // ͨ�������ؽ��зָ�
    for (const auto& door : door_pixels) 
    {
        std::vector<std::pair<int, int>> door_segment = bresenham_line(door.first.first, door.first.second, door.second.first, door.second.second);
        for (const auto& point : door_segment) 
        {
            int x = point.first;
            int y = point.second;
            if (is_valid_pixel(x, y, rows, cols)) 
            {
                segmented_matrix[x][y] = 0;//�����������ڵĵ���Ϊ0
            }
        }
    }

    // ���������е�ÿ�����أ����з��������ͱ��
    for (int i = 0; i < rows; i++) 
    {
        for (int j = 0; j < cols; j++) 
        {
            if (segmented_matrix[i][j] == 1 && !visited[i][j]) 
            {
                Room room(room_id);//����һ���µķ������
                dfs(i, j, room);//��ʼ�������������������ͬһ�������������ӵ����������
                rooms.push_back(room);//�����������ӵ������б���
                room_id++;
            }
        }
    }

    return std::make_pair(segmented_matrix, rooms);//���طָ��ľ���ͷ�������б�
}


/*  ��cv::Mat��Ϊ������������İ汾
cv::Mat extract_filled_image(const cv::Mat& connected_region) 
{
    // ����ʾ����ͨ����Ķ�ֵͼ��
    cv::Mat matrix = connected_region.clone();

    // �������Ե����
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(matrix, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // �����հ�ͼ��
    cv::Mat filled_image = cv::Mat::zeros(matrix.size(), CV_8UC1);

    // ����������
    cv::drawContours(filled_image, contours, -1, cv::Scalar(255), cv::FILLED);

    return filled_image;
}
*/

std::vector<std::vector<int>> extract_filled_image(const std::vector<std::vector<int>>& connected_region) 
{
    // ����ʾ����ͨ����Ķ�ֵͼ��
    cv::Mat matrix(connected_region.size(), connected_region[0].size(), CV_8UC1);
    for (size_t i = 0; i < connected_region.size(); i++) 
    {
        for (size_t j = 0; j < connected_region[i].size(); j++) 
        {
            matrix.at<uchar>(i, j) = connected_region[i][j];
        }
    }

    // �������Ե����
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(matrix, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // �����հ�ͼ��
    cv::Mat filled_image(matrix.size(), CV_8UC1, cv::Scalar(0));

    // ����������
    cv::drawContours(filled_image, contours, -1, cv::Scalar(1), cv::FILLED);

    // �����ת��Ϊ��ά����
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


/*  ��cv::Mat�ṹ��Ϊ��������İ汾
cv::Mat extract_edges(const cv::Mat& filled_image) 
{
    // ����01����
    cv::Mat matrix = filled_image.clone();

    // �ھ�����Χ��һȦ0
    cv::Mat padded_matrix;
    cv::copyMakeBorder(matrix, padded_matrix, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);

    // ����3x3��ȫ1�����
    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);

    // ��ÿ�����ؽ��о�����ж��Ƿ�Ϊ��Ե
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
    // ����01����
    std::vector<std::vector<int>> matrix = filled_image;

    // �ھ�����Χ��һȦ0
    std::vector<std::vector<int>> padded_matrix(matrix.size() + 2, std::vector<int>(matrix[0].size() + 2, 0));
    for (int i = 0; i < matrix.size(); i++) 
    {
        for (int j = 0; j < matrix[i].size(); j++) 
        {
            padded_matrix[i + 1][j + 1] = matrix[i][j];
        }
    }

    // ����3x3��ȫ1�����
    std::vector<std::vector<int>> kernel(3, std::vector<int>(3, 1));

    // ��ÿ�����ؽ��о�����ж��Ƿ�Ϊ��Ե
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

    // ��ԭʼ�����Ƶ��м䲿��
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            paddedMatrix[i + padRows][j + padCols] = binaryMatrix[i][j];
        }
    }

    std::vector<std::vector<int>> openedMatrix(rows, std::vector<int>(cols, 0));

    // ��ʴ����
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

    // ���Ͳ���
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

    // ��ԭʼ�����Ƶ��м䲿��
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            paddedMatrix[i + padRows][j + padCols] = binaryMatrix[i][j];
        }
    }

    std::vector<std::vector<int>> closedMatrix(rows, std::vector<int>(cols, 0));

    // ���Ͳ���
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

    // ��ʴ����
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

    //����ÿһ�Է���
    for (size_t i = 0; i < rooms.size(); i++) 
    {
        for (size_t j = i + 1; j < rooms.size(); j++) 
        {
            Room& room1 = rooms[i];
            Room& room2 = rooms[j];

            //Ϊÿ��������ȡ���ͼ����ȡ��Ե
            auto room1_filled_image = extract_filled_image(pixels_to_matrix(room1.get_pixels(), rows, cols));
            auto room2_filled_image = extract_filled_image(pixels_to_matrix(room2.get_pixels(), rows, cols));

            auto room1_edges = extract_edges(room1_filled_image);
            auto room2_edges = extract_edges(room2_filled_image);

            //���ÿ�����Ƿ�������������
            for (const auto& door : door_pixels) 
            {
                auto door_segment = bresenham_line(door.first.first, door.first.second, door.second.first, door.second.second);

                bool room1_connected = false;
                bool room2_connected = false;

                //����ŵ�ÿ�������Ƿ����ӵ�����ı�Ե
                for (const auto& pixel : door_segment) 
                {
                    if (room1_edges[pixel.first][pixel.second] == 1)
                        room1_connected = true;

                    if (room2_edges[pixel.first][pixel.second] == 1)
                        room2_connected = true;
                }

                //����������䶼��������������ӵ����ӵķ����б�
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
    //����ÿһ�Է���
    for (size_t i = 0; i < rooms.size(); i++)
    {
        for (size_t j = i + 1; j < rooms.size(); j++)
        {
            Room& room1 = rooms[i];
            Room& room2 = rooms[j];

            // ��ȡÿ��������������ص�
            const auto& room1_outline_pixels = room1.get_outline_pixels();
            const auto& room2_outline_pixels = room2.get_outline_pixels();

            //���ÿ�����Ƿ�������������
            for (const auto& door : door_pixels)
            {
                auto door_segment = bresenham_line(door.first.first, door.first.second, door.second.first, door.second.second);

                bool room1_connected = false;
                bool room2_connected = false;

                //����ŵ�ÿ�������Ƿ����ӵ����������
                for (const auto& pixel : door_segment)
                {
                    if (std::find(room1_outline_pixels.begin(), room1_outline_pixels.end(), pixel) != room1_outline_pixels.end())
                        room1_connected = true;

                    if (std::find(room2_outline_pixels.begin(), room2_outline_pixels.end(), pixel) != room2_outline_pixels.end())
                        room2_connected = true;
                }

                //����������䶼��������������ӵ����ӵķ����б�
                if (room1_connected && room2_connected)
                {
                    room1.add_connected_room(room2.get_room_id(), door);
                    room2.add_connected_room(room1.get_room_id(), door);
                }
            }
        }
    }
}


std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms) 
{
    // ����һ���µķ����б���Ϊ����
    std::vector<Room> expanded_rooms = rooms;

    // ����һ���µľ��󸱱�
    std::vector<std::vector<int>> expanded_matrix = segmented_matrix;

    // ��ȡ����Ĵ�С
    int height = segmented_matrix.size();
    int width = segmented_matrix[0].size();

    // ����һ����־��������ʾ�Ƿ������ؿ��Խ�������
    bool expansion_occurred = true;

    // ֻҪ�������ؿ������ͣ��ͼ���ѭ��
    while (expansion_occurred) 
    {
        // �ڿ�ʼ��һ�ֵ�ѭ��ʱ�����Ƚ���־����ΪFalse
        expansion_occurred = false;

        // ��������
        for (int i = 0; i < height; i++) 
        {
            for (int j = 0; j < width; j++) 
            {
                // �����ص㲻Ϊ0ʱ�����õ�λ��ĳ��������
                if (expanded_matrix[i][j] != 0) 
                {
                    int room_id = expanded_matrix[i][j];

                    // ��ȡ��Χ8���������
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

                    // ��ȡ��Χ8���������ֵ
                    std::vector<int> neighbor_pixels;
                    for (const auto& coord : neighbor_coords) 
                    {
                        neighbor_pixels.push_back(expanded_matrix[coord.first][coord.second]);
                    }

                    // �ж���Χ8�����Ƿ񶼵��ڷ���Ż�0
                    if (std::all_of(neighbor_pixels.begin(), neighbor_pixels.end(), [room_id](int pixel) { return pixel == room_id || pixel == 0; })) 
                    {
                        // ������Χ8������0������
                        int num_zeros = std::count(neighbor_pixels.begin(), neighbor_pixels.end(), 0);

                        // ���ֻ��һ��������0���򽫸����ص���Ϊ����ţ�����¼��������
                        if (num_zeros == 1 || num_zeros == 2)
                        {
                            auto zero_it = neighbor_pixels.begin();
                            while ((zero_it = std::find(zero_it, neighbor_pixels.end(), 0)) != neighbor_pixels.end())
                            {
                                int zero_index = std::distance(neighbor_pixels.begin(), zero_it);
                                std::pair<int, int> zero_coord = neighbor_coords[zero_index];
                                expanded_matrix[zero_coord.first][zero_coord.second] = room_id;
                                expanded_rooms[room_id - 1].add_pixel(zero_coord);

                                // ��Ϊ�����ص㱻�����ˣ����Խ���־����ΪTrue
                                expansion_occurred = true;

                                zero_it++;
                            }
                        }
                    }
                }
            }
        }
    }

    // ������չ��ķ�������
    for (Room& room : expanded_rooms) 
    {
        room.calculate_outline(expanded_matrix);
    }

    return { expanded_matrix, expanded_rooms };
}

void draw_map(std::vector<std::vector<int>>& segmented_matrix, std::vector<Room>& expanded_rooms, std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& door_pixels) 
{
    int h = segmented_matrix.size();
    int w = segmented_matrix[0].size();

    // ����һ��RGB��������ɫ����
    cv::Mat final_map(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    // �������RGB��ɫ
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < expanded_rooms.size(); i++) 
    {
        colors.push_back(cv::Vec3b(dis(gen), dis(gen), dis(gen)));
    }

    // Ⱦɫÿ������
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

    // ����м�ͼ��
    cv::imshow("Colored Rooms", final_map);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\Colored Rooms.jpg", final_map);
    //cv::waitKey(0);


    // ������չ��ķ�������
    std::vector<std::vector<int>> floor_plan_matrix(h, std::vector<int>(w, 0));

    for (Room& room : expanded_rooms) 
    {
        for (auto& pixel : room.get_outline_pixels()) 
        {
            //final_map.at<cv::Vec3b>(pixel.first, pixel.second) = cv::Vec3b(0, 0, 0);  // ʹ�ú�ɫ����������
            int u = pixel.first;
            int v = pixel.second;
            floor_plan_matrix[u][v] = 1;
        }
    }
    cv::Mat floor_plan_img(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
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

    zhangSuenThinning(floor_plan_matrix);
    removeBranches(floor_plan_matrix);

    cv::Mat floor_plan_img_thin(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            if (floor_plan_matrix[x][y] != 0)
            {
                floor_plan_img_thin.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    cv::imshow("floor_plan_img_thin", floor_plan_img_thin);
    cv::imwrite("C:\\Users\\13012\\Desktop\\result\\floor_plan_img_thin.jpg", floor_plan_img_thin);



    // �����ŵ��߶�
    for (auto& door : door_pixels) 
    {
        cv::line(final_map, cv::Point(door.first.second, door.first.first), cv::Point(door.second.second, door.second.first), cv::Scalar(0, 0, 255), 2);  // ��ɫ
    }

    // ��ʾ���յ�ͼ
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

    // ��������
    for (int i = 0; i < img.size(); ++i) 
    {
        for (int j = 0; j < img[i].size(); ++j) 
        {
            // ��⵽ǰ��
            if (img[i][j] != 0) 
            {
                int connectedPoints = 0;
                // ��������ͨ����ǰ��������
                for (const auto& offset : offsets) 
                {
                    int ni = i + offset.first, nj = j + offset.second;
                    if (ni >= 0 && ni < img.size() && nj >= 0 && nj < img[i].size() && img[ni][nj] != 0) 
                    {
                        connectedPoints++;
                    }
                }
                // ����������ӵ�
                if (connectedPoints > 2) 
                {
                    std::pair<int, int> intersectionPoint = { i, j };
                    lines.push_back(Line{ static_cast<int>(lines.size()), Line::INTERSECTION, {intersectionPoint}, intersectionPoint, intersectionPoint });
                    img[i][j] = 0;  // ���������Ϊ����
                }
            }
        }
    }

    return lines;
}

void extractOrthogonalLines(std::vector<std::vector<int>>& img, std::vector<Line>& lines)
{
    //��һ������ȡˮƽ��
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

                // ֻ���ǳ���5���ص��߶�
                if (end - start + 1 > 5) 
                {
                    Line line;
                    line.id = lines.size();
                    line.direction = Line::HORIZONTAL;
                    line.startPoint = { i, start };
                    line.endPoint = { i, end };
                    for (int k = start; k <= end; ++k) 
                    {
                        line.points.push_back({ i, k });
                        img[i][k] = 0;  // ����Ϊ����
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

    //�ڶ�������ȡ��ֱ��
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

                // ֻ���ǳ���5���ص��߶�
                if (end - start + 1 > 5) 
                {
                    Line line;
                    line.id = lines.size();
                    line.direction = Line::VERTICAL;
                    line.startPoint = { start, j };
                    line.endPoint = { end, j };
                    for (int k = start; k <= end; ++k) 
                    {
                        line.points.push_back({ k, j });
                        img[k][j] = 0;  // ����Ϊ����
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
    int lineId = lines.size();  //ȷ��line��id

    std::vector<std::pair<int, int>> offsets = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };  //����ͨƫ����

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (img[i][j] != 0) {  // �ҵ�һ���ߵ�һ����
                Line line;
                line.id = lineId++;
                line.direction = Line::NONLINEAR;

                std::stack<std::pair<int, int>> dfsStack;
                dfsStack.push({ i, j });

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
                            img[next.first][next.second] = 0;  //��ȡ����������Ϊ����
                        }
                    }
                }

                for (auto& point : line.points)
                {
                    int connected = 0;
                    for (auto& p : line.points)
                    {
                        if (std::abs(p.first - point.first) + std::abs(p.second - point.second) == 1) //�����پ���Ϊ1
                        {
                            connected++;
                        }
                    }

                    // ����connected��ֵ��ȷ���߶ε������յ�
                    switch (connected)
                    {
                    case 0:  //�������һ������������
                        line.startPoint = point;
                        line.endPoint = point;
                        break;
                    case 1:  //�������һ���˵�
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
                    case 2://����õ����ߵ��м䣬�ͼ���
                        break;
                    default:  //����õ��ж��������ھӣ��ͱ���
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
    // ����ͨƫ����
    std::vector<std::pair<int, int>> offsets = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

    std::priority_queue<Node> pq;
    pq.emplace(start, std::vector<std::pair<int, int>>{start}, 0); // ����ʼ�ڵ���ӵ�������

    while (!pq.empty()) {
        Node curNode = pq.top();
        pq.pop();

        std::pair<int, int> curPos = curNode.pos;
        if (curPos == end) 
        { // �����Ѿ��ҵ���һ��ͨ���յ�ĵ�·
            return curNode.path; // �������·��
        }

        for (const auto& offset : offsets) 
        {
            std::pair<int, int> newPos = { curPos.first + offset.first, curPos.second + offset.second };

            // �߽�����ϰ�����
            if (newPos.first < 0 || newPos.first >= mask.size() || newPos.second < 0 || newPos.second >= mask[0].size() || mask[newPos.first][newPos.second] == 0)
                continue;

            // ����ת����
            int newTurns = curNode.turns;
            if (curNode.path.size() > 1 && getDirection(curNode.path[curNode.path.size() - 2], curPos) != getDirection(curPos, newPos))
            {
                newTurns++;
            }

            // ����һ���µĽڵ㲢������ӵ�������
            std::vector<std::pair<int, int>> newPath = curNode.path;
            newPath.push_back(newPos);
            pq.emplace(newPos, newPath, newTurns);
        }
    }

    // ���û�д���㵽�յ��·��
    return std::vector<std::pair<int, int>>{};
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

    // ��ȡ��ͼ�ļ���ת��Ϊ01����
    std::vector<std::vector<uint8_t>> binaryMatrix = readMapFile(filename);

    std::vector<std::vector<int>> origin_map = ConvertMatrixToInt(binaryMatrix);

    // ��01����ת��Ϊ��ֵͼ�񲢴�ӡ
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

    draw_map(segmented_matrix, expanded_rooms, door_pixels);

}


int main() {
    test_final_map();
    return 0;
}


