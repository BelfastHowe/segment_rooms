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

/*
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
    //std::stack<std::pair<int, int>> expansion;

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
*/

std::pair<std::vector<std::vector<int>>, std::vector<Room>> expand_rooms(const std::vector<std::vector<int>>& segmented_matrix, const std::vector<Room>& rooms)
{
    // ����һ���µķ����б���Ϊ����
    std::vector<Room> expanded_rooms = rooms;

    // ����һ���µľ��󸱱�
    std::vector<std::vector<int>> expanded_matrix = segmented_matrix;

    // ��ȡ����Ĵ�С
    int height = segmented_matrix.size();
    int width = segmented_matrix[0].size();

    std::stack<std::pair<std::pair<int, int>, int>> expansion;
    bool expansion_occurred = true;

    while (expansion_occurred)
    {
        //��־λ��0
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
                                //expanded_matrix[zero_coord.first][zero_coord.second] = room_id;
                                //expanded_rooms[room_id - 1].add_pixel(zero_coord);

                                expansion.push(std::make_pair(zero_coord, room_id));

                                // ��Ϊ�����ص㱻�����ˣ����Խ���־����ΪTrue
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

    // ������չ��ķ�������
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
    /*
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
    */

    // ������չ��ķ�������
    std::vector<std::vector<int>> floor_plan_matrix(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> floor_plan_optimization_matrix(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> floor_plan_optimization_matrix1(h, std::vector<int>(w, 0));

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

    completion_link(floor_plan_optimization_matrix1);//����ͨ���Ӵ���ȫ




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

    // ����м�ͼ��
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
    int threshold = 15;//�����߶γ�����ֵ


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

                // ֻ���ǳ�����ֵ���߶�
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

                // ֻ���ǳ�����ֵ���߶�
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

    //��1������ȡ�����
    std::vector<Line> lines = extractIntersections(img);

    //printBinaryImage(img, 2, "img1");
    //cv::waitKey(0);

    // ��2������ȡ������
    extractOrthogonalLines(img, lines);

    //printBinaryImage(img, 2, "img2");
    //cv::waitKey(0);

    // ��3����Ѱ�ҷ���������
    findNonLinearLines(img, lines);

    printBinaryImage(img, 2, "img3");
    //cv::waitKey(0);

    //��4��������������������ʹ�������С��ת�۵�
    //������ܵĻ�������Сת���·������������Ե���·
    
    for (auto& line : lines)
    {
        if (line.direction == Line::NONLINEAR)
        {
            //�����е����������Ƿ�����Ĥ��
            bool allInMask = std::all_of(line.points.begin(), line.points.end(), [&mask](const std::pair<int, int>& p) 
                {
                    return mask[p.first][p.second] == 1; 
                });

            //������е����ض���maskǰ���У������滻����
            if (allInMask)
            {
                auto newPath = getLeastTurnPath(line.startPoint, line.endPoint, mask);
                if (!newPath.empty()) line.points = std::move(newPath);
            }
        }
    }
    

    // ���岽�����������������һ���µ�ͼ����
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
                        std::cerr << "case=1ʱ�Ҳ��������ǵ�" << std::endl;
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
                        std::cerr << "case=2ʱ�Ҳ������������Ľǵ��" << std::endl;
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
    int erode_times = 3;//��ʴ����
    double threshold = 10;//�ſ򱣻��Ŀ��

    int h = floor_plan.size();
    int w = floor_plan[0].size();

    std::vector<std::vector<int>> tidy_room(h, std::vector<int>(w, 0));

    //����ͼ����ǰ����������
    for (int x = 0; x < h; x++)
    {
        for (int y = 0; y < w; y++)
        {
            tidy_room[x][y] = floor_plan[x][y] == 0 ? 1 : 0;
        }
    }

    std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };//����ͨ

    std::stack<std::pair<int, int>> background_stack;
    background_stack.push(std::make_pair(0, 0));
    tidy_room[0][0] = 0;

    //������Ȧ��Ϊ����
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

    //������ʴ
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

    //����id�̳�
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

    //����id��ɢ
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
        //��¼��Ҫ��ʴ�ĵ�Ķ���
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

        //�Զ����еĵ���и�ʴ
        while (!points_to_erode.empty())
        {
            std::pair<int, int> point = points_to_erode.front();
            points_to_erode.pop();

            tidy_room_matrix[point.first][point.second] = 0;
        }
    }

    for (int round = 0; round < rounds; round++) 
    {
        // ʹ��һ���������洢ÿ����Ҫ��չ�ĵ�
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

        // ����Ҫ��չ�ĵ��������
        while (!points_to_expand.empty()) 
        {
            std::pair<int, int> point = points_to_expand.front();
            points_to_expand.pop();

            for (int i = 0; i < 8; i++)
            {
                int new_x = point.first + dx[i];
                int new_y = point.second + dy[i];

                // �ж��µ��Ƿ��ھ���Χ�ڣ��Լ��Ƿ���Ա�����
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
    //������ľ���ת��Ϊopencv��Mat��ʽ
    cv::Mat binary_image(room_matrix.size(), room_matrix[0].size(), CV_8U);
    for (size_t i = 0; i < room_matrix.size(); i++)
    {
        for (size_t j = 0; j < room_matrix[0].size(); j++)
        {
            binary_image.at<uchar>(i, j) = room_matrix[i][j] * 255;
        }
    }

    std::vector<std::vector<cv::Point>> contours;

    //ʹ��findContours�ҳ��ⲿ����
    cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approx_contour;

    // ���ҵ����������ж�������
    cv::approxPolyDP(contours[0], approx_contour, epsilon, true);

    // ����һ���հ�ͼ�����ڻ�����Ϻ��ͼ��
    cv::Mat result = cv::Mat::zeros(binary_image.size(), CV_8U);

    // ʹ��drawContours������Ϻ��ͼ��
    cv::drawContours(result, std::vector<std::vector<cv::Point>>{approx_contour}, -1, cv::Scalar(255), cv::FILLED);

    // ��ԭ����ȫ����0
    for (auto& row : room_matrix)
    {
        fill(row.begin(), row.end(), 0);
    }

    // �����ת���ض�ֵ���󣬲�ֱ���޸�ԭ����
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
            //������
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
            //������
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
            //���
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
            //���
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

            // ������ֵ������ֵ
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
                    //����3*3�����б�
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
                    //����3*3�����б�
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

    draw_map(segmented_matrix, rooms, expanded_rooms, door_pixels);

}


int main() {
    test_final_map();
    return 0;
}


