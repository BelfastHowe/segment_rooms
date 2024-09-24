#include <generate_connected_region.h>

using namespace std;

int imwrite_mdy_private(cv::InputArray input, const string file_name)
{
    cv::Mat src = input.getMat().clone();

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::tm now_tm;
    localtime_s(&now_tm, &now_c);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S_") << now.time_since_epoch().count() << string("_");

    std::string output_file_name = string("C:\\Users\\Belfast\\OneDrive\\Desktop\\result\\") + oss.str() + file_name + string(".png");

    std::cout << output_file_name << std::endl;
    cv::imwrite(output_file_name, src);
    cv::waitKey(1);

    return 0;
}

std::vector<std::vector<int>> generate_connected_region(int length, int width, int desired_area) 
{
    // 创建一个空白的矩阵
    std::vector<std::vector<int>> matrix(length, std::vector<int>(width, 0));
    std::vector<std::vector<bool>> visited(length, std::vector<bool>(width, false));
    int connected_area = 0;

    // 计算矩阵的中心点坐标
    int center_x = length / 2;
    int center_y = width / 2;

    // 随机选择一个起始点，使其尽可能位于中央
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist_x(center_x - length / 4, center_x + length / 4);
    std::uniform_int_distribution<int> dist_y(center_y - width / 4, center_y + width / 4);
    int start_x = dist_x(gen);
    int start_y = dist_y(gen);

    std::vector<std::pair<int, int>> stack;
    stack.push_back(std::make_pair(start_x, start_y));

    while (!stack.empty()) 
    {
        int x = stack.back().first;
        int y = stack.back().second;
        stack.pop_back();

        if (visited[x][y]) 
        {
            continue;
        }

        visited[x][y] = true;
        matrix[x][y] = 1;
        connected_area++;

        // 随机选择邻居节点
        std::vector<std::pair<int, int>> neighbors = { {x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1} };
        std::shuffle(neighbors.begin(), neighbors.end(), gen);

        for (const auto& neighbor : neighbors) 
        {
            int neighbor_x = neighbor.first;
            int neighbor_y = neighbor.second;
            if (1 <= neighbor_x && neighbor_x < length - 1 && 1 <= neighbor_y && neighbor_y < width - 1 && !visited[neighbor_x][neighbor_y]) 
            {
                stack.push_back(std::make_pair(neighbor_x, neighbor_y));
            }
        }

        // 如果已达到所需面积，则停止扩展
        if (connected_area >= desired_area) 
        {
            break;
        }
    }

    return matrix;
}

void printBinaryImage(const std::vector<std::vector<int>>& matrix, int scale, const char* windowName)
{
    int length = matrix.size();
    int width = matrix[0].size();

    cv::Mat image(length, width, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < length; i++) 
    {
        for (int j = 0; j < width; j++) 
        {
            if (matrix[i][j] > 0) 
            {
                image.at<uchar>(i, j) = 255;
            }
        }
    }

    cv::Mat scaledImage;
    cv::resize(image, scaledImage, cv::Size(width * scale, length * scale),cv::INTER_NEAREST);

    cv::imshow(windowName, scaledImage);
    //cv::waitKey(0);

    std::string outputPath = "C:\\Users\\13012\\Desktop\\result\\" + std::string(windowName) + ".png"; // 修改为你想要保存的路径
    cv::imwrite(outputPath, scaledImage);

}


