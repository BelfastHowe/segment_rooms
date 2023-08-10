#include <mapFileAnalysis.h>


std::vector<std::vector<uint8_t>> readMapFile(const char* filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file) 
    {
        throw std::runtime_error("Failed to open map file");
    }

    // ��ȡ��ͼ�ṹ��Ϣ
    int16_t symbol, min_x, min_y, delta_x, delta_y;
    file.read(reinterpret_cast<char*>(&symbol), sizeof(symbol));
    file.read(reinterpret_cast<char*>(&min_x), sizeof(min_x));
    file.read(reinterpret_cast<char*>(&min_y), sizeof(min_y));
    file.read(reinterpret_cast<char*>(&delta_x), sizeof(delta_x));
    file.read(reinterpret_cast<char*>(&delta_y), sizeof(delta_y));

    // �����ͼ���ݵ��ֽ���
    int mapSize = delta_x * delta_y;

    // ��ȡ��ͼ����
    std::vector<uint8_t> mapData(mapSize);
    file.read(reinterpret_cast<char*>(mapData.data()), mapSize);

    // �ر��ļ�
    file.close();

    // ������ֵ����
    std::vector<std::vector<uint8_t>> binaryMatrix(delta_x, std::vector<uint8_t>(delta_y, 0));

    // ������ͼ���ݲ����ɶ�ֵ����
    for (int i = 0; i < mapSize; ++i) {
        uint8_t value = mapData[i];
        if ((value & 0xC1) == 0xC1) {
            int x = i / delta_y;
            int y = i % delta_y;
            binaryMatrix[x][y] = 1;
        }
    }

    return binaryMatrix;

}

void printBinaryMatrix(const std::vector<std::vector<uint8_t>>& binaryMatrix)
{

    // ������ֵͼ��
    cv::Mat image(binaryMatrix.size(), binaryMatrix[0].size(), CV_8UC1);
    for (int i = 0; i < binaryMatrix.size(); ++i) 
    {
        for (int j = 0; j < binaryMatrix[i].size(); ++j)
        {
            image.at<uint8_t>(i, j) = binaryMatrix[i][j] * 255;  // ��01�����е�0��1ӳ�䵽ͼ��ĺڰ�ֵ
        }
    }
    // ��ʾ��ֵͼ��
    cv::imshow("Binary Image", image);
    //cv::waitKey(0);
   // cv::destroyAllWindows();
}

/*int main()
{
    const char* filename = "D:/files/seg_ori_20230509_073109_647.debug";

    // ��ȡ��ͼ�ļ���ת��Ϊ01����
    std::vector<std::vector<uint8_t>> binaryMatrix = readMapFile(filename);

    // ��01����ת��Ϊ��ֵͼ�񲢴�ӡ
    printBinaryMatrix(binaryMatrix);

    return 0;
}
*/


std::vector<std::vector<int>> ConvertImageToMatrix(const std::string& imagePath) 
{
    // Load the image
    cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);

    // Check if image is loaded successfully
    if (img.empty()) 
    {
        std::cerr << "Failed to load image: " << imagePath << std::endl;
        return {};
    }

    // Convert the image to grayscale
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    // Binarize the image
    cv::Mat img_binary;
    cv::threshold(img_gray, img_binary, 128, 1, cv::THRESH_BINARY_INV);

    // Convert the binary image to a 2D vector
    std::vector<std::vector<int>> matrix;
    matrix.resize(img_binary.rows, std::vector<int>(img_binary.cols));

    for (int i = 0; i < img_binary.rows; ++i) 
    {
        for (int j = 0; j < img_binary.cols; ++j) 
        {
            matrix[i][j] = img_binary.at<uchar>(i, j);
        }
    }

    return matrix;
}

