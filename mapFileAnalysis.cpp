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
