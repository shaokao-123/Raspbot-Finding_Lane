#include <iostream>  
#include <vector>    
#include <cmath>     
#include <fstream>   
#include <sstream>   
#include <string>    
#include <opencv2/opencv.hpp> 

using namespace std;
using namespace cv;

// 车道中心线计算结果结构体
struct LaneResult {
    int center_x;
    int center_y;
    int left_bound;
    int right_bound;
    bool detected;

    LaneResult() : center_x(0), center_y(0), left_bound(0), right_bound(0), detected(false) {}
};

// 车道中心线计算类
class LaneCalculator {
private:
    int image_width;
    int image_height;
    int min_white_pixels;

public:
    LaneCalculator(int width = 320, int height = 240)
        : image_width(width), image_height(height), min_white_pixels(6) {}


    LaneResult calculateLaneCenter(const vector<vector<int>>& binary_image) {
        LaneResult result;  // 创建结果对象，用来存储计算结果

        // 第一步：检查输入数据是否有效
        if (binary_image.empty() || binary_image[0].empty()) {
            return result;
        }

        // 第二步：使用 Canny边缘检测获取车道边缘
        Mat edges = detectEdgesWithOpenCV(binary_image);

        // 第三步：从边缘中提取车道边界
        vector<int> left_edges, right_edges;  // 存储左右车道边缘点
        extractLaneEdges(edges, left_edges, right_edges);  // 提取车道边缘

        // 第四步：计算车道中心线
        vector<int> valid_columns;  // 存储有效列的X坐标

        if (!left_edges.empty() && !right_edges.empty()) {
            // 使用边缘点计算车道中心
            for (int x = 0; x < image_width; ++x) {  // 检查每一列
                int edge_count = 0;  // 统计这一列的边缘点数量
                for (int y = 0; y < image_height; ++y) {  // 遍历这一列的所有行
                    if (y < edges.rows && x < edges.cols) {  // 检查边界
                        if (edges.at<uchar>(y, x) == 255) {  // 如果是边缘点
                            edge_count++;  // 边缘点数量+1
                        }
                    }
                }
                if (edge_count > min_white_pixels) {  // 如果这一列有足够的边缘点
                    valid_columns.push_back(x);  // 把这一列加入有效列列表
                }
            }
        }

        // 第五步：检查是否找到有效车道
        if (valid_columns.empty()) {
            return result;  // 如果没有找到有效车道，返回检测失败的结果
        }

        // 第六步：计算车道边界和中心
        int left_bound = valid_columns[0];  // 左边界：有效列中最小的X坐标
        int right_bound = valid_columns[valid_columns.size() - 1];  // 右边界：有效列中最大的X坐标
        int lane_center = calculateEdgeBasedCenter(left_edges, right_edges, valid_columns);  // 使用边缘检测计算车道中心

        // 第七步：保存计算结果
        result.center_x = lane_center;      // 车道中心X坐标
        result.center_y = image_height / 2; // 车道中心Y坐标（图片中间位置）
        result.left_bound = left_bound;     // 左边界X坐标
        result.right_bound = right_bound;   // 右边界X坐标
        result.detected = true;              // 标记为检测成功

        return result;  // 返回检测结果
    }

private:
    /**
     * @brief 使用 Canny边缘检测算法
     * @param binary_image 输入的二值化图像
     * @return 边缘检测结果图像
     */
    Mat detectEdgesWithOpenCV(const vector<vector<int>>& binary_image) {
        int height = binary_image.size();  // 获取图像高度
        int width = binary_image[0].size();  // 获取图像宽度

        // 将vector转换为OpenCV Mat格式
        Mat binary_mat(height, width, CV_8UC1);  // 创建8位单通道图像
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                binary_mat.at<uchar>(y, x) = static_cast<uchar>(binary_image[y][x]);  // 复制像素值
            }
        }

        // 使用OpenCV的Canny边缘检测
        Mat edges;
        Canny(binary_mat, edges, 50, 150);  // 低阈值50，高阈值150

        return edges;  // 返回边缘检测结果
    }

    /**
     * @brief 从OpenCV边缘图像中提取车道边缘
     * @param edges OpenCV边缘检测结果
     * @param left_edges 输出：左车道边缘点
     * @param right_edges 输出：右车道边缘点
     * @details 通过分析边缘点分布提取车道边界
     */
    void extractLaneEdges(const Mat& edges,
        vector<int>& left_edges, vector<int>& right_edges) {
        int height = edges.rows;  // 获取图像高度
        int width = edges.cols;   // 获取图像宽度

        vector<int> edge_counts(width, 0);  // 统计每列的边缘点数量
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                if (edges.at<uchar>(y, x) == 255) {  // 如果是边缘点
                    edge_counts[x]++;  // 这一列的边缘点数量+1
                }
            }
        }

        int mid_x = width / 2;  // 图像中心X坐标
        int left_max = 0, right_max = 0;  // 左右半部分的最大边缘密度
        int left_peak = 0, right_peak = 0;  // 左右半部分的最大边缘密度位置

        // 在左半部分找最大边缘密度
        for (int x = 0; x < mid_x; ++x) {
            if (edge_counts[x] > left_max) {
                left_max = edge_counts[x];
                left_peak = x;
            }
        }

        // 在右半部分找最大边缘密度
        for (int x = mid_x; x < width; ++x) {
            if (edge_counts[x] > right_max) {
                right_max = edge_counts[x];
                right_peak = x;
            }
        }

        // 提取左车道边缘点
        if (left_max > 0) {
            for (int y = 0; y < height; ++y) {
                if (edges.at<uchar>(y, left_peak) == 255) {  // 如果是边缘点
                    left_edges.push_back(left_peak);  // 添加到左边缘列表
                }
            }
        }

        // 提取右车道边缘点
        if (right_max > 0) {
            for (int y = 0; y < height; ++y) {
                if (edges.at<uchar>(y, right_peak) == 255) {  // 如果是边缘点
                    right_edges.push_back(right_peak);  // 添加到右边缘列表
                }
            }
        }
    }

    // 基于边缘检测计算车道中心
    int calculateEdgeBasedCenter(const vector<int>& left_edges, const vector<int>& right_edges,
        const vector<int>& valid_columns) {
        if (valid_columns.empty()) {
            return 0;
        }

        if (!left_edges.empty() && !right_edges.empty()) {
            int left_avg = 0, right_avg = 0;

            for (int edge : left_edges) {
                left_avg += edge;
            }
            left_avg /= left_edges.size();

            for (int edge : right_edges) {
                right_avg += edge;
            }
            right_avg /= right_edges.size();

            return (left_avg + right_avg) / 2;
        }

        return (valid_columns[0] + valid_columns[valid_columns.size() - 1]) / 2;
    }
};

/**
 * @brief 从文本文件读取黑白二值图数据
 * @param filename 图片数据文件名
 * @return 二值化图片数据
 * @details 读取黑白二值图文本文件
 */
vector<vector<int>> loadBinaryImageFromFile(const string& filename) {
    vector<vector<int>> binary_image;
    ifstream file(filename);

    if (!file.is_open()) {
        cout << "错误: 无法打开文件 " << filename << endl;
        return binary_image;  // 返回空图像
    }

    string line;
    while (getline(file, line) && !line.empty()) {  // 读取每一行
        vector<int> row;  // 创建一行数据
        istringstream iss(line);  // 创建字符串流
        int pixel_value;

        // 解析每一行的像素值
        while (iss >> pixel_value) {  // 读取每个像素值
            row.push_back(pixel_value);  // 添加到行中
        }

        if (!row.empty()) {
            binary_image.push_back(row);  // 添加到图像中
        }
    }

    file.close();
    cout << "成功读取图片数据: " << binary_image[0].size() << "x" << binary_image.size() << endl;

    return binary_image;
}

/**
 * @brief 创建测试用的二值化图片数据
 */
vector<vector<int>> createTestImage(int width = 320, int height = 240) {
    vector<vector<int>> binary_image(height, vector<int>(width, 0));

    int left_start = static_cast<int>(width * 0.2);
    int left_end = static_cast<int>(width * 0.25);
    int right_start = static_cast<int>(width * 0.75);
    int right_end = static_cast<int>(width * 0.8);

    for (int y = 0; y < height; ++y) {
        for (int x = left_start; x < left_end; ++x) {
            if (x < width) binary_image[y][x] = 255;
        }

        int right_width = right_end - right_start;
        if (y > height / 2) {
            right_width = right_width / 2;
        }
        for (int x = right_start; x < right_start + right_width; ++x) {
            if (x < width) binary_image[y][x] = 255;
        }
    }

    return binary_image;
}


// 主函数
int main() {
    LaneCalculator calculator(320, 240);
    vector<vector<int>> binary_image;

    // 自动尝试读取数据文件
    string data_file = "lane_data.txt";  // 默认数据文件名
    binary_image = loadBinaryImageFromFile(data_file);

    if (binary_image.empty()) {
        // 如果读取失败，使用测试图片
        cout << "未找到实际车道图片" << endl;
        binary_image = createTestImage(320, 240);
    }
    else {
        cout << "成功读取数据文件" << endl;
    }

    // 计算车道中心线坐标
    LaneResult result = calculator.calculateLaneCenter(binary_image);

    // 输出结果（适合循迹小车使用）
    if (result.detected) {
        // 输出车道中心坐标，供循迹小车使用
        cout << result.center_x << " " << result.center_y << endl;
    }
    else {
        // 检测失败时输出-1 -1，表示未检测到车道
        cout << "-1 -1" << endl;
    }

    return 0;
}