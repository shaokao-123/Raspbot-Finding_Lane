#include <iostream>  
#include <vector>    
#include <cmath>     
#include <opencv2/opencv.hpp>  

using namespace std;  
using namespace cv;   

extern "C" {

    // 车道中心线计算结果结构体
    struct LaneResult {
        int center_x;
        int center_y;
        int left_bound;
        int right_bound;
        bool detected;

        LaneResult() : center_x(0), center_y(0), left_bound(0), right_bound(0), detected(false) {}
    };

    // 车道中心线计算器类
    class LaneCalculator {
    private:
        int image_width;
        int image_height;
        int min_white_pixels;

    public:
        LaneCalculator(int width = 320, int height = 240)
            : image_width(width), image_height(height), min_white_pixels(6) {}

        /**
         * @brief 从二值化图片计算车道中心线坐标
         * @param binary_image 二值化图片数据 (0=黑色, 255=白色)
         * @return 车道中心线计算结果
         * @details 这是核心函数，实现基于Canny边缘检测的车道检测完整流程
         */
        LaneResult calculateLaneCenter(const vector<vector<int>>& binary_image) {
            LaneResult result;  // 创建结果对象，用来存储计算结果

            // 第一步：检查输入数据是否有效
            if (binary_image.empty() || binary_image[0].empty()) {
                return result;  // 如果数据无效，返回空结果
            }

            // 第二步：使用OpenCV Canny边缘检测获取车道边缘
            Mat edges = detectEdgesWithOpenCV(binary_image);  // 使用OpenCV检测图像边缘

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
                            if (edges.at<uchar>(y, x) == 255) {  // 如果是边缘点（使用OpenCV的访问方式）
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
         * @brief 使用OpenCV Canny边缘检测算法
         * @param binary_image 输入的二值化图像
         * @return 边缘检测结果图像（OpenCV Mat格式）
         * @details 使用OpenCV的Canny函数进行边缘检测，更准确更高效
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
         * @details 通过分析OpenCV边缘点分布提取车道边界
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
     * @brief Python调用的主要函数：计算车道中心线坐标
     * @param image_data 二值化图像数据指针（一维数组）
     * @param width 图像宽度
     * @param height 图像高度
     * @param result 结果数组指针 [center_x, center_y]
     * @return 1表示成功，0表示失败
     * @details 这是Python通过ctypes调用的主要接口函数
     */
    int calculate_lane_center(int* image_data, int width, int height, int* result) {
        // 初始化结果
        result[0] = -1;  // center_x
        result[1] = -1;  // center_y

        // 检查输入参数有效性
        if (image_data == nullptr || result == nullptr || width <= 0 || height <= 0) {
            return 0;  // 参数无效，返回失败
        }

        try {
            // 将一维数组转换为二维vector格式
            vector<vector<int>> binary_image(height, vector<int>(width));
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    binary_image[y][x] = image_data[y * width + x];
                }
            }

            // 创建计算器并计算车道中心
            LaneCalculator calculator(width, height);
            LaneResult lane_result = calculator.calculateLaneCenter(binary_image);

            // 检查检测结果
            if (lane_result.detected) {
                result[0] = lane_result.center_x;  // 设置center_x
                result[1] = lane_result.center_y;  // 设置center_y
                return 1;  // 检测成功
            }
            else {
                return 0;  // 检测失败
            }

        }
        catch (...) {
            // 捕获任何异常，返回失败
            return 0;
        }
    }

}