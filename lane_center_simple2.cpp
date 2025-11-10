/**
 * @file lane_center_simple.cpp
 * @brief 车道中心线坐标计算器（DLL版本）
 * @details 从Python传递的二值化图片数据计算车道中心线坐标
 * @author 循迹小车项目组
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

extern "C" {

struct LaneResult {
    int center_x;
    int center_y;
    int left_bound;
    int right_bound;
    bool detected;
    
    LaneResult() : center_x(0), center_y(0), left_bound(0), right_bound(0), detected(false) {}
};

class LaneCalculator {
private:
    int image_width;
    int image_height;
    int min_white_pixels;
    
public:
    LaneCalculator(int width = 320, int height = 240) 
        : image_width(width), image_height(height), min_white_pixels(2) {}
    
    LaneResult calculateLaneCenter(const vector<vector<int>>& binary_image) {
        LaneResult result;
        
        if (binary_image.empty() || binary_image[0].empty()) {
            return result;
        }
        
        Mat processed_image = preprocessImage(binary_image);
        Mat edges = detectEdgesWithOpenCV(processed_image);
        vector<int> left_edges, right_edges;
        extractLaneEdges(edges, left_edges, right_edges);
        vector<int> valid_columns = extractValidColumnsFromEdges(edges);
        
        if (valid_columns.empty()) {
            valid_columns = extractValidColumnsFromPixels(processed_image);
        }
        
        if (valid_columns.empty()) {
            return result;
        }
        
        // 改进边界提取：不按图像中心分割，找到两个最大的连续区域
        int left_bound = valid_columns[0];
        int right_bound = valid_columns[valid_columns.size() - 1];
        
        if (valid_columns.size() > 8) {
            // 找到所有连续区域
            vector<pair<int, int>> regions;  // (起始索引, 区域大小)
            int current_region_start = 0;
            int current_region_size = 1;
            
            for (int i = 1; i < valid_columns.size(); ++i) {
                if (valid_columns[i] - valid_columns[i-1] <= 3) {
                    current_region_size++;
                } else {
                    regions.push_back(make_pair(current_region_start, current_region_size));
                    current_region_start = i;
                    current_region_size = 1;
                }
            }
            regions.push_back(make_pair(current_region_start, current_region_size));
            
            // 找到两个最大的区域（不限定位置）
            int region1_idx = -1, region1_size = 0;
            int region2_idx = -1, region2_size = 0;
            int min_region_distance = 20;  // 两个区域之间的最小距离
            
            for (int i = 0; i < regions.size(); ++i) {
                if (regions[i].second > region1_size) {
                    region2_idx = region1_idx;
                    region2_size = region1_size;
                    region1_idx = i;
                    region1_size = regions[i].second;
                } else if (regions[i].second > region2_size) {
                    // 检查与region1的距离
                    if (region1_idx < 0) {
                        region2_idx = i;
                        region2_size = regions[i].second;
                    } else {
                        int x1 = valid_columns[regions[i].first];
                        int x2 = valid_columns[regions[region1_idx].first];
                        int distance = (x1 > x2) ? (x1 - x2) : (x2 - x1);
                        if (distance >= min_region_distance) {
                            region2_idx = i;
                            region2_size = regions[i].second;
                        }
                    }
                }
            }
            
            // 如果只有一个区域，尝试找到第二个区域（距离足够远）
            if (region1_idx >= 0 && region2_idx < 0) {
                for (int i = 0; i < regions.size(); ++i) {
                    if (i != region1_idx) {
                        int x1 = valid_columns[regions[i].first];
                        int x2 = valid_columns[regions[region1_idx].first];
                        int distance = (x1 > x2) ? (x1 - x2) : (x2 - x1);
                        if (distance >= min_region_distance && regions[i].second > region2_size) {
                            region2_idx = i;
                            region2_size = regions[i].second;
                        }
                    }
                }
            }
            
            // 确定左右边界（较小的x是左边界，较大的x是右边界）
            if (region1_idx >= 0 && region2_idx >= 0) {
                int x1 = valid_columns[regions[region1_idx].first];
                int x2 = valid_columns[regions[region2_idx].first];
                
                if (x1 < x2) {
                    left_bound = x1;
                    right_bound = x2;
                } else {
                    left_bound = x2;
                    right_bound = x1;
                }
                
                // 使用区域的中点作为边界（更准确）
                if (regions[region1_idx].second >= 3) {
                    int region1_end = regions[region1_idx].first + regions[region1_idx].second - 1;
                    int region1_center = (valid_columns[regions[region1_idx].first] + valid_columns[region1_end]) / 2;
                    if (x1 < x2) {
                        left_bound = region1_center;
                    } else {
                        right_bound = region1_center;
                    }
                }
                
                if (regions[region2_idx].second >= 3) {
                    int region2_end = regions[region2_idx].first + regions[region2_idx].second - 1;
                    int region2_center = (valid_columns[regions[region2_idx].first] + valid_columns[region2_end]) / 2;
                    if (x1 < x2) {
                        right_bound = region2_center;
                    } else {
                        left_bound = region2_center;
                    }
                }
            } else if (region1_idx >= 0) {
                // 只有一个区域，分析区域位置和有效列分布
                int region_start = valid_columns[regions[region1_idx].first];
                int region_end = valid_columns[regions[region1_idx].first + regions[region1_idx].second - 1];
                int region_center = (region_start + region_end) / 2;
                
                // 检查有效列的分布：如果区域左边有很多有效列，说明可能是右边界
                // 如果区域右边有很多有效列，说明可能是左边界
                int left_count = 0, right_count = 0;
                for (int x : valid_columns) {
                    if (x < region_start) left_count++;
                    if (x > region_end) right_count++;
                }
                
                // 根据有效列分布和区域位置判断
                if (left_count > right_count && left_count > 3) {
                    // 左边有效列多，说明这个区域可能是右边界
                    right_bound = region_center;
                    // 找到左边有效列的最大值（最接近区域的值）
                    int max_left = -1;
                    for (int x : valid_columns) {
                        if (x < region_start && x > max_left) {
                            max_left = x;
                        }
                    }
                    if (max_left >= 0) {
                        left_bound = max_left;
                    } else {
                        // 如果没有左边有效列，估算
                        left_bound = max(0, right_bound - image_width / 3);
                    }
                } else if (right_count > left_count && right_count > 3) {
                    // 右边有效列多，说明这个区域可能是左边界
                    left_bound = region_center;
                    // 找到右边有效列的最小值（最接近区域的值）
                    int min_right = image_width;
                    for (int x : valid_columns) {
                        if (x > region_end && x < min_right) {
                            min_right = x;
                        }
                    }
                    if (min_right < image_width) {
                        right_bound = min_right;
                    } else {
                        // 如果没有右边有效列，估算
                        right_bound = min(image_width - 1, left_bound + image_width / 3);
                    }
                } else {
                    // 无法确定，使用区域边界（可能是车道的一部分）
                    left_bound = region_start;
                    right_bound = region_end;
                    // 如果区域很小，可能是噪声，尝试扩展或使用全局边界
                    if (region_end - region_start < image_width / 6) {
                        // 区域太小，可能是噪声，使用全局最小/最大值
                        left_bound = valid_columns[0];
                        right_bound = valid_columns[valid_columns.size() - 1];
                    }
                }
            }
        }
        
        int lane_center = calculateRobustCenter(left_edges, right_edges, valid_columns, processed_image);
        
        result.center_x = lane_center;
        result.center_y = image_height / 2;
        result.left_bound = left_bound;
        result.right_bound = right_bound;
        result.detected = true;
        
        return result;
    }
    
private:
    Mat preprocessImage(const vector<vector<int>>& binary_image) {
        int height = binary_image.size();
        int width = binary_image[0].size();
        
        Mat binary_mat(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                binary_mat.at<uchar>(y, x) = static_cast<uchar>(binary_image[y][x]);
            }
        }
        
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat processed;
        morphologyEx(binary_mat, processed, MORPH_OPEN, kernel);
        morphologyEx(processed, processed, MORPH_CLOSE, kernel);
        
        return processed;
    }
    
    Mat detectEdgesWithOpenCV(const Mat& binary_mat) {
        Mat edges;
        double mean_val = mean(binary_mat)[0];
        int low_threshold = max(30, static_cast<int>(mean_val * 0.3));
        int high_threshold = min(200, static_cast<int>(mean_val * 1.5));
        Canny(binary_mat, edges, low_threshold, high_threshold);
        return edges;
    }
    
    void extractLaneEdges(const Mat& edges, 
                         vector<int>& left_edges, vector<int>& right_edges) {
        int height = edges.rows;
        int width = edges.cols;
        int roi_start = height / 2;
        
        // 统计所有列的边缘点数量
        vector<int> edge_counts(width, 0);
        for (int x = 0; x < width; ++x) {
            for (int y = roi_start; y < height; ++y) {
                if (edges.at<uchar>(y, x) == 255) {
                    edge_counts[x]++;
                }
            }
        }
        
        // 找到所有峰值（局部最大值）
        // 峰值定义：比左右邻居都大，且值>=阈值
        vector<pair<int, int>> peaks;  // (x坐标, 边缘点数量)
        int min_peak_value = 2;  // 峰值最小阈值
        int min_peak_distance = 15;  // 两个峰值之间的最小距离
        
        for (int x = 1; x < width - 1; ++x) {
            if (edge_counts[x] >= min_peak_value &&
                edge_counts[x] > edge_counts[x-1] &&
                edge_counts[x] > edge_counts[x+1]) {
                peaks.push_back(make_pair(x, edge_counts[x]));
            }
        }
        
        // 如果没有找到峰值，尝试找全局最大值
        if (peaks.empty()) {
            int max_count = 0, max_x = 0;
            for (int x = 0; x < width; ++x) {
                if (edge_counts[x] > max_count) {
                    max_count = edge_counts[x];
                    max_x = x;
                }
            }
            if (max_count >= min_peak_value) {
                peaks.push_back(make_pair(max_x, max_count));
            }
        }
        
        // 找到两个最大的峰值（不限定位置）
        int peak1_x = -1, peak1_count = 0;
        int peak2_x = -1, peak2_count = 0;
        
        for (const auto& peak : peaks) {
            if (peak.second > peak1_count) {
                peak2_x = peak1_x;
                peak2_count = peak1_count;
                peak1_x = peak.first;
                peak1_count = peak.second;
            } else if (peak.second > peak2_count) {
                // 检查与peak1的距离
                int distance = (peak.first > peak1_x) ? (peak.first - peak1_x) : (peak1_x - peak.first);
                if (peak1_x < 0 || distance >= min_peak_distance) {
                    peak2_x = peak.first;
                    peak2_count = peak.second;
                }
            }
        }
        
        // 如果只有一个峰值，尝试找到第二个峰值（距离足够远）
        if (peak1_x >= 0 && peak2_x < 0) {
            for (const auto& peak : peaks) {
                if (peak.first != peak1_x) {
                    int distance = (peak.first > peak1_x) ? (peak.first - peak1_x) : (peak1_x - peak.first);
                    if (distance >= min_peak_distance && peak.second > peak2_count) {
                        peak2_x = peak.first;
                        peak2_count = peak.second;
                    }
                }
            }
        }
        
        // 确定左右边缘（较小的x是左边缘，较大的x是右边缘）
        int left_peak = -1, right_peak = -1;
        
        if (peak1_x >= 0 && peak2_x >= 0) {
            if (peak1_x < peak2_x) {
                left_peak = peak1_x;
                right_peak = peak2_x;
            } else {
                left_peak = peak2_x;
                right_peak = peak1_x;
            }
        } else if (peak1_x >= 0) {
            // 只有一个峰值，需要判断是左边缘还是右边缘
            // 方法：检查峰值周围的边缘点分布
            // 如果峰值左边边缘点多，可能是右边缘；如果右边边缘点多，可能是左边缘
            
            int left_side_count = 0, right_side_count = 0;
            int check_range = min(20, width / 4);  // 检查范围
            
            // 统计峰值左边和右边的边缘点数量
            for (int x = max(0, peak1_x - check_range); x < peak1_x; ++x) {
                if (edge_counts[x] > 0) {
                    left_side_count += edge_counts[x];
                }
            }
            for (int x = peak1_x + 1; x <= min(width - 1, peak1_x + check_range); ++x) {
                if (edge_counts[x] > 0) {
                    right_side_count += edge_counts[x];
                }
            }
            
            // 根据分布判断：如果左边边缘点多，说明峰值可能是右边缘
            // 如果右边边缘点多，说明峰值可能是左边缘
            // 使用整数比较：left_side_count * 2 > right_side_count * 3 等价于 left_side_count > right_side_count * 1.5
            if (left_side_count * 2 > right_side_count * 3 && left_side_count > 3) {
                // 左边边缘点多，可能是右边缘
                right_peak = peak1_x;
            } else if (right_side_count * 2 > left_side_count * 3 && right_side_count > 3) {
                // 右边边缘点多，可能是左边缘
                left_peak = peak1_x;
            } else {
                // 无法确定，根据峰值位置判断（辅助判断）
                if (peak1_x < width / 3) {
                    left_peak = peak1_x;
                } else if (peak1_x > width * 2 / 3) {
                    right_peak = peak1_x;
                } else {
                    // 在中间，默认作为左边缘（保守策略）
                    left_peak = peak1_x;
                }
            }
        }
        
        // 提取左边缘点
        if (left_peak >= 0) {
            for (int y = 0; y < height; ++y) {
                if (edges.at<uchar>(y, left_peak) == 255) {
                    left_edges.push_back(left_peak);
                }
            }
        }
        
        // 提取右边缘点
        if (right_peak >= 0) {
            for (int y = 0; y < height; ++y) {
                if (edges.at<uchar>(y, right_peak) == 255) {
                    right_edges.push_back(right_peak);
                }
            }
        }
    }
    
    vector<int> extractValidColumnsFromEdges(const Mat& edges) {
        vector<int> valid_columns;
        int height = edges.rows;
        int width = edges.cols;
        int roi_start = height / 2;
        
        for (int x = 0; x < width; ++x) {
            int edge_count = 0;
            for (int y = roi_start; y < height; ++y) {
                if (edges.at<uchar>(y, x) == 255) {
                    edge_count++;
                }
            }
            if (edge_count >= min_white_pixels) {
                valid_columns.push_back(x);
            }
        }
        return valid_columns;
    }
    
    vector<int> extractValidColumnsFromPixels(const Mat& binary_mat) {
        vector<int> valid_columns;
        int height = binary_mat.rows;
        int width = binary_mat.cols;
        int roi_start = height / 2;
        
        for (int x = 0; x < width; ++x) {
            int white_count = 0;
            for (int y = roi_start; y < height; ++y) {
                if (binary_mat.at<uchar>(y, x) == 255) {
                    white_count++;
                }
            }
            if (white_count >= min_white_pixels) {
                valid_columns.push_back(x);
            }
        }
        return valid_columns;
    }
    
    int calculateRobustCenter(const vector<int>& left_edges, const vector<int>& right_edges,
                             const vector<int>& valid_columns, const Mat& binary_mat) {
        if (valid_columns.empty()) {
            return image_width / 2;
        }
        
        // 方法1：使用左右边缘平均值
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
            
            if (left_avg < right_avg && left_avg >= 0 && right_avg < image_width) {
                return (left_avg + right_avg) / 2;
            }
        }
        
        // 方法2a：只有左边缘
        if (!left_edges.empty() && right_edges.empty()) {
            int left_avg = 0;
            for (int edge : left_edges) {
                left_avg += edge;
            }
            left_avg /= left_edges.size();
            int right_bound = valid_columns[valid_columns.size() - 1];
            int estimated_right = min(image_width - 1, left_avg + image_width / 3);
            return (left_avg + estimated_right) / 2;
        }
        
        // 方法2b：只有右边缘
        if (left_edges.empty() && !right_edges.empty()) {
            int right_avg = 0;
            for (int edge : right_edges) {
                right_avg += edge;
            }
            right_avg /= right_edges.size();
            int left_bound = valid_columns[0];
            int estimated_left = max(0, right_avg - image_width / 3);
            return (estimated_left + right_avg) / 2;
        }
        
        // 方法3：使用有效列的边界
        int left_bound = valid_columns[0];
        int right_bound = valid_columns[valid_columns.size() - 1];
        
        // 方法4：加权中心（如果有效列太少）
        if (valid_columns.size() < 10) {
            int height = binary_mat.rows;
            int roi_start = height / 2;
            vector<int> weighted_x;
            for (int x : valid_columns) {
                int weight = 0;
                for (int y = roi_start; y < height; ++y) {
                    if (binary_mat.at<uchar>(y, x) == 255) {
                        weight++;
                    }
                }
                for (int i = 0; i < weight; ++i) {
                    weighted_x.push_back(x);
                }
            }
            if (!weighted_x.empty()) {
                int sum = 0;
                for (int x : weighted_x) {
                    sum += x;
                }
                return sum / weighted_x.size();
            }
        }
        
        // 默认方法：使用加权边界计算中心
        int height = binary_mat.rows;
        int roi_start = height / 2;
        int left_weight = 0, right_weight = 0;
        
        for (int y = roi_start; y < height; ++y) {
            if (binary_mat.at<uchar>(y, left_bound) == 255) {
                left_weight++;
            }
        }
        
        for (int y = roi_start; y < height; ++y) {
            if (binary_mat.at<uchar>(y, right_bound) == 255) {
                right_weight++;
            }
        }
        
        if (left_weight > 0 && right_weight > 0) {
            int total_weight = left_weight + right_weight;
            int weighted_center = (left_bound * left_weight + right_bound * right_weight) / total_weight;
            return weighted_center;
        }
        
        return (left_bound + right_bound) / 2;
    }
};

int calculate_lane_center(int* image_data, int width, int height, int* result) {
    result[0] = -1;
    result[1] = -1;
    
    if (image_data == nullptr || result == nullptr || width <= 0 || height <= 0) {
        return 0;
    }
    
    try {
        vector<vector<int>> binary_image(height, vector<int>(width));
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                binary_image[y][x] = image_data[y * width + x];
            }
        }
        
        LaneCalculator calculator(width, height);
        LaneResult lane_result = calculator.calculateLaneCenter(binary_image);
        
        if (lane_result.detected) {
            result[0] = lane_result.center_x;
            result[1] = lane_result.center_y;
            return 1;
        } else {
            return 0;
        }
        
    } catch (...) {
        return 0;
    }
}

}  // extern "C"
