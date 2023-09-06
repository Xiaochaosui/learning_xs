//
// Created by xcsy on 23-8-21.
//
#include <iostream>
#include <vector>
#include <algorithm>
#include "Algorithm.h"

// 中值滤波函数
std::vector<int> medianFilter(const std::vector<int>& input, int windowSize) {
    int halfWindow = windowSize / 2;
    std::vector<int> filteredData;

    for (int i = 0; i < input.size(); ++i) {
        // 窗口范围
        int start = std::max(0, i - halfWindow);
        int end = std::min(static_cast<int>(input.size()) - 1, i + halfWindow);

        // 在窗口内取数据
        std::vector<int> windowData;
        for (int j = start; j <= end; ++j) {
            windowData.push_back(input[j]);
        }

        // 排序窗口数据并取中值
        std::sort(windowData.begin(), windowData.end());
        int median = windowData[windowData.size() / 2];

        filteredData.push_back(median);
    }

    return filteredData;
}
#include <iostream>
#include <vector>

// 滑动窗口平滑函数
std::vector<int> slidingWindowSmoothing(std::vector<int>& input, int windowSize,std::vector<int>& observe) {
    std::vector<int> smoothedData;
//    smoothedData = observe;
    int pre;
    smoothedData.push_back(input[0]);
    for (int i = 1; i < input.size(); ++i) {
        pre = input[i-1];
        int count_0 = 0,count_1 = 0;
        int pre_count_0 = 0,pre_count_1 = 0;
        int flag=0,pre_flag=0;
            for (int j = i+1; j <input.size() &&j<i+windowSize ; ++j) {
                if(input[j]==1)
                    count_1++;
                if (input[j]==0)
                    count_0++;
            }
            for (int j = i-1; j >=0 &&j>i-windowSize/2 ; --j) {
                if(input[j]==1)
                    pre_count_1++;
                if (input[j]==0)
                    pre_count_0++;
            }
            if(count_1>count_0)
                flag = 1;
            else
                flag = 0;
            if (pre_count_1>=pre_count_0)
                pre_flag = 1;
            else
                pre_flag = 0;

           smoothedData.push_back(flag);
           input[i] = flag;






//        observe.push_back(input[i]);
    }

    return smoothedData;
}


// 指数加权移动平均函数
std::vector<int> exponentialMovingAverage(const std::vector<int>& input, double smoothingFactor) {
    std::vector<int> smoothedData;
    double prevSmoothed = input[0];

    for (int i = 0; i < input.size(); ++i) {
        double smoothed = smoothingFactor * input[i] + (1 - smoothingFactor) * prevSmoothed;
        smoothedData.push_back(static_cast<int>(smoothed + 0.5)); // 四舍五入
        prevSmoothed = smoothed;
    }

    return smoothedData;
}
// 移动平均函数
std::vector<int> movingAverage(const std::vector<int>& input, int windowSize) {
    std::vector<int> smoothedData;

    for (int i = 0; i < input.size(); ++i) {
        double sum = 0.0;
        int count = 0;

        // 计算窗口内数据的平均值
        for (int j = std::max(0, i - windowSize + 1); j <= i; ++j) {
            sum += input[j];
            count++;
        }

        smoothedData.push_back(sum / count);
    }

    return smoothedData;
}