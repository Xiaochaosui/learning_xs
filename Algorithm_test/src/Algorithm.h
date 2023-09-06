//
// Created by xcsy on 23-8-21.
//

#ifndef LEARNING_CPP_ALGORITHM_H
#define LEARNING_CPP_ALGORITHM_H
#include "cyber/CyberMsg.h"
#endif //LEARNING_CPP_ALGORITHM_H
std::vector<int> medianFilter(const std::vector<int>& input, int windowSize);
std::vector<int> movingAverage(const std::vector<int>& input, int windowSize);
std::vector<int> exponentialMovingAverage(const std::vector<int>& input, double smoothingFactor) ;

std::vector<int> slidingWindowSmoothing(std::vector<int>& input, int windowSize,std::vector<int>& observe);