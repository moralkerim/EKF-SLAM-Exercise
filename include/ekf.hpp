/*
 * EKF Implementation
 * Author: Kerim Moral
 * E-mail: moralkerim@gmail.com
 * License: MIT License
 *
 * Copyright (c) 2025 Kerim Moral
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include<iostream>
#include<memory>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <vector>
#include "robot.hpp"



class EKF {
    public:
        EKF(Eigen::Matrix2f& R_,
         Eigen::Matrix3f& Q_,
         Eigen::Matrix3f& P0_,
         Eigen::Vector3f& X0_,
         std::shared_ptr<Logger>& logger_);

        void initLandmark(Measurement& Z, int& id);

        void predict(Eigen::Vector2f& U);

        void update(const Measurement& Z, const int& id);

    private:
        Eigen::Matrix3f F;
        Eigen::MatrixXf H;
        Eigen::Matrix2f R;
        Eigen::Matrix3f Q;
        Eigen::MatrixXf Kt;
        Eigen::MatrixXf S_in;

        const float RANGE_LIMIT = 10.0;
        const float BEAR_LIM_DEG = 90.0;
        const float BEAR_LIM_RAD = BEAR_LIM_DEG * M_PI / 180.0;
        std::shared_ptr<Logger> logger;

    public:
        Eigen::MatrixXf P, P_hat;
        Eigen::VectorXf X, X_hat;
        Eigen::Vector2f Z_hat;
        std::vector<int> landmark_list;


};