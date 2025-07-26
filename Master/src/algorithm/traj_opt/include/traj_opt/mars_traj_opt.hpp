#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "gcopter/lbfgs.hpp"

namespace uneven_fast_planner {

    class MarsTrajOpt {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保Eigen的内存对齐

        public:
        rclcpp::Node::SharedPtr node_;
        MarsTrajOpt(rclcpp::Node::SharedPtr node);


        private:
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_corridor;
        void rcvCorridorCallBack(visualization_msgs::msg::MarkerArray::SharedPtr msg);
        int Optimize(visualization_msgs::msg::MarkerArray::SharedPtr msg);

    };




}