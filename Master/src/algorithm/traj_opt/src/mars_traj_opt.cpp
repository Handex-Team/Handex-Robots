#include "traj_opt/mars_traj_opt.hpp"

namespace uneven_fast_planner {
    MarsTrajOpt::MarsTrajOpt(rclcpp::Node::SharedPtr node): node_(node){
        std::cout << "MarsTrajOpt constructor called" << std::endl;
        sub_corridor = node_->create_subscription<visualization_msgs::msg::MarkerArray>
                        ("corridor", 10, std::bind(&MarsTrajOpt::rcvCorridorCallBack, this, std::placeholders::_1));
        
    }
    void MarsTrajOpt::rcvCorridorCallBack(visualization_msgs::msg::MarkerArray::SharedPtr msg){

        Optimize(msg);
        

        
    }


    int MarsTrajOpt::Optimize(visualization_msgs::msg::MarkerArray::SharedPtr msg){
        int ret_code = 0;


        // 开始求解
        lbfgs::lbfgs_parameter_t param;
        param.g_epsilon = 1e-5;
        param.delta = 1e-6;
        param.max_iterations = 1000;
        param.mem_size = 20;

        return ret_code;
    }

}