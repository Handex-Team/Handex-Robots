#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 用于四元数转换
#include <nlopt.hpp>
#include <vector>
#include "visualization_msgs/msg/marker.hpp"
// #include "eigen3/Eigen/Core"
using namespace std::chrono_literals;
double dt = 0.1; // 控制周期
int N_P = 10,N_C=1; // 预测时域和控制时域
int point_ind=5;//当前目标点索引
// 机器人状态：x, y, theta
struct State
{
    double x;
    double y;
    double theta;
};

// 控制输入：线速度 v 和角速度 omega
struct Control
{
    double v;
    double omega;
};
class MPCController;
struct optData{
nav_msgs::msg::Path path_data;
MPCController* mpc_controller;
};

State pre_state(const State& current_state,const std::vector<double>&x){
    State pre_state;
    pre_state.x = current_state.x+x[0]*cos(current_state.theta)*dt;
    pre_state.y = current_state.y+x[0]*sin(current_state.theta)*dt;
    pre_state.theta = current_state.theta+x[1]*dt;
    return pre_state;
};
double cul_cost(const std::vector<double>&x,optData* optdata);



class MPCController : public rclcpp::Node
{
public:
    MPCController()
        : Node("mpc_controller")
    {
        // 订阅路径话题
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&MPCController::path_callback, this, std::placeholders::_1));

        // 订阅机器人当前状态
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_sim_ground_truth_pose", 10,
            std::bind(&MPCController::odom_callback, this, std::placeholders::_1));
        

        // 发布控制命令给差速小车
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
        control_timer_ = this->create_wall_timer(0.1s, std::bind(&MPCController::control_loop, this));
        
        // 初始化路径
        // nav_msgs::msg::Path default_path;
        // default_path.header.frame_id = "odom";  // 确保与其他数据一致
        // default_path.header.stamp = this->get_clock()->now();

        // for (int i = 0; i < 2*N_P; ++i) {
        //     geometry_msgs::msg::PoseStamped pose;
        //     pose.pose.position.x = 0.0;
        //     pose.pose.position.y = 0.0;
        //     pose.pose.orientation.w = 1.0;  // 默认方向无旋转
        //     default_path.poses.push_back(pose);
        // }
        // path_ = default_path;  // 将默认路径赋值给成员变量
        // 初始化控制参数
        v_opt_ = 0.0;
        omega_opt_ = 0.0;
    }

    State get_current_state()
    {
        // 返回更新后的当前状态
        return current_state_;
    }
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty path!");
            return;
        }

        // 如果路径不为空，则正常处理
        path_ = *msg;
    }




    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 更新当前状态
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;

        // 使用 tf2 库转换四元数为 yaw 角度
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);  // 将四元数转换为 tf2::Quaternion
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // 获取 roll, pitch, yaw
        current_state_.theta = yaw;  // 只关心 yaw 角度
        
    }
    
    static double cost_function(const std::vector<double>& x, std::vector<double>& grad, void* data)
    {
        
        auto optdata = static_cast<optData*>(data);
        // 计算当前状态
        auto path_data = optdata->path_data;

        auto goal=path_data.poses[point_ind].pose.position;
        optdata->mpc_controller->publish_marker(goal);
        double cost=cul_cost(x,optdata);
        // 数值差分计算梯度
        double epsilon = 1e-4;  // 小的扰动量
        std::vector<double> x_plus_epsilon = x;
        std::vector<double> x_minus_epsilon = x;
        
        std::fill(grad.begin(), grad.end(), 0.0);
        
        // 对 v (x) 求导
        for (int i = 0; i < 2 * N_P; i++){
            x_plus_epsilon[i] += epsilon;
            x_minus_epsilon[i] -= epsilon;
            

            double cost_plus_epsilon =cul_cost(x_plus_epsilon,optdata);
            double cost_minus_epsilon =cul_cost(x_minus_epsilon,optdata);
            
            grad[i] = (cost_plus_epsilon - cost_minus_epsilon) / (2 * epsilon);
            x_plus_epsilon=x;
            x_minus_epsilon=x;
        }
        return cost;
    }


    void solve_mpc(State current_state)
    {
        auto start_time = std::chrono::steady_clock::now();
        // 使用 nlopt 求解优化问题
        nlopt::opt opt(nlopt::LD_MMA, 2* N_P);  // 使用 MMA 算法，2 表示优化变量 v 和 omega
        optData optdata;
        optdata.path_data = path_;
        if(path_.poses.size()<point_ind+1){
            RCLCPP_ERROR(this->get_logger(), "没有生成路径");
            v_opt_ = 0.0;
            omega_opt_ = 0.0;
            return;
        }
        optdata.mpc_controller=this;
        // 设置成本函数
        opt.set_min_objective(cost_function, &optdata);
        opt.set_xtol_rel(1e-4);  // 设置收敛容差

        std::vector<double> x(2 * N_P, 0.0); // 初始化优化变量，所有元素都赋值为 0.0


        double minf; // 最优值
        nlopt::result result = opt.optimize(x, minf);  // 进行优化

        // 获取优化结果：v 和 omega
        v_opt_ = x[0];
        omega_opt_ = x[N_P];
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
        if (result < nlopt::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Optimization failed with result code %d", result);
        }

        RCLCPP_INFO(this->get_logger(), "MPC optimization time: %f seconds", elapsed_seconds);
        RCLCPP_INFO(this->get_logger(), "Optimized v: %f, omega: %f", v_opt_, omega_opt_);
    }

    void apply_control(double v, double omega)
    {
        // 通过 ROS2 发布速度命令给小车的驱动系统
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = v;
        twist_msg.angular.z = omega;

        twist_pub_->publish(twist_msg);  // 发布到指定的控制话题
        RCLCPP_INFO(this->get_logger(), "Published v: %f, omega: %f", v, omega);
    }


    void control_loop()
    {
        // 获取当前状态
        State current_state = get_current_state();

        // 求解 MPC
        solve_mpc(current_state);

        // 应用控制
        apply_control(v_opt_, omega_opt_);
    }

    void publish_marker(const geometry_msgs::msg::Point goal)
    {
        // 创建一个 Marker 消息
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";  // 坐标系选择（例如：map，odom）
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "goal_marker";  // 命名空间
        marker.id = 0;  // 唯一标识符

        marker.type = visualization_msgs::msg::Marker::SPHERE;  // 使用球体来表示目标
        marker.action = visualization_msgs::msg::Marker::ADD;  // 添加标记

        marker.pose.position = goal;  // 设置目标坐标
        marker.pose.orientation.w = 1.0;  // 没有旋转

        marker.scale.x = 0.2;  // 球体的半径
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0;  // 透明度
        marker.color.r = 1.0;  // 颜色（红色）
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // 发布 Marker
        goal_pub_->publish(marker);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path path_; // 存储路径数据
    State current_state_; // 当前机器人状态

    double v_opt_=0.0;   // 最优线速度
    double omega_opt_=0.0; // 最优角速度

    
};

double cul_cost(const std::vector<double>&x,optData* optdata)
{
    State current_state = optdata->mpc_controller->get_current_state();
    auto path_data = optdata->path_data;
    double weight_d = 0.8,weight_d_sum=0.2;  // 最终路径距离权重
    
    // 多步预测
    State Pre_state[N_P];
    std::vector<std::vector<double>> error(N_P,std::vector<double>(3));

    double cost1=0.0,cost2=0.0,cost=0.0;// 累计误差成本,终点误差成本
    for(int i=0;i<N_P;i++){
        std::vector<double>input={x[i],x[i+N_P]};
        if(i==0){
            Pre_state[i] = pre_state(current_state,input);
        }else{
            Pre_state[i] = pre_state(Pre_state[i-1],input);
        }
        error[i][0] = path_data.poses[point_ind].pose.position.x - Pre_state[i].x;
        error[i][1] = path_data.poses[point_ind].pose.position.y - Pre_state[i].y;
        cost1+=weight_d_sum*std::hypot(error[i][0],error[i][1])*dt;
    }
    
    cost2=weight_d*std::hypot(path_data.poses[point_ind].pose.position.x - Pre_state[N_P-1].x,path_data.poses[point_ind].pose.position.y - Pre_state[N_P-1].y);
    cost = cost1+cost2;
    return cost;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCController>());
    rclcpp::shutdown();
    return 0;
}
