#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>
#include <iostream>

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/RayShape.hh>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"  // 导入 Pose 消息类型
#include "std_msgs/msg/string.hpp"  // 导入 String 消息类型

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->world = _parent->GetWorld();

      // 获取模型名称
      this->modelName = this->model->GetName();

      ignition::math::Pose3d initialPose = this->model->WorldPose();
      this->startPoint = ignition::math::Vector3d(-14.96, -9.59, 0.5);
      this->endPoint = ignition::math::Vector3d(-6.65, -7.20, 0.55);
      this->speed = 0.05;
      this->direction = 1;  // 1表示朝着结束点运动，-1表示朝着起点运动
      this->time = 0.0;

      // 初始化 ROS 2 节点
      this->rosNode = rclcpp::Node::make_shared("model_push_node");
      
      // 根据模型名称创建独特的 ROS 话题
      this->posePublisher = this->rosNode->create_publisher<geometry_msgs::msg::Pose>(
          "model_pose/" + this->modelName, 10);

      std::cout << "ModelPush plugin loaded for model: " << this->modelName << std::endl;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    void OnUpdate()
    {
      double dt = 0.0001; //动态障碍物位置更新周期
      this->time += dt;

      // 计算当前位置的插值
      double t = std::fmod(this->time, 2.0);  // 2秒周期来回一次
      double factor = t < 1.0 ? t : 2.0 - t;  // 用于插值的因子
      factor = factor * factor * (3.0 - 2.0 * factor);  // 平滑插值（平滑的立方缓动）
      
      ignition::math::Vector3d currentPos = this->startPoint + (this->endPoint - this->startPoint) * factor;
      double terrainHeight = 0.5;  // 保持固定高度
      ignition::math::Pose3d newPose(currentPos.X(), currentPos.Y(), terrainHeight, 0.0, 0.0, 0.0);

      // 使用 SetLinearVel 替代 SetWorldPose 避免模型漂移
      this->model->SetLinearVel(ignition::math::Vector3d((endPoint.X() - startPoint.X()) * speed, 
                                                          (endPoint.Y() - startPoint.Y()) * speed, 0));

      // 发布位姿
      geometry_msgs::msg::Pose poseMsg;
      
      poseMsg.position.x = currentPos.X();
      poseMsg.position.y = currentPos.Y();
      poseMsg.position.z = terrainHeight;
      poseMsg.orientation.x = 0.0;
      poseMsg.orientation.y = 0.0;
      poseMsg.orientation.z = 0.0;
      poseMsg.orientation.w = 1.0;
      this->posePublisher->publish(poseMsg);

      // 更新模型的位姿
      this->model->SetWorldPose(newPose);
    }

  private: 
    double GetTerrainHeight(double x, double y, const physics::WorldPtr& world)
    {
        return 0.5;  // 假设固定高度
    }

    rclcpp::Node::SharedPtr rosNode;  // ROS 2 节点
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr posePublisher;  // 位姿发布者

    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    std::string modelName;  // 存储模型名称
    ignition::math::Vector3d startPoint;  // 运动起始点
    ignition::math::Vector3d endPoint;    // 运动结束点
    double speed;
    double time;
    int direction;  // 控制运动方向
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
