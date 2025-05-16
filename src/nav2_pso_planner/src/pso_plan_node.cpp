#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <iostream>
#include "nav2_pso_planner/pso_planner.hpp"

using std::placeholders::_1;

namespace nav2_pso_planner
{

/**
 * @brief This node wraps the PSO planner (globalMotionPlannerWithCostmap) and subscribes to a "goal" topic.
 *        When a goal is received, it retrieves the robot pose, computes a plan, and publishes the plan.
 */
class PSOPlanNode : public rclcpp::Node
{
public:
  PSOPlanNode()
  : Node("pso_planner")
  {
    // Tạo costmap node; đối với ROS2 Nav2, costmap thường được tạo ra và quản lý bởi costmap2d_ros node
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap", tf_buffer_);
    // Khởi tạo Planner plugin (PSO Planner)
    // Giả sử bạn đã đăng ký plugin trong tệp XML, ta tạo đối tượng planner thông qua API của pluginlib
    // Tuy nhiên, ở ví dụ đơn giản này, ta tạo trực tiếp đối tượng của PSOPlanner.
    // Lưu ý: Trong thực tế, việc khởi tạo plugin sẽ thông qua pluginlib.
    pso_planner_ = std::make_shared<PSOPlanner>();

    // Giả sử chúng ta cấu hình planner với các tham số từ Node (trong thực tế, bạn sẽ dùng YAML)
    pso_planner_->configure(shared_from_this(), "pso_planner", tf_buffer_, costmap_ros_);
    
    // Đăng ký subscriber nhận goal (ROS2 sử dụng lambda callback)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 10,
      std::bind(&PSOPlanNode::goalCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PSO Plan Node has been started.");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    // Lấy vị trí hiện tại của robot thông qua costmap_ros_
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
      RCLCPP_WARN(this->get_logger(), "Cannot get robot pose");
      return;
    }
    
    // Kiểm tra nếu điểm goal có nằm trong costmap không bằng cách chuyển đổi
    unsigned int mx = 0, my = 0;
    if (!costmap_ros_->getCostmap()->worldToMap(goal_msg->pose.position.x, goal_msg->pose.position.y, mx, my))
    {
      RCLCPP_WARN(this->get_logger(), "worldToMap error for goal point");
      return;
    }
    
    // Kiểm tra nếu ô goal không phải vật cản
    if (costmap_ros_->getCostmap()->getCost(mx, my) != costmap_2d::FREE_SPACE)
    {
      RCLCPP_WARN(this->get_logger(), "The target point is unreachable.");
      return;
    }
    
    // Tạo vector plan để lưu path
    std::vector<geometry_msgs::msg::PoseStamped> plan;
    bool success = pso_planner_->makePlan(robot_pose, *goal_msg, plan);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan computed successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute plan.");
    }
  }

  // Các thành phần thành viên:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  tf2_ros::Buffer tf_buffer_{this->get_clock()};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{
    std::make_shared<tf2_ros::TransformListener>(tf_buffer_)};
  std::shared_ptr<PSOPlanner> pso_planner_;
};

} // namespace nav2_pso_planner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Sử dụng make_shared để tạo node, và rclcpp::spin() sẽ giữ node chạy
  auto node = std::make_shared<nav2_pso_planner::PSOPlanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
