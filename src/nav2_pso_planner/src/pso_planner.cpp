#include <iostream>
#include <cmath>
#include <chrono>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_pso_planner/pso_planner.hpp"

namespace nav2_pso_planner
{

PSOPlanner::PSOPlanner()
  : costmap_(nullptr),
    costmap_ros_(nullptr),
    g_planner_(nullptr),
    initialized_(false)
{
}

PSOPlanner::~PSOPlanner()
{
    if (g_planner_) {
        delete g_planner_;
        g_planner_ = nullptr;
    }
}

void PSOPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    if (!initialized_) {
      auto node = parent.lock();
      if (!node) {
        throw std::runtime_error{"Failed to lock node"};
      }
      node_ = node;
      name_ = name;
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      frame_id_ = costmap_ros_->getGlobalFrameID();
      nx_ = costmap_->getSizeInCellsX();
      ny_ = costmap_->getSizeInCellsY();
      origin_x_ = costmap_->getOriginX();
      origin_y_ = costmap_->getOriginY();
      resolution_ = costmap_->getResolution();
  
      // Khởi tạo lifecycle publisher
      plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "plan", 
        rclcpp::SystemDefaultsQoS().reliable().transient_local());
  
      // Khai báo và lấy tham số sử dụng nav2_util
      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".n_particles", rclcpp::ParameterValue(50));
      node_->get_parameter(name + ".n_particles", n_particles_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".n_inherited", rclcpp::ParameterValue(20));
      node_->get_parameter(name + ".n_inherited", n_inherited_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".pointNum", rclcpp::ParameterValue(5));
      node_->get_parameter(name + ".pointNum", pointNum_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".obs_factor", rclcpp::ParameterValue(0.39));
      node_->get_parameter(name + ".obs_factor", obs_factor_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".max_speed", rclcpp::ParameterValue(40.0));
      node_->get_parameter(name + ".max_speed", max_speed_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".w_inertial", rclcpp::ParameterValue(1.0));
      node_->get_parameter(name + ".w_inertial", w_inertial_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".w_social", rclcpp::ParameterValue(2.0));
      node_->get_parameter(name + ".w_social", w_social_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".w_cognitive", rclcpp::ParameterValue(1.2));
      node_->get_parameter(name + ".w_cognitive", w_cognitive_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".initposmode", rclcpp::ParameterValue(2));
      node_->get_parameter(name + ".initposmode", initposmode_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".pub_particles", rclcpp::ParameterValue(false));
      node_->get_parameter(name + ".pub_particles", pub_particles_);

      nav2_util::declare_parameter_if_not_declared(
        node_, name + ".pso_max_iter", rclcpp::ParameterValue(5));
      node_->get_parameter(name + ".pso_max_iter", pso_max_iter_);
  
      // Tạo đối tượng PSO (file thuật toán PSO - logic được giữ nguyên)
      g_planner_ = new PSO(
                      nx_, ny_, resolution_, origin_x_, origin_y_,
                      n_particles_, n_inherited_, pointNum_,
                      w_inertial_, w_social_, w_cognitive_,
                      obs_factor_, max_speed_, initposmode_,
                      pub_particles_, pso_max_iter_, node_);
  
      RCLCPP_INFO(node_->get_logger(), "PSO planner initialized successfully");
      initialized_ = true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Planner already initialized");
    }
  }
                
  
  void PSOPlanner::cleanup()
  {
    // Neu can giải phóng tài nguyên thì làm ở đây
  }
  
  void PSOPlanner::activate()
  {
    plan_pub_->on_activate();
  }
  
  void PSOPlanner::deactivate()
  {
    plan_pub_->on_deactivate();
  }
  
  nav_msgs::msg::Path PSOPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
  {
    nav_msgs::msg::Path plan_msg;
    plan_msg.header.frame_id = frame_id_;
    plan_msg.header.stamp = node_->now();
  
    if (!initialized_) {
      RCLCPP_ERROR(node_->get_logger(), "Planner has not been initialized yet");
      return plan_msg;
    }
  
    std::pair<int, int> start_node, goal_node;
    unsigned int mx = 0, my = 0;
  
    // Kiểm tra điểm bắt đầu không là vật cản
    if (collision(start.pose.position.x, start.pose.position.y)) {
      RCLCPP_WARN(node_->get_logger(), "Failed to get path: start point is an obstacle");
      return plan_msg;
    } else {
      costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
      start_node.first = static_cast<int>(mx);
      start_node.second = static_cast<int>(my);
    }
  
    // Kiểm tra điểm đích không là vật cản
    if (collision(goal.pose.position.x, goal.pose.position.y)) {
      RCLCPP_WARN(node_->get_logger(), "Failed to get path: goal point is an obstacle");
      return plan_msg;
    } else {
      costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
      goal_node.first = static_cast<int>(mx);
      goal_node.second = static_cast<int>(my);
    }
  
    std::vector<std::pair<int, int>> path;
    plan_msg.poses.clear();
    
    //double start_time = node_->now().seconds();
  
    bool path_found = g_planner_->plan(costmap_->getCharMap(), start_node, goal_node, path);
  
    if (path_found) {
      if (getPlanFromPath(path, plan_msg.poses)) {
        geometry_msgs::msg::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = node_->now();
        plan_msg.poses.push_back(goal_copy);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get a plan from path even though path found.");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute a valid path.");
    }
  
    // Sửa lại hướng của các điểm (nếu cần)
    optimizationOrientation(plan_msg.poses);
  
    // Xuất plan cho visualization
    nav_msgs::msg::Path vis_plan;
    vis_plan.header.frame_id = frame_id_;
    vis_plan.header.stamp = node_->now();
    vis_plan.poses = plan_msg.poses;
    plan_pub_->publish(vis_plan);
  
    return plan_msg;
  }
  
  bool PSOPlanner::getPlanFromPath(std::vector< std::pair<int, int> >& path, std::vector<geometry_msgs::msg::PoseStamped>& plan)
  {
    plan.clear();
    unsigned int mx = 0, my = 0;
    double wx, wy;
    for (size_t i = 0; i < path.size(); i++) {
      mx = static_cast<unsigned int>(path[i].first);
      my = static_cast<unsigned int>(path[i].second);
      costmap_->mapToWorld(mx, my, wx, wy);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      plan.push_back(pose);
    }
    return !plan.empty();
  }
  
  void PSOPlanner::optimizationOrientation(std::vector<geometry_msgs::msg::PoseStamped>& plan)
  {
    if (plan.size() < 2)
      return;
    
    for (size_t i = 0; i < plan.size() - 1; i++) {
      double yaw = atan2(plan[i+1].pose.position.y - plan[i].pose.position.y,
                         plan[i+1].pose.position.x - plan[i].pose.position.x);
      // Sử dụng tf2 để tạo quaternion từ yaw
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      plan[i].pose.orientation = tf2::toMsg(q);
    }
  }
  
  bool PSOPlanner::collision(double x, double y)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my))
      return true;
    if ((mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
      return true;
    if (costmap_->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return true;
    return false;
  }
  
  double PSOPlanner::distance(double px1, double py1, double px2, double py2)
  {
    return std::hypot(px1 - px2, py1 - py2);
  }
  
  bool PSOPlanner::pointCircleCollision(double x1, double y1, double x2, double y2, double goal_radius)
  {
    double dist = distance(x1, y1, x2, y2);
    return (dist < goal_radius);
  }
  
  bool PSOPlanner::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
  {
    std::pair<double, double> ptmp{0.0, 0.0};
    double dist = std::hypot(p2.second - p1.second, p2.first - p1.first);
    if (dist < resolution_)
      return true;
    else {
      int value = static_cast<int>(std::floor(dist / resolution_));
      double theta = atan2(p2.second - p1.second, p2.first - p1.first);
      for (int i = 1; i <= value; i++) {
        ptmp.first = p1.first + resolution_ * cos(theta) * i;
        ptmp.second = p1.second + resolution_ * sin(theta) * i;
        if (collision(ptmp.first, ptmp.second))
          return false;
      }
      return true;
    }
  }
  
  bool PSOPlanner::isAroundFree(double wx, double wy)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my))
      return false;
    if (mx <= 1 || my <= 1 || mx >= (costmap_->getSizeInCellsX() - 1) || my >= (costmap_->getSizeInCellsY() - 1))
      return false;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        unsigned int x = mx + i, y = my + j;
        if (costmap_->getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return false;
      }
    }
    return true;
  }
  
  } // namespace nav2_pso_planner
  
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_pso_planner::PSOPlanner, nav2_core::GlobalPlanner)
