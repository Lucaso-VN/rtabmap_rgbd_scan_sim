#ifndef NAV2_IPSO_PLANNER__IPSO_PLANNER_HPP_
#define NAV2_IPSO_PLANNER__IPSO_PLANNER_HPP_

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_ipso_planner/ipso.hpp"

namespace nav2_ipso_planner
{

class IPSOPlanner : public nav2_core::GlobalPlanner
{
public:
 /**
  * @brief constructor
  */
 IPSOPlanner();
 /**
  * @brief destructor
  */
 ~IPSOPlanner();
 
 /**
  * @brief Configuring plugin
  * @param parent Lifecycle node pointer
  * @param name Name of plugin map
  * @param tf Shared ptr of TF2 buffer
  * @param costmap_ros Costmap2DROS object
  */
 void configure(
   const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
   std::string name, 
   std::shared_ptr<tf2_ros::Buffer> tf,
   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

 /**
  * @brief Cleanup lifecycle node
  */
 void cleanup() override;
 
 /**
  * @brief Activate lifecycle node
  */
 void activate() override;
 
 /**
  * @brief Deactivate lifecycle node
  */
 void deactivate() override;

 /**
  * @brief Creating a plan from start and goal poses
  * @param start Start pose
  * @param goal Goal pose
  * @return nav_msgs::Path of the generated path
  */
 nav_msgs::msg::Path createPlan(
   const geometry_msgs::msg::PoseStamped & start,
   const geometry_msgs::msg::PoseStamped & goal) override;

 bool getPlanFromPath(std::vector<std::pair<int, int>>& path, std::vector<geometry_msgs::msg::PoseStamped>& plan);
 double distance(double px1, double py1, double px2, double py2);
 bool collision(double x, double y);
 bool isAroundFree(double wx, double wy);
 bool pointCircleCollision(double x1, double y1, double x2, double y2, double radius);
 void optimizationOrientation(std::vector<geometry_msgs::msg::PoseStamped> & plan);
 bool isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2);

protected:
 nav2_costmap_2d::Costmap2D* costmap_;
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
 rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

 std::string name_;
 std::shared_ptr<tf2_ros::Buffer> tf_;

 unsigned int nx_, ny_;
 double origin_x_, origin_y_;
 double resolution_;
 std::string frame_id_;
 nav2_ipso_planner::IPSO* g_planner_;
 rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

 bool initialized_;

 // IPSO parameters
 int n_particles_;
 int n_inherited_;
 int pointNum_;
 double obs_factor_;
 double max_speed_;
 double w_inertial_min_;
 double w_inertial_max_;
 double w_social_lower_;
 double w_social_upper_;
 double w_cognitive_lower_;
 double w_cognitive_upper_;
 int initposmode_;
 bool pub_particles_;
 int ipso_max_iter_;
};

}  // namespace nav2_ipso_planner

#endif  // NAV2_IPSO_PLANNER__IPSO_PLANNER_HPP_
