#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <thread>
#include <random>
#include "nav2_pso_planner/pso.hpp"

// ROS2 includes for time and marker messages
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace nav2_pso_planner
{
  /**
   * @brief Construct a new PSO object
   *
   * @param nx              Number of pixels in costmap x direction
   * @param ny              Number of pixels in costmap y direction
   * @param resolution      Costmap resolution
   * @param origin_x        Origin coordinate in the x direction
   * @param origin_y        Origin coordinate in the y direction
   * @param n_particles     Number of particles
   * @param n_inherited     Number of inherited particles
   * @param pointNum        Number of position points contained in each particle
   * @param w_inertial      Inertia weight
   * @param w_social        Social weight
   * @param w_cognitive     Cognitive weight
   * @param obs_factor      Obstacle factor (greater means obstacles are more penalizing)
   * @param max_speed       The maximum movement speed of particles (as double)
   * @param initposmode     Mode for generating initial particle positions
   * @param pub_particles   Flag to publish particles
   * @param pso_max_iter        Maximum iterations
   * @param node            Shared pointer to an rclcpp::Node (ROS2)
   */
  PSO::PSO(
    int nx, int ny, double resolution, double origin_x, double origin_y,
    int n_particles, int n_inherited, int pointNum, double w_inertial,
    double w_social, double w_cognitive, double obs_factor, int max_speed, // Thiếu max_speed trong implementation
    int initposmode, bool pub_particles, int pso_max_iter,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : nx_(nx),
      ny_(ny),
      ns_(nx * ny),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      n_particles_(n_particles),
      n_inherited_(n_inherited),
      pointNum_(pointNum),
      w_inertial_(w_inertial),
      w_social_(w_social),
      w_cognitive_(w_cognitive),
      obs_factor_(obs_factor),
      max_speed_(max_speed),
      initposmode_(initposmode),
      pub_particles_(pub_particles),
      pso_max_iter_(pso_max_iter),
      node_(node)  // Store the ROS2 node pointer for later use
  {
    // Initialize inherited_particles_ as before; ensure the container is defined in your class.
    inherited_particles_.emplace_back(
      std::vector<std::pair<int, int>>(pointNum, std::make_pair(1, 1)),
      std::vector<std::pair<int, int>>(pointNum, std::make_pair(0, 0)),
      0.0);

    // Create ROS2 publisher for visualization_msgs::msg::Marker using the passed node pointer
    particle_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("particle_swarm_markers", 10);

    RCLCPP_INFO(node_->get_logger(), "PSO constructor called, publisher initialized.");
  }

  PSO::~PSO()
  {
  }

  /**
   * @brief PSO implementation.
   * @param global_costmap Global costmap data (as a pointer to unsigned char)
   * @param start         Start node (grid coordinates)
   * @param goal          Goal node (grid coordinates)
   * @param path          Output optimal path (vector of grid indices)
   * @return true if path found, else false
   */
  bool PSO::plan(const unsigned char* global_costmap, const std::pair<int, int>& start, const std::pair<int, int>& goal, std::vector<std::pair<int, int>>& path)
  {
    std::cout << "PSO planning started..." << std::endl;

    // Variable initialization
    double pathLength;
    double initial_fitness;
    double obstacle_cost;
    Particle Best_particle;
    PositionSequence initialPositions;
    std::vector<Particle> particles;
    std::vector<std::pair<double, double>> initial_point;
    std::pair<double, double> start_d(static_cast<double>(start.first), static_cast<double>(start.second));
    std::pair<double, double> goal_d(static_cast<double>(goal.first), static_cast<double>(goal.second));

    // Generate initial positions of the particle swarm
    if (initposmode_ == 1)
    {
      generateRandomInitialPositions(initialPositions, start_d, goal_d);
    }
    else
    {
      generateCircularInitialPositions(initialPositions, start_d, goal_d);
    }
    
    std::cout << "PSO: Successfully generated initial particle swarm positions" << std::endl;

    // Particle initialization
    for (int i = 0; i < n_particles_; ++i)
    {
      std::vector<std::pair<int, int>> initial_position;
      if ((i < n_inherited_) && (inherited_particles_.size() == static_cast<size_t>(n_inherited_)))
      {
        initial_position = inherited_particles_[i].personal_best_pos;
      }
      else
      {
        initial_position = initialPositions[i];
      }
      
      std::vector<std::pair<int, int>> initial_velocity(pointNum_, std::make_pair(0, 0));
      // Generate B-spline curve control points
      path_generation.GenerateControlPoints(start_d, goal_d, initial_position, initial_point);
      // Generate B-spline curves
      path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
      // Calculate path length
      pathLength = path_generation.calculatePathLength(initial_point);
      // Collision detection
      obstacle_cost = ObstacleCost(global_costmap, initial_point);
      // Calculate fitness for this particle
      initial_fitness = 100000.0 / (pathLength + 1000 * obstacle_cost);

      if ((i == 0) || (initial_fitness > Best_particle.fitness))
      {
        GlobalBest_particle_ = i;
        Best_particle.fitness = initial_fitness;
      }
      // Create and add particle
      particles.emplace_back(initial_position, initial_velocity, initial_fitness);
    }

    Best_particle.position = particles[GlobalBest_particle_].position;
    std::cout << "PSO: Successfully initialized particles" << std::endl;

    // Create random generator for particle updates
    std::random_device rd;
    std::mt19937 gen(rd());

    std::cout << "PSO: Particle swarm optimization iteration progress:" << std::endl;

    // Iterative optimization
    for (size_t iter = 0; iter < static_cast<size_t>(pso_max_iter_); iter++)
    {
      std::cout << "PSO iteration " << iter << " ";
      std::vector<std::thread> particle_list(n_particles_);
      for (size_t i = 0; i < static_cast<size_t>(n_particles_); ++i)
      {
        particle_list[i] = std::thread(&PSO::optimizeParticle, this, std::ref(particles[i]), std::ref(Best_particle),
                                       std::cref(global_costmap), std::cref(start_d), std::cref(goal_d), static_cast<int>(i),
                                       std::ref(gen));
      }
      for (size_t i = 0; i < static_cast<size_t>(n_particles_); ++i)
      {
        particle_list[i].join();
      }
      Best_particle.position = particles[GlobalBest_particle_].personal_best_pos;
    }

    // Generating Path from Optimal Particle
    path_generation.GenerateControlPoints(start_d, goal_d, Best_particle.position, initial_point);
    path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
   
    std::cout << "PSO: Iteration completed, optimal fitness is: " << Best_particle.fitness << std::endl;

    // Convert the generated path (vector of double-pairs) to integer grid indices
    path.clear();
    if (!initial_point.empty())
    {
      path.emplace_back(static_cast<int>(initial_point[0].first), static_cast<int>(initial_point[0].second));
      for (size_t p = 1; p < initial_point.size(); ++p)
      {
        int x = static_cast<int>(initial_point[p].first);
        int y = static_cast<int>(initial_point[p].second);
        if (x != path.back().first || y != path.back().second)
        {
          path.emplace_back(x, y);
        }
      }
    }

    // Update inherited particles based on fitness
    std::sort(particles.begin(), particles.end(),
              [](const Particle& a, const Particle& b) {
                return a.personal_best_fitness > b.personal_best_fitness;
              });
    inherited_particles_.clear();
    for (size_t inherit = 0; inherit < static_cast<size_t>(n_inherited_); ++inherit)
    {
      inherited_particles_.emplace_back(particles[inherit]);
    }

    if (!path.empty())
    {
      std::cout << "PSO: Planning Successful!" << std::endl;
    }

    return !path.empty();
  }

  // Transform from grid map coordinates (x, y) to grid index
  int PSO::grid2Index(int x, int y)
  {
    return x + nx_ * y;
  }

  // Generate random initial positions for particles
  void PSO::generateRandomInitialPositions(PositionSequence &initialPositions,
                                           const std::pair<double, double> start_d,
                                           const std::pair<double, double> goal_d)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> x(pointNum_);
    std::vector<int> y(pointNum_);
    int point_id;
    bool xorder = (goal_d.first > start_d.first);
    bool yorder = (goal_d.second > start_d.second);

    for (int i = 0; i < n_particles_; ++i)
    {
      std::unordered_set<int> visited;
      std::vector<std::pair<int, int>> particlePositions;
      point_id = 0;
      while (point_id < pointNum_)
      {
        x[point_id] = std::uniform_int_distribution<int>(0, nx_ - 1)(gen);
        y[point_id] = std::uniform_int_distribution<int>(0, ny_ - 1)(gen);
        int uniqueId = x[point_id] * (ny_ + 1) + y[point_id];
        if (visited.find(uniqueId) == visited.end())
        {
          point_id++;
          visited.insert(uniqueId);
        }
      }
      if (xorder)
	std::sort(x.begin(), x.end(), [](int a, int b) { return PSO::ascendingOrder(a, b); });
      else
	std::sort(x.begin(), x.end(), [](int a, int b) { return PSO::descendingOrder(a, b); });
      if (yorder)
	std::sort(y.begin(), y.end(), [](int a, int b) { return PSO::ascendingOrder(a, b); });
      else
	std::sort(y.begin(), y.end(), [](int a, int b) { return PSO::descendingOrder(a, b); });
      for (int ii = 0; ii < pointNum_; ++ii)
      {
        particlePositions.emplace_back(x[ii], y[ii]);
      }
      initialPositions.push_back(particlePositions);
    }
  }

  // Generate circular initial positions for particles
  void PSO::generateCircularInitialPositions(PositionSequence &initialPositions,
                                             const std::pair<double, double> start_d,
                                             const std::pair<double, double> goal_d)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> x(pointNum_);
    std::vector<int> y(pointNum_);
    int point_id;
    bool xorder = (goal_d.first > start_d.first);
    bool yorder = (goal_d.second > start_d.second);
    int centerX = static_cast<int>((start_d.first + goal_d.first) / 2);
    int centerY = static_cast<int>((start_d.second + goal_d.second) / 2);
    double radius = path_generation.calculateDistance(start_d, goal_d) / 2.0;
    if (radius < 5)
      radius = 5;

    for (int i = 0; i < n_particles_; ++i)
    {
      std::unordered_set<int> visited;
      std::vector<std::pair<int, int>> particlePositions;
      point_id = 0;
      while (point_id < pointNum_)
      {
        double angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(gen);
        double r = std::sqrt(std::uniform_real_distribution<double>(0, 1)(gen)) * radius;
        x[point_id] = static_cast<int>(std::round(centerX + r * std::cos(angle)));
        y[point_id] = static_cast<int>(std::round(centerY + r * std::sin(angle)));
        if (x[point_id] >= 0 && x[point_id] < nx_ && y[point_id] >= 0 && y[point_id] < ny_)
        {
          int uniqueId = x[point_id] * (ny_ + 1) + y[point_id];
          if (visited.find(uniqueId) == visited.end())
          {
            point_id++;
            visited.insert(uniqueId);
          }
        }
      }
      if (xorder)
	std::sort(x.begin(), x.end(), [](int a, int b) { return PSO::ascendingOrder(a, b); });
      else
	std::sort(x.begin(), x.end(), [](int a, int b) { return PSO::descendingOrder(a, b); });
      if (yorder)
	std::sort(y.begin(), y.end(), [](int a, int b) { return PSO::ascendingOrder(a, b); });
      else
	std::sort(y.begin(), y.end(), [](int a, int b) { return PSO::descendingOrder(a, b); });
      for (int ii = 0; ii < pointNum_; ++ii)
      {
        particlePositions.emplace_back(x[ii], y[ii]);
      }
      initialPositions.push_back(particlePositions);
    }
  }

  // Calculate obstacle avoidance cost
  double PSO::ObstacleCost(const unsigned char* global_costmap, const std::vector<std::pair<double, double>>& pso_path)
  {
    int point_index;
    double Obscost = 1.0;
    for (size_t i = 1; i < pso_path.size(); ++i)
    {
      point_index = grid2Index(static_cast<int>(pso_path[i].first), static_cast<int>(pso_path[i].second));
      if ((point_index < 0) || (point_index >= ns_) || (global_costmap[point_index] >= lethal_cost_ * obs_factor_))
      {
        Obscost += 1.0;
      }
    }
    return Obscost;
  }

  // Update particle velocity
  void PSO::updateParticleVelocity(Particle& particle, const Particle& global_best, std::mt19937& gen)
  {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (size_t i = 0; i < static_cast<size_t>(pointNum_); ++i)
    {
      double rand1 = dist(gen);
      double rand2 = dist(gen);
      particle.velocity[i].first = static_cast<int>(w_inertial_ * particle.velocity[i].first +
                                                     w_social_ * rand1 * (particle.personal_best_pos[i].first - particle.position[i].first) +
                                                     w_cognitive_ * rand2 * (global_best.position[i].first - particle.position[i].first));
      particle.velocity[i].second = static_cast<int>(w_inertial_ * particle.velocity[i].second +
                                                      w_social_ * rand1 * (particle.personal_best_pos[i].second - particle.position[i].second) +
                                                      w_cognitive_ * rand2 * (global_best.position[i].second - particle.position[i].second));
      // Limit velocity
      particle.velocity[i].first = clamp(particle.velocity[i].first, -max_speed_, max_speed_);
      particle.velocity[i].second = clamp(particle.velocity[i].second, -max_speed_, max_speed_);
    }
  }

  // Update particle position
  void PSO::updateParticlePosition(Particle& particle)
  {
    for (size_t i = 0; i < static_cast<size_t>(pointNum_); ++i)
    {
      particle.position[i].first += particle.velocity[i].first;
      particle.position[i].second += particle.velocity[i].second;
      // Clamp positions to the map boundaries
      particle.position[i].first = clamp(particle.position[i].first, 1, nx_ - 1);
      particle.position[i].second = clamp(particle.position[i].second, 1, ny_ - 1);
    }
  }

  // Particle update optimization iteration
  void PSO::optimizeParticle(Particle& particle, Particle& best_particle,
                             const unsigned char* global_costmap,
                             const std::pair<double, double>& start_d,
                             const std::pair<double, double>& goal_d,
                             const int& index_i, std::mt19937& gen)
  {
    std::vector<std::pair<double, double>> process_path;
    updateParticleVelocity(particle, best_particle, gen);
    updateParticlePosition(particle);
    
    // Generate B-spline curve control points and compute the path
    path_generation.GenerateControlPoints(start_d, goal_d, particle.position, process_path);
    path_generation.B_spline_curve(process_path, path_generation.splineOrder);
    double pathLength = path_generation.calculatePathLength(process_path);
    double obstacle_cost = ObstacleCost(global_costmap, process_path);
    particle.fitness = 100000.0 / (pathLength + 1000 * obstacle_cost);

    // Update individual optimum
    if (particle.fitness > particle.personal_best_fitness)
    {
      particle.personal_best_fitness = particle.fitness;
      particle.personal_best_pos = particle.position;
    }

    // Publish particle markers if enabled
    if (pub_particles_)
    {
      publishParticleMarkers(particle.position, index_i);
    }

    // Update global optimal particle (lock for thread safety)
    particles_lock_.lock();
    if (particle.personal_best_fitness > best_particle.fitness)
    {
      best_particle.fitness = particle.personal_best_fitness;
      GlobalBest_particle_ = index_i;
    }
    particles_lock_.unlock();
  }

  // Publish particle markers for visualization in RViz
  void PSO::publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, const int& index)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now(); // Use node's time stamp in ROS2
    marker.ns = "particle_swarm";
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // Convert grid cell positions into world coordinates using the resolution and origin
    for (const auto& position : positions) {
      geometry_msgs::msg::Point p;
      p.x = origin_x_ + position.first * resolution_;
      p.y = origin_y_ + position.second * resolution_;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    // Set marker lifetime (using rclcpp::Duration in ROS2)
    marker.lifetime = builtin_interfaces::msg::Duration();
    auto duration = rclcpp::Duration(1, 0);
    marker.lifetime.sec = duration.seconds();
    marker.lifetime.nanosec = duration.nanoseconds() % 1000000000;

    particle_pub_->publish(marker);
  }
  
}  // namespace nav2_pso_planner

