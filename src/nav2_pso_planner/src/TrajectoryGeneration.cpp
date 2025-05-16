#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "nav2_pso_planner/TrajectoryGeneration.hpp"

namespace nav2_pso_planner
{
    // Constructor
    TrajectoryGeneration::TrajectoryGeneration()
    {
    }
    
    // Destructor
    TrajectoryGeneration::~TrajectoryGeneration()
    {
    }
    
    int TrajectoryGeneration::splineOrder = 3; // Set the order of the B-spline curve
    
    // Generate B-spline curve control points
    void TrajectoryGeneration::GenerateControlPoints(const std::pair<double, double>& start_d,
                                                     const std::pair<double, double>& goal_d,
                                                     const std::vector<std::pair<int, int>>& initial_position,
                                                     std::vector<std::pair<double, double>>& initial_point)
    {
        initial_point.clear();  // Clear the vector to start fresh
        initial_point.push_back(start_d);
        std::pair<double, double> last_point = start_d;
    
        for (const auto& position : initial_position)
        {
            double x = static_cast<double>(position.first);
            double y = static_cast<double>(position.second);
            // Check if the current point is different from the last point
            if (x != last_point.first || y != last_point.second)
            {
                initial_point.emplace_back(x, y);
                last_point = {x, y};
            }
        }
    
        // Append goal point if it is different from the last point
        if (goal_d != last_point)
        {
            initial_point.push_back(goal_d);
        }
    }
    
    // De Boor-Cox recursion to calculate Bik(u)
    double TrajectoryGeneration::BaseFun(int i, int k, double u, std::vector<double> NodeVector)
    {
        if (k == 0)
        {
            if ((u >= NodeVector[i]) && (u < NodeVector[i + 1]))
            {
                return 1.0;
            }
            else
            {
                return 0.0;
            }
        }
        else
        {
            double Length1 = NodeVector[i + k] - NodeVector[i];
            double Length2 = NodeVector[i + k + 1] - NodeVector[i + 1];
    
            if (Length1 == 0.0)
            {
                Length1 = 1.0;
            }
            if (Length2 == 0.0)
            {
                Length2 = 1.0;
            }
    
            return ((u - NodeVector[i]) / Length1) * BaseFun(i, k - 1, u, NodeVector) +
                   ((NodeVector[i + k + 1] - u) / Length2) * BaseFun(i + 1, k - 1, u, NodeVector);
        }
    }
    
    // B-spline curve for trajectory smoothing.
    // This function replaces the original plan with the smoothed B-spline curve.
    void TrajectoryGeneration::B_spline_curve(std::vector<std::pair<double, double>> &plan, int k)
    {
        if (plan.size() < 2)
            return;
    
        double plan_length = 0.0;
        for (size_t i = 1; i < plan.size(); i++)
        {
            plan_length += std::sqrt(
                (plan[i].first - plan[i - 1].first) * (plan[i].first - plan[i - 1].first) +
                (plan[i].second - plan[i - 1].second) * (plan[i].second - plan[i - 1].second)
            );
        }
    
        double d;
        std::pair<double, double> new_control_point;
        d = std::sqrt(
            (plan[1].first - plan[0].first) * (plan[1].first - plan[0].first) +
            (plan[1].second - plan[0].second) * (plan[1].second - plan[0].second)
        );
        new_control_point.first = plan[0].first - (0.1 / d) * (plan[1].first - plan[0].first);
        new_control_point.second = plan[0].second - (0.1 / d) * (plan[1].second - plan[0].second);
        plan.insert(plan.begin(), new_control_point);
    
        int n = plan.size();
    
        std::vector<double> NodeVector;
        for (int i = 0; i < k + 1; i++) {
            NodeVector.push_back(0.0);
        }
        for (int i = 1; i <= (n - k - 1); i++) {
            NodeVector.push_back(double(i) / (n - k));
        }
        for (int i = 0; i <= k + 1; i++) {
            NodeVector.push_back(1.0);
        }
    
        std::vector<double> u;
        double temp_u = 0.0;
        double end_u = 1.0;
        double temp_dt = 1.0 / plan_length;
        while (temp_u < end_u) {
            u.push_back(temp_u);
            temp_u += temp_dt;
        }
    
        std::vector<double> Bik(n, 0);
        std::vector<std::pair<double, double>> B_plan;
        B_plan.reserve(u.size());
    
        for (size_t i = 0; i < u.size(); i++) {
            double B_plan_x = 0.0;
            double B_plan_y = 0.0;
            for (int j = 0; j < n; j++) {
                Bik[j] = BaseFun(j, k - 1, u[i], NodeVector);
            }
            for (int m = 0; m < n; m++) {
                B_plan_x += plan[m].first * Bik[m];
                B_plan_y += plan[m].second * Bik[m];
            }
            B_plan.emplace_back(B_plan_x, B_plan_y);
        }
    
        plan.swap(B_plan);
    }
    
    // Calculate the Euclidean distance between two points.
    double TrajectoryGeneration::calculateDistance(const std::pair<double, double>& point1,
                                                   const std::pair<double, double>& point2)
    {
        double dx = point1.first - point2.first;
        double dy = point1.second - point2.second;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Calculate the path length.
    double TrajectoryGeneration::calculatePathLength(const std::vector<std::pair<double, double>>& path)
    {
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            length += calculateDistance(path[i - 1], path[i]);
        }
        return length;
    }
    
} // namespace nav2_pso_planner

