#include "PathSmoother.h"
#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>
#include <random>

PathSmoother::PathSmoother() {}

std::vector<std::tuple<double, double, double>> PathSmoother::smooth_path(
    const std::vector<std::tuple<double, double, double>>& path,
    double smoothing_factor,
    int num_points) {
    
    if (path.size() <= 2) {
        return path;  // Can't smooth a path with 2 or fewer points
    }

    auto smoothed_path = path;  // Copy original path
    
    // Apply iterative smoothing - default to 10 iterations if num_points not specified
    int iterations = (num_points == -1) ? 10 : num_points;
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<std::tuple<double, double, double>> new_path = smoothed_path;
        
        // Don't modify start and end points
        for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
            double x_i = std::get<0>(smoothed_path[i]);
            double y_i = std::get<1>(smoothed_path[i]);
            double z_i = std::get<2>(smoothed_path[i]);
            
            double x_prev = std::get<0>(smoothed_path[i-1]);
            double y_prev = std::get<1>(smoothed_path[i-1]);
            double z_prev = std::get<2>(smoothed_path[i-1]);
            
            double x_next = std::get<0>(smoothed_path[i+1]);
            double y_next = std::get<1>(smoothed_path[i+1]);
            double z_next = std::get<2>(smoothed_path[i+1]);
            
            // Calculate new position as weighted average of current position and neighbors
            double new_x = x_i + smoothing_factor * (x_prev + x_next - 2*x_i);
            double new_y = y_i + smoothing_factor * (y_prev + y_next - 2*y_i);
            double new_z = z_i + smoothing_factor * (z_prev + z_next - 2*z_i);
            
            new_path[i] = std::make_tuple(new_x, new_y, new_z);
        }
        
        smoothed_path = new_path;
    }
    
    return smoothed_path;
}

std::vector<std::tuple<double, double, double>> PathSmoother::smooth_path_with_obstacle_avoidance(
    const std::vector<std::tuple<double, double, double>>& path,
    std::shared_ptr<Map3D> map3d,
    double smoothing_factor,
    int num_points,
    double safety_distance,
    int max_iterations) {
    
    if (path.size() <= 2) {
        return path;
    }
    
    // First, perform basic smoothing
    auto smoothed_path = smooth_path(path, smoothing_factor, num_points);
    
    // Then detect conflicts with obstacles
    auto conflict_points_info = detect_obstacle_conflicts(smoothed_path, map3d);
    
    // If there are conflicts, try to fix them with local resmoothing
    if (!conflict_points_info.empty()) {
        smoothed_path = _local_resmooth(smoothed_path, conflict_points_info, map3d);
    }
    
    return smoothed_path;
}

std::vector<std::tuple<int, std::tuple<double, double, double>, std::tuple<double, double, double>>> 
PathSmoother::detect_obstacle_conflicts(
    const std::vector<std::tuple<double, double, double>>& path,
    std::shared_ptr<Map3D> map3d) {
    
    std::vector<std::tuple<int, std::tuple<double, double, double>, std::tuple<double, double, double>>> conflicts;
    
    for (size_t i = 0; i < path.size(); ++i) {
        double x = std::get<0>(path[i]);
        double y = std::get<1>(path[i]);
        double z = std::get<2>(path[i]);
        
        if (map3d->isObstacle(x, y, z)) {
            // Add conflict info: index, conflict point, original point (same in this case)
            conflicts.push_back(std::make_tuple(i, path[i], path[i]));
        }
    }
    
    return conflicts;
}

std::vector<std::tuple<double, double, double>> PathSmoother::_optimize_straight_lines(
    const std::vector<std::tuple<double, double, double>>& path,
    std::shared_ptr<Map3D> map3d) {
    // Simplified implementation - just return the original path
    return path;
}

bool PathSmoother::_is_line_clear(
    const std::tuple<double, double, double>& start,
    const std::tuple<double, double, double>& end,
    std::shared_ptr<Map3D> map3d,
    int num_check_points) {
    // Calculate step size
    double step_x = (std::get<0>(end) - std::get<0>(start)) / (num_check_points + 1);
    double step_y = (std::get<1>(end) - std::get<1>(start)) / (num_check_points + 1);
    double step_z = (std::get<2>(end) - std::get<2>(start)) / (num_check_points + 1);
    
    for (int i = 1; i <= num_check_points; ++i) {
        double x = std::get<0>(start) + step_x * i;
        double y = std::get<1>(start) + step_y * i;
        double z = std::get<2>(start) + step_z * i;
        
        if (map3d->isObstacle(x, y, z)) {
            return false;
        }
    }
    
    return true;
}

std::vector<std::tuple<double, double, double>> PathSmoother::_linear_interpolate_path(
    const std::vector<std::tuple<double, double, double>>& path,
    int num_points) {
    if (path.empty()) {
        return path;
    }
    
    if (num_points <= 0) {
        return path;  // Return original path if num_points not specified
    }
    
    std::vector<std::tuple<double, double, double>> interpolated_path;
    
    // Add the start point
    interpolated_path.push_back(path.front());
    
    // Calculate total distance along the path
    double total_distance = 0.0;
    std::vector<double> distances;
    distances.push_back(0.0);
    
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = std::get<0>(path[i]) - std::get<0>(path[i-1]);
        double dy = std::get<1>(path[i]) - std::get<1>(path[i-1]);
        double dz = std::get<2>(path[i]) - std::get<2>(path[i-1]);
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        total_distance += dist;
        distances.push_back(total_distance);
    }
    
    // Interpolate points along the path
    for (int i = 1; i < num_points - 1; ++i) {
        double target_distance = total_distance * i / (num_points - 1);
        
        // Find the segment that contains the target distance
        int seg_idx = 0;
        for (size_t j = 1; j < distances.size(); ++j) {
            if (distances[j] >= target_distance) {
                seg_idx = j - 1;
                break;
            }
        }
        
        if (seg_idx >= static_cast<int>(path.size()) - 1) {
            break;
        }
        
        // Interpolate between the two points of the segment
        double segment_start_dist = distances[seg_idx];
        double segment_end_dist = distances[seg_idx + 1];
        double ratio = (target_distance - segment_start_dist) / (segment_end_dist - segment_start_dist);
        
        double x1 = std::get<0>(path[seg_idx]);
        double y1 = std::get<1>(path[seg_idx]);
        double z1 = std::get<2>(path[seg_idx]);
        
        double x2 = std::get<0>(path[seg_idx + 1]);
        double y2 = std::get<1>(path[seg_idx + 1]);
        double z2 = std::get<2>(path[seg_idx + 1]);
        
        double interp_x = x1 + ratio * (x2 - x1);
        double interp_y = y1 + ratio * (y2 - y1);
        double interp_z = z1 + ratio * (z2 - z1);
        
        interpolated_path.push_back(std::make_tuple(interp_x, interp_y, interp_z));
    }
    
    // Add the end point
    if (num_points > 1) {
        interpolated_path.push_back(path.back());
    }
    
    return interpolated_path;
}

std::tuple<double, double, double> PathSmoother::_find_free_point_heuristic(
    const std::tuple<double, double, double>& point,
    const std::tuple<double, double, double>& original_point,
    std::shared_ptr<Map3D> map3d,
    double safety_distance) {
    // For simplicity, we'll use the safety distance as the resolution multiplier
    double offset = (safety_distance > 0) ? safety_distance : map3d->getResolution() * 2.0;
    
    // Try different directions from the conflict point toward the original point
    double cx = std::get<0>(point);
    double cy = std::get<1>(point);
    double cz = std::get<2>(point);
    
    double ox = std::get<0>(original_point);
    double oy = std::get<1>(original_point);
    double oz = std::get<2>(original_point);
    
    // Direction vector from conflict point to original point
    double dx = ox - cx;
    double dy = oy - cy;
    double dz = oz - cz;
    
    // Normalize and scale by offset
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist > 0) {
        dx = dx / dist * offset;
        dy = dy / dist * offset;
        dz = dz / dist * offset;
    } else {
        dx = dy = dz = offset;
    }
    
    // Try moving in the direction of the original point
    std::vector<std::tuple<double, double, double>> candidates = {
        std::make_tuple(cx + dx, cy + dy, cz + dz),  // Toward original point
        std::make_tuple(cx - dx, cy - dy, cz - dz),  // Away from original point
        std::make_tuple(cx + offset, cy, cz),        // +X
        std::make_tuple(cx - offset, cy, cz),        // -X
        std::make_tuple(cx, cy + offset, cz),        // +Y
        std::make_tuple(cx, cy - offset, cz),        // -Y
        std::make_tuple(cx, cy, cz + offset),        // +Z
        std::make_tuple(cx, cy, cz - offset)         // -Z
    };
    
    for (const auto& candidate : candidates) {
        double x = std::get<0>(candidate);
        double y = std::get<1>(candidate);
        double z = std::get<2>(candidate);
        
        if (map3d->isValidPosition(x, y, z) && !map3d->isObstacle(x, y, z)) {
            return candidate;
        }
    }
    
    // If no free point found, return the original point
    return point;
}

std::tuple<double, double, double> PathSmoother::_find_free_point(
    const std::tuple<double, double, double>& point,
    std::shared_ptr<Map3D> map3d,
    double safety_distance) {
    return _find_free_point_heuristic(point, point, map3d, safety_distance);
}

std::vector<std::tuple<double, double, double>> PathSmoother::_local_resmooth(
    const std::vector<std::tuple<double, double, double>>& path,
    const std::vector<std::tuple<int, std::tuple<double, double, double>, std::tuple<double, double, double>>>& conflict_points_info,
    std::shared_ptr<Map3D> map3d) {
    
    auto smoothed_path = path;
    
    // Process each conflict point
    for (const auto& conflict_info : conflict_points_info) {
        int idx = std::get<0>(conflict_info);
        auto conflict_point = std::get<1>(conflict_info);
        auto original_point = std::get<2>(conflict_info);
        
        // Find a free point near the conflict point
        auto free_point = _find_free_point_heuristic(conflict_point, original_point, map3d, map3d->getResolution() * 2.0);
        
        // Replace the conflict point with the free point
        if (idx < static_cast<int>(smoothed_path.size()) && idx >= 0) {
            smoothed_path[idx] = free_point;
        }
    }
    
    // Now apply a gentler smoothing to avoid sharp turns
    // But only smooth points that are not at the ends
    for (int iter = 0; iter < 3; ++iter) {  // Only a few iterations to prevent over-smoothing
        std::vector<std::tuple<double, double, double>> new_path = smoothed_path;
        
        for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
            // Check if this point is in conflict again after relocation
            double x = std::get<0>(smoothed_path[i]);
            double y = std::get<1>(smoothed_path[i]);
            double z = std::get<2>(smoothed_path[i]);
            
            if (!map3d->isObstacle(x, y, z)) {
                // Apply gentle smoothing to non-conflicting points
                double x_i = std::get<0>(smoothed_path[i]);
                double y_i = std::get<1>(smoothed_path[i]);
                double z_i = std::get<2>(smoothed_path[i]);
                
                double x_prev = std::get<0>(smoothed_path[i-1]);
                double y_prev = std::get<1>(smoothed_path[i-1]);
                double z_prev = std::get<2>(smoothed_path[i-1]);
                
                double x_next = std::get<0>(smoothed_path[i+1]);
                double y_next = std::get<1>(smoothed_path[i+1]);
                double z_next = std::get<2>(smoothed_path[i+1]);
                
                // Use a very small smoothing factor to avoid creating new conflicts
                double sf = 0.1;  // Much smaller to prevent excessive deviation
                double new_x = x_i + sf * (x_prev + x_next - 2*x_i);
                double new_y = y_i + sf * (y_prev + y_next - 2*y_i);
                double new_z = z_i + sf * (z_prev + z_next - 2*z_i);
                
                // Verify this new point doesn't cause an obstacle conflict
                if (!map3d->isObstacle(new_x, new_y, new_z)) {
                    new_path[i] = std::make_tuple(new_x, new_y, new_z);
                }
            }
        }
        
        smoothed_path = new_path;
    }
    
    return smoothed_path;
}