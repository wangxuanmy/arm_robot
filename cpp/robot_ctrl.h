#ifndef ROBOT_CTRL_H
#define ROBOT_CTRL_H

#include "arm_utils/darwin.h"
#include "plan/Map3D.h"
#include "plan/AStar3D.h"
#include "plan/PathSmoother.h"
#include <vector>
#include <tuple>
#include <string>
#include <memory>
#include <jsoncpp/json/json.h>  // Using JsonCpp for JSON parsing

namespace arm_robot {

class RobotCtrl {
private:
    std::string config_path_;
    std::shared_ptr<Human> robot_;
    std::shared_ptr<Human> robot_copy_;
    std::shared_ptr<Map3D> map3d_;
    std::shared_ptr<AStar3D> path_planner_;
    std::shared_ptr<PathSmoother> path_smoother_;
    
    // Configuration data loaded from JSON
    std::vector<std::vector<double>> dh_params_body_;
    std::vector<std::vector<double>> dh_params_left_hand_;
    std::vector<std::vector<double>> dh_params_right_hand_;
    std::vector<std::vector<double>> dh_params_head_;
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;
    std::vector<int> enable_joint_;
    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> init_offset_;
    
    // Current path data
    std::vector<std::tuple<double, double, double>> current_path_;
    std::vector<std::tuple<double, double, double>> smoothed_path_;

    double MOVE_THETA_THRESHOLD_ = 0.3;

public:
    /**
     * Constructor - Initialize the robot controller with JSON config
     * @param json_config_path Path to JSON config file (e.g., darwin_02.json)
     */
    explicit RobotCtrl(const std::string& json_config_path);

    /**
     * Load configuration from JSON file
     * @param json_config_path Path to JSON config file
     */
    void loadConfig(const std::string& json_config_path);

    /**
     * Initialize the robot model based on loaded configuration
     */
    void initRobot();

    /**
     * Reset robot to initial position
     */
    void resetToInitialPosition();

    /**
     * Initialize 3D map
     * @param width Map width (X-axis) in meters
     * @param height Map height (Y-axis) in meters
     * @param depth Map depth (Z-axis) in meters
     * @param resolution Map resolution (actual distance per grid, in meters)
     * @param origin_x Map origin X coordinate (world coordinate)
     * @param origin_y Map origin Y coordinate (world coordinate)
     * @param origin_z Map origin Z coordinate (world coordinate)
     */
    void initMap(int width, int height, int depth, double resolution = 1.0,
                 double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0);

    /**
     * Set obstacle inflation radius
     * @param radius Inflation radius (actual units, in meters)
     */
    void setInflationRadius(double radius);

    /**
     * Add obstacle at specified position
     * @param x, y, z Obstacle world coordinates
     * @return true if added successfully, otherwise false
     */
    bool addObstacle(double x, double y, double z);

    /**
     * Add obstacles from point cloud data
     * @param points Point cloud data list, each element is (x, y, z) world coordinate tuple
     * @return Number of obstacles successfully added
     */
    int addObstaclesFromPointCloud(const std::vector<std::tuple<double, double, double>>& points);

    /**
     * Remove obstacle at specified position
     * @param x, y, z Obstacle world coordinates
     * @return true if removed successfully, otherwise false
     */
    bool removeObstacle(double x, double y, double z);

    void clearMap();

    /**
     * Plan path from start to goal
     * @param start Start world coordinate (x, y, z)
     * @param goal Goal world coordinate (x, y, z)
     * @return Path point list (world coordinates), return empty list if no path found
     */
    std::vector<std::tuple<double, double, double>> planPath(
        std::tuple<double, double, double> start,
        std::tuple<double, double, double> goal);

    /**
     * Smooth the path
     * @param path Path to smooth, if nullptr use most recently planned path
     * @param smoothing_factor Smoothing factor, larger value means smoother
     * @param num_points Number of output path points
     * @param with_obstacle_avoidance Whether to consider obstacle avoidance during smoothing
     * @return Smoothed path point list
     */
    std::vector<std::tuple<double, double, double>> smoothPath(
        const std::vector<std::tuple<double, double, double>>& path = {},
        double smoothing_factor = 0.5,
        int num_points = -1,
        bool with_obstacle_avoidance = true);


    /**
     * Update robot joint angles
     * @param theta Joint angle list
     */
    void updateRobotTheta(const std::vector<double>& theta);

    /**
     * Get current robot state
     * @return Vector containing all joint angles
     */
    std::vector<double> getRobotState();

    /**
     * Check collision at given pose
     * @param pose Pose to check (x, y, z)
     * @param safety_margin Safety margin
     * @return true if collision occurs, otherwise false
     */
    bool isCollisionAtPose(const std::tuple<double, double, double>& pose, 
                          double safety_margin = 0.1);


    std::vector<std::vector<double>> smoothJointAngles(const std::vector<std::vector<double>>& thetas,
            const Eigen::MatrixXd& theta_min,
            const Eigen::MatrixXd& theta_max,
            double max_threshold);

    void syncBodyAndCopy(); 

    bool isSameTheta(std::string part_name, double theta_threshold = 0.008);

    std::pair<bool, std::vector<std::vector<double>>> findFollowPath(const Eigen::Matrix4d& aim_mat,
        const std::string& part_name = "body",
        bool ignore_angle = false);

    std::pair<bool, std::vector<std::vector<double>>> goAim(const Eigen::Matrix4d& aim_mat,const std::string& part_name = "body",
        bool ignore_angle = false);

    std::vector<double> moveViaVelocity(const std::string& part_name, Eigen::VectorXd vel);

    // 双臂镜像同步
    std::pair<bool, std::vector<std::vector<double>>> cooperationJoint(const std::string& main_name, const std::string& sub_name, const std::vector<std::vector<double>>& theta_input);
    
    // 身体动 双臂保持不动
    std::pair<bool, std::map<std::string, std::vector<std::vector<double>>>> cooperationBody(const std::vector<std::string>& sub_name, const std::vector<std::vector<double>>& theta_input);
    
    // 关节插值
    std::vector<std::vector<double>> jointInterpolation(const std::vector<double>& thetas,
                                                       const std::string& part_name,
                                                       double speed);

    // Getter methods
    std::shared_ptr<Human> getRobot() const { return robot_; }
    std::shared_ptr<Human> getRobotCopy() const { return robot_copy_; }
    std::shared_ptr<Map3D> getMap() const { return map3d_; }
    std::vector<std::tuple<double, double, double>> getCurrentPath() const { return current_path_; }
    std::vector<std::tuple<double, double, double>> getSmoothedPath() const { return smoothed_path_; }




};

} // namespace arm_robot

#endif // ROBOT_CTRL_H