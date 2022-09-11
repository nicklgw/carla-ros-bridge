#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "pid_controller.h"
#include <std_msgs/msg/bool.hpp>

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_status.hpp"

using std::placeholders::_1;

class VehicleControlPublisher : public rclcpp::Node
{
public:
    VehicleControlPublisher();
    ~VehicleControlPublisher();

    double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y);
    
    TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    void VehicleControlIterationCallback(); // 收到仿真器返回的状态后，产生控制信号
    // void VehicleControlIterationCallback(carla_msgs::msg::CarlaStatus::SharedPtr msg); // 收到仿真器返回的状态后，产生控制信号

public:
    double V_set_;
    double T_gap_;

    bool first_record_;
    bool cout_distance_;
    bool cout_speed_;

    int cnt;
    int qos;

    double controller_frequency;

    double acceleration_cmd;
    double steer_cmd;

    std::vector<TrajectoryPoint> trajectory_points_;
    TrajectoryData planning_published_trajectory;

    // Input
    VehicleState vehicle_state_;

    tf2::Quaternion localization_quaternion_transform;

    rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer_;

    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr vehicle_control_publisher;
    carla_msgs::msg::CarlaEgoVehicleControl control_cmd;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vehicle_control_manual_override_publisher;
    std_msgs::msg::Bool vehicle_control_manual_override;

    rclcpp::Subscription<carla_msgs::msg::CarlaStatus>::SharedPtr carla_status_subscriber;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscriber;

    
};

#endif