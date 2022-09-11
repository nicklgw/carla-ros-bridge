#include "carla_shenlan_pid_controller/vehicle_longitudinal_controller_pid.h"

#include <math.h>

#include <fstream>

#include "carla_shenlan_pid_controller/common.h"

using namespace std;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("carla_shenlan_pid_controller_publisher");

// Controller
shenlan::control::PIDController yaw_pid_controller(0.5, 0.3, 0.1);      // 转向角pid
shenlan::control::PIDController speed_pid_controller(0.1, 0.6, 0.001);    // 速度pid Kp Ki Kd

VehicleControlPublisher::VehicleControlPublisher()
    : Node("carla_shenlan_pid_controller")
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    V_set_ = 30; // km/h
    // V_set_ = V_set_ / 3.6; // m/s
    T_gap_ = 2.0;

    first_record_ = true;
    cout_distance_ = true;
    cout_speed_ = true;

    cnt = 0;
    controller_frequency = 100;
    qos = 10;

    vehicle_control_iteration_timer_ = this->create_wall_timer(50ms, std::bind(&VehicleControlPublisher::VehicleControlIterationCallback, this));    // 10ms根据 100Hz换算得到
    // carla_status_subscriber = this->create_subscription<carla_msgs::msg::CarlaStatus>("/carla/status", qos, std::bind(&VehicleControlPublisher::VehicleControlIterationCallback, this, _1));

    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", qos, std::bind(&VehicleControlPublisher::odomCallback, this, _1));

    vehicle_control_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
    control_cmd.header.stamp = this->now();
    // control_cmd.gear = 1;
    control_cmd.manual_gear_shift = false;
    // control_cmd.reverse = false;
    control_cmd.hand_brake = false;

    // vehicle_control_manual_override_publisher = this->create_publisher<std_msgs::msg::Bool>("/carla/ego_vehicle/vehicle_control_manual_override", rclcpp::QoS(10).reliable().transient_local());
    // vehicle_control_manual_override.data = true;

    // vehicle_control_manual_override_publisher->publish(vehicle_control_manual_override);

    // 读取参考线路径
    std::ifstream infile("src/ros-bridge/carla_shenlan_project_1/carla_shenlan_pid_controller/data/gps_data_2022_09_09_15_18_45.csv", ios::in);    //将文件流对象与文件连接起来
    assert(infile.is_open());                                                                                                                      //若失败,则输出错误消息,并终止程序运行

    std::vector<std::pair<double, double>> xy_points;
    std::string _line;
    std::string count_csv;
    std::string timestamp;
    std::string x;
    std::string y;
    std::string z;
    std::string yaw;
    std::string velocity;

    while (getline(infile, _line)) {
        std::cout << _line << std::endl;

        //解析每行的数据
        stringstream ss(_line);
        string _sub;
        vector<string> subArray;

        //按照逗号分隔
        while (getline(ss, _sub, ',')) {
            subArray.push_back(_sub);
        }
        double pt_x = std::atof(subArray[2].c_str());
        double pt_y = std::atof(subArray[3].c_str());
        xy_points.push_back(std::make_pair(pt_x, pt_y));
    }
    infile.close();

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<shenlan::control::ReferenceLine> reference_line = std::make_unique<shenlan::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    for (size_t i = 0; i < headings.size(); i++) {
        std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
    }

    size_t _count_points = headings.size();
    size_t _stop_begin_point = ceil(_count_points * 0.85);    // 需要 include <math.h>
    size_t _stop_point = ceil(_count_points * 0.95);          // 需要 include <math.h>

    std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

    for (size_t i = 0; i < headings.size(); i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
        if (i < _stop_begin_point) {
            trajectory_pt.v = V_set_;
        } else {
            if (trajectory_pt.v > 1.0) {
                trajectory_pt.v = V_set_ * ((double)i / ((double)_stop_begin_point - (double)_stop_point) - (double)_stop_point / ((double)_stop_begin_point - (double)_stop_point));
            } else {
                trajectory_pt.v = 0;
            }
        }
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = headings[i];
        trajectory_pt.kappa = kappas[i];

        planning_published_trajectory.trajectory_points.push_back(trajectory_pt);
    }

    trajectory_points_ = planning_published_trajectory.trajectory_points;

    acceleration_cmd = 0.0;
    steer_cmd = 0.0;
}

VehicleControlPublisher::~VehicleControlPublisher() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/

double VehicleControlPublisher::PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : 两点之间的距离
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

TrajectoryPoint VehicleControlPublisher::QueryNearestPointByPosition(const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    // cout << "vehicle.x: " << x << " "
    //      << "vehicle.y: " << y << endl;
    // cout << "trajectory_points.x: " << trajectory_points_[index_min].x << " "
    //      << "trajectory_points.y: " << trajectory_points_[index_min].y;

    // cout << endl;

    return trajectory_points_[index_min];
}

void VehicleControlPublisher::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->pose.pose.position.x);
    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicle_state_.roll, vehicle_state_.pitch, vehicle_state_.yaw);

    if (first_record_) {
        vehicle_state_.start_point_x = msg->pose.pose.position.x;
        vehicle_state_.start_point_x = msg->pose.pose.position.x;
        // vehicle_state_.start_heading = vehicle_state_.yaw;
        vehicle_state_.start_heading = -M_PI / 2;
        first_record_ = false;
    }
    vehicle_state_.x = msg->pose.pose.position.x;
    vehicle_state_.y = msg->pose.pose.position.y;
    vehicle_state_.vx = msg->twist.twist.linear.x;
    vehicle_state_.vy = msg->twist.twist.linear.y;
    vehicle_state_.v = std::sqrt(vehicle_state_.vx * vehicle_state_.vx + vehicle_state_.vy * vehicle_state_.vy);    // 本车速度
    vehicle_state_.heading = vehicle_state_.yaw;                                                                    // pose.orientation是四元数
}

// void VehicleControlPublisher::VehicleControlIterationCallback(carla_msgs::msg::CarlaStatus::SharedPtr msg)
void VehicleControlPublisher::VehicleControlIterationCallback()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    TrajectoryPoint target_point_;

    target_point_ = this->QueryNearestPointByPosition(vehicle_state_.x, vehicle_state_.y);

    double v_err = target_point_.v - vehicle_state_.v * 3.6;                  // 速度误差
    double yaw_err = vehicle_state_.heading - target_point_.heading;    // 横摆角误差

    if (yaw_err > M_PI / 6) {
        yaw_err = M_PI / 6;
    } else if (yaw_err < -M_PI / 6) {
        yaw_err = -M_PI / 6;
    }

    if (cnt % 1 == 0) {
        // cout << "start_heading: " << vehicle_state_.start_heading << endl;
        // cout << "heading: " << vehicle_state_.heading << endl;
        cout << "v_err: " << v_err << endl;
        // cout << "yaw_err: " << yaw_err << endl;
        // cout << "control_cmd.target_wheel_angle: " << control_cmd.target_wheel_angle << endl;
    }

    acceleration_cmd = speed_pid_controller.Control(v_err, 0.05);
    // steer_cmd = yaw_pid_controller.Control(yaw_err, 0.01);

    // acceleration_cmd = -0.5;
    steer_cmd = 0;

    if (acceleration_cmd >= 1.0) {
        acceleration_cmd = 1.0;
    }
    if (acceleration_cmd <= -1) {
        acceleration_cmd = -1.0;
    }

    control_cmd.header.stamp = this->now();
    if (acceleration_cmd <= 0) {
        control_cmd.brake = -acceleration_cmd;
        control_cmd.throttle = 0;
    } else {
        control_cmd.throttle = acceleration_cmd;
        control_cmd.brake = 0;
    }

    control_cmd.steer = steer_cmd;

    std::cout << "acceleration_cmd: " << acceleration_cmd << std::endl;

    // control_cmd.gear = 1;
    // control_cmd.reverse = false;
    control_cmd.hand_brake = false;
    control_cmd.manual_gear_shift = false;

    vehicle_control_publisher->publish(control_cmd);
    cnt++;
}

int main(int argc, char **argv)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    RCLCPP_INFO(LOGGER, "Initialize node");

    rclcpp::init(argc, argv);

    auto n = std::make_shared<VehicleControlPublisher>();

    rclcpp::spin(n);

    rclcpp::shutdown();

    return 0;
}
