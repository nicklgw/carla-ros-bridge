#include "carla_shenlan_stanley_pid_controller/stanely_controller.h"
#include "carla_shenlan_stanley_pid_controller/pid_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

namespace shenlan {
    namespace control {

        shenlan::control::PIDController e_theta_pid_controller(1.0, 0.0, 0.4); // PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上

        double atan2_to_PI(const double atan2) 
        {
            return atan2 * M_PI / 180;
        }

        double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) 
        {
            const double dx = point.x - x;
            const double dy = point.y - y;
            return dx * dx + dy * dy;
        }

        void StanleyController::LoadControlConf() 
        {
            k_y_ = 1.0;
        }

        // /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
        // 提示，在该函数中你需要调用计算误差
        // 控制器中，前轮转角的命令是弧度单位，发送给carla的横向控制指令，范围是 -1~1
        void StanleyController::ComputeControlCmd(const VehicleState &vehicle_state, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) 
        {  
            trajectory_points_= planning_published_trajectory.trajectory_points;

            double vehicle_x = vehicle_state.x;
            double vehicle_y = vehicle_state.y;
            double vehicle_theta = vehicle_state.heading;
            double vehicle_vel = vehicle_state.v + 0.001;

            double e_y;
            double e_theta;
            ComputeLateralErrors(vehicle_x, vehicle_y, vehicle_theta, e_y, e_theta);
            
            double e_delta = atan2(k_y_*e_y, vehicle_vel);
            double target_theta = e_theta + e_delta;
            if(target_theta > M_PI/3) 
            {
                target_theta=  M_PI / 3 ;
            }
            else if (target_theta < -M_PI/3)
            {
                target_theta =  -M_PI / 3;
            }   

            cmd.steer_target = target_theta;

            cout << "StanleyController::ComputeControlCmd" << endl;
            cout << "target_theta: "<< target_theta <<endl;            
        }

        // /** to-do **/ 计算需要的误差，包括横向误差，纵向误差，误差计算函数没有传入主车速度，因此返回的位置误差就是误差，不是根据误差计算得到的前轮转角
        void StanleyController::ComputeLateralErrors(const double x, const double y, const double theta, double &e_y, double &e_theta) 
        {
            // 获取距离车辆最近的路径点信息
            TrajectoryPoint closest_waypoint = QueryNearestPointByPosition(x, y);
            // 横向误差e_y
            double dx = closest_waypoint.x - x;
            double dy = closest_waypoint.y - y;
            
            // 将最近路径点从世界坐标系转换到坐标系  (vehicle_x, vehicle_y, closest_waypoint.heading)
            // 注意 vehicle_theta + e_theta = closest_waypoint.heading
            double closest_waypoint_y_in_vehicle_coordinate = -dx*sin(closest_waypoint.heading) + dy*cos(closest_waypoint.heading);
            e_y = -closest_waypoint_y_in_vehicle_coordinate;

            // 路径点距离车辆最近点的参考航向角，大于车辆当前航向角的话，车辆应该左转以跟踪航向
            e_theta = theta - closest_waypoint.heading;
            if(e_theta > M_PI)
            {
                e_theta = e_theta - 2 * M_PI ;
            }
            else if (e_theta < -M_PI)
            {
               e_theta = e_theta + 2 * M_PI ;
            }

            cout << "StanleyController::ComputeLateralErrors" << endl;
            cout << "e_y: "<< e_y <<endl;
            cout << "e_theta: "<< e_theta <<endl;
        }



        // 返回参考路径上和车辆当前位置距离最近的点，返回的是点结构体
        TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x, const double y) 
        {
            double d_min = PointDistanceSquare(trajectory_points_.front(), x, y); // 得到当前位置距离参考路径的第一个点的距离
            size_t index_min = 0;

            for (size_t i = 1; i < trajectory_points_.size(); ++i) 
            {
                double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
                if (d_temp < d_min) 
                {
                    d_min = d_temp;
                    index_min = i;
                }
            }
            // cout << " index_min: " << index_min << endl;
            //cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
            theta_ref_ = trajectory_points_[index_min].heading; // 获得距离车辆当前位置最近的路径点的航向角

            return trajectory_points_[index_min];
        }
    }  // namespace control
}  // namespace shenlan