#include "common.hh"
#include <chrono>  //时钟，时间日期
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
namespace laser_copilot_applications {
// constexpr 在编译时计算常量表达式  寻找最优方向
constexpr int FILTER_SIZE = 120.0f / 2.0f * DEG_TO_RAD / RAD_INC;  //60度转换成弧度制/某个增量 
constexpr int SEARCH_RANGE = 120.0f / 2.0f * DEG_TO_RAD / RAD_INC;
constexpr double WEIGHT_DISTANCE = 10.0;  
constexpr double WEIGHT_DEVIATION = 0.1;

using namespace std::chrono_literals;  //引入该命名空间下的时间字面量
class safe_fly_controller : public rclcpp::Node {
public:
  explicit safe_fly_controller(const rclcpp::NodeOptions &options)  //explicit防止进行隐式转换
      : Node("safe_fly_controller", options) {
    init_cb();
    load_param();
    init_members();
  }

private:
  void init_cb() {
    using std::placeholders::_1;  //命名空间提供了占位符
#ifdef OPT_CONTROLLER_USE_PX4_MSG
    sub_px4_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),  //Qos质量服务配置
        std::bind(&safe_fly_controller::cb_px4_odom, this, _1));  //std::bind 用于绑定函数的部分参数，生成一个新的可调用对象 _1为占位符
    pub_px4_cmd_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
    pub_px4_mode_ctrl_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
#endif

#ifdef OPT_CONTROLLER_USE_MAVROS_MSG
    pub_mavros_pos_target_ = create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
#endif

    sub_vel_ = create_subscription<geometry_msgs::msg::Twist>(  //遥控器-offboard
        "sub/vel", 5, std::bind(&safe_fly_controller::cb_vel, this, _1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "sub/goal", 5, std::bind(&safe_fly_controller::cb_goal, this, _1)); 
    sub_objs_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "sub/objs", 5, std::bind(&safe_fly_controller::cb_objs, this, _1));
    pub_dbg_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("pub/debug", 5);
    timer_50hz_ = this->create_wall_timer(
        20ms, std::bind(&safe_fly_controller::cb_50hz, this));  

    timer_once_1s_ = create_wall_timer(1s, [this]() {
      this->timer_once_1s_->cancel();
      this->sub_px4_odom_.reset(); // cancle px4 sub
      this->sub_nav_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "sub/odom", rclcpp::SensorDataQoS(),
          std::bind(&safe_fly_controller::cb_nav_odometry, this, _1));
      this->T_odomflu_odomned_.linear() = Eigen::Matrix3d(  //flu-ned：先把自身坐标系flu旋转至北,再旋转180度到世界坐标系NED
          Eigen::AngleAxisd(this->cur_pose_.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      this->T_odomned_odomflu_ = this->T_odomflu_odomned_.inverse();
      this->T_localflu_localfrd_.linear() =  //flu和frd均为自身坐标系
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();    
      this->T_localfrd_localflu_ = this->T_localflu_localfrd_;
      this->T_odomflu_odomfrd_ = this->T_localflu_localfrd_;
      this->T_odomfrd_odomflu_ = this->T_odomflu_odomfrd_;
      RCLCPP_INFO_STREAM(get_logger(),
                         "detect init yaw(degree) in odom_ned frame: "
                             << this->cur_pose_.yaw / M_PI * 180.0);
    });
  }

  void load_param() {
    max_speed_ = declare_parameter("max_speed", 2.0);  //declare_parameter声明设置参数
    max_acc_ = declare_parameter("max_acc", 1.0);
    max_jerk_ = declare_parameter("max_jerk", 0.2);  //最大加加速度
    max_yaw_speed_ = declare_parameter("max_yaw_speed", 30.0) / 180.0 * M_PI;
    min_dist_ = declare_parameter("min_distance", 1.0);
    obj_sensor_delay_ns_ =
        declare_parameter("object_sensor_delay_ms", 100) * 1e6;
    target_.position.z() = declare_parameter("takeoff_height", 1.0);
  }

  void init_members() {
    //初始化PID控制器
    vel_pid_.kp = 0.3;
    vel_pid_.all_err = Eigen::Vector3d::Zero();
    vel_pid_.last_err = Eigen::Vector3d::Zero();
    
    // 计算 PID 输出
    // double p_term = vel_pid_.kp * current_error.norm();
    // double i_term = vel_pid_.all_err.norm();
    // double d_term = (current_error - vel_pid_.last_err).norm();
    // double pid_output = p_term + i_term + d_term;

    yaw_pid_.all_err = 0.0;
    yaw_pid_.last_err = 0.0;

    objs_msg_.range_min = 100.0;
    objs_msg_.range_max = 0.0;
    
    //采用速度控制
    ctrl_msg_.position = false;
    ctrl_msg_.attitude = true;
    ctrl_msg_.velocity = true;
    ctrl_msg_.acceleration = true;
    ctrl_msg_.body_rate = true;
    ctrl_msg_.actuator = false;
  }

    //处理订阅的里程计数据
  void cb_nav_odometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto &pos = msg->pose.pose.position;
    const auto &ori = msg->pose.pose.orientation;
    cur_pose_.position << pos.x, pos.y, pos.z;
    cur_pose_.orientation = Eigen::Quaterniond(ori.w, ori.x, ori.y, ori.z);
    cur_pose_.yaw = quaternion_to_yaw(cur_pose_.orientation);  //将四元数转换为偏航角
    const auto &vel = msg->twist.twist.linear;
    const auto &ang_vel = msg->twist.twist.angular;
    cur_pose_.linear_vel << vel.x, vel.y, vel.z;
    cur_pose_.angle_vel << ang_vel.x, ang_vel.y, ang_vel.z;
    cur_pose_.linear_vel = cur_pose_.orientation * cur_pose_.linear_vel;  //坐标变换
    cur_pose_.angle_vel = cur_pose_.orientation * cur_pose_.angle_vel;
    cur_pose_.stamp = static_cast<uint64_t>(msg->header.stamp.sec) *  //合并时间辍纳秒级
                          static_cast<uint64_t>(1e9) +
                      static_cast<uint64_t>(msg->header.stamp.nanosec);  //static_cast<uint64_t>的作用是将变量显式转换为64位无符号整数类型uint64_t
  }

  void cb_px4_odom(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    cur_pose_.yaw = quaternion_to_yaw(Eigen::Quaterniond(
        msg->q[0], msg->q[1], msg->q[2], msg->q[3]));
  }

  //用于遥控器离线模式
  // void cb_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg) {  //SharedPtr：是一种智能指针，用于管理动态分配的对象的生命周期，自动处理内存释放
  //   target_.position[0] = std::abs(msg->linear.x) > 0.05
  //                             ? (cur_pose_.position[0] + msg->linear.x)
  //                             : target_.position[0];
  //   target_.position[1] = std::abs(msg->linear.y) > 0.05
  //                             ? (cur_pose_.position[1] + msg->linear.y)
  //                             : target_.position[1];
  //   target_.position[2] = std::abs(msg->linear.z) > 0.05
  //                             ? (cur_pose_.position[2] + msg->linear.z)
  //                             : target_.position[2];
  //   target_.yaw = std::abs(msg->angular.z) > 0.05
  //                     ? (cur_pose_.yaw + msg->angular.z)
  //                     : target_.yaw;
  }

  //接收目标位置
  void cb_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    target_ = setpoint_t({msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z < 0.2 ? cur_pose_.position.z()  //target_z小于0.2时，保持当前高度
                                                     : msg->pose.position.z},
                         {msg->pose.orientation.w, msg->pose.orientation.x,
                          msg->pose.orientation.y, msg->pose.orientation.z});
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 500,
                                "goint to: " << target_.position.transpose()
                                             << " "
                                             << target_.yaw / M_PI * 180.0);
  }

  void cb_50hz() { go_to_target(target_); }

  void go_to_target(const setpoint_t &tgt) {
    Eigen::Vector3d speed_vec = vel_pid_(tgt.position, cur_pose_.position);  
    Eigen::Vector3d acc_vec = speed_vec - cur_pose_.linear_vel;
    if (speed_vec.norm() > 0.05) {
      Eigen::Vector3d speed_dir = speed_vec / speed_vec.norm();
      Eigen::Vector3d acc_dir = acc_vec / acc_vec.norm();  
      acc_vec = std::min(max_acc_, acc_vec.norm()) * acc_dir;  //如果超越最大加速度，实际的加速度小于上述计算值和max_acc_
      speed_vec =
          std::min(max_speed_, (cur_pose_.linear_vel + acc_vec).norm()) *  //利用新得到的加速度反推速度
          speed_dir;
      adjust_velocity_setpoint(speed_vec);
      acc_vec = speed_vec - cur_pose_.linear_vel;  //再次调整加速度
    }

    double vyaw = tgt.yaw - cur_pose_.yaw;
    if (vyaw > M_PI) {
      vyaw -= 2 * M_PI;  //调整后,yaw介于-pai-pai之间
    } else if (vyaw < -M_PI) {
      vyaw += 2 * M_PI;
    }
    int vyaw_dir = vyaw / std::abs(vyaw);
    vyaw = std::min(std::abs(vyaw), max_yaw_speed_) * vyaw_dir;

#ifdef OPT_CONTROLLER_USE_PX4_MSG  // 使用px4控制
    ctrl_msg_.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    pub_px4_mode_ctrl_->publish(ctrl_msg_);
    pub_px4_setpoint(speed_vec, acc_vec, vyaw);
#endif

#ifdef OPT_CONTROLLER_USE_MAVROS_MSG  // 使用mavros控制
    pub_mavros_setpoint(speed_vec, acc_vec, vyaw);
#endif

    std_msgs::msg::Float64MultiArray dbg_msg;
    dbg_msg.data.push_back(cur_pose_.position[0]); // 0
    dbg_msg.data.push_back(cur_pose_.position[1]);
    dbg_msg.data.push_back(cur_pose_.position[2]);
    dbg_msg.data.push_back(cur_pose_.linear_vel.norm()); // 3
    dbg_msg.data.push_back(cur_pose_.linear_vel[0]);     // 4
    dbg_msg.data.push_back(cur_pose_.linear_vel[1]);
    dbg_msg.data.push_back(cur_pose_.linear_vel[2]);
    dbg_msg.data.push_back(cur_pose_.yaw * RAD_TO_DEG); // 7
    dbg_msg.data.push_back(speed_vec.norm());           // 8
    dbg_msg.data.push_back(speed_vec[0]);               // 9
    dbg_msg.data.push_back(speed_vec[1]);
    dbg_msg.data.push_back(speed_vec[2]);
    dbg_msg.data.push_back(acc_vec.norm()); // 12
    dbg_msg.data.push_back(acc_vec[0]);     // 13
    dbg_msg.data.push_back(acc_vec[1]);
    dbg_msg.data.push_back(acc_vec[2]);
    dbg_msg.data.push_back(vyaw);            // 16
    dbg_msg.data.push_back(tgt.position[0]); // 17
    dbg_msg.data.push_back(tgt.position[1]);
    dbg_msg.data.push_back(tgt.position[2]);
    dbg_msg.data.push_back(tgt.yaw * RAD_TO_DEG); // 20
    pub_dbg_->publish(dbg_msg);
  }

  //传递坐标变换后的速度和加速度，偏航速度
  void pub_px4_setpoint(Eigen::Vector3d vel, Eigen::Vector3d acc, double vyaw) {
    vel = T_odomflu_odomned_ * vel;  //坐标变换
    acc = T_odomflu_odomned_ * acc;
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = get_clock()->now().nanoseconds() * 1e-3;
    msg.position = {NAN, NAN, NAN};
    msg.yaw = NAN;
    msg.velocity = {float(vel[0]), float(vel[1]), float(vel[2])};
    msg.acceleration = {float(acc[0]), float(acc[1]), float(acc[2])};
    msg.jerk = {NAN, NAN, NAN};
    msg.yawspeed = (T_odomflu_odomned_ * Eigen::Vector3d{0, 0, vyaw})[2];  
    pub_px4_cmd_->publish(msg);
  }

  void pub_mavros_setpoint(Eigen::Vector3d vel, Eigen::Vector3d acc,
                           double vyaw) {
    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "odom"; // mavros donot use this
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |  //忽略 X、Y、Z 轴的位置和偏航角
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    msg.velocity.x = vel[0];
    msg.velocity.y = vel[1];
    msg.velocity.z = vel[2];
    msg.acceleration_or_force.x = acc[0];
    msg.acceleration_or_force.y = acc[1];
    msg.acceleration_or_force.z = acc[2];
    msg.yaw_rate = vyaw;
    pub_mavros_pos_target_->publish(msg);
  }

  //分配扫描范围
  void cb_objs(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    objs_msg_.header = msg->header;
    objs_msg_.range_max = msg->range_max;
    objs_msg_.range_min = msg->range_min;
    objs_msg_.ranges.resize(GROUP_NUM);
    int id_drone = id(cur_pose_.yaw);
    for (int i = 0; i < GROUP_NUM; i++) {
      objs_msg_.ranges[wrap_id(id_drone + i)] = msg->ranges[i];
    }
  }

  void adjust_velocity_setpoint(Eigen::Vector3d &setpoint) {
    if (objs_msg_.range_min > objs_msg_.range_max) {
      return;
    }
    if (std::abs(setpoint.z()) > 0.05 || setpoint.norm() < 0.05) {  //z轴速度大于0.05，或速度小于0.05，不进行速度控制
      return;
    }
    double speed = setpoint.norm();
    setpoint.normalize();
    // adjust_move_direction(setpoint);
    adjust_speed(setpoint, speed);  //速度控制的核心函数
  }

  //被注释掉：找到最优的移动方向
  // void adjust_move_direction(Eigen::Vector3d &direction) {
  //   const double sp_idx_original = id(direction.x(), direction.y());
  //   double sp_idx_new = sp_idx_original;
  //   double best_cost = std::numeric_limits<double>::max();
  //   for (int i = sp_idx_original - SEARCH_RANGE;
  //        i <= sp_idx_original + SEARCH_RANGE; i++) {
  //     const double mean_dist = mean_distance(i);
  //     const int id = wrap_id(i);
  //     const double deviation_cost = std::abs(i - sp_idx_original) / min_dist_;
  //     const double cost =
  //         deviation_cost * WEIGHT_DEVIATION - mean_dist * WEIGHT_DISTANCE;
  //     if (cost < best_cost) {
  //       best_cost = cost;
  //       sp_idx_new = id;
  //     }
  //   }
  //   if (sp_idx_new != sp_idx_original) {
  //     double angle = sp_idx_new * RAD_INC;
  //     direction << std::cos(angle), std::sin(angle), 0;
  //   }
  }

  void adjust_speed(Eigen::Vector3d &sp_direction, double sp_speed) {
    Eigen::Vector3d direction;
    const double min_distance_to_keep =
        std::min<double>(objs_msg_.range_min, min_dist_);
    // const double min_distance_to_keep = 1;  //20241212 change 1 固定停止距离
    for (int i = 0; i < GROUP_NUM; i++) {
      if (objs_msg_.ranges[i] >= MAX_DIST)  //忽略远范围障碍物
        continue;
      double angle = i * RAD_INC;  //计算障碍物角度：以传感器前进方向为0度
      direction << std::cos(angle), std::sin(angle), 0.;
      if (sp_direction.dot(direction) <= 0)   // 计算内积，忽略三四象限的障碍物，忽略与欲行进方向相反的障碍物
        continue;
      const double cur_vel_parallel =  // 计算当前速度在障碍物方向的分量
          std::max(0.0, cur_pose_.linear_vel.dot(direction));
      const double delay_distance =  // 计算因传感器延迟导致的额外距离
          obj_sensor_delay_ns_ * cur_vel_parallel * 1e-9;
      // if(objs_msg_.ranges[i]< 0.3) //20241212 change 2 再次滤除自身一定范围内的点
      //   continue;
      const double stop_distance = std::max(  // 计算需要停止的距离
          0.0, objs_msg_.ranges[i] - delay_distance - min_distance_to_keep);
      const double vel_max_pid = stop_distance * 0.3;  //根据停止距离计算最大速度
      const double vel_max_smooth =  // 平滑处理速度与距离的关系
          smooth_velocity_from_distance(stop_distance);
      const double projection = direction.dot(sp_direction);  // 两个方向向量的内积
      double vel_max_dir = sp_speed;  //不赋值貌似也可以
      if (projection > 0.01) {  // 处理与前进方向-pai-pai的障碍物 感觉不加判断也可以
        vel_max_dir = std::max(vel_max_pid, vel_max_smooth) / projection;  // 夹角越垂直，速度越大(如果没障碍物gaga提速，但不会超过刚开始规划的速度)
      }
      vel_max_dir = std::max(vel_max_dir, 0.); //确保速度为非负
      sp_speed = std::min(sp_speed, vel_max_dir);
    }
    sp_direction *= sp_speed;  //好像不再是方向向量
  }

  //被注释了
  // double mean_distance(int idx) {
  //   double mean_dist = 0;
  //   double range_max = std::max<double>(objs_msg_.range_max, min_dist_);
  //   for (int j = idx - FILTER_SIZE; j <= idx + FILTER_SIZE; j++) {
  //     int id = wrap_id(j);
  //     if (objs_msg_.ranges[id] == MAX_DIST) {
  //       mean_dist += range_max;
  //     } else {
  //       mean_dist += objs_msg_.ranges[id];
  //     }
  //   }
  //   return mean_dist / (2.f * FILTER_SIZE + 1.f);
  // }

  // copy from px4 TrajMath.hpp L61
  // https://github.com/PX4/PX4-Autopilot/blob/b8c541dd7277ed735139d7d1bfb829d61fbe29fb/src/lib/mathlib/math/TrajMath.hpp#L61
  double smooth_velocity_from_distance(const double stop_distance) {  //构造一个二元函数ax^2 + bx + c = 0 来计算最大速度，其中 a = 1, b = -b, c = c，求解
    static const double b = 4.0 * max_acc_ * max_acc_ / max_jerk_;
    double c = -2.0 * max_acc_ * stop_distance;
    double max_speed = 0.5 * (-b + std::sqrt(b * b - 4.0 * c));
    return std::max(max_speed, 0.);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_nav_odom_ =
      nullptr;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      sub_px4_odom_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_ =
      nullptr;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_objs_ =
      nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_ = nullptr;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_px4_cmd_ =
      nullptr;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      pub_px4_mode_ctrl_ = nullptr;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr
      pub_mavros_pos_target_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_dbg_ =
      nullptr;
  rclcpp::TimerBase::SharedPtr timer_50hz_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_once_1s_ = nullptr;
  px4_msgs::msg::OffboardControlMode ctrl_msg_;
  Eigen::Isometry3d T_odomflu_odomned_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odomned_odomflu_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odomfrd_odomflu_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odomflu_odomfrd_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_localflu_localfrd_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_localfrd_localflu_ = Eigen::Isometry3d::Identity();
  double max_speed_;
  double max_acc_;
  double max_jerk_;
  double max_yaw_speed_;
  double min_dist_;
  pose_t cur_pose_;
  pose_t prv_pose_;
  setpoint_t target_;
  pid_controller<Eigen::Vector3d> vel_pid_;
  pid_controller<double> yaw_pid_;
  sensor_msgs::msg::LaserScan objs_msg_;
  double obj_sensor_delay_ns_;
};
}; // namespace laser_copilot_applications

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot_applications::safe_fly_controller)  