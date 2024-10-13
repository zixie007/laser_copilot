#include "common.hh"
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <queue>
#include <tf2_ros/static_transform_broadcaster.h>
namespace laser_copilot_applications {
class traj_replayer : public rclcpp::Node {
public:
  explicit traj_replayer(const rclcpp::NodeOptions &options)
      : Node("traj_replayer", options) {
    load_param();
    regist_callback();
  }

private:
  void load_param() {
    T_localflu_localfrd_.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    T_localfrd_localflu_ = T_localflu_localfrd_.inverse();  //加不加逆都可以
    load_setpoints(declare_parameter("file_path", std::string("")),  //导入第一次手飞航点
                   declare_parameter("topic", std::string("")));
  }

  void regist_callback() {
    using namespace std::placeholders;
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "sub/odom", rclcpp::SensorDataQoS(),
        std::bind(&traj_replayer::cb_odom, this, _1));
    pub_sp_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("pub/setpoint", 5);
    pub_path_ = 
        create_publisher<nav_msgs::msg::Path>("pub/path", 5);

    using namespace std::chrono_literals;
    timer_100hz_ =  //发布pub/setpoint话题  每10ms触发一次
        create_wall_timer(10ms, std::bind(&traj_replayer::cb_100hz, this));
    timer_1hz_ = create_wall_timer(1s, std::bind(&traj_replayer::cb_1hz, this));  //发布pub/path话题 每秒触发一次
    
  }

  void load_setpoints(std::string file_path, std::string topic) {
    rosbag2_cpp::Reader reader;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
    nav_msgs::msg::Odometry odom_msg;
    reader.open(file_path);
    double yaw_frd_ned = NAN;
    while (reader.has_next()) {  //循环读取bag文件中的消息，直到没有更多消息
      auto msg = reader.read_next();
      if (msg->topic_name != topic)
        continue;
      rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};  //将读取到的序列化消息反序列化为Odometry消息
      serialization.deserialize_message(&serialized_msg, &odom_msg);
      setpoints_.emplace_back(
          Eigen::Vector3d{odom_msg.pose.pose.position.x,
                          odom_msg.pose.pose.position.y,
                          odom_msg.pose.pose.position.z},
          Eigen::Quaterniond{odom_msg.pose.pose.orientation.w,
                             odom_msg.pose.pose.orientation.x,
                             odom_msg.pose.pose.orientation.y,
                             odom_msg.pose.pose.orientation.z});
    }
    RCLCPP_INFO_STREAM(get_logger(), "load: " << setpoints_.size()
                                              << " points from " << file_path);
  }

  //接受里程计数据
  void cb_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto &pos = msg->pose.pose.position;
    const auto &ori = msg->pose.pose.orientation;
    cur_pose_.position << pos.x, pos.y, pos.z;
    cur_pose_.orientation = Eigen::Quaterniond(ori.w, ori.x, ori.y, ori.z);
    cur_pose_.yaw = quaternion_to_yaw(cur_pose_.orientation);
    const auto &vel = msg->twist.twist.linear;
    const auto &ang_vel = msg->twist.twist.angular;
    cur_pose_.linear_vel << vel.x, vel.y, vel.z;
    cur_pose_.angle_vel << ang_vel.x, ang_vel.y, ang_vel.z;
    cur_pose_.linear_vel = cur_pose_.orientation * cur_pose_.linear_vel;
    cur_pose_.angle_vel = cur_pose_.orientation * cur_pose_.angle_vel;
    cur_pose_.stamp = static_cast<uint64_t>(msg->header.stamp.sec) *
                          static_cast<uint64_t>(1e9) +
                      static_cast<uint64_t>(msg->header.stamp.nanosec);
  }

  void cb_1hz(){
    static bool is_init_path_msg = false;
    static nav_msgs::msg::Path msg;
    if(!is_init_path_msg) {
      is_init_path_msg = true;
      msg.header.frame_id = "odom";
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      for (const auto& sp : setpoints_) {  //遍历bag的setpoints_
        pose.pose.position.x = sp.position[0];
        pose.pose.position.y = sp.position[1];
        pose.pose.position.z = sp.position[2];
        Eigen::Quaterniond q(Eigen::AngleAxisd(sp.yaw, Eigen::Vector3d::UnitZ())
                                 .toRotationMatrix());  //将yaw转换为四元数
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        msg.poses.push_back(pose);
      }
    }
    msg.header.stamp = get_clock()->now();
    pub_path_->publish(msg);
  }

  void cb_100hz() {
    if (setpoints_.empty()) {  //如果为空表示完成返航
      RCLCPP_INFO(get_logger(), "traj replay finish, node exit!");
      exit(0);
    }
    if (!is_need_update_sp()) {
      return;
    }
    auto sp = setpoints_.front().to_affine();  //取出集合中的首设定点，并将其转换为仿射变换形式
    setpoints_.pop_front();  //从集合中移除取出的点
    Eigen::Vector3d position {sp.translation()};  //提取设定点的位置和姿态信息
    Eigen::Quaterniond orientation{sp.linear()};  
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = get_clock()->now();
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    pub_sp_->publish(msg);
  }

  bool is_need_update_sp() {
    bool norm_ok = false;
    norm_ok = (cur_pose_.position - setpoints_.front().position).norm() < 3;  //检查当前点与下个弹出航点的距离是否小于3米  小于3m：1.防止航点大跳变跟丢 2.3m内继续发航点，依速度控制（3m产生速度控制上限值）若无障碍物速度非常快，但不会超过设定值
    return norm_ok && setpoints_.size() > 1;  //小于3m && 不是最后一个航点  
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_sp_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  rclcpp::TimerBase::SharedPtr timer_100hz_;
  std::deque<setpoint_t> setpoints_;
  pose_t cur_pose_;
  Eigen::Affine3d T_odomflu_odomned_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_odomned_odomflu_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localflu_localfrd_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localfrd_localflu_ = Eigen::Affine3d::Identity();
};
}; // namespace laser_copilot_applications

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(laser_copilot_applications::traj_replayer)