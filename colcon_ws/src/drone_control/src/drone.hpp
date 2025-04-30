#ifndef DRONE_HPP
#define DRONE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include <chrono>
#include <memory>
#include <string>

class DRONE : public rclcpp::Node
{
public:
  DRONE() : Node("DRONE"),
            takeoff_duration_(5.0),
            flight_duration_(10.0),   // longer outbound
            return_duration_(10.0),   // longer return
            distance_(5.0)
  {
    // Configure quality-of-service parameters for best-effort communication.
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                           .keep_last(10);

    // Publisher for setpoint positions.
    setpoint_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", qos_profile);

    // Subscription to odometry data.
    odom_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos_profile,
      std::bind(&DRONE::odom_callback, this, std::placeholders::_1));

    // Client for the set_mode service.
    client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  }

  void main_loop()
  {
    // Slow update rate for smoother motion
    rclcpp::Rate rate(10.0);  // 10 Hz instead of 50
    int toggle = 1;
    while (rclcpp::ok())
    {
        // Publish next waypoint
        setpoint.pose.position.x = setPoints[current_setpoint_ind][0];
        setpoint.pose.position.y = setPoints[current_setpoint_ind][1];
        setpoint.pose.position.z = setPoints[current_setpoint_ind][2];
        setpoint.header.stamp = this->now();
        setpoint.header.frame_id = "map";

        if (check_setpoint_reached(current_pose, setpoint)) {
            // Move to next
            if(current_setpoint_ind == total_setpoints - 1) {
                // Reset to start
                current_setpoint_ind = total_setpoints - 1;
            }else
              current_setpoint_ind = (current_setpoint_ind + toggle);
          if (current_setpoint_ind == 0) {
                // Reset to end
                toggle = 1;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Current setpoint: %d", current_setpoint_ind);
        publish_setpoint(setpoint);
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
  }

private:
  // ... existing methods unchanged ...

  // Extended list of waypoints for slower motion:
  static constexpr int total_setpoints = 13;
  float setPoints[total_setpoints][3] = {
    // Takeoff to 10m
    // { 0.0f,  0.0f, 10.0f},
    // // Outbound path in 1m steps
    // { 0.25f, -0.25f, 10.0f},
    // { 0.5f, -0.5f, 10.0f},
    // { 0.75f, -0.75f, 10.0f},
    // { 1.0f, -1.0f, 10.0f},
    // { 1.25f, -1.25f, 10.0f},
    // { 1.5f, -1.5f, 10.0f},
    // { 1.75f, -1.75f, 10.0f},
    // { 2.0f, -2.0f, 10.0f},
    // { 2.25f, -2.25f, 10.0f},
    // { 2.5f, -2.5f, 10.0f},
    // { 2.75f, -2.75f, 10.0f},
    // { 3.0f, -3.0f, 10.0f},
    { 0.0f,  0.0f, 10.0f},
    {0.25f, 0.0f, 10.0f},
    {0.75f, 0.0f, 10.0f},
    {1.75f, 0.0f, 10.0f},
    {2.75f, 0.0f, 10.0f},
    {3.75f, 0.0f, 10.0f},
    {4.75f, 0.0f, 10.0f},
    {5.75f, 0.0f, 10.0f},
    {6.75f, 0.0f, 10.0f},
    {7.75f, 0.0f, 10.0f},
    {8.75f, 0.0f, 10.0f},
    {9.75f, 0.0f, 10.0f},
    {10.0f, 0.0f, 10.0f},

  };

  int current_setpoint_ind = 0;

  // ROS communication objects
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_;

  geometry_msgs::msg::PoseStamped setpoint;
  geometry_msgs::msg::PoseStamped current_pose;

  // Flight variables
  double takeoff_duration_, flight_duration_, return_duration_, distance_;

  void setMode(const std::string &mode);

  // Check reach threshold
  bool check_setpoint_reached(const geometry_msgs::msg::PoseStamped &current_pose,
                              const geometry_msgs::msg::PoseStamped &target)
  {
    double tol = 0.2;  // Increase tolerance for smooth switching
    return (std::abs(current_pose.pose.position.x - target.pose.position.x) < tol &&
            std::abs(current_pose.pose.position.y - target.pose.position.y) < tol &&
            std::abs(current_pose.pose.position.z - target.pose.position.z) < tol);
  }

  void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose = *msg;
  }

  void publish_setpoint(const geometry_msgs::msg::PoseStamped &setpoint)
  {
    setpoint_pub->publish(setpoint);
  }
};

#endif // DRONE_HPP
