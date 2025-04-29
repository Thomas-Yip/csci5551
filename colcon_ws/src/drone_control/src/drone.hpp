// #ifndef DRONE_HPP
// #define DRONE_HPP

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "mavros_msgs/srv/set_mode.hpp"
// #include "mavros_msgs/msg/state.hpp"
// #include <chrono>
// #include <memory>
// #include <string>

// class DRONE : public rclcpp::Node
// {
// public:
//   DRONE() : Node("DRONE"),
//             takeoff_duration_(5.0),
//             flight_duration_(5.0),
//             return_duration_(5.0),
//             distance_(5.0)
//   {
//     // Configure quality-of-service parameters for best-effort communication.
//     auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
//                            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
//                            .keep_last(10);

//     // Create publisher for setpoint positions.
//     setpoint_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
//       "/mavros/setpoint_position/local", qos_profile);

//     // Create subscription to odometry data.
//     odom_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/mavros/local_position/pose", qos_profile,
//       std::bind(&DRONE::odom_callback, this, std::placeholders::_1));

//     // Create a client for the set_mode service.
//     client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

//   }

//   void main_loop()
//   {
//     rclcpp::Rate rate(50.0);
//     while(rclcpp::ok()) 
//     {
//         setpoint.pose.position.x = setPoints[current_setpoint_ind][0];
//         setpoint.pose.position.y = setPoints[current_setpoint_ind][1];
//         setpoint.pose.position.z = setPoints[current_setpoint_ind][2];
//         setpoint.header.stamp = this->now();
//         setpoint.header.frame_id = "map"; // Use "map" as the frame of reference.
//         if(check_setpoint_reached(current_pose, setpoint)) { // use the odometry to update the current waypoint
//             current_setpoint_ind++;
//         }
//         if(current_setpoint_ind == final_setpoint_ind) {
//             // setpoint.pose.position.x = 0;
//             // setpoint.pose.position.y = 0;
//             // setpoint.pose.position.z = 10; // Return to the initial position
//             current_setpoint_ind = 0;
//         }

//         publish_setpoint(setpoint);
        
//         rclcpp::spin_some(this->get_node_base_interface());
//         rate.sleep();
//     }
//   }

// private:
//   // Function to request a mode change for the drone.
//   void setMode(const std::string &mode);

//   // Odometry callback: logs the current position.
//   void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received odometry data: Position (%f, %f, %f)",
//                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     current_pose = *msg;
//   }

//   // Helper function to publish the setpoint.
//   void publish_setpoint(const geometry_msgs::msg::PoseStamped &setpoint)
//   {
//     setpoint_pub->publish(setpoint);
//   }

//   bool check_setpoint_reached(const geometry_msgs::msg::PoseStamped &current_pose, 
//                                   const geometry_msgs::msg::PoseStamped &target_setpoint)
//   {
//     // Check if the current position is within a threshold of the target setpoint.
//     double threshold = 0.1; // 10 cm tolerance
//     return (std::abs(current_pose.pose.position.x - target_setpoint.pose.position.x) < threshold &&
//             std::abs(current_pose.pose.position.y - target_setpoint.pose.position.y) < threshold &&
//             std::abs(current_pose.pose.position.z - target_setpoint.pose.position.z) < threshold);
//   }

//   // ROS communication objects.
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
//   rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_;
//   rclcpp::TimerBase::SharedPtr timer;
  
//   // Flight scheduling variables.
//   rclcpp::Time start_time;
//   double takeoff_duration_;  // Duration (s) for the takeoff phase.
//   double flight_duration_;   // Duration (s) for flying outwards.
//   double return_duration_;   // Duration (s) for the return flight.
//   double distance_;          // Distance (m) for the outbound flight along the x-axis.
//   geometry_msgs::msg::PoseStamped setpoint;
//   geometry_msgs::msg::PoseStamped current_pose; // Current position of the drone.
//   float setPoints[5][3] = { 
//     {0, 0, 10},
//     {0, -5, 10},
//     // {10, 10, 10},
//     // {0, 10, 10},
//     // {0, 0, 10}
    
//   };

//   // {x, y, z}
//   // red, green, blue  
//   int current_setpoint_ind = 0; // Index to track the current setpoint.
//   int final_setpoint_ind = 2;
// };

// #endif // DRONE_HPP
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
            current_setpoint_ind = (current_setpoint_ind + 1) % total_setpoints;
        }

        publish_setpoint(setpoint);
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
  }

private:
  // ... existing methods unchanged ...

  // Extended list of waypoints for slower motion:
  static constexpr int total_setpoints = 12;
  float setPoints[total_setpoints][3] = {
    // Takeoff to 10m
    { 0.0f,  0.0f, 10.0f},
    // Outbound path in 1m steps
    { 0.0f, -1.0f, 10.0f},
    { 0.0f, -2.0f, 10.0f},
    { 0.0f, -3.0f, 10.0f},
    { 0.0f, -4.0f, 10.0f},
    { 0.0f, -5.0f, 10.0f},
    // Return path in 1m steps
    { 0.0f, -4.0f, 10.0f},
    { 0.0f, -3.0f, 10.0f},
    { 0.0f, -2.0f, 10.0f},
    { 0.0f, -1.0f, 10.0f},
    // Hover back at origin
    { 0.0f,  0.0f, 10.0f},
    // Land / final point
    // { 0.0f,  0.0f,  0.0f}
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
