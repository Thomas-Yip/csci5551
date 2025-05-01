#ifndef DRONE_HPP
#define DRONE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include <chrono>
#include <memory>
#include <string>
using namespace std;
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

    final_dest_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone/final_dest", qos_profile);
    
    // Timer for periodic tasks.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DRONE::pub_final_dest, this));

      if (no_setpoint) {
        // Set mode to OFFBOARD
        // setMode("OFFBOARD");
        no_setpoint = false;
        cout << "Enter search range: x_s, y_s, x_e, y_e, altitude, step_size" << endl;
        int x_s, y_s, x_e, y_e;
        int z;
        int step_size;
        step_size = 1;
        cin >> x_s >> y_s >> x_e >> y_e >> z >> step_size;
        generate_waypoints(x_s, y_s, x_e, y_e, z, step_size);        
        final_dest.pose.position.x = waypoints[current_setpoint_ind][0];
        final_dest.pose.position.y = waypoints[current_setpoint_ind][1];
        final_dest.pose.position.z = waypoints[current_setpoint_ind][2];
        string buf;
    }

  }

  ~DRONE()
  {
    if (waypoints != nullptr) {
      delete[] waypoints;
    }
  }

  void main_loop()
  {
    // Slow update rate for smoother motion
    rclcpp::Rate rate(10.0);  // 10 Hz instead of 50
    int toggle = 1;
    while (rclcpp::ok())
    {
        
        // Publish next waypoint
        if (!enter_go) {
            cout << "Enter go to fly" << endl;
            string buf;
            cin >> buf;
            if (buf == "go") {
                cout << "Flying..." << endl;
                current_setpoint_ind = 0;
                arrived = false;
                enter_go = true;
            }
        }
      
        setpoint.pose.position.x = waypoints[current_setpoint_ind][0];
        setpoint.pose.position.y = waypoints[current_setpoint_ind][1];
        setpoint.pose.position.z = waypoints[current_setpoint_ind][2];
        setpoint.header.stamp = this->now();
        setpoint.header.frame_id = "map";

        if (check_setpoint_reached(current_pose, setpoint)) {
            if (current_setpoint_ind < num_waypoints && arrived == false) {
              current_setpoint_ind  += 1;
            }else{
              current_setpoint_ind = num_waypoints - 1;
              arrived = true;
            }
              cout << "Reached setpoint: " << current_setpoint_ind << endl;
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
  bool no_setpoint = true;
  bool arrived = false;
  bool enter_go = false;
  int current_setpoint_ind = 0;

  // ROS communication objects
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr final_dest_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped setpoint;
  geometry_msgs::msg::PoseStamped current_pose;
  geometry_msgs::msg::PoseStamped final_dest;

  // Flight variables
  double takeoff_duration_, flight_duration_, return_duration_, distance_;

  int **waypoints = nullptr;
  int num_waypoints = 0;
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

  void generate_waypoints(int x_s, int y_s, int x_e, int y_e, int z, int step_size)
  {
    int search_start_range[2] = {x_s,y_s};
    int search_end_range[2] = {x_e,y_e};
    num_waypoints = 
    (search_end_range[0] - search_start_range[0] + 1) * (search_end_range[1] - search_start_range[1] + 1);
    cout << num_waypoints << endl;
    int rows = search_end_range[1] - search_start_range[1];
    int cols = search_end_range[0] - search_start_range[0];
    int direction = 1;
    int row_transition = false;
    int x = search_start_range[0];
    int y = search_start_range[1];
    int lock = false;
    // int waypoints[num_waypoints][2];
    waypoints = new int*[num_waypoints];
    for (int i = 0; i < num_waypoints; i++) {
        waypoints[i] = new int[3];
    }
    for (int i = 0; i < num_waypoints; i++) {
        // cout << x << ',' << y << endl;
        waypoints[i][0] = x;
        waypoints[i][1] = y;
        waypoints[i][2] = z;
        if(!row_transition){
            if (direction == 1) {
                x += step_size;
            } else {
                x -= step_size;
            }
        }else{
            row_transition = false;
            lock = true;
            y += step_size;
        }
        if ((x == search_start_range[0] || x == search_end_range[0]) && !lock) {
            // row_transition =  true;
            direction *= -1;
            row_transition = true;
        }
        if(lock) 
            lock = false;
    }
  }

  void pub_final_dest()
  {
    final_dest.pose.position.x = waypoints[num_waypoints-1][0];
    final_dest.pose.position.y = waypoints[num_waypoints-1][1];
    final_dest.pose.position.z = waypoints[num_waypoints-1][2];
    final_dest.header.stamp = this->now();
    final_dest.header.frame_id = "map";
    final_dest_pub->publish(final_dest);
  }
};

#endif // DRONE_HPP
