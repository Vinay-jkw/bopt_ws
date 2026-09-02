#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>
#include <mutex>
#include <limits>
#include <chrono>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>
class LiDARListener : public rclcpp::Node {
public:
    LiDARListener() : Node("lidar_listener"),slowdown_initiated(false),stop_initiated(false) {
        subscription_left = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_behind", 10, std::bind(&LiDARListener::leftCallback, this, std::placeholders::_1));
        
        //  subscription_right = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "filtered_scan_right", 10, std::bind(&LiDARListener::rightCallback, this, std::placeholders::_1));
    //      safety_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    // "safety_turnoff", 10, std::bind(&LiDARListener::safety_turnoff_callback, this, std::placeholders::_1));
     loadYAMLConfig();
     std::cout<<"constructor called"<<std::endl;
     velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/odometry/filtered", 10, std::bind(&LiDARListener::velocity_callback, this, std::placeholders::_1));
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&LiDARListener::velocity_callback2, this, std::placeholders::_1));
   //  publisher_ = this->create_publisher<std_msgs::msg::String>("danger_zone", 10);
        publisher_ = this->create_publisher<std_msgs::msg::String>("/byd/safety_trigger", 10);
          velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_remapped", 10);
    }

private:
 /*  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& sensor_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Determine the closest object within the scan
        float closest_range = std::numeric_limits<float>::max();
        int denger_element_count=0;
       int warning_element_count=0;
       for (size_t i = 0; i < msg->ranges.size(); ++i) {
       float range = msg->ranges[i];
         if (range>=0.05&&range <= 0.2) {
                  denger_element_count++;
        } else if (range > 0.2 && range <= 0.4) {
                    warning_element_count++;
        }
         if(denger_element_count>=5){
                 publishState("red");
                break;
            }
            else if(warning_element_count>=2){
                 publishState("yellow");
                 break;
            }
       }

        // for (auto range : msg->ranges) {
        //   //  if (range < 0.05) continue; // Skip invalid ranges
        //     if (range >= msg->range_min && range <= msg->range_max) {
        //         closest_range = std::min(closest_range, range);
        //     }
        // }
       
        // Update sensor-specific state
        // std::string new_state = closest_range <= 0.1 ? "0x48" :
        //                         closest_range > 0.1 && closest_range <= 0.2 ? "0x10" : "0x20";
        //  RCLCPP_INFO(this->get_logger(), "element detected at distance: %f sensor_id:%s", closest_range,sensor_id.c_str());
        // last_updates[sensor_id] = std::chrono::steady_clock::now();
        // sensor_states[sensor_id] = new_state;

        // // Check if all sensors have updated their state since the last publish
        // if (std::all_of(last_updates.begin(), last_updates.end(), [&](const auto& lu) {
        //         return lu.second > last_publish_time;
        //     })) {
            // Evaluate combined state from all sensors
            // std::string combined_state = std::accumulate(sensor_states.begin(), sensor_states.end(), std::string("0x20"),
            //                                              [](const std::string& acc, const auto& state) {
            //                                                  return state.second < acc ? state.second : acc;
            //                                              });
            // Assuming sensor_states is defined as std::map<std::string, std::string>
// std::string combined_state = std::accumulate(
//     sensor_states.begin(), sensor_states.end(), std::string("0x20"),
//     [](const std::string& acc, const std::pair<const std::string, std::string>& state) {
//         // Ensure "red" has the highest priority
//         if (acc == "0x48" || state.second == "0x48") {
//             return std::string("0x48");
//         }
//         // If any state is "yellow" and no "red" has been found, set to "yellow"
//         else if (acc == "0x10" || state.second == "0x10") {
//             return std::string("0x10");
//         }
//         return acc; // Keep the accumulator's state if no higher priority state is found
//     });



            // Publish new state if it has changed and is more critical
            // if (combined_state != current_state) {
            //     current_state = combined_state;
            //     publishState(current_state);
            //     last_publish_time = std::chrono::steady_clock::now();
            // }
       // }
    }
*/

struct Point {
    float x, y;
};

std::vector<std::vector<Point>> zones;
std::vector<Point> parseZone(const std::string& line) {
    std::vector<Point> zone;
    std::stringstream ss(line);
    std::string pointStr;

    while (getline(ss, pointStr, ';')) { // Split line into points using ';' as delimiter
        std::stringstream pointStream(pointStr);
        std::string coordinate;
        Point point;
        getline(pointStream, coordinate, ','); // Split point into coordinates using ',' as delimiter
        point.x = std::stof(coordinate);
        getline(pointStream, coordinate, ',');
        point.y = std::stof(coordinate);
        zone.push_back(point);
    }

    return zone;
}
void loadYAMLConfig() {
         std::ifstream file("/home/lenovo/lidar_ws/src/ldlidar_stl_ros2/src/example.txt"); // Open the file
    std::string line;

    while (getline(file, line)) { // Read file line by line
        zones.push_back(parseZone(line)); // Parse each line and add to zones
    }
      for (const auto& zone : zones) {
        for (const auto& point : zone) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }
    }
bool isPointInPolygon(const std::vector<Point>& polygon, const Point& point/*Point point, vector<Point> polygon*/)
{
	int num_vertices = polygon.size();
	double x = point.x, y = point.y;
	bool inside = false;

	// Store the first point in the polygon and initialize
	// the second point
	Point p1 = polygon[0], p2;

	// Loop through each edge in the polygon
	for (int i = 1; i <= num_vertices; i++) {
		// Get the next point in the polygon
		p2 = polygon[i % num_vertices];

		// Check if the point is above the minimum y
		// coordinate of the edge
		if (y > std::min(p1.y, p2.y)) {
			// Check if the point is below the maximum y
			// coordinate of the edge
			if (y <= std::max(p1.y, p2.y)) {
				// Check if the point is to the left of the
				// maximum x coordinate of the edge
				if (x <= std::max(p1.x, p2.x)) {
					// Calculate the x-intersection of the
					// line connecting the point to the edge
					double x_intersection
						= (y - p1.y) * (p2.x - p1.x)
							/ (p2.y - p1.y)
						+ p1.x;

					// Check if the point is on the same
					// line as the edge or to the left of
					// the x-intersection
					if (p1.x == p2.x
						|| x <= x_intersection) {
						// Flip the inside flag
						inside = !inside;
					}
				}
			}
		}

		// Store the current point as the first point for
		// the next iteration
		p1 = p2;
	}

	// Return the value of the inside flag
	return inside;
}

// Function to convert polar coordinates to Cartesian, assuming origin (0,0) and angle in radians
Point polarToCartesian(float range, float angle) {
    Point p;
    p.x = range * cos(angle);
    p.y = range * sin(angle);
    return p;
}

void processScan_left(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Define your polygon here based on the sensor position and the specified dimensions
    // This is a simplified placeholder for the actual logic you'd need to implement
   // RCLCPP_INFO(this->get_logger(), "Publishing...");
   std::vector<Point> danger_Zone3 = {
        {0.0,0.0},
        {-0.50, 0.48},  // Up
        {-0.50, 1.0}, // Left
        {0.50, 1.0}, // Down

        {0.50, 0.48}   // Right
    };
    std::vector<Point> danger_Zone2 = {
        {0.0,0.0},
        {-0.50, 0.48},  // Up
        {-0.50, 1.6}, // Left
        {0.50, 1.6}, // Down

        {0.50, 0.48}   // Right
    };
    std::vector<Point> danger_Zone1 = {
        {0.0,0.0},
        {-0.60, 0.48},  // Up
        {-0.60, 1.8}, // Left
        {0.60, 1.8}, // Down

        {0.60, 0.48}   // Right
    };
   // std::vector<Point> danger_Zone1=zones[2];
    std::vector<Point> warning_Zone = {
        {0.0,0.0},
        {0.71, 0.28},  // Up
        {0.71, 2.1}, // Left
        {-0.71, 2.1}, // Down

        {-0.71, 0.28}   // Right
    };
    int denger_point_count=0;
    int warning_point_count=0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max) continue; // Skip invalid ranges
        if(range<=0.2)continue;
        float angle = msg->angle_min + i * msg->angle_increment;
            Point p = polarToCartesian(range, angle);
            // if(range>0.3&&range<2.1){
            // RCLCPP_INFO(this->get_logger(), " element detected range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
            // }
            // Point p={0.0,0.0};
            
         if(current_velocity_.linear.x<-1.0){
          if(denger_point_count>=5){
            std_msgs::msg::String message;
                            message.data = "0x48";
                          publisher_->publish(message);
                          //if(stop_initiated==false&&current_velocity_.linear.x<0.0){
                          stop_robot();
                          RCLCPP_INFO(this->get_logger(), "stop command sent");
                            break;
          }
          if(warning_point_count>=5){
            std_msgs::msg::String message;
                            message.data = "0x10";
                          publisher_->publish(message);
                      // if(slowdown_initiated==false){
                          slow_down_robot();
                          break;
          }
    
    if (isPointInPolygon(danger_Zone1, p)) {
    //  RCLCPP_INFO(this->get_logger(), "incoming cmd_vel:%f ",cmd_vel_velocity_.linear.x);
            //  stop_robot(); // Assuming stop_robot is implemented elsewhere
              //auto message = std_msgs::msg::String();
            //  message.data = "danger zone";
              // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
              // publisher_->publish(message);
              denger_point_count++;
              
                  // }
              RCLCPP_INFO(this->get_logger(), " element detected in danger zone 1 range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
          }
        else if (isPointInPolygon(warning_Zone, p)) {
          //    auto message = std_msgs::msg::String();
          //      message.data = "warning zone";
          warning_point_count++;
             
                  //}
                // stop_initiated=false;
              RCLCPP_INFO(this->get_logger(), "element detected in waring zone range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
          } 
          else{
            //  std_msgs::msg::String message;
             // message.data = "0x20";
            // stop_initiated=false;
            //  publisher_->publish(message);
          }
     }
    else if(-0.1>current_velocity_.linear.x>-1.0){
        //  RCLCPP_INFO(this->get_logger(), "incoming cmd_vel:%f ",cmd_vel_velocity_.linear.x);
         
         
        if(denger_point_count>=5){
            std_msgs::msg::String message;
                            message.data = "0x48";
                          publisher_->publish(message);
                          //if(stop_initiated==false&&current_velocity_.linear.x<0.0){
                          stop_robot();
                          RCLCPP_INFO(this->get_logger(), "stop command sent");
                            break;
          }
          if(warning_point_count>=5){
            std_msgs::msg::String message;
                            message.data = "0x10";
                          publisher_->publish(message);
                      // if(slowdown_initiated==false){
                          slow_down_robot();
                          break;
          }
    
    if (isPointInPolygon(danger_Zone2, p)) {
    //  RCLCPP_INFO(this->get_logger(), "incoming cmd_vel:%f ",cmd_vel_velocity_.linear.x);
            //  stop_robot(); // Assuming stop_robot is implemented elsewhere
              //auto message = std_msgs::msg::String();
            //  message.data = "danger zone";
              // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
              // publisher_->publish(message);
              denger_point_count++;
              
                  // }
              RCLCPP_INFO(this->get_logger(), " element detected in danger zone 2 range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
          }
        else if (isPointInPolygon(warning_Zone, p)) {
          //    auto message = std_msgs::msg::String();
          //      message.data = "warning zone";
          warning_point_count++;
             
                  //}
                // stop_initiated=false;
              RCLCPP_INFO(this->get_logger(), "element detected in waring zone range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
          } 
          else{
            //  std_msgs::msg::String message;
             // message.data = "0x20";
            // stop_initiated=false;
            //  publisher_->publish(message);
          }
      }
      if(current_velocity_.linear.x>-0.1){
          if(denger_point_count>=5){
            std_msgs::msg::String message;
                            message.data = "0x48";
                          publisher_->publish(message);
                          //if(stop_initiated==false&&current_velocity_.linear.x<0.0){
                          stop_robot();
                          RCLCPP_INFO(this->get_logger(), "stop command sent");
                            break;
          }
          if(warning_point_count>=5){
            std_msgs::msg::String message;
                            message.data = "0x10";
                          publisher_->publish(message);
                      // if(slowdown_initiated==false){
                          slow_down_robot();
                          break;
          }
    
    if (isPointInPolygon(danger_Zone3, p)) {
    //  RCLCPP_INFO(this->get_logger(), "incoming cmd_vel:%f ",cmd_vel_velocity_.linear.x);
            //  stop_robot(); // Assuming stop_robot is implemented elsewhere
              //auto message = std_msgs::msg::String();
            //  message.data = "danger zone";
              // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
              // publisher_->publish(message);
              denger_point_count++;
              
                  // }
              RCLCPP_INFO(this->get_logger(), " element detected in danger zone 3 range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
          }
        else if (isPointInPolygon(warning_Zone, p)) {
          //    auto message = std_msgs::msg::String();
          //      message.data = "warning zone";
          warning_point_count++;
             
                  //}
                // stop_initiated=false;
              RCLCPP_INFO(this->get_logger(), "element detected in waring zone range:%f coordinate:x=%f,y=%f",range,p.x,p.y);
          } 
          else{
            //  std_msgs::msg::String message;
             // message.data = "0x20";
            // stop_initiated=false;
            //  publisher_->publish(message);
          }
     }
      // RCLCPP_INFO(this->get_logger(), "outgoing cmd_vel:%f ",cmd_vel_velocity_.linear.x);
    }
    if(denger_point_count<5&&warning_point_count<5){
     velocity_publisher_->publish(cmd_vel_velocity_);
     std_msgs::msg::String message;
      message.data = "0x20";
      publisher_->publish(message);
    }
}


    void leftCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        processScan_left(msg/*, "left"*/);
    }

   
     void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Update the current velocity of the robot
        current_velocity_ = *msg;
    }
    void velocity_callback2(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Update the current velocity of the robot
        cmd_vel_velocity_ = *msg;
    }
    void safety_turnoff_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
    // isSafetyTurnOff =true;
      RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }
     void slow_down_robot()
    {
        // Halve the speed of the robot
        cmd_vel_velocity_.linear.x *= 0.33;
        cmd_vel_velocity_.angular.z *= 0.33;

        // Publish the new velocity
        velocity_publisher_->publish(cmd_vel_velocity_);
        RCLCPP_INFO(this->get_logger(), "outgoing cmd_vel:%f ",cmd_vel_velocity_.linear.x);
        slowdown_initiated = true; 
    }
    void stop_robot(){
        cmd_vel_velocity_.linear.x *= 0.0;
        cmd_vel_velocity_.angular.z *= 0.0;
        
        // Publish the new velocity
        velocity_publisher_->publish(cmd_vel_velocity_);
        RCLCPP_INFO(this->get_logger(), "outgoing cmd_vel:%f ",cmd_vel_velocity_.linear.x);
       // stop_initiated=true;
    }

    void publishState(const std::string& state) {
        std_msgs::msg::String message;
       // message.data = state;
       // publisher_->publish(message);

       if(state=="red"){  //red="0x48"
        RCLCPP_INFO(this->get_logger(), "element detected in red zone");
       // stop_robot();
       }
       else if(state=="yellow"&& !slowdown_initiated){
       // slow_down_robot();  //yellow="0x10"
         RCLCPP_INFO(this->get_logger(), "element detected in warning zone");
       }
       // RCLCPP_INFO(this->get_logger(), "Published State: %s", state.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_left;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_right;
   // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::map<std::string, std::string> sensor_states; // Tracks the latest state reported by each sensor
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  //   rclcpp::Subscription<std::msg::Bool>::SharedPtr safety_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    geometry_msgs::msg::Twist current_velocity_;
    geometry_msgs::msg::Twist cmd_vel_velocity_;
     bool slowdown_initiated;
     bool stop_initiated;
    // bool isSafetyTurnOff;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}