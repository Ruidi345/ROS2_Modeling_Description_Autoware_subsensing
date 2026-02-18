#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.h> // Required for NavSatStatus constants
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> // Now subscribing to this
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp> // Required for Doppler velocity
#include "llh_converter/llh_converter.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// For debugging output (optional)
#include <iostream>
#include <iomanip>
#include <cmath> // For M_PI
#include <array> // For std::array

// Include the header that provides llh2xyz
// Assuming these functions are provided by eagleye_coordinate package as per your reference
#include "eagleye_coordinate/eagleye_coordinate.hpp" 

class GnssMgrsConverterNode : public rclcpp::Node
{
public:
  GnssMgrsConverterNode() : Node("gnss_mgrs_node")
  {
    // 1. Dynamically get the path to the geoid data file
    std::string geoid_file_path;
    try {
      geoid_file_path = ament_index_cpp::get_package_share_directory("llh_converter") + "/data/gsigeo2011_ver2_1.asc";
      RCLCPP_INFO(this->get_logger(), "Geoid data file path: %s", geoid_file_path.c_str());
    } catch (const std::runtime_error & e) {
      RCLCPP_FATAL(this->get_logger(), "Package 'llh_converter' not found: %s", e.what());
      rclcpp::shutdown(); // Cannot proceed without the package
      return;
    }

    // 2. Initialize LLHConverter with the geoid file path
    llh_converter_ = std::make_unique<llh_converter::LLHConverter>(geoid_file_path);

    // 3. Set up LLHParam for MGRS conversion
    param_.use_mgrs = true;
    this->declare_parameter("mgrs_grid_code", "54SUE");
    param_.mgrs_code = this->get_parameter("mgrs_grid_code").as_string();
    param_.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
    param_.geoid_type = llh_converter::GeoidType::EGM2008;

    // Declare and get parameters for NavSatFix message
    this->declare_parameter("nav_sat_fix_frame_id", "gnss_link");
    this->declare_parameter("nav_sat_fix_status", static_cast<int>(sensor_msgs::msg::NavSatStatus::STATUS_FIX));
    this->declare_parameter("nav_sat_fix_service", static_cast<int>(
      sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO));
    this->declare_parameter("nav_sat_fix_covariance_type", static_cast<int>(sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN));
    this->declare_parameter("output_doppler_velocity_topic", "/sensing/gnss/simulated_gnss_doppler_velocity");

    nav_sat_fix_frame_id_ = this->get_parameter("nav_sat_fix_frame_id").as_string();
    nav_sat_fix_status_ = this->get_parameter("nav_sat_fix_status").as_int();
    nav_sat_fix_service_ = this->get_parameter("nav_sat_fix_service").as_int();
    nav_sat_fix_covariance_type_ = this->get_parameter("nav_sat_fix_covariance_type").as_int();
    output_doppler_velocity_topic_ = this->get_parameter("output_doppler_velocity_topic").as_string();

    // 4. Create Subscriber for PoseWithCovarianceStamped
    subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/sensing/gnss/pose_with_covariance",
      rclcpp::SensorDataQoS(),
      std::bind(&GnssMgrsConverterNode::poseCallback, this, std::placeholders::_1));

    // 5. Create Publishers
    publisher_navsatfix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/sensing/gnss/nav_sat_fix",
      rclcpp::SensorDataQoS().reliable());

    doppler_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      output_doppler_velocity_topic_, rclcpp::SensorDataQoS().reliable());

    RCLCPP_INFO(this->get_logger(), "GNSS MGRS to LLH Converter Node Initialized.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /sensing/gnss/pose_with_covariance");
    RCLCPP_INFO(this->get_logger(), "Publishing NavSatFix to: /sensing/gnss/nav_sat_fix");
    RCLCPP_INFO(this->get_logger(), "Publishing Doppler Velocity to: %s", output_doppler_velocity_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "MGRS Grid Code for conversion: %s", param_.mgrs_code.c_str());
  }

private:
  // Member variables to store previous ECEF position and timestamp for velocity calculation
  std::array<double, 3> prev_ecef_pos_ = {0.0, 0.0, 0.0};
  rclcpp::Time prev_timestamp_;
  bool first_data_received_ = false;

  // New callback function for PoseWithCovarianceStamped messages
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // Extract MGRS coordinates and ellipsoidal altitude from the PoseWithCovarianceStamped message.
    double mgrs_x = msg->pose.pose.position.x;
    double mgrs_y = msg->pose.pose.position.y;
    double ellipsoidal_altitude_z = msg->pose.pose.position.z;

    double lat_deg, lon_deg;
    // Convert MGRS (x,y) to Latitude and Longitude (degrees).
    llh_converter_->revertXYZ2Deg(mgrs_x, mgrs_y, lat_deg, lon_deg, param_);

    // --- NavSatFix Publishing ---
    // Populate and publish the NavSatFix message.
    sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
    nav_sat_fix_msg.header.stamp = msg->header.stamp;
    nav_sat_fix_msg.header.frame_id = nav_sat_fix_frame_id_;
    nav_sat_fix_msg.latitude = lat_deg;
    nav_sat_fix_msg.longitude = lon_deg;
    nav_sat_fix_msg.altitude = ellipsoidal_altitude_z; // This is the ellipsoidal height
    nav_sat_fix_msg.status.status = nav_sat_fix_status_;
    nav_sat_fix_msg.status.service = nav_sat_fix_service_;

    // Set position covariance for NavSatFix message
    // Initialize all elements to 0.0
    std::fill(nav_sat_fix_msg.position_covariance.begin(), nav_sat_fix_msg.position_covariance.end(), 0.0);
    // Set diagonal elements to 0.0025
    nav_sat_fix_msg.position_covariance[0] = 0.0025; // x-x covariance
    nav_sat_fix_msg.position_covariance[4] = 0.0025; // y-y covariance
    nav_sat_fix_msg.position_covariance[8] = 0.0025; // z-z covariance
    
    nav_sat_fix_msg.position_covariance_type = nav_sat_fix_covariance_type_;

    publisher_navsatfix_->publish(nav_sat_fix_msg);

    // --- Doppler Velocity Publishing ---
    geometry_msgs::msg::TwistWithCovarianceStamped doppler_msg;
    doppler_msg.header.stamp = msg->header.stamp;
    doppler_msg.header.frame_id = "ecef"; // The output Doppler velocity will be in the ECEF frame.

    // Prepare LLH array for llh2xyz function.
    // Latitude and Longitude must be in radians, Altitude in meters.
    double llh_rad[3] = {
      lat_deg * M_PI / 180.0, // Latitude in radians
      lon_deg * M_PI / 180.0, // Longitude in radians
      ellipsoidal_altitude_z  // Ellipsoidal height in meters
    };
    std::array<double, 3> current_ecef_pos; // Current ECEF position of the vehicle

    // --- Use llh2xyz for LLH to ECEF conversion ---
    llh2xyz(llh_rad, current_ecef_pos.data());

    rclcpp::Time current_timestamp = msg->header.stamp;

    // Calculate linear velocity using displacement / time difference
    if (first_data_received_) {
      double dt = (current_timestamp - prev_timestamp_).seconds();

      if (dt > 0.0) {
        // Calculate displacement in ECEF
        double delta_x = current_ecef_pos[0] - prev_ecef_pos_[0];
        double delta_y = current_ecef_pos[1] - prev_ecef_pos_[1];
        double delta_z = current_ecef_pos[2] - prev_ecef_pos_[2];

        // Calculate velocity (displacement / time)
        doppler_msg.twist.twist.linear.x = delta_x / dt;
        doppler_msg.twist.twist.linear.y = delta_y / dt;
        doppler_msg.twist.twist.linear.z = delta_z / dt;
      } else {
        // If dt is zero or negative (e.g., duplicate timestamp), set velocity to zero
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Time difference for linear velocity calculation is zero or negative. Setting linear velocity to zero.");
        doppler_msg.twist.twist.linear.x = 0.0;
        doppler_msg.twist.twist.linear.y = 0.0;
        doppler_msg.twist.twist.linear.z = 0.0;
      }
    } else {
      // First data point, cannot calculate velocity yet. Set to zero.
      RCLCPP_INFO(this->get_logger(), "First pose received. Initializing linear velocity to zero.");
      doppler_msg.twist.twist.linear.x = 0.0;
      doppler_msg.twist.twist.linear.y = 0.0;
      doppler_msg.twist.twist.linear.z = 0.0;
    }

    // --- Angular Velocity Publishing (set to constant zero) ---
    // As per your request, angular velocity is set to a constant zero.
    doppler_msg.twist.twist.angular.x = 0.0;
    doppler_msg.twist.twist.angular.y = 0.0;
    doppler_msg.twist.twist.angular.z = 0.0;

    // Set covariance for the Doppler velocity message.
    // Initialize all elements to 0.0
    std::fill(doppler_msg.twist.covariance.begin(), doppler_msg.twist.covariance.end(), 0.0);
    // Set diagonal elements to 0.0025 for linear (x,y,z) and angular (roll,pitch,yaw)
    doppler_msg.twist.covariance[0] = 0.0025;  // linear x-x
    doppler_msg.twist.covariance[7] = 0.0025;  // linear y-y
    doppler_msg.twist.covariance[14] = 0.0025; // linear z-z
    doppler_msg.twist.covariance[21] = 0.0025; // angular x-x (roll)
    doppler_msg.twist.covariance[28] = 0.0025; // angular y-y (pitch)
    doppler_msg.twist.covariance[35] = 0.0025; // angular z-z (yaw)

    doppler_velocity_publisher_->publish(doppler_msg);

    // RCLCPP_INFO(this->get_logger(),
    //            "Doppler Vel ECEF: Linear(%.2f, %.2f, %.2f), Angular(%.2f, %.2f, %.2f)",
    //            doppler_msg.twist.twist.linear.x, doppler_msg.twist.twist.linear.y, doppler_msg.twist.twist.linear.z,
    //            doppler_msg.twist.twist.angular.x, doppler_msg.twist.twist.angular.y, doppler_msg.twist.twist.angular.z);
    
    // Store current position and timestamp for the next iteration
    prev_ecef_pos_ = current_ecef_pos;
    prev_timestamp_ = current_timestamp;
    first_data_received_ = true;
  }

  // Private member variables
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_navsatfix_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr doppler_velocity_publisher_;
  std::unique_ptr<llh_converter::LLHConverter> llh_converter_;
  llh_converter::LLHParam param_;

  // Parameters for NavSatFix and Doppler velocity topics/settings
  std::string nav_sat_fix_frame_id_;
  int nav_sat_fix_status_;
  int nav_sat_fix_service_;
  int nav_sat_fix_covariance_type_;
  std::string output_doppler_velocity_topic_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssMgrsConverterNode>());
  rclcpp::shutdown();
  return 0;
}
