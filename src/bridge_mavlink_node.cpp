#include "bridge_mavlink.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/StreamRate.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

BridgeMavlink::BridgeMavlink(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // Read configuration parameters
    pnh.param<double>("publish_rate", publish_rate_, 20.0);
    pnh.param<bool>("use_degrees", use_degrees_, false);

    // Enforce valid rate
    if (publish_rate_ <= 0.0) {
        ROS_WARN("Invalid publish_rate specified: %f. Resetting to default 20.0 Hz.", publish_rate_);
        publish_rate_ = 20.0;
    }

    // Subscribe directly to standard mavros state and telemetry topics
    sub_state_ = nh.subscribe("/mavros/state", 10, &BridgeMavlink::stateCallback, this);
    sub_local_odom_ = nh.subscribe("/mavros/local_position/odom", 10, &BridgeMavlink::localOdomCallback, this);
    sub_global_gps_ = nh.subscribe("/mavros/global_position/global", 10, &BridgeMavlink::globalGpsCallback, this);
    sub_gps_raw_ = nh.subscribe("/mavros/gpsstatus/gps1/raw", 10, &BridgeMavlink::gpsRawCallback, this);

    // Advertise state output topic directly
    pub_state_ = nh.advertise<std_msgs::String>("/dank/state", 10);

    // Set up timer for fixed-rate publishing
    pub_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), &BridgeMavlink::publishTimerCallback, this);

    ROS_INFO("BridgeMavlink initialized. Publishing on /dank/state at %.1f Hz.", publish_rate_);
}

void BridgeMavlink::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    bool connected = msg->connected;
    bool was_connected = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        was_connected = fcu_connected_;
        fcu_connected_ = connected;
    }
    
    if (connected && !was_connected) {
        ROS_INFO("FCU connected. Requesting stream rates and message intervals...");
        std::thread(&BridgeMavlink::setupMavrosStreams, this, publish_rate_).detach();
    }
}

void BridgeMavlink::localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // pos_enu
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;

    // Convert orientation quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    if (use_degrees_) {
        roll_ = r * 180.0 / M_PI;
        pitch_ = p * 180.0 / M_PI;
        yaw_ = y * 180.0 / M_PI;
    } else {
        roll_ = r;
        pitch_ = p;
        yaw_ = y;
    }
}


void BridgeMavlink::globalGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // gps: [lon, lat, alt]
    lon_ = msg->longitude;
    lat_ = msg->latitude;
    alt_ = msg->altitude;
}

void BridgeMavlink::gpsRawCallback(const mavros_msgs::GPSRAW::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // gps_fix_type
    gps_fix_type_ = msg->fix_type;
}

void BridgeMavlink::publishTimerCallback(const ros::TimerEvent& event) {
    std_msgs::String out_msg;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        out_msg.data = formatJsonString();
    }
    pub_state_.publish(out_msg);
}

std::string BridgeMavlink::formatJsonString() {
    std::ostringstream ss;
    // Set precision to capture coordinates accurately (double has ~15 decimal digits)
    ss << std::fixed << std::setprecision(9);
    ss << "{\n"
       << "  \"pos_enu\": [" << pos_x_ << ", " << pos_y_ << ", " << pos_z_ << "],\n"
       << "  \"roll\": " << roll_ << ",\n"
       << "  \"pitch\": " << pitch_ << ",\n"
       << "  \"yaw\": " << yaw_ << ",\n"
       << "  \"gps\": [" << lon_ << ", " << lat_ << ", " << alt_ << "],\n"
       << "  \"gps_fix_type\": " << gps_fix_type_ << "\n"
       << "}";
    return ss.str();
}

void BridgeMavlink::setupMavrosStreams(double rate) {
    ros::NodeHandle nh;
    
    // Wait for the MAVROS services to become available (e.g. timeout after 30 seconds)
    ROS_INFO("setupMavrosStreams: Waiting for MAVROS services to start...");
    
    if (ros::service::waitForService("/mavros/set_stream_rate", ros::Duration(30.0))) {
        ros::ServiceClient client = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
        mavros_msgs::StreamRate srv;
        srv.request.stream_id = 0; // STREAM_ALL
        srv.request.message_rate = static_cast<uint16_t>(rate);
        srv.request.on_off = true;
        if (client.call(srv)) {
            ROS_INFO("Successfully requested all MAVROS streams at %.1f Hz via set_stream_rate.", rate);
        } else {
            ROS_WARN("Failed to call MAVROS service /mavros/set_stream_rate.");
        }
    } else {
        ROS_WARN("MAVROS service /mavros/set_stream_rate not available (timeout).");
    }

    if (ros::service::waitForService("/mavros/set_message_interval", ros::Duration(10.0))) {
        ros::ServiceClient client = nh.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");
        
        auto set_interval = [&](uint32_t msg_id, float msg_rate) {
            mavros_msgs::MessageInterval srv;
            srv.request.message_id = msg_id;
            srv.request.message_rate = msg_rate;
            if (client.call(srv) && srv.response.success) {
                ROS_INFO("Successfully set message interval for message ID %u to %.1f Hz.", msg_id, msg_rate);
            } else {
                ROS_WARN("Failed to set message interval for message ID %u.", msg_id);
            }
        };

        // Call the service for attitude, position, and rangefinder as requested
        set_interval(30, static_cast<float>(rate));  // MAVLINK_MSG_ID_ATTITUDE (attitude)
        set_interval(32, static_cast<float>(rate));  // MAVLINK_MSG_ID_LOCAL_POSITION_NED (position)
        set_interval(132, static_cast<float>(rate)); // MAVLINK_MSG_ID_DISTANCE_SENSOR (rangefinder)
    } else {
        ROS_WARN("MAVROS service /mavros/set_message_interval not available (timeout).");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bridge_mavlink_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    BridgeMavlink bridge(nh, pnh);

    // Multi-threaded spinner to handle callbacks concurrently with safety locks
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
