#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/StreamRate.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

#include "bridge_mavlink.hpp"
#include "std_msgs/String.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

BridgeMavlink::BridgeMavlink(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    // Read configuration parameters
    pnh.param<double>("publish_rate", publish_rate_, 20.0);
    pnh.param<bool>("use_degrees", use_degrees_, false);
    pnh.param<std::string>("odom_topic", odom_topic_,
                           "/mavros/local_position/odom");

    // Enforce valid rate
    if (publish_rate_ <= 0.0) {
        ROS_WARN(
            "Invalid publish_rate specified: %f. Resetting to default 20.0 Hz.",
            publish_rate_);
        publish_rate_ = 20.0;
    }

    // Subscribe directly to standard mavros state and telemetry topics
    sub_state_ =
        nh.subscribe("/mavros/state", 10, &BridgeMavlink::stateCallback, this);
    sub_local_odom_ =
        nh.subscribe(odom_topic_, 10, &BridgeMavlink::localOdomCallback, this);
    sub_global_gps_ = nh.subscribe("/mavros/global_position/global", 10,
                                   &BridgeMavlink::globalGpsCallback, this);
    sub_gps_raw_ = nh.subscribe("/mavros/gpsstatus/gps1/raw", 10,
                                &BridgeMavlink::gpsRawCallback, this);
    sub_rel_alt_ = nh.subscribe("/mavros/global_position/rel_alt", 10,
                                &BridgeMavlink::relAltCallback, this);

    // Advertise state output topic directly
    pub_state_ = nh.advertise<std_msgs::String>("/dank/status", 10);

    // Set up timer for fixed-rate publishing
    pub_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_),
                                &BridgeMavlink::publishTimerCallback, this);

    srv_get_gps_ =
        nh.advertiseService("get_gps", &BridgeMavlink::getGpsCallback, this);

    ROS_INFO(
        "BridgeMavlink initialized. Publishing on /dank/status at %.1f Hz.",
        publish_rate_);
}

void BridgeMavlink::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    bool connected = msg->connected;
    bool was_connected = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        was_connected = fcu_connected_;
        fcu_connected_ = connected;
        connected_ = connected;
        mode_ = msg->mode;
    }

    if (connected && !was_connected) {
        ROS_INFO(
            "FCU connected. Requesting stream rates and message intervals...");
        std::thread(&BridgeMavlink::setupMavrosStreams, this, publish_rate_)
            .detach();
    }
}

void BridgeMavlink::localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // pos_enu
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;

    // Convert orientation quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
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

    yaw_ = -yaw_ + M_PI * 0.5;  // ENU -> NED

    // Update body and global velocities
    x_vel_body_ = msg->twist.twist.linear.x;
    y_vel_body_ = msg->twist.twist.linear.y;

    tf2::Vector3 v_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    tf2::Vector3 v_global = tf2::quatRotate(q, v_body);
    x_vel_ = v_global.getX();
    y_vel_ = v_global.getY();

    // Push to datum synchronizer
    double time_now = msg->header.stamp.toSec();
    datum_sync_.pushENU(Eigen::Vector3d(pos_x_, pos_y_, pos_z_), time_now);

    auto opt_datum = datum_sync_.getReliableDatum();
    if (opt_datum.has_value()) {
        last_reliable_datum_ = opt_datum;
    }
}

void BridgeMavlink::globalGpsCallback(
    const sensor_msgs::NavSatFix::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // gps: [lon, lat, alt]
    lon_ = msg->longitude;
    lat_ = msg->latitude;
    alt_ = msg->altitude;

    // Push to datum synchronizer if GPS fix is good
    if (gps_fix_type_ >= 3) {
        double time_now = msg->header.stamp.toSec();
        datum_sync_.pushGPS(Eigen::Vector3d(lon_, lat_, alt_), time_now);

        auto opt_datum = datum_sync_.getReliableDatum();
        if (opt_datum.has_value()) {
            last_reliable_datum_ = opt_datum;
        }
    }
}

void BridgeMavlink::gpsRawCallback(const mavros_msgs::GPSRAW::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // gps_fix_type
    gps_fix_type_ = msg->fix_type;
    gps_nsats_ = msg->satellites_visible;
}

void BridgeMavlink::relAltCallback(const std_msgs::Float64::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rel_alt_ = msg->data;
}

bool BridgeMavlink::getGpsCallback(bridge_routes::StringSrv::Request &req,
                                   bridge_routes::StringSrv::Response &res) {
    double lon_out = 0.0;
    double lat_out = 0.0;
    double alt_out = 0.0;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        lon_out = lon_;
        lat_out = lat_;
        alt_out = alt_;

        if (gps_fix_type_ < 3 && last_reliable_datum_.has_value()) {
            const double R_e = 6378137.0;  // WGS-84 equatorial radius
            const double PI = 3.14159265358979323846;
            const auto &datum = last_reliable_datum_.value();

            double lat_ref_rad = datum.gps.y() * PI / 180.0;
            double delta_x = pos_x_ - datum.enu.x();
            double delta_y = pos_y_ - datum.enu.y();
            double delta_z = pos_z_ - datum.enu.z();

            lat_out = datum.gps.y() + (delta_y / R_e) * (180.0 / PI);
            lon_out = datum.gps.x() +
                      (delta_x / (R_e * std::cos(lat_ref_rad))) * (180.0 / PI);
            alt_out = datum.gps.z() + delta_z;
        }
    }

    nlohmann::json response_json;
    response_json["msg"] = {lon_out, lat_out, alt_out};
    response_json["status"] = "success";
    res.response = response_json.dump();

    return true;
}

void BridgeMavlink::publishTimerCallback(const ros::TimerEvent &event) {
    nlohmann::json state_mqtt;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_mqtt = getState();
    }

    std_msgs::String out_msg;
    out_msg.data = state_mqtt.dump();
    pub_state_.publish(out_msg);
}

nlohmann::json BridgeMavlink::getState() {
    double lon_out = lon_;
    double lat_out = lat_;
    double alt_out = alt_;

    if (gps_fix_type_ < 3 && last_reliable_datum_.has_value()) {
        const double R_e = 6378137.0;  // WGS-84 equatorial radius
        const double PI = 3.14159265358979323846;
        const auto &datum = last_reliable_datum_.value();

        double lat_ref_rad = datum.gps.y() * PI / 180.0;
        double delta_x = pos_x_ - datum.enu.x();
        double delta_y = pos_y_ - datum.enu.y();
        double delta_z = pos_z_ - datum.enu.z();

        lat_out = datum.gps.y() + (delta_y / R_e) * (180.0 / PI);
        lon_out = datum.gps.x() +
                  (delta_x / (R_e * std::cos(lat_ref_rad))) * (180.0 / PI);
        alt_out = datum.gps.z() + delta_z;
    }

    nlohmann::json j;
    j["pos_enu"] = {pos_x_, pos_y_, pos_z_};
    j["roll"] = roll_;
    j["pitch"] = pitch_;
    j["yaw"] = yaw_;
    j["gps"] = {lon_out, lat_out, alt_out};
    j["gps_fix_type"] = gps_fix_type_;
    j["x_vel"] = x_vel_;
    j["y_vel"] = y_vel_;
    j["x_vel_body"] = x_vel_body_;
    j["y_vel_body"] = y_vel_body_;
    j["connected"] = connected_;  // FCU
    j["rel_alt"] = rel_alt_;
    j["gps_nsats"] = gps_nsats_;
    j["mode"] = mode_;  // FCU
    return j;
}

void BridgeMavlink::setupMavrosStreams(double rate) {
    ros::NodeHandle nh;

    // Wait for the MAVROS services to become available (e.g. timeout after 30
    // seconds)
    ROS_INFO("setupMavrosStreams: Waiting for MAVROS services to start...");

    if (ros::service::waitForService("/mavros/set_stream_rate",
                                     ros::Duration(30.0))) {
        ros::ServiceClient client = nh.serviceClient<mavros_msgs::StreamRate>(
            "/mavros/set_stream_rate");
        mavros_msgs::StreamRate srv;
        srv.request.stream_id = 0;  // STREAM_ALL
        srv.request.message_rate = static_cast<uint16_t>(rate);
        srv.request.on_off = true;
        if (client.call(srv)) {
            ROS_INFO(
                "Successfully requested all MAVROS streams at %.1f Hz via "
                "set_stream_rate.",
                rate);
        } else {
            ROS_WARN("Failed to call MAVROS service /mavros/set_stream_rate.");
        }
    } else {
        ROS_WARN(
            "MAVROS service /mavros/set_stream_rate not available (timeout).");
    }

    if (ros::service::waitForService("/mavros/set_message_interval",
                                     ros::Duration(10.0))) {
        ros::ServiceClient client =
            nh.serviceClient<mavros_msgs::MessageInterval>(
                "/mavros/set_message_interval");

        auto set_interval = [&](uint32_t msg_id, float msg_rate) {
            mavros_msgs::MessageInterval srv;
            srv.request.message_id = msg_id;
            srv.request.message_rate = msg_rate;
            if (client.call(srv) && srv.response.success) {
                ROS_INFO(
                    "Successfully set message interval for message ID %u to "
                    "%.1f Hz.",
                    msg_id, msg_rate);
            } else {
                ROS_WARN("Failed to set message interval for message ID %u.",
                         msg_id);
            }
        };

        // Call the service for attitude, position, and rangefinder as requested
        set_interval(30, static_cast<float>(
                             rate));  // MAVLINK_MSG_ID_ATTITUDE (attitude)
        set_interval(
            32, static_cast<float>(
                    rate));  // MAVLINK_MSG_ID_LOCAL_POSITION_NED (position)
        set_interval(
            132,
            static_cast<float>(
                rate));  // MAVLINK_MSG_ID_DISTANCE_SENSOR (rangefinder)
    } else {
        ROS_WARN(
            "MAVROS service /mavros/set_message_interval not available "
            "(timeout).");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bridge_mavlink_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    BridgeMavlink bridge(nh, pnh);

    // Multi-threaded spinner to handle callbacks concurrently with safety locks
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
