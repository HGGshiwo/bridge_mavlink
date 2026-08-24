#pragma once

#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

#include "datum_synchronizer.hpp"

class BridgeMavlink {
   public:
    BridgeMavlink(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~BridgeMavlink() = default;

   private:
    // Subscriber callbacks
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void globalGpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void gpsRawCallback(const mavros_msgs::GPSRAW::ConstPtr &msg);

    // Timer callback for publishing
    void publishTimerCallback(const ros::TimerEvent &event);

    // 或者当前状态
    nlohmann::json getState();

    // 转为云端格式
    nlohmann::json convertKey(nlohmann::json);

    // Helper to request MAVROS streams and message intervals asynchronously
    void setupMavrosStreams(double rate);

    // ROS interfaces
    ros::Subscriber sub_state_;
    ros::Subscriber sub_local_odom_;
    ros::Subscriber sub_global_gps_;
    ros::Subscriber sub_gps_raw_;
    ros::Publisher pub_state_;
    ros::Publisher pub_state_ws_;
    ros::Timer pub_timer_;

    // Mutex for thread safety (callbacks might run concurrently in
    // multi-threaded spinners)
    std::mutex data_mutex_;

    // Telemetry storage
    double pos_x_{0.0};
    double pos_y_{0.0};
    double pos_z_{0.0};
    double roll_{0.0};
    double pitch_{0.0};
    double yaw_{0.0};
    double lon_{0.0};
    double lat_{0.0};
    double alt_{0.0};
    int gps_fix_type_{0};  // Default to 0 (No GPS)

    // Configuration flags
    bool use_degrees_{false};
    double publish_rate_{20.0};
    bool fcu_connected_{false};
    std::string odom_topic_{"/mavros/local_position/odom"};

    // Datum synchronization and estimation
    DatumSynchronizer datum_sync_;
    std::optional<SyncedPair> last_reliable_datum_;
};