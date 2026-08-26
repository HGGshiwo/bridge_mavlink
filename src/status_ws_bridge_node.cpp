#include <ros/ros.h>
#include <std_msgs/String.h>

#include <exception>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

// Simple map to map JSON keys for websocket publishing
const std::map<std::string, std::string> key_map{
    {"gps", "gpsLocation"},     {"x_vel", "xVel"},
    {"y_vel", "yVel"},          {"x_vel_body", "xVelBody"},
    {"y_vel_body", "yVelBody"}, {"rel_alt", "relAlt"},
    {"gps_nsats", "gpsNsats"},  {"pos_enu", "mapLocation"}};

class StatusWsBridge {
   public:
    StatusWsBridge(ros::NodeHandle &nh) {
        sub_status_ = nh.subscribe("/dank/status", 10,
                                   &StatusWsBridge::statusCallback, this);
        pub_status_ws_ = nh.advertise<std_msgs::String>("/dank/status_ws", 10);
        ROS_INFO(
            "StatusWsBridge node initialized. Subscribed to /dank/status, "
            "publishing to /dank/status_ws.");
    }

   private:
    void statusCallback(const std_msgs::String::ConstPtr &msg) {
        try {
            auto j = nlohmann::json::parse(msg->data);
            nlohmann::json out = nlohmann::json::object();

            for (auto &el : j.items()) {
                auto it = key_map.find(el.key());
                if (it != key_map.end()) {
                    out[it->second] = el.value();
                } else {
                    out[el.key()] = el.value();
                }
            }

            std_msgs::String out_msg;
            out_msg.data = out.dump();
            pub_status_ws_.publish(out_msg);
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM("StatusWsBridge: Failed to parse or convert JSON: "
                             << e.what());
        }
    }

    ros::Subscriber sub_status_;
    ros::Publisher pub_status_ws_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "status_ws_bridge_node");
    ros::NodeHandle nh;
    StatusWsBridge bridge(nh);
    ros::spin();
    return 0;
}
