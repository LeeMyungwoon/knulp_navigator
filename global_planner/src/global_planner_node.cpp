#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>    
#include <tf2/LinearMath/Quaternion.h> 

#include "global_planner.hpp"

class globalPlanner : public rclcpp::Node {
public:
    globalPlanner() : Node("global_planner") {
        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);

        // Subscriber
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                mapCallBack(msg);
            });
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                goalCallBack(msg);
            });
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/odometry", 50 , [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odomCallBack(msg);
            });

        RCLCPP_INFO(this->get_logger(), "global_planner node started.");
    }   
private:
    void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        const auto &info = msg->info;
        
        std::lock_guard<std::mutex> lk(map_mtx_);

        const bool need_new = 
            !gm_ ||
            gm_->width != static_cast<int>(info.width) ||
            gm_->height != static_cast<int>(info.height) ||
            gm_->resolution != info.resolution ||
            gm_->origin_x != info.origin.position.x ||
            gm_->origin_y != info.origin.position.y;
         
        if (need_new) {
            gm_ = std::make_shared<GridMap>(
                static_cast<int>(info.width),
                static_cast<int>(info.height),
                info.resolution,
                info.origin.position.x,
                info.origin.position.y,
                -1
            );
            gm_->unknown_is_free = false;
            map_frame_ = msg->header.frame_id;

            RCLCPP_INFO(this->get_logger(), "Map NEW: %dx%d res=%.3f frame='%s'", gm_->width, gm_->height, gm_->resolution, map_frame_.c_str());
        }

        // Only Data change
        gm_->data.assign(msg->data.begin(), msg->data.end());
    }

    void goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (!gm_) {
            RCLCPP_WARN(this->get_logger(), "Map not ready");
            return;
        }
        if (!have_odom_) {
            RCLCPP_WARN(this->get_logger(), "Odometry not ready");
            return;
        }
        gwx_ = msg->point.x;
        gwy_ = msg->point.y;
        Pose start_pose(swx_, swy_, yaw_);
        Pose goal_pose(gwx_, gwy_, yaw_);

        std::shared_ptr<GridMap> snap; {
            std::lock_guard<std::mutex> lk(map_mtx_);
            snap = std::make_shared<GridMap>(*gm_); 
        }

        HybridAStarRoutePlanner planner(*snap);
        auto result = planner.search_route(start_pose, goal_pose);
        if (result.x.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "Path not found");
            return;
        }

        // make path message
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = !map_frame_.empty() ? map_frame_ : "map";

        path.poses.reserve(result.x.size());
        for (size_t i = 0; i < result.x.size(); i++) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = result.x[i];
            ps.pose.position.y = result.y[i];
            ps.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, result.theta[i]);
            ps.pose.orientation = tf2::toMsg(q);
            path.poses.push_back(std::move(ps));
        }
        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu poses.", path.poses.size());
    }

    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
        swx_ = msg->pose.pose.position.x;
        swy_ = msg->pose.pose.position.y;
        yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        have_odom_ = true;
    }

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Subcription
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;

    std::shared_ptr<GridMap> gm_;
    std::mutex map_mtx_;

    double swy_ = 0.0, swx_ = 0.0, gwy_ = 0.0, gwx_ = 0.0;
    double yaw_ = 0.0;
    bool have_odom_ = false;
    std::string map_frame_ = "map";
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<globalPlanner>());
    rclcpp::shutdown();    

    return 0;
}
