#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

class GlobalMapBuilder : public rclcpp::Node {
public:
    GlobalMapBuilder() : Node("map_builder"), resolution_(0.1), map_size_x_(1000), map_size_y_(1000) {
        // Grid 초기화
        grid_.header.frame_id = "world";
        grid_.info.resolution = resolution_;
        grid_.info.width = map_size_x_;
        grid_.info.height = map_size_y_;
        grid_.info.origin.position.x = -map_size_x_ * resolution_ / 2;
        grid_.info.origin.position.y = -map_size_y_ * resolution_ / 2;
        grid_.info.origin.position.z = 0.0;
        grid_.data.assign(map_size_x_ * map_size_y_, -1);  // unknown

        // Publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Subscription
        point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/vins_estimator/keyframe_point", 10, [this](const sensor_msgs::msg::PointCloud::SharedPtr msg) {
                // updateMap(msg);
                pointCallBack(msg);
            });
        frame_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/keyframe_pose", 10 , [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                origin_x_ = msg->pose.pose.position.x;
                origin_y_ = msg->pose.pose.position.y;
                have_origin_ = true;
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/odometry", 50 , [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odomCallBack(msg);
            });
        
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { publishMap(); });

        RCLCPP_INFO(this->get_logger(), "Global Map Builder initialized: %dx%d (%.1fm)", 
                    map_size_x_, map_size_y_, map_size_x_ * resolution_);
    }

private:
    inline bool worldToGrid(double xw, double yw, int& ix, int& iy) const {
        ix = static_cast<int>((xw - grid_.info.origin.position.x) / resolution_);
        iy = static_cast<int>((yw - grid_.info.origin.position.y) / resolution_);

        return (ix >= 0 && ix < map_size_x_ && iy >= 0 && iy < map_size_y_);    
    }

    void pointCallBack(sensor_msgs::msg::PointCloud::SharedPtr msg) {
        if (msg->header.frame_id != "world") {
            RCLCPP_WARN(this->get_logger(),
                "Expected frame_id 'world' but got '%s'. Skip.", msg->header.frame_id.c_str());
            return;
        }

        if (!have_origin_) {
            RCLCPP_WARN(this->get_logger(), 
                "No origin pose yet. Waiting for /vins_estimator/keyframe_pose...");
            return;
        }

        int ox, oy;
        if (!worldToGrid(origin_x_, origin_y_, ox, oy)) {
            RCLCPP_WARN(this->get_logger(), "Origin outside map bounds.");
            return;
        }

        int updated_occ = 0;
        for (const auto& p : msg->points) {
            const double xw = p.x, yw = p.y;
            int ix, iy;
            if (!worldToGrid(xw, yw, ix, iy)) continue;

            // 1) 원점→히트셀까지 free(0) 레이 마킹
            markFreeLine(ox, oy, ix, iy);

            // 2) 히트셀은 occupied(100)로
            int hit_idx = iy * map_size_x_ + ix;
            if (grid_.data[hit_idx] != 100) {
                grid_.data[hit_idx] = 100;
                updated_occ++;
            }
        }

        if (updated_occ > 0) {
            RCLCPP_INFO(this->get_logger(), "Updated occupied: %d", updated_occ);
        }
    }

    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg->header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = msg->pose.pose.position.z;
        tf.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf);
    }

    void markFreeLine(int x0, int y0, int x1, int y1) {
        // Bresenham Algorithm
        int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;

        int x = x0, y = y0;
        while (true) {
            if (x < 0 || x >= map_size_x_ || y < 0 || y >= map_size_y_)
                break;
            int idx = y * map_size_x_ + x;
            if (grid_.data[idx] != 100) grid_.data[idx] = 0;
            if (x == x1 && y == y1) break; 
            int e2 = 2 * err;
            if (e2 >= dy) { 
                err += dy; x += sx; 
            }
            if (e2 <= dx) { 
                err += dx; y += sy; 
            }
        }
    }

    void publishMap() {
        grid_.header.stamp = this->now();
        map_pub_->publish(grid_);

        // nav_msgs::msg::OccupancyGrid grid__;
        // grid__.header.stamp = this->now();
        // grid__.header.frame_id = "world";
        // grid__.info.resolution = 0.1;
        // grid__.info.width = 1000;
        // grid__.info.height = 1000;
        // grid__.info.origin.position.x = -map_size_x_ * resolution_ / 2;
        // grid__.info.origin.position.y = -map_size_y_ * resolution_ / 2;
        // grid__.info.origin.position.z = 0.0;
        // grid__.data.assign(map_size_x_ * map_size_y_, 0);  
        // for (size_t i = 100000; i <= 100550; i++) {
        //     grid__.data[i] = 100;
        // }
    
        // map_pub_->publish(grid__);
    }

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frame_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid grid_;

    const double resolution_;
    const int map_size_x_, map_size_y_;

    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    bool have_origin_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMapBuilder>());
    rclcpp::shutdown();

    return 0;
}
