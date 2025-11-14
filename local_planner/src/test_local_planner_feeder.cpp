#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class TestLocalPlannerFeeder : public rclcpp::Node {
public:
    TestLocalPlannerFeeder()
    : Node("test_local_planner_feeder"),
      t_(0.0)
    {
        map_pub_  = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 10);

        // 50 ms (20 Hz) 타이머
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TestLocalPlannerFeeder::onTimer, this)
        );

        RCLCPP_INFO(this->get_logger(), "test_local_planner_feeder started.");
    }

private:
    void onTimer()
    {
        // 한 번만 맵과 전역 경로 publish
        if (!map_sent_) {
            publishDummyMap();
            publishDummyPath();
            map_sent_ = true;
        }

        publishDummyOdom();
    }

    static geometry_msgs::msg::Quaternion makeQuatFromYaw(double yaw)
    {
        // 간단한 z축 회전 쿼터니언
        geometry_msgs::msg::Quaternion q;
        double half = 0.5 * yaw;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(half);
        q.w = std::cos(half);
        return q;
    }

    void publishDummyMap()
    {
        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = this->now();
        map.header.frame_id = "map";

        map.info.resolution = 0.2;        // 20cm grid
        map.info.width  = 100;            // 20m x 20m
        map.info.height = 100;
        map.info.origin.position.x = -10.0;
        map.info.origin.position.y = -10.0;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation = makeQuatFromYaw(0.0);

        map.data.assign(map.info.width * map.info.height, 0);  // 전부 free

        map_pub_->publish(map);
        RCLCPP_INFO(this->get_logger(), "Published dummy /map");
    }

    void publishDummyPath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        // x 방향으로 0 ~ 10m 직선 경로 (0.5m 간격)
        const double step = 0.5;
        for (double x = 0.0; x <= 10.0 + 1e-6; x += step) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = x;
            ps.pose.position.y = 0.0;
            ps.pose.position.z = 0.0;

            ps.pose.orientation = makeQuatFromYaw(0.0); // x축 방향

            path.poses.push_back(ps);
        }

        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published dummy /global_path with %zu poses.", path.poses.size());
    }

    void publishDummyOdom()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "map";
        odom.child_frame_id  = "base_link";

        // t_를 이용해 0.5 m/s로 x 방향 이동
        double vx = 0.5;                 // 0.5 m/s
        double x  = vx * t_;
        double y  = 0.0;
        double yaw = 0.0;

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = makeQuatFromYaw(yaw);

        odom.twist.twist.linear.x  = vx;
        odom.twist.twist.angular.z = 0.0;

        odom_pub_->publish(odom);

        t_ += 0.05;   // 50ms 타이머 기준

        // 너무 멀리 가면 다시 0으로 리셋
        if (x > 12.0) {
            t_ = 0.0;
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool map_sent_ = false;
    double t_;  // "시간" 비슷하게 쓰는 변수
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestLocalPlannerFeeder>());
    rclcpp::shutdown();
    return 0;
}
