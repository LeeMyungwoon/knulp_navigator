#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstdio>

using ackermann_msgs::msg::AckermannDriveStamped;

class KeyboardAckermannNode : public rclcpp::Node {
public:
    KeyboardAckermannNode() : Node("keyboard_ackermann_node"),
      speed_(0.0),
      steering_angle_(0.0) {
        // Publisher
        pub_ = this->create_publisher<AckermannDriveStamped>("/ackermann_cmd", 10);

        // 터미널을 non-canonical 모드로 변경
        enableRawMode();

        RCLCPP_INFO(this->get_logger(),
                    "Keyboard Ackermann node started.\n"
                    "Controls:\n"
                    "  w : speed up\n"
                    "  s : slow down / reverse\n"
                    "  a : steer left\n"
                    "  d : steer right\n"
                    "  x : stop (speed=0, steering=0)\n"
                    "  q : quit");

        // 타이머: 주기적으로 키 입력 체크 + 메시지 퍼블리시
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&KeyboardAckermannNode::timerCallback, this));
    }

    ~KeyboardAckermannNode() override
    {
        disableRawMode();
    }

private:
    void enableRawMode()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO); // canonical 모드, echo 끄기
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void disableRawMode()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    // non-blocking 키 입력 체크
    bool kbhit()
    {
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        return ret > 0 && FD_ISSET(STDIN_FILENO, &fds);
    }

    char getch() {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) < 0)
            return 0;
        return c;
    }

    void timerCallback() {
        // 키가 눌려 있으면 처리
        if (kbhit())
        {
            char c = getch();
            handleKey(c);
        }

        // 현재 speed_, steering_angle_ 값으로 항상 publish
        AckermannDriveStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";  // 필요시 변경

        msg.drive.speed = speed_;
        msg.drive.steering_angle = steering_angle_;

        pub_->publish(msg);
    }

    void handleKey(char c) {
        const double speed_step = 0.2;        // m/s 증가량
        const double steering_step = 0.05;    // rad 증가량
        const double max_speed = 5.0;         // m/s
        const double max_steering = 0.5;      // rad

        switch (c) {
        case 'w':
        case 'W':
            speed_ += speed_step;
            if (speed_ > max_speed) speed_ = max_speed;
            RCLCPP_INFO(this->get_logger(), "speed up: %.2f", speed_);
            break;

        case 's':
        case 'S':
            speed_ -= speed_step;
            if (speed_ < -max_speed) speed_ = -max_speed;
            RCLCPP_INFO(this->get_logger(), "speed down: %.2f", speed_);
            break;

        case 'a':
        case 'A':
            steering_angle_ += steering_step;
            if (steering_angle_ > max_steering) steering_angle_ = max_steering;
            RCLCPP_INFO(this->get_logger(), "steer left: %.2f", steering_angle_);
            break;

        case 'd':
        case 'D':
            steering_angle_ -= steering_step;
            if (steering_angle_ < -max_steering) steering_angle_ = -max_steering;
            RCLCPP_INFO(this->get_logger(), "steer right: %.2f", steering_angle_);
            break;

        case 'x':
        case 'X':
            speed_ = 0.0;
            steering_angle_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "stop");
            break;

        case 'q':
        case 'Q':
            RCLCPP_INFO(this->get_logger(), "quit requested");
            rclcpp::shutdown();
            break;

        default:
            break;
        }
    }

    rclcpp::Publisher<AckermannDriveStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double speed_;
    double steering_angle_;

    termios orig_termios_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardAckermannNode>());
    rclcpp::shutdown();

    return 0;
}
