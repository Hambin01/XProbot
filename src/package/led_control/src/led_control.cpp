#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <algorithm>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <asm/ioctls.h>

#ifndef B250000
#define B250000 0010001
#endif

enum LightMode {
    MODE_OFF = 0,
    MODE_BLUE_FLOW = 1,
    MODE_RED_FLASH_FAST = 2,
    MODE_YELLOW_FLASH_SLOW = 3,
    MODE_WHITE_STEADY = 4
};

class DMX512LightController {
public:
    DMX512LightController() : 
        dmx_channels_(512, 0), 
        serial_fd_(-1), 
        baudrate_(250000),
        current_mode_(MODE_OFF), 
        led_count_(60), 
        animation_step_(0),
        reconnect_interval_(2.0),
        last_reconnect_time_(0.0) {

        ros::NodeHandle nh("~");
        nh.param<std::string>("serial_port", serial_port_name_, "/dev/ttyUSB0");
        nh.param<int>("send_rate", send_rate_, 30);
        nh.param<int>("animation_rate", animation_rate_, 30);
        nh.param<double>("reconnect_interval", reconnect_interval_, 2.0);

        initSerial();

        mode_sub_ = nh.subscribe("/dmx512/set_mode", 10, &DMX512LightController::modeCallback, this);
        light_cmd_sub_ = nh.subscribe("/dmx512/light_command", 10, &DMX512LightController::lightCmdCallback, this);
        
        send_timer_ = nh.createTimer(ros::Duration(1.0/send_rate_), &DMX512LightController::sendDMXData, this);
        animation_timer_ = nh.createTimer(ros::Duration(1.0/animation_rate_), &DMX512LightController::updateAnimation, this);
        reconnect_timer_ = nh.createTimer(ros::Duration(0.5), &DMX512LightController::checkAndReconnect, this);

        ROS_INFO("=== DMX512 Light Controller Started ===");
        ROS_INFO("Serial: %s | Send Rate: %dHz | Animation Rate: %dHz", serial_port_name_.c_str(), send_rate_, animation_rate_);
        ROS_INFO("Supported commands: off, blue_flow, red_flash_fast, yellow_flash_slow, white_steady");
    }

    ~DMX512LightController() {
        if(serial_fd_ >= 0) {
            close(serial_fd_);
            ROS_INFO("Serial port closed");
        }
    }

    bool initSerial() {
        if(serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }

        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if(serial_fd_ < 0) {
            ROS_WARN("Open serial %s failed: %s", serial_port_name_.c_str(), strerror(errno));
            return false;
        }

        struct termios options;
        if(tcgetattr(serial_fd_, &options) != 0) {
            ROS_WARN("Get serial attr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        cfsetispeed(&options, B250000);
        cfsetospeed(&options, B250000);

        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8 | CMSPAR | CREAD | CLOCAL;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        if(tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            ROS_WARN("Set serial attr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        ROS_INFO("Serial %s initialized", serial_port_name_.c_str());
        return true;
    }

    bool sendBulkDMXData() {
        if(serial_fd_ < 0) return false;

        struct termios options;
        if(tcgetattr(serial_fd_, &options) != 0) return false;

        try {
            options.c_cflag &= ~PARODD;
            tcsetattr(serial_fd_, TCSANOW, &options);
            uint8_t start_code = 0x00;
            write(serial_fd_, &start_code, 1);

            options.c_cflag |= PARODD;
            tcsetattr(serial_fd_, TCSANOW, &options);
            ssize_t ret = write(serial_fd_, dmx_channels_.data(), dmx_channels_.size());
            
            if(ret != static_cast<ssize_t>(dmx_channels_.size())) {
                ROS_WARN_THROTTLE(5, "Bulk send failed: sent %zd, expected 512", ret);
                return false;
            }

            tcdrain(serial_fd_);
            return true;
        } catch(...) {
            return false;
        }
    }

    void checkAndReconnect(const ros::TimerEvent& event) {
        if(serial_fd_ >= 0) {
            struct termios test_attr;
            if(tcgetattr(serial_fd_, &test_attr) != 0) {
                ROS_WARN("Serial error, prepare reconnect");
                close(serial_fd_);
                serial_fd_ = -1;
            } else {
                return;
            }
        }

        double now = ros::Time::now().toSec();
        if(now - last_reconnect_time_ < reconnect_interval_) return;

        ROS_INFO("Reconnecting serial %s", serial_port_name_.c_str());
        last_reconnect_time_ = now;
        if(initSerial()) {
            ROS_INFO("Reconnected, restore mode: %s", getModeName(current_mode_).c_str());
        }
    }

    void modeCallback(const std_msgs::String::ConstPtr& msg) {
        std::string cmd = msg->data;
        cmd.erase(remove_if(cmd.begin(), cmd.end(), isspace), cmd.end());
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        bool cmd_valid = true;
        if(cmd == "off") current_mode_ = MODE_OFF;
        else if(cmd == "blue_flow") current_mode_ = MODE_BLUE_FLOW;
        else if(cmd == "red_flash_fast") current_mode_ = MODE_RED_FLASH_FAST;
        else if(cmd == "yellow_flash_slow") current_mode_ = MODE_YELLOW_FLASH_SLOW;
        else if(cmd == "white_steady") current_mode_ = MODE_WHITE_STEADY;
        else cmd_valid = false;

        if(cmd_valid) {
            animation_step_ = 0;
            ROS_INFO("Mode changed: %s -> %s", cmd.c_str(), getModeName(current_mode_).c_str());
        } else {
            ROS_WARN("Invalid command: %s | Supported: off, blue_flow, red_flash_fast, yellow_flash_slow, white_steady", msg->data.c_str());
        }
    }

    void lightCmdCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
        current_mode_ = MODE_OFF;
        for(size_t i = 0; i < msg->data.size(); i += 2) {
            if(i+1 >= msg->data.size()) break;
            uint8_t channel = msg->data[i];
            uint8_t value = msg->data[i+1];
            if(channel >= 1 && channel <= 512) {
                dmx_channels_[channel-1] = value;
                ROS_DEBUG("Set channel %d to %d", channel, value);
            } else {
                ROS_WARN("Invalid channel: %d (1-512)", channel);
            }
        }
    }

    std::string getModeName(LightMode mode) {
        switch(mode) {
            case MODE_OFF: return "Off";
            case MODE_BLUE_FLOW: return "Blue Flow";
            case MODE_RED_FLASH_FAST: return "Red Fast Flash";
            case MODE_YELLOW_FLASH_SLOW: return "Yellow Slow Flash";
            case MODE_WHITE_STEADY: return "White Steady";
            default: return "Unknown";
        }
    }

    void updateAnimation(const ros::TimerEvent& event) {
        std::fill(dmx_channels_.begin(), dmx_channels_.end(), 0);
        switch(current_mode_) {
            case MODE_BLUE_FLOW: updateBlueFlow(); break;
            case MODE_RED_FLASH_FAST: updateRedFastFlash(); break;
            case MODE_YELLOW_FLASH_SLOW: updateYellowSlowFlash(); break;
            case MODE_WHITE_STEADY: updateWhiteSteady(); break;
            default: break;
        }
        animation_step_ = (animation_step_ + 1) % 1000;
    }

    void updateBlueFlow() {
        int active_led = animation_step_ % led_count_;
        int fade_range = 5;
        for(int i = 0; i < led_count_; i++) {
            int distance = abs(i - active_led);
            if(distance > fade_range) continue;
            uint8_t brightness = 255 * (1.0 - (double)distance / fade_range);
            int blue_channel = i * 4 + 3;
            if(blue_channel <= 512) dmx_channels_[blue_channel-1] = brightness;
        }
    }

    void updateRedFastFlash() {
        bool on = (animation_step_ % 5) < 3;
        uint8_t brightness = on ? 255 : 0;
        for(int i = 0; i < led_count_; i++) {
            int red_channel = i * 4 + 1;
            if(red_channel <= 512) dmx_channels_[red_channel-1] = brightness;
        }
    }

    void updateYellowSlowFlash() {
        bool on = (animation_step_ % 30) < 15;
        uint8_t brightness = on ? 200 : 0;
        for(int i = 0; i < led_count_; i++) {
            int r_ch = i * 4 + 1;
            int g_ch = i * 4 + 2;
            if(r_ch <= 512) dmx_channels_[r_ch-1] = brightness;
            if(g_ch <= 512) dmx_channels_[g_ch-1] = brightness;
        }
    }

    void updateWhiteSteady() {
        for(int i = 0; i < led_count_; i++) {
            int w_ch = i * 4 + 4;
            if(w_ch <= 512) dmx_channels_[w_ch-1] = 220;
        }
    }

    void sendDMXData(const ros::TimerEvent& event) {
        if(serial_fd_ < 0) return;
        sendBulkDMXData();
    }

    void run() {
        ros::spin();
    }

private:
    std::vector<uint8_t> dmx_channels_;
    int serial_fd_;
    std::string serial_port_name_;
    int baudrate_;
    int send_rate_;
    int animation_rate_;
    LightMode current_mode_;
    int led_count_;
    int animation_step_;
    double reconnect_interval_;
    double last_reconnect_time_;

    ros::Subscriber mode_sub_;
    ros::Subscriber light_cmd_sub_;
    ros::Timer send_timer_;
    ros::Timer animation_timer_;
    ros::Timer reconnect_timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dmx512_light_controller");
    DMX512LightController controller;
    controller.run();
    return 0;
}