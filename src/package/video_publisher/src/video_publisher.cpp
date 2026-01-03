/*
 * @Author: Hambin.Lu
 * @Description: ##
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <std_msgs/String.h>
#include <regex>

std::queue<cv::Mat> imageQueue;
image_transport::Publisher image_pub;
ros::Publisher img64;
int frame_width;
int frame_height;
int image_resize = 1;
bool save_flag = false;
bool pub_thread_flag = true;
bool use_nvdec = true;
static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string base64_encode(unsigned char const *bytes_to_encode, unsigned int in_len)
{
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--)
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while ((i++ < 3))
            ret += '=';
    }

    return ret;
}

void PubtDataThread(void)
{
    ros::Rate loop_rate(50);
    cv::Mat frame;
    cv::Mat resizedImage;
    sensor_msgs::ImagePtr image_msg;
    cv::VideoWriter video_writer("/home/nvidia" + ros::this_node::getName() + "1.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(frame_width, frame_height));

    while (ros::ok() && pub_thread_flag)
    {
        if (imageQueue.size() > 0)
        {
            try
            {
                frame = imageQueue.front();
                if (frame.cols == 0 || frame.rows == 0)
                    continue;
                // std::vector<cv::Point> points;
                // points.push_back(cv::Point(400, 1080));
                // points.push_back(cv::Point(frame_width - 400, 1080));
                // points.push_back(cv::Point(frame_width - 600, 900));
                // points.push_back(cv::Point(600, 900));
                // std::vector<std::vector<cv::Point>> contours;
                // contours.push_back(points);
                // cv::polylines(frame, contours, true, cv::Scalar(0, 0, 255), 10);

                cv::resize(frame, resizedImage, cv::Size(frame.cols / image_resize, frame.rows / image_resize));
                if (save_flag)
                    video_writer.write(resizedImage);
                if (image_pub.getNumSubscribers() != 0)
                {
                    std_msgs::Header header;
                    header.stamp = ros::Time::now();
                    image_msg = cv_bridge::CvImage(header, "bgr8", resizedImage).toImageMsg();
                    image_pub.publish(image_msg);
                }
                if (img64.getNumSubscribers() > 0)
                {
                    std::vector<uchar> buffer;
                    cv::imencode(".jpg", resizedImage, buffer);
                    std_msgs::String base64Image;
                    base64Image.data = base64_encode(buffer.data(), buffer.size());
                    img64.publish(base64Image);
                }
                imageQueue.pop();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        loop_rate.sleep();
    }

    video_writer.release();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;
    std::string url;
    nh.getParam(ros::this_node::getName() + "/save", save_flag);
    nh.getParam(ros::this_node::getName() + "/image_resize", image_resize);
    nh.getParam(ros::this_node::getName() + "/use_nvdec", use_nvdec);
    if (!nh.getParam(ros::this_node::getName() + "/url", url))
    {
        ROS_ERROR("No url param");
        return -1;
    };
    std::regex ipRegex(R"rtsp(rtsp://\w+:\w+@(\d+\.\d+\.\d+\.\d+):(\d+)/\w+)rtsp");
    std::smatch match;
    if (std::regex_search(url, match, ipRegex))
    {
        if (match.size() > 1)
        {
            bool net_ok = false;
            std::string ip = match[1];
            while (ros::ok() && !net_ok)
            {
                std::string command = "ping -c 1 " + ip + " > /dev/null";
                int status = system(command.c_str());
                if (status == 0)
                    net_ok = true;
                else
                    ROS_WARN("%s network offline!", ip.c_str());
                ros::Rate(0.5).sleep();
            }
        }
    }

    img64 = nh.advertise<std_msgs::String>(ros::this_node::getName() + "/img64", 1);
    image_transport::ImageTransport it(nh);
    image_pub = it.advertise(ros::this_node::getName() + "/image", 1);
    std::shared_ptr<std::thread> pub_data_thread;
    pub_data_thread = std::make_shared<std::thread>(std::thread(&PubtDataThread));
    if (pub_data_thread->joinable())
        pub_data_thread->detach();

    cv::VideoCapture cap;
    std::string pipeline = "rtspsrc location=" + url + " ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! appsink sync = false ";
    if (use_nvdec)
        cap.open(pipeline, cv::CAP_GSTREAMER);
    else
        cap.open(url,cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open RTSP video stream");
        return -1;
    }

    frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    cv::Mat frame;
    while (ros::ok())
    {
        try
        {
            cap >> frame;
            if (!frame.empty())
            {
                if (imageQueue.size() > 2)
                {
                    ROS_INFO("too much image ");
                    imageQueue.pop();
                }
                imageQueue.push(frame);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        ros::Rate(30).sleep();
    }

    cap.release();
    pub_thread_flag = false;

    return 0;
}
