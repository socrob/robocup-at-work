#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>

using namespace cv;

int main(int argc, char** argv)
{
    if (argc != 2)
        std::cout << "Please provide an image file as argument" << std::endl;

    ros::init(argc, argv, "mcr_fake_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_gray = it.advertise("/stereo/left/image_mono", 1);
    image_transport::Publisher pub_color = it.advertise("/stereo/left/image_color", 1);

    cv::Mat image_gray = cv::imread(argv[1], IMREAD_GRAYSCALE);
    cv::Mat image_color = cv::imread(argv[1], IMREAD_COLOR);

    cv_bridge::CvImage out_msg_gray, out_msg_color;

    out_msg_gray.image = image_gray;
    out_msg_gray.encoding = "mono8";
    out_msg_color.image = image_color;
    out_msg_color.encoding ="bgr8";

    ros::Rate loop_rate(10);
    while (nh.ok())
    {
        out_msg_gray.header.stamp = ros::Time::now();
        pub_gray.publish(out_msg_gray.toImageMsg());
        pub_color.publish(out_msg_color.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }
}

