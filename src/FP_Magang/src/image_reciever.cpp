#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

ros::Publisher image_pub;

void processAndPublishImage(cv::Mat& frame) {

    cv::resize(frame, frame, cv::Size(900, 600));

    sensor_msgs::Image img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.header.frame_id = "camera";
    img_msg.height = frame.rows;
    img_msg.width = frame.cols;
    img_msg.encoding = "bgr8";
    img_msg.is_bigendian = false;
    img_msg.step = frame.cols * 3;
    img_msg.data.resize(img_msg.step * img_msg.height);

    memcpy(&img_msg.data[0], frame.data, frame.total() * frame.elemSize());

    image_pub.publish(img_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_receiver");

    ros::NodeHandle nh;

    image_pub = nh.advertise<sensor_msgs::Image>("/image_resized", 1);

    cv::Mat frame = cv::imread("/home/rasya/Documents/ROBOTIK/FINAL_PROJECT/fp_ws/src/bola3.jpg");

    if (frame.empty()) {
        ROS_ERROR("Failed to load image.");
        return 1;
    }

    ros::Rate loop_rate(1);

    while (ros::ok()) {

        processAndPublishImage(frame);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
