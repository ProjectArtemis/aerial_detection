#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "classifier.h"

const std::string RECEIVE_IMG_TOPIC_NAME = "/cam2/image_rect";
const std::string PUBLISH_RET_TOPIC_NAME = "/detections";

Classifier* classifier;
std::string model_path;
std::string weights_path;
std::string mean_file;
std::string label_file;
std::string image_path;

ros::Publisher gPublisher;

void publishRet(const std::vector<Prediction>& predictions);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        //cv::imwrite("rgb.png", cv_ptr->image);
		cv::Mat img = cv_ptr->image;
		std::vector<Prediction> predictions = classifier->Classify(img);
		publishRet(predictions);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// TODO: Define a msg or create a service
// Try to receive : $rostopic echo /caffe_ret
void publishRet(const std::vector<Prediction>& predictions)  {
    std_msgs::String msg;
    std::stringstream ss;
    for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        ss << "[" << p.second << " - " << p.first << "]" << std::endl;
    }
    msg.data = ss.str();
    gPublisher.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aerial_detector_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    // To receive an image from the topic, PUBLISH_RET_TOPIC_NAME
    image_transport::Subscriber sub = it.subscribe(RECEIVE_IMG_TOPIC_NAME, 1, imageCallback);
	gPublisher = nh.advertise<std_msgs::String>(PUBLISH_RET_TOPIC_NAME, 100);
    
    std::string DATA_DIR = "/home/artemis/data";
    
    model_path = DATA_DIR + "/deploy.prototxt";
    weights_path = DATA_DIR + "/bvlc_reference_caffenet.caffemodel";
    mean_file = DATA_DIR + "/imagenet_mean.binaryproto";
    label_file = DATA_DIR + "/synset_words.txt";
    image_path = DATA_DIR + "/cows.jpg";

    classifier = new Classifier(model_path, weights_path, mean_file, label_file);

    // TODO : read direct camera stream from callback
    cv::Mat img = cv::imread(image_path, -1);
    std::vector<Prediction> predictions = classifier->Classify(img);
    /* Print the top N predictions. */
    
    std::cout << "Test default image" << std::endl;
    for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        std::cout << std::fixed << std::setprecision(4) << p.second << " - \"" << p.first << "\"" << std::endl;
    }
    
    publishRet(predictions);

    ros::spin();
    delete classifier;
    ros::shutdown();
    return 0;
}
