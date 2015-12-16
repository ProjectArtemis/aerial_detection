#include <ros/ros.h>
#include <ros/package.h>
#include <aerial_detection/detections.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "classifier.h"

Classifier *classifier;

ros::Publisher pred_pub;
void publishPred(const std::vector<Prediction> &predictions);

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

		cv::Mat img = cv_ptr->image;
		std::vector<Prediction> predictions = classifier->Classify(img);
		publishPred(predictions);

	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void publishPred(const std::vector<Prediction> &predictions)
{
	aerial_detection::detections msg;
	
	// Publish 5 top predictions
	for (size_t i = 0; i < 5; ++i) {	
		Prediction p = predictions[i];
		msg.detection[i] = p.first;
		msg.detection_id[i] = p.second;
	}

	pred_pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aerial_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	std::string data_directory;
	std::string camera_topic;
	nh.param<std::string>("data_directory", data_directory, "/home/artemis/data");
	nh.param<std::string>("camera_topic", camera_topic, "/stereo/cam0/image_rect");

	image_transport::Subscriber image_sub = it.subscribe(camera_topic, 1, imageCallback);
	pred_pub = nh.advertise<aerial_detection::detections>("detections", 10);
	
	ROS_INFO("Starting neural network classifier");

	std::string model_path = data_directory + "/deploy.prototxt";
	std::string weights_path = data_directory + "/bvlc_reference_caffenet.caffemodel";
	std::string mean_file = data_directory + "/imagenet_mean.binaryproto";
	std::string label_file = data_directory + "/synset_words.txt";
	std::string image_path = data_directory + "/cows.jpg";

	classifier = new Classifier(model_path, weights_path, mean_file, label_file);
	
	while(ros::ok())
	{
		ros::spin();
	}
	
	delete classifier;
	ros::shutdown();
	return 0;
}
