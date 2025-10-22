#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <string>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>



class CameraNode : public rclcpp::Node {
public:
	CameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("camera_node", options)
	{
		RCLCPP_INFO(this->get_logger(), "Camera node started");
		flash_sub_ = this->create_subscription<std_msgs::msg::Int32>(
			"flash_light_intensity", 10,
			std::bind(&CameraNode::flash_callback, this, std::placeholders::_1));
		status_pub_ = this->create_publisher<std_msgs::msg::String>("camera_status", 10);
	}

	void setup_image_transport() {
		it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
		publisher_ = it_->advertise("camera/image_raw", 1);
	}

	void stream_video(const std::string& url) {
		cv::VideoCapture cap(url);
		std_msgs::msg::String status_msg;
		if (!cap.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open video stream: %s", url.c_str());
			status_msg.data = "offline";
			status_pub_->publish(status_msg);
			return;
		}
		status_msg.data = "online";
		status_pub_->publish(status_msg);
		cv::Mat frame;
		rclcpp::Rate rate(30);
		while (rclcpp::ok()) {
			cap >> frame;
			if (frame.empty()) {
				status_msg.data = "offline";
				status_pub_->publish(status_msg);
				continue;
			} else {
				status_msg.data = "online";
				status_pub_->publish(status_msg);
			}
			auto msg = sensor_msgs::msg::Image();
			msg.header.stamp = this->now();
			msg.header.frame_id = "camera";
			msg.height = frame.rows;
			msg.width = frame.cols;
			msg.encoding = "bgr8";
			msg.is_bigendian = false;
			msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
			msg.data.assign(frame.datastart, frame.dataend);
			publisher_.publish(msg);
			rate.sleep();
		}
	}

	void flash_callback(const std_msgs::msg::Int32::SharedPtr msg) {
		int intensity = msg->data;
		if (intensity < 0) intensity = 0;
		if (intensity > 255) intensity = 255;
		bool ok = set_flash_intensity(intensity);
		if (ok) {
			RCLCPP_INFO(this->get_logger(), "Set flash intensity to %d", intensity);
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to set flash intensity");
		}
	}

	bool set_flash_intensity(int intensity) {
		CURL* curl = curl_easy_init();
		if (!curl) return false;
		std::string url = "http://10.206.232.232/control?var=led_intensity&val=" + std::to_string(intensity);
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		CURLcode res = curl_easy_perform(curl);
		curl_easy_cleanup(curl);
		return (res == CURLE_OK);
	}

private:
	std::unique_ptr<image_transport::ImageTransport> it_;
	image_transport::Publisher publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flash_sub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraNode>();
	node->setup_image_transport();
	std::string stream_url = "http://10.206.232.232:81/stream";
	std::thread video_thread([&]() { node->stream_video(stream_url); });
	rclcpp::spin(node);
	video_thread.join();
	rclcpp::shutdown();
	return 0;
}

