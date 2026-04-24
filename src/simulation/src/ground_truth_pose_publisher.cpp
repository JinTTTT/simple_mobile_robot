#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <algorithm>
#include <string>
#include <vector>

class GroundTruthPosePublisher : public rclcpp::Node
{
public:
  GroundTruthPosePublisher()
  : Node("ground_truth_pose_publisher")
  {
    candidate_child_frames_ = declare_parameter<std::vector<std::string>>(
      "candidate_child_frames",
      std::vector<std::string>{"my_first_robot", "base_link"});
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/world/empty/dynamic_pose/info");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/ground_truth_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/ground_truth_path");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    max_path_length_ = declare_parameter<int>("max_path_length", 2000);

    pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic_,
      10,
      std::bind(&GroundTruthPosePublisher::poseCallback, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "Ground truth pose publisher started. Listening on %s and publishing %s / %s.",
      input_topic_.c_str(),
      pose_topic_.c_str(),
      path_topic_.c_str());
  }

private:
  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & transform : msg->transforms) {
      if (!matchesCandidateFrame(transform.child_frame_id)) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = transform.header;
      pose_msg.header.frame_id = output_frame_id_;
      pose_msg.pose.position.x = transform.transform.translation.x;
      pose_msg.pose.position.y = transform.transform.translation.y;
      pose_msg.pose.position.z = transform.transform.translation.z;
      pose_msg.pose.orientation = transform.transform.rotation;

      pose_pub_->publish(pose_msg);

      path_msg_.header.frame_id = output_frame_id_;
      path_msg_.header.stamp = pose_msg.header.stamp;
      path_msg_.poses.push_back(pose_msg);

      if (static_cast<int>(path_msg_.poses.size()) > max_path_length_) {
        path_msg_.poses.erase(path_msg_.poses.begin());
      }

      path_pub_->publish(path_msg_);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "No matching robot frame found in ground truth stream yet.");
  }

  bool matchesCandidateFrame(const std::string & child_frame_id) const
  {
    for (const auto & candidate : candidate_child_frames_) {
      if (child_frame_id == candidate) {
        return true;
      }

      const std::string suffix = "/" + candidate;
      if (child_frame_id.size() > suffix.size() &&
        child_frame_id.compare(child_frame_id.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> candidate_child_frames_;
  std::string input_topic_;
  std::string pose_topic_;
  std::string path_topic_;
  std::string output_frame_id_;
  int max_path_length_ = 2000;

  nav_msgs::msg::Path path_msg_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
