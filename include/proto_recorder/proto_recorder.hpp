#ifndef PROTO_RECORDER__PROTO_RECORDER_HPP_
#define PROTO_RECORDER__PROTO_RECORDER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"

namespace proto_recorder
{

class ProtoRecorder : public rclcpp::Node
{
public:
  // Default constructor for composition
  explicit ProtoRecorder(const rclcpp::NodeOptions & options);
  
  // Destructor
  virtual ~ProtoRecorder();

  // Start recording
  void record();

  // Stop recording
  void stop();

  // Pause recording
  void pause();

  // Resume recording
  void resume();

  // Check if recording is paused
  bool is_paused() const;

private:
  // Subscribe to topics
  void subscribe_topics(const std::vector<std::string> & topics);
  
  // Create a subscription for a topic
  void subscribe_topic(const std::string & topic_name, const std::string & topic_type);
  
  // Create a generic subscription
  std::shared_ptr<rclcpp::GenericSubscription> create_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos);

  // Get topic names and types
  std::unordered_map<std::string, std::string> fetch_topic_names_and_types();

  // Retry subscribing to topics
  void retry_topics();

  // Member variables
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::RecordOptions record_options_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  std::atomic<bool> paused_{false};
  std::string serialization_format_;
  
  // Timer for retrying subscription to specified topics
  rclcpp::TimerBase::SharedPtr topic_retry_timer_;
  
  // Flag to indicate if all topics are subscribed
  std::atomic<bool> all_topics_subscribed_{false};
};

}  // namespace proto_recorder

#endif  // PROTO_RECORDER__PROTO_RECORDER_HPP_
