#ifndef PROTO_RECORDER__PROTO_RECORDER_HPP_
#define PROTO_RECORDER__PROTO_RECORDER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <proto_recorder_msgs/msg/recorder_status.hpp>
#include <proto_recorder_msgs/msg/topic_status.hpp>
#include <std_msgs/msg/bool.hpp>

namespace proto_recorder
{

struct TopicInfo {
  std::string name;
  std::string type;
  std::deque<rclcpp::Time> message_times;
  std::mutex mutex;
  double rate{0.0};
  double min_rate{0.0};  // Minimum expected rate in Hz
  double max_rate{0.0};  // Maximum expected rate in Hz
};

class ProtoRecorder : public rclcpp::Node
{
public:
  // Default constructor for composition
  explicit ProtoRecorder(const rclcpp::NodeOptions & options);
  
  // Destructor
  virtual ~ProtoRecorder();

  // Start recording (initializes writer)
  void start();

  // Stop recording (closes writer)
  void stop();

  // Pause recording (sets paused_ flag)
  void pause();

  // Resume recording (clears paused_ flag)
  void resume();

  // Initialize subscriptions (called once from constructor)
  void initialize_subscriptions();


  // Check if recording is paused
  bool is_paused() const;

private:
  // Load topics from YAML file
  std::vector<std::string> load_topics_from_file(const std::string & file_path);
  
  // Subscribe to topics
  void subscribe_topics(const std::vector<std::string> & topics);
  
  // Create a subscription for a topic
  void subscribe_topic(const std::string & topic_name, const std::string & topic_type);
  
  // Create a generic subscription
  std::shared_ptr<rclcpp::GenericSubscription> create_generic_topic_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos);

  // Get topic names and types
  std::unordered_map<std::string, std::string> fetch_topic_names_and_types();

  // Retry subscribing to topics
  void retry_topics();
  
  // Update topic rate statistics
  void update_topic_rate(const std::string & topic_name, const rclcpp::Time & now);
  
  // Diagnostics callback
  void check_topic_rates();
  
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
  
  // Topic rate checking
  std::unordered_map<std::string, TopicInfo> topic_info_;
  size_t rate_check_window_size_{10};
  
  // Status publishing
  rclcpp::Publisher<proto_recorder_msgs::msg::RecorderStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  double diagnostics_period_{1.0};
  std::string hardware_id_;

  // Flag to indicate if we are waiting for stable rates
  std::atomic<bool> wait_for_stable_rates_{false};

  // Control subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_resume_sub_;

  // Recording state
  std::atomic<bool> is_recording_{false};
  
  // Original URI prefix (without timestamp)
  std::string original_uri_prefix_;
  std::string dir_permission_;
  std::string file_permission_;

  // Generate timestamped URI
  std::string generate_timestamped_uri(const std::string & prefix);
  
  // Callback functions for control subscriptions
  void on_start(const std_msgs::msg::Bool::SharedPtr msg);
  void on_pause(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::QoS get_subscription_qos_for_topic(const std::string & topic_name);
  rclcpp::QoS adapt_qos_to_publishers(const std::string & topic_name);
  
  // Initialize writer and open storage
  void initialize_writer();
  
  // Register all existing topics to writer
  void register_all_topics_to_writer();
  
  // Register a single topic to writer
  void register_topic_to_writer(const std::string & topic_name, const std::string & topic_type);
  
  // Serialize QoS profiles for a topic
  std::string serialized_offered_qos_profiles_for_topic(const std::string & topic_name);
};

}  // namespace proto_recorder

#endif  // PROTO_RECORDER__PROTO_RECORDER_HPP_
