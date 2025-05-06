#include "proto_recorder/proto_recorder.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

namespace proto_recorder
{

ProtoRecorder::ProtoRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("proto_recorder", options)
{
  // Declare parameters
  this->declare_parameter("storage_id", "sqlite3");
  this->declare_parameter("uri", "proto_recording");
  this->declare_parameter("topics", std::vector<std::string>{});
  this->declare_parameter("all", false);
  this->declare_parameter("serialization_format", "cdr");
  this->declare_parameter("start_paused", false);

  // Get parameters
  storage_options_.storage_id = this->get_parameter("storage_id").as_string();
  storage_options_.uri = this->get_parameter("uri").as_string();
  
  record_options_.topics = this->get_parameter("topics").as_string_array();
  record_options_.all = this->get_parameter("all").as_bool();
  record_options_.rmw_serialization_format = this->get_parameter("serialization_format").as_string();
  record_options_.start_paused = this->get_parameter("start_paused").as_bool();
  
  // Initialize writer
  writer_ = std::make_shared<rosbag2_cpp::Writer>();
  
  // Set initial paused state
  paused_ = record_options_.start_paused;
  
  // Set serialization format
  serialization_format_ = record_options_.rmw_serialization_format;
  
  RCLCPP_INFO(
    this->get_logger(),
    "Initialized ProtoRecorder with storage_id: %s, uri: %s",
    storage_options_.storage_id.c_str(),
    storage_options_.uri.c_str());
  record();
}

ProtoRecorder::~ProtoRecorder()
{
  stop();
}

void ProtoRecorder::record()
{
  RCLCPP_INFO(get_logger(), "Recording started.");
  if (record_options_.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }
  RCLCPP_INFO(get_logger(), "Opening writer with serialization format: %s", record_options_.rmw_serialization_format.c_str());
  writer_->open(
    storage_options_,
    {rmw_get_serialization_format(), record_options_.rmw_serialization_format});

  if (!record_options_.topics.empty()) {
    RCLCPP_INFO(get_logger(), "Subscribing to specified topics");
    // Subscribe to specified topics
    subscribe_topics(record_options_.topics);
    
    // Start a timer to retry subscribing to topics that weren't found
    topic_retry_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ProtoRecorder::retry_topics, this));
  } else {
    RCLCPP_WARN(get_logger(), "No topics specified for recording.");
  }

  if (record_options_.start_paused) {
    RCLCPP_INFO(
      get_logger(), "Recording started but paused. Call resume() to start recording.");
  } else {
    RCLCPP_INFO(get_logger(), "Recording started.");
  }
}

void ProtoRecorder::stop()
{
  // Cancel timer if it exists
  if (topic_retry_timer_) {
    topic_retry_timer_->cancel();
  }
  
  subscriptions_.clear();
  if (writer_) {
    writer_->close();
  }
  RCLCPP_INFO(get_logger(), "Recording stopped.");
}

void ProtoRecorder::pause()
{
  paused_.store(true);
  RCLCPP_INFO(get_logger(), "Recording paused.");
}

void ProtoRecorder::resume()
{
  paused_.store(false);
  RCLCPP_INFO(get_logger(), "Recording resumed.");
}

bool ProtoRecorder::is_paused() const
{
  return paused_.load();
}

void ProtoRecorder::subscribe_topics(const std::vector<std::string> & topics)
{
  // Fetch topic names and types once
  auto topics_and_types = fetch_topic_names_and_types();
  RCLCPP_INFO(get_logger(), "Successfully got topic names and types");
  
  for (const auto & topic : topics) {
    RCLCPP_INFO(get_logger(), "Subscribing to topic: %s", topic.c_str());
    auto it = topics_and_types.find(topic);
    if (it != topics_and_types.end()) {
      subscribe_topic(topic, it->second);
    } else {
      RCLCPP_WARN(
        get_logger(), "Topic '%s' not found or not yet available.", topic.c_str());
    }
  }
}

void ProtoRecorder::subscribe_topic(const std::string & topic_name, const std::string & topic_type)
{
  RCLCPP_INFO(get_logger(), "Subscribing to topic '%s' with type '%s'", topic_name.c_str(), topic_type.c_str());
  // Create topic metadata
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = serialization_format_;
  
  // Create topic in writer
  writer_->create_topic(topic_metadata);
  
  // Create subscription with default QoS
  auto subscription = create_subscription(topic_name, topic_type, rclcpp::QoS(10));
  
  if (subscription) {
    subscriptions_[topic_name] = subscription;
    RCLCPP_INFO(
      get_logger(),
      "Subscribed to topic '%s' with type '%s'",
      topic_name.c_str(),
      topic_type.c_str());
  } else {
    writer_->remove_topic(topic_metadata);
    RCLCPP_ERROR(
      get_logger(),
      "Failed to subscribe to topic '%s'",
      topic_name.c_str());
  }
}

std::shared_ptr<rclcpp::GenericSubscription> ProtoRecorder::create_subscription(
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos)
{
  auto subscription = this->create_generic_subscription(
    topic_name,
    topic_type,
    qos,
    [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> message) {
      // 全てのトピックが購読されるまで記録を開始しない
      if (!paused_.load() && all_topics_subscribed_) {
        writer_->write(message, topic_name, topic_type, this->get_clock()->now());
      }
    });
  return subscription;
}

std::unordered_map<std::string, std::string> ProtoRecorder::fetch_topic_names_and_types()
{
  RCLCPP_INFO(get_logger(), "Getting topic names and types");
  std::unordered_map<std::string, std::string> topics_and_types;
  
  // Use the Node's method to get topic names and types
  auto topic_names_and_types = this->get_node_graph_interface()->get_topic_names_and_types();
  
  for (const auto & topic_name_and_types : topic_names_and_types) {
    const auto & topic_name = topic_name_and_types.first;
    const auto & type_names = topic_name_and_types.second;
    
    RCLCPP_INFO(get_logger(), "Topic name: %s, Type count: %zu", 
                topic_name.c_str(), type_names.size());
    
    if (type_names.empty() || type_names.size() > 1) {
      continue;
    }
    
    topics_and_types[topic_name] = type_names[0];
    RCLCPP_INFO(get_logger(), "Topic name: %s, Type: %s", topic_name.c_str(), type_names[0].c_str());
  }
  RCLCPP_INFO(get_logger(), "Topics and types: %zu", topics_and_types.size());
  return topics_and_types;
}

void ProtoRecorder::retry_topics()
{
  if (!record_options_.topics.empty()) {
    std::vector<std::string> topics_to_retry;
    
    for (const auto & topic : record_options_.topics) {
      // Check if we're already subscribed to this topic
      if (subscriptions_.find(topic) == subscriptions_.end()) {
        topics_to_retry.push_back(topic);
      }
    }
    
    if (!topics_to_retry.empty()) {
      RCLCPP_INFO(get_logger(), "Retrying subscription to %zu topics", topics_to_retry.size());
      subscribe_topics(topics_to_retry);
    } else {
      // All topics are subscribed, cancel the timer
      RCLCPP_INFO(get_logger(), "All requested topics are now subscribed");
      all_topics_subscribed_ = true;
      topic_retry_timer_->cancel();
    }
  }
}

}  // namespace proto_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(proto_recorder::ProtoRecorder)
