#include "proto_recorder/proto_recorder.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "yaml-cpp/yaml.h"

namespace proto_recorder
{

ProtoRecorder::ProtoRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("proto_recorder", options)
{
  // Declare parameters and get values in one step
  storage_options_.storage_id = declare_parameter<std::string>("storage_id", "mcap");
  storage_options_.uri = declare_parameter<std::string>("uri", "proto_recording");
  
  // Add storage configuration parameters - use int64_t instead of uint64_t to avoid ambiguity
  storage_options_.max_bagfile_size = static_cast<uint64_t>(declare_parameter<int64_t>("max_bagfile_size", 0));
  storage_options_.max_bagfile_duration = static_cast<uint64_t>(declare_parameter<int64_t>("max_bagfile_duration", 0));
  storage_options_.max_cache_size = static_cast<uint64_t>(declare_parameter<int64_t>("max_cache_size", 0));
  storage_options_.storage_preset_profile = declare_parameter<std::string>("storage_preset_profile", "");
  storage_options_.storage_config_uri = declare_parameter<std::string>("storage_config_uri", "");
  
  // Get topics from parameter or file
  auto topics_file = declare_parameter<std::string>("topics_file", "");
  if (!topics_file.empty()) {
    record_options_.topics = load_topics_from_file(topics_file);
  }
  
  record_options_.rmw_serialization_format = declare_parameter<std::string>("serialization_format", "cdr");
  record_options_.start_paused = declare_parameter<bool>("start_paused", false);
  
  // Add compression options - use int64_t for numeric parameters
  record_options_.compression_mode = declare_parameter<std::string>("compression_mode", "");
  record_options_.compression_format = declare_parameter<std::string>("compression_format", "");
  record_options_.compression_queue_size = static_cast<uint64_t>(declare_parameter<int64_t>("compression_queue_size", 1));
  record_options_.compression_threads = static_cast<uint64_t>(declare_parameter<int64_t>("compression_threads", 0));
  
  // Topic rate check parameters
  rate_check_window_size_ = static_cast<size_t>(declare_parameter<int64_t>("rate_check_window_size", 10));
  diagnostics_period_ = declare_parameter<double>("diagnostics_period", 1.0);
  
  // Initialize writer
  writer_ = std::make_shared<rosbag2_cpp::Writer>();
  
  // Set initial paused state
  paused_ = record_options_.start_paused;
  
  // Set serialization format
  serialization_format_ = record_options_.rmw_serialization_format;
  
  // Initialize diagnostics updater
  updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  updater_->setHardwareID("proto_recorder");
  updater_->add("topic_rates", std::bind(&ProtoRecorder::check_topic_rates, this, std::placeholders::_1));
  updater_->setPeriod(diagnostics_period_);
  
  RCLCPP_INFO(
    this->get_logger(),
    "Initialized ProtoRecorder with storage_id: %s, uri: %s",
    storage_options_.storage_id.c_str(),
    storage_options_.uri.c_str());
    
  if (!storage_options_.storage_config_uri.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Using storage configuration file: %s",
      storage_options_.storage_config_uri.c_str());
  }
  
  if (!storage_options_.storage_preset_profile.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Using storage preset profile: %s",
      storage_options_.storage_preset_profile.c_str());
  }
  
  record();
}

ProtoRecorder::~ProtoRecorder()
{
  stop();
}

std::vector<std::string> ProtoRecorder::load_topics_from_file(const std::string & file_path)
{
  std::vector<std::string> topics;
  
  try {
    RCLCPP_INFO(get_logger(), "Loading topics from file: %s", file_path.c_str());
    YAML::Node config = YAML::LoadFile(file_path);
    
    if (config["topics"]) {
      for (const auto & topic : config["topics"]) {
        if (topic["name"]) {
          std::string topic_name = topic["name"].as<std::string>();
          topics.push_back(topic_name);
          RCLCPP_INFO(get_logger(), "Added topic from file: %s", topic_name.c_str());
          
          // Load hz_range if available
          if (topic["hz_range"] && topic["hz_range"].IsSequence() && topic["hz_range"].size() == 2) {
            double min_rate = topic["hz_range"][0].as<double>();
            double max_rate = topic["hz_range"][1].as<double>();
            topic_info_[topic_name].min_rate = min_rate;
            topic_info_[topic_name].max_rate = max_rate;
            RCLCPP_INFO(get_logger(), "  Rate range: %.1f - %.1f Hz", min_rate, max_rate);
          }
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error loading topics from file: %s", e.what());
  }
  
  return topics;
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
  
  // Initialize topic info for rate checking
  topic_info_[topic_name].name = topic_name;
  topic_info_[topic_name].type = topic_type;
  
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
      // Update topic rate statistics
      update_topic_rate(topic_name, this->get_clock()->now());
      
      // Do not start recording until all topics are subscribed
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

void ProtoRecorder::update_topic_rate(const std::string & topic_name, const rclcpp::Time & now)
{
  auto & info = topic_info_[topic_name];
  std::lock_guard<std::mutex> lock(info.mutex);
  
  // Only record the timestamp
  info.message_times.push_back(now);
  
  // Remove old timestamps to keep within window size
  while (info.message_times.size() > rate_check_window_size_) {
    info.message_times.pop_front();
  }
  
  // Rate calculation is done in check_topic_rates
}

void ProtoRecorder::check_topic_rates(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Topic rates are normal";
  std::vector<std::string> abnormal_topics;
  
  rclcpp::Time now = this->get_clock()->now();
  
  // Check all subscribed topics
  for (const auto & topic_entry : record_options_.topics) {
    // Get topic name
    const std::string & topic_name = topic_entry;
    
    // Initialize topic_info_ if the topic doesn't exist
    if (topic_info_.find(topic_name) == topic_info_.end()) {
      topic_info_[topic_name].name = topic_name;
      // Type is unknown, so empty string
      topic_info_[topic_name].type = "";
    }
    
    TopicInfo & info = topic_info_[topic_name];
    std::lock_guard<std::mutex> lock(info.mutex);
          
    // Calculate rate if there are at least 2 messages
    if (info.message_times.size() >= 2) {
      auto oldest = info.message_times.front();
      auto newest = info.message_times.back();
      double duration = (newest - oldest).seconds();
      
      if (duration > 0.0) {
        // Calculation: (message count - 1) / duration
        info.rate = static_cast<double>(info.message_times.size() - 1) / duration;
      }
      
      // If too much time has passed since the last message, reduce the rate
      double time_since_last_msg = (now - newest).seconds();
      if (time_since_last_msg > diagnostics_period_) {
        // Adjust rate based on time elapsed since last message
        // Example: If 2 seconds have passed since the last message, halve the rate
        double decay_factor = diagnostics_period_ / time_since_last_msg;
        info.rate *= decay_factor;
      }
    } else if (info.message_times.size() == 1) {
      // If there's only one message
      double time_since_msg = (now - info.message_times.front()).seconds();
      if (time_since_msg > 0.0) {
        // 1 message / elapsed time (approaches 0 if time is too long)
        info.rate = 1.0 / std::max(time_since_msg, diagnostics_period_);
      } else {
        info.rate = 0.0;
      }
    } else {
      // No messages
      info.rate = 0.0;
    } 
    
    // Format rate string with 2 decimal places
    std::string rate_str = std::to_string(info.rate);
    rate_str = rate_str.substr(0, rate_str.find(".") + 3); // 小数点以下2桁まで
    stat.add(topic_name, rate_str);
    
    // Check if rate is within expected range
    if (info.min_rate > 0.0 && info.max_rate > 0.0) {
      if (info.rate < info.min_rate || info.rate > info.max_rate) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        abnormal_topics.push_back(topic_name + " (" + rate_str + " Hz)");
      }
    }    
  }
  
  // Update summary message if there are abnormal topics
  if (!abnormal_topics.empty()) {
    message = "Abnormal rates for topics: " + abnormal_topics[0];
    for (size_t i = 1; i < abnormal_topics.size(); ++i) {
      message += ", " + abnormal_topics[i];
    }
  }
  
  stat.summary(level, message);
}

}  // namespace proto_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(proto_recorder::ProtoRecorder)
