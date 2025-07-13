#include "proto_recorder/proto_recorder.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/bool.hpp>

namespace proto_recorder
{

ProtoRecorder::ProtoRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("proto_recorder", options)
{
  // Declare parameters and get values in one step
  storage_options_.storage_id = declare_parameter<std::string>("storage_id", "mcap");
  storage_options_.uri = declare_parameter<std::string>("uri", "proto_recording");
  
  // Save original URI prefix
  original_uri_prefix_ = storage_options_.uri;
  
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
  
  // Add parameter to wait for stable rates before recording
  wait_for_stable_rates_ = declare_parameter<bool>("wait_for_stable_rates", false);
  
  // Hardware ID parameter for diagnostics
  hardware_id_ = declare_parameter<std::string>("hardware_id", "proto_recorder");
  
  // Initialize writer
  writer_ = std::make_shared<rosbag2_cpp::Writer>();
  
  // Set initial paused state - pause if we're waiting for stable rates or start_paused is true
  paused_ = record_options_.start_paused || wait_for_stable_rates_;
  
  // Set serialization format
  serialization_format_ = record_options_.rmw_serialization_format;
  
  // Initialize status publisher
  status_publisher_ = this->create_publisher<proto_recorder_msgs::msg::RecorderStatus>(
    "~/output/status", rclcpp::QoS(10));
  
  // Create timer for periodic status updates
  status_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(diagnostics_period_),
    std::bind(&ProtoRecorder::check_topic_rates, this));
  
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
  
  // Create control subscriptions
  start_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/input/start",
    rclcpp::QoS(1).transient_local(),
    std::bind(&ProtoRecorder::on_start, this, std::placeholders::_1));

  pause_resume_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/input/pause",
    rclcpp::QoS(1).transient_local(),
    std::bind(&ProtoRecorder::on_pause, this, std::placeholders::_1));

  // Start recording first to open the writer
  start();
  
  // Initialize subscriptions after writer is opened
  initialize_subscriptions();
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
          
          // Load min_rate and max_rate if available
          if (topic["min_rate"] && topic["max_rate"]) {
            double min_rate = topic["min_rate"].as<double>();
            double max_rate = topic["max_rate"].as<double>();
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

void ProtoRecorder::start()
{
  if (is_recording_.load()) {
    RCLCPP_WARN(get_logger(), "Recording is already started");
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Starting recording...");
  
  if (record_options_.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }
  
  try {
    // Generate timestamped URI using original prefix
    storage_options_.uri = generate_timestamped_uri(original_uri_prefix_);
    
    RCLCPP_INFO(get_logger(), "Opening writer with URI: %s, serialization format: %s", 
                storage_options_.uri.c_str(), record_options_.rmw_serialization_format.c_str());
    
    // Create new writer instance
    writer_ = std::make_shared<rosbag2_cpp::Writer>();
    writer_->open(
      storage_options_,
      {rmw_get_serialization_format(), record_options_.rmw_serialization_format});

    // Re-create topics in the new writer
    for (const auto & [topic_name, subscription] : subscriptions_) {
      auto & info = topic_info_[topic_name];
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = topic_name;
      topic_metadata.type = info.type;
      topic_metadata.serialization_format = serialization_format_;
      writer_->create_topic(topic_metadata);
    }

    is_recording_.store(true);
    RCLCPP_INFO(get_logger(), "Recording started.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", e.what());
    if (writer_) {
      writer_->close();
      writer_.reset();
    }
    throw;
  }
}

void ProtoRecorder::stop()
{
  if (!is_recording_.load()) {
    RCLCPP_WARN(get_logger(), "Recording is not started");
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Stopping recording...");
  
  is_recording_.store(false);
  paused_.store(false);  // Reset paused state
  
  try {
    if (writer_) {
      writer_->close();
      writer_.reset();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error closing writer: %s", e.what());
  }
  
  RCLCPP_INFO(get_logger(), "Recording stopped.");
}

void ProtoRecorder::pause()
{
  if (!is_recording_.load()) {
    RCLCPP_WARN(get_logger(), "Cannot pause: recording is not started");
    return;
  }
  
  if (paused_.load()) {
    RCLCPP_WARN(get_logger(), "Recording is already paused");
    return;
  }
  
  paused_.store(true);
  RCLCPP_INFO(get_logger(), "Recording paused.");
}

void ProtoRecorder::resume()
{
  if (!is_recording_.load()) {
    RCLCPP_WARN(get_logger(), "Cannot resume: recording is not started");
    return;
  }
  
  if (!paused_.load()) {
    RCLCPP_WARN(get_logger(), "Recording is not paused");
    return;
  }
  
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
  
  // Get appropriate QoS for this topic
  auto qos = get_subscription_qos_for_topic(topic_name);
  
  // Create subscription with adapted QoS
  auto subscription = create_generic_topic_subscription(topic_name, topic_type, qos);
  
  if (subscription) {
    subscriptions_[topic_name] = subscription;
    RCLCPP_INFO(
      get_logger(),
      "Subscribed to topic '%s' with type '%s' using %s reliability and %s durability",
      topic_name.c_str(),
      topic_type.c_str(),
      qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ? "RELIABLE" : "BEST_EFFORT",
      qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "TRANSIENT_LOCAL" : "VOLATILE");
  } else {
    writer_->remove_topic(topic_metadata);
    RCLCPP_ERROR(
      get_logger(),
      "Failed to subscribe to topic '%s'",
      topic_name.c_str());
  }
}

std::shared_ptr<rclcpp::GenericSubscription> ProtoRecorder::create_generic_topic_subscription(
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos)
{
  auto subscription = this->create_generic_subscription(
    topic_name,
    topic_type,
    qos,
    [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> message) {
      // Update topic rate statistics
      update_topic_rate(topic_name, this->get_clock()->now());
      
      // Only record if recording is started, not paused, and all topics are subscribed
      if (is_recording_.load() && !paused_.load() && all_topics_subscribed_) {
        try {
          if (writer_) {
            writer_->write(message, topic_name, topic_type, this->get_clock()->now());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Error writing message to bag for topic '%s': %s", 
                       topic_name.c_str(), e.what());
        }
      }
    });
  return subscription;
}

std::unordered_map<std::string, std::string> ProtoRecorder::fetch_topic_names_and_types()
{
  std::unordered_map<std::string, std::string> topics_and_types;
  
  // Use the Node's method to get topic names and types
  auto topic_names_and_types = this->get_node_graph_interface()->get_topic_names_and_types();
  
  for (const auto & topic_name_and_types : topic_names_and_types) {
    const auto & topic_name = topic_name_and_types.first;
    const auto & type_names = topic_name_and_types.second;
        
    if (type_names.empty() || type_names.size() > 1) {
      continue;
    }
    
    topics_and_types[topic_name] = type_names[0];
  }
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

void ProtoRecorder::check_topic_rates()
{
  auto status_msg = std::make_unique<proto_recorder_msgs::msg::RecorderStatus>();
  status_msg->header.stamp = this->get_clock()->now();
  status_msg->hardware_id = hardware_id_;
  status_msg->is_recording = (writer_ != nullptr && !paused_);
  
  
  uint8_t error_level = proto_recorder_msgs::msg::RecorderStatus::ERROR_LEVEL_OK;
  bool all_rates_stable = true;
  
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
    
    // Create TopicStatus message
    proto_recorder_msgs::msg::TopicStatus topic_status_msg;
    topic_status_msg.name = topic_name;
    topic_status_msg.type = info.type;
    topic_status_msg.rate = info.rate;
    
    // Determine rate status
    if (info.message_times.empty()) {
      topic_status_msg.rate_status = proto_recorder_msgs::msg::TopicStatus::RATE_STATUS_NO_MESSAGES;
      error_level = proto_recorder_msgs::msg::RecorderStatus::ERROR_LEVEL_WARN;
      all_rates_stable = false;
    } else if (info.min_rate > 0.0 && info.max_rate > 0.0) {
      if (info.rate < info.min_rate) {
        topic_status_msg.rate_status = proto_recorder_msgs::msg::TopicStatus::RATE_STATUS_TOO_LOW;
        error_level = proto_recorder_msgs::msg::RecorderStatus::ERROR_LEVEL_WARN;
        all_rates_stable = false;
      } else if (info.rate > info.max_rate) {
        topic_status_msg.rate_status = proto_recorder_msgs::msg::TopicStatus::RATE_STATUS_TOO_HIGH;
        error_level = proto_recorder_msgs::msg::RecorderStatus::ERROR_LEVEL_WARN;
        all_rates_stable = false;
      } else {
        topic_status_msg.rate_status = proto_recorder_msgs::msg::TopicStatus::RATE_STATUS_NORMAL;
      }
    } else if (info.message_times.size() < rate_check_window_size_) {
      topic_status_msg.rate_status = proto_recorder_msgs::msg::TopicStatus::RATE_STATUS_UNKNOWN;
      // UNKNOWNは一時的な状態なので、error_levelは上げない
      all_rates_stable = false;
    } else {
      topic_status_msg.rate_status = proto_recorder_msgs::msg::TopicStatus::RATE_STATUS_NORMAL;
    }
    
    status_msg->topic_statuses.push_back(topic_status_msg);
  }
  
  status_msg->error_level = error_level;
  
  // Publish status message
  status_publisher_->publish(std::move(status_msg));
  
  // If we're waiting for stable rates and all rates are now stable, resume recording
  if (wait_for_stable_rates_ && all_rates_stable && paused_.load() && all_topics_subscribed_) {
    RCLCPP_INFO(get_logger(), "All topic rates are now stable. Resuming recording.");
    wait_for_stable_rates_ = false;  // Don't auto-resume again
    resume();
  }
}

rclcpp::QoS ProtoRecorder::get_subscription_qos_for_topic(const std::string & topic_name)
{
  // Adapt to publisher QoS
  return adapt_qos_to_publishers(topic_name);
}

rclcpp::QoS ProtoRecorder::adapt_qos_to_publishers(const std::string & topic_name)
{
  auto publishers_info = this->get_publishers_info_by_topic(topic_name);
  
  // Always initialize with a depth value (10 is a common default)
  rclcpp::QoS adapted_qos(10);
  
  if (publishers_info.empty()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "No publishers found for topic " << topic_name << ", using default QoS");
    return adapted_qos;  // Return the default QoS we just created
  }
  
  // Count publishers with different QoS policies
  size_t num_publishers = publishers_info.size();
  size_t reliable_publishers = 0;
  size_t transient_local_publishers = 0;
  
  for (const auto & info : publishers_info) {
    const auto & profile = info.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliable_publishers++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      transient_local_publishers++;
    }
  }
  
  // Set reliability policy
  if (reliable_publishers == num_publishers) {
    adapted_qos.reliable();
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "All publishers for topic " << topic_name << " use RELIABLE QoS, matching");
  } else {
    adapted_qos.best_effort();
    if (reliable_publishers > 0) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Mixed reliability for publishers on topic " << topic_name << 
        ". Using BEST_EFFORT to connect to all publishers, but may miss messages from RELIABLE publishers");
    } else {
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "All publishers for topic " << topic_name << " use BEST_EFFORT QoS, matching");
    }
  }
  
  // Set durability policy
  if (transient_local_publishers == num_publishers) {
    adapted_qos.transient_local();
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "All publishers for topic " << topic_name << " use TRANSIENT_LOCAL QoS, matching");
  } else {
    adapted_qos.durability_volatile();
    if (transient_local_publishers > 0) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Mixed durability for publishers on topic " << topic_name << 
        ". Using VOLATILE to connect to all publishers, but will not receive latched messages");
    } else {
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "All publishers for topic " << topic_name << " use VOLATILE QoS, matching");
    }
  }
  
  return adapted_qos;
}

std::string ProtoRecorder::generate_timestamped_uri(const std::string & prefix)
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  
  std::stringstream ss;
  ss << prefix << "_" << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H-%M-%S");
  
  // Add timezone offset
  auto local_time = std::localtime(&time_t);
  char tz_buffer[16];
  std::strftime(tz_buffer, sizeof(tz_buffer), "%z", local_time);
  
  // Convert +0900 format to +09:00 format if needed, but ROS bag uses +0900 format
  ss << tz_buffer;
  
  return ss.str();
}

void ProtoRecorder::on_start(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_recording_.load()) {
    start();
  } else if (!msg->data && is_recording_.load()) {
    stop();
  }
}

void ProtoRecorder::on_pause(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_paused()) {
    pause();
  } else if (!msg->data && is_paused()) {
    resume();
  }
}

void ProtoRecorder::initialize_subscriptions()
{
  if (subscriptions_initialized_.load()) {
    return;
  }
  
  if (!record_options_.topics.empty()) {
    RCLCPP_INFO(get_logger(), "Initializing subscriptions to specified topics");
    subscribe_topics(record_options_.topics);
    
    topic_retry_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ProtoRecorder::retry_topics, this));
  } else {
    RCLCPP_WARN(get_logger(), "No topics specified for recording.");
  }
  
  subscriptions_initialized_.store(true);
}

}  // namespace proto_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(proto_recorder::ProtoRecorder)
