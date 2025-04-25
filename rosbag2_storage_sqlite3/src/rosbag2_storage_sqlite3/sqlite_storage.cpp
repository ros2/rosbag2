// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2_storage_sqlite3/sqlite_storage.hpp"

#include <sys/stat.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>  // NOLINT cpplint: FP, filesystem is a C++17 system header
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/env.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_storage_sqlite3/sqlite_exception.hpp"
#include "rosbag2_storage_sqlite3/sqlite_pragmas.hpp"
#include "rosbag2_storage_sqlite3/sqlite_statement_wrapper.hpp"

#include "logging.hpp"

namespace
{
std::string to_string(rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND:
      return "APPEND";
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      return "READ_ONLY";
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
      return "READ_WRITE";
    default:
      return "UNKNOWN";
  }
}

bool is_read_write(const rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  return io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE;
}

// Return pragma-name to full statement map
inline std::unordered_map<std::string, std::string> parse_pragmas(
  const std::string & storage_config_uri, const rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  std::unordered_map<std::string, std::string> pragmas;
  if (storage_config_uri.empty()) {
    return pragmas;
  }

  std::vector<std::string> pragma_entries;
  try {
    auto key =
      io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY ? "read" : "write";
    YAML::Node yaml_file = YAML::LoadFile(storage_config_uri);
    pragma_entries = yaml_file[key]["pragmas"].as<std::vector<std::string>>();
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(
            std::string("Exception on parsing sqlite3 config file: ") +
            ex.what());
  }
  // poor developer's sqlinjection prevention ;-)
  std::string invalid_characters = {"';\""};
  auto throw_on_invalid_character = [](const auto & pragmas, const auto & invalid_characters) {
      for (const auto & pragma_string : pragmas) {
        auto pos = pragma_string.find_first_of(invalid_characters);
        if (pos != std::string::npos) {
          throw std::runtime_error(
                  std::string("Invalid characters in sqlite3 config file: ") +
                  pragma_string[pos] +
                  ". Avoid following characters: " +
                  invalid_characters);
        }
      }
    };
  throw_on_invalid_character(pragma_entries, invalid_characters);

  // Extract pragma name and map to full pragma statement
  for (const auto & pragma : pragma_entries) {
    if (pragma.empty()) {
      continue;
    }

    const std::string pragma_assign = "=";
    const std::string pragma_bracket = "(";
    auto found_value_assignment = pragma.find(pragma_assign);

    // Extract pragma name. It is the same as statement for read only pragmas
    auto pragma_name = pragma;

    // Find assignment operator and strip value assignment part
    if (found_value_assignment == std::string::npos) {
      found_value_assignment = pragma.find(pragma_bracket);
    }
    if (found_value_assignment != std::string::npos) {
      if (found_value_assignment == 0) {
        // Incorrect syntax, starts with = or (
        std::stringstream errmsg;
        errmsg << "Incorrect storage setting syntax: " << pragma;
        throw std::runtime_error{errmsg.str()};
      }
      // Strip value assignment part, trim trailing whitespaces before = or (
      pragma_name = pragma.substr(0, found_value_assignment);
      const std::string whitespaces(" \t");
      pragma_name = pragma_name.substr(0, pragma_name.find_last_not_of(whitespaces) + 1);
    }

    auto full_pragma_statement = "PRAGMA " + pragma + ";";
    pragmas.insert({pragma_name, full_pragma_statement});
  }
  return pragmas;
}

void apply_resilient_storage_settings(std::unordered_map<std::string, std::string> & pragmas)
{
  auto robust_pragmas = rosbag2_storage_plugins::SqlitePragmas::robust_writing_pragmas();
  for (const auto & kv : robust_pragmas) {
    // do not override settings from configuration file, otherwise apply
    if (pragmas.count(kv.first) == 0) {
      pragmas[kv.first] = kv.second;
    }
  }
}

constexpr const auto FILE_EXTENSION = ".db3";

// Minimum size of a sqlite3 database file in bytes (84 kiB).
constexpr const uint64_t MIN_SPLIT_FILE_SIZE = 86016;
}  // namespace

namespace rosbag2_storage_plugins
{
SqliteStorage::~SqliteStorage()
{
  if (active_transaction_) {
    commit_transaction();
  }
}

SqliteStorage::PresetProfile SqliteStorage::parse_preset_profile(const std::string & profile_string)
{
  if (profile_string == "resilient") {
    return SqliteStorage::PresetProfile::Resilient;
  } else if (profile_string == "none" || profile_string == "") {
    return SqliteStorage::PresetProfile::WriteOptimized;
  } else {
    throw std::runtime_error(
            "Invalid storage preset profile string: " + profile_string + "\n"
            "Valid options are:\n"
            "'none': configure writer for maximum write throughput and to minimize dropped "
            "messages.\n"
            "'resilient': indicate preference for avoiding data corruption in case of crashes, "
            "at the cost of performance. Setting this flag disables optimization settings for "
            "storage."
    );
  }
}

void SqliteStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  if (page_count_statement_) {
    page_count_statement_->reset();  // Reset statement to avoid DB lock
    page_count_statement_.reset();
  }
  storage_mode_ = io_flag;
  const auto preset = parse_preset_profile(storage_options.storage_preset_profile);
  auto pragmas = parse_pragmas(storage_options.storage_config_uri, io_flag);
  if (preset == PresetProfile::Resilient && is_read_write(io_flag)) {
    apply_resilient_storage_settings(pragmas);
  }

  if (is_read_write(io_flag)) {
    relative_path_ = storage_options.uri + FILE_EXTENSION;

    // READ_WRITE requires the DB to not exist.
    if (std::filesystem::exists(std::filesystem::path(relative_path_))) {
      throw std::runtime_error(
              "Failed to create bag: File '" + relative_path_ + "' already exists!");
    }
  } else {  // APPEND and READ_ONLY
    relative_path_ = storage_options.uri;

    // APPEND and READ_ONLY require the DB to exist
    if (!std::filesystem::exists(std::filesystem::path(relative_path_))) {
      throw std::runtime_error(
              "Failed to read from bag: File '" + relative_path_ + "' does not exist!");
    }
  }

  try {
    database_ = std::make_unique<SqliteWrapper>(relative_path_, io_flag, std::move(pragmas));
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  page_count_statement_ = database_->prepare_statement("PRAGMA page_count;");
  db_page_size_ = get_page_size();
  db_file_size_ = db_page_size_ * read_total_page_count_locked();

  // initialize only for READ_WRITE since the DB is already initialized if in APPEND.
  if (is_read_write(io_flag)) {
    db_schema_version_ = kDBSchemaVersion_;
    std::lock_guard<std::mutex> db_lock(db_read_write_mutex_);
    initialize();
  } else {
    db_schema_version_ = read_db_schema_version();
    read_metadata();
  }

  // Reset the read and write statements in case the database changed.
  // These will be reinitialized lazily on the first read or write.
  read_statement_ = nullptr;
  write_statement_ = nullptr;

  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_INFO_STREAM(
    "Opened database '" << relative_path_ << "' for " << to_string(io_flag) << ".");
}

void SqliteStorage::update_metadata(const rosbag2_storage::BagMetadata & metadata)
{
  metadata_ = metadata;
  if (db_schema_version_ >= 3) {  // We don't have metadata table before schema version 3
    YAML::Node metadata_node = YAML::convert<rosbag2_storage::BagMetadata>::encode(metadata);
    std::string serialized_metadata = YAML::Dump(metadata_node);

    auto insert_metadata = database_->prepare_statement(
      "INSERT INTO metadata (metadata_version, metadata) VALUES (?, ?)");
    insert_metadata->bind(metadata.version, serialized_metadata);
    std::lock_guard<std::mutex> db_lock(db_read_write_mutex_);
    insert_metadata->execute_and_reset();
    db_file_size_ = db_page_size_ * read_total_page_count_locked();
  }
}

void SqliteStorage::activate_transaction()
{
  if (active_transaction_) {
    return;
  }

  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_DEBUG_STREAM("begin transaction");
  database_->prepare_statement("BEGIN TRANSACTION;")->execute_and_reset();

  active_transaction_ = true;
}

void SqliteStorage::commit_transaction()
{
  if (!active_transaction_) {
    return;
  }

  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_DEBUG_STREAM("commit transaction");
  database_->prepare_statement("COMMIT;")->execute_and_reset();

  active_transaction_ = false;
}

void SqliteStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  std::lock_guard<std::mutex> db_lock(db_read_write_mutex_);
  write_locked(message);
  db_file_size_ = db_page_size_ * read_total_page_count_locked();
}

void SqliteStorage::write_locked(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  if (!write_statement_) {
    prepare_for_writing();
  }
  auto topic_entry = topics_.find(message->topic_name);
  if (topic_entry == end(topics_)) {
    throw SqliteException(
            "Topic '" + message->topic_name +
            "' has not been created yet! Call 'create_topic' first.");
  }

  try {
    write_statement_->bind(
      message->recv_timestamp, topic_entry->second,
      message->serialized_data);
  } catch (const SqliteException & exc) {
    if (SQLITE_TOOBIG == exc.get_sqlite_return_code()) {
      // Get the sqlite string/blob limit.
      const size_t sqlite_limit = sqlite3_limit(
        this->get_sqlite_database_wrapper().get_database(),
        SQLITE_LIMIT_LENGTH,
        -1);
      ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_WARN_STREAM(
        "Message on topic '" << message->topic_name << "' of size '" <<
          message->serialized_data->buffer_length <<
          "' bytes failed to write because it exceeds the maximum size sqlite can store ('" <<
          sqlite_limit << "' bytes): " <<
          exc.what());
      return;
    } else {
      // Rethrow.
      throw;
    }
  }
  write_statement_->execute_and_reset();
}

void SqliteStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
{
  std::lock_guard<std::mutex> db_lock(db_read_write_mutex_);
  if (!write_statement_) {
    prepare_for_writing();
  }

  activate_transaction();

  for (auto & message : messages) {
    write_locked(message);
    // Update db_file_size_ with the estimated data size written to the DB
    // + 3 * sizeof(int64_t) This is for storing timestamp, topic_id and index for messages table
    db_file_size_ += message->serialized_data->buffer_length + 3 * sizeof(int64_t);
  }

  commit_transaction();
  // Correct db_file_size_ by reading out the exact numbers from the DB
  db_file_size_ = db_page_size_ * read_total_page_count_locked();
}

bool SqliteStorage::set_read_order(const rosbag2_storage::ReadOrder & read_order)
{
  if (read_order.sort_by == rosbag2_storage::ReadOrder::PublishedTimestamp) {
    ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_DEBUG("ReadOrder::PublishedTimestamp not implemented");
    return false;
  }
  if (read_order.sort_by == rosbag2_storage::ReadOrder::File) {
    ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_DEBUG("ReadOrder::File not implemented");
    return false;
  }

  read_order_ = read_order;
  read_statement_ = nullptr;
  return true;
}

bool SqliteStorage::has_next()
{
  if (!read_statement_) {
    prepare_for_reading();
  }

  return current_message_row_ != message_result_.end();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SqliteStorage::read_next()
{
  if (!read_statement_) {
    prepare_for_reading();
  }

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->serialized_data = std::get<0>(*current_message_row_);
  bag_message->recv_timestamp = std::get<1>(*current_message_row_);
  bag_message->topic_name = std::get<2>(*current_message_row_);

  // set start time to current time
  // and set seek_row_id to the new row id up
  seek_time_ = bag_message->recv_timestamp;
  seek_row_id_ = std::get<3>(*current_message_row_) + (read_order_.reverse ? -1 : 1);

  ++current_message_row_;
  return bag_message;
}

std::vector<rosbag2_storage::TopicMetadata> SqliteStorage::get_all_topics_and_types()
{
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types();
  }

  return all_topics_and_types_;
}

void SqliteStorage::get_all_message_definitions(
  std::vector<rosbag2_storage::MessageDefinition> & definitions)
{
  definitions.clear();
  if (db_schema_version_ < 4) {return;}
  auto statement = database_->prepare_statement(
    "SELECT topic_type, encoding, encoded_message_definition, type_description_hash FROM "
    "message_definitions ORDER BY id;");
  auto query_results =
    statement->execute_query<std::string, std::string, std::string, std::string>();

  for (auto result : query_results) {
    definitions.push_back(
      {std::get<0>(result), std::get<1>(result), std::get<2>(result), std::get<3>(result)});
  }
}

uint64_t SqliteStorage::get_bagfile_size() const
{
  if (!database_ || !page_count_statement_) {
    // Trying to call get_bagfile_size when SqliteStorage::open was not called or db
    // failed to open. Fallback to the filesystem call.
    const auto bag_path = std::filesystem::path{get_relative_file_path()};
    return std::filesystem::exists(bag_path) ? std::filesystem::file_size(bag_path) : 0u;
  } else {
    return db_file_size_;
  }
}

void SqliteStorage::initialize()
{
  std::string create_stmt = "CREATE TABLE schema(" \
    "schema_version INTEGER PRIMARY KEY," \
    "ros_distro TEXT NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();

  create_stmt = "CREATE TABLE metadata(" \
    "id INTEGER PRIMARY KEY," \
    "metadata_version INTEGER NOT NULL," \
    "metadata TEXT NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();

  create_stmt = "CREATE TABLE topics(" \
    "id INTEGER PRIMARY KEY," \
    "name TEXT NOT NULL," \
    "type TEXT NOT NULL," \
    "serialization_format TEXT NOT NULL," \
    "offered_qos_profiles TEXT NOT NULL," \
    "type_description_hash TEXT NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();

  create_stmt = "CREATE TABLE message_definitions(" \
    "id INTEGER PRIMARY KEY," \
    "topic_type TEXT NOT NULL," \
    "encoding TEXT NOT NULL," \
    "encoded_message_definition TEXT NOT NULL," \
    "type_description_hash TEXT NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();

  create_stmt = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY," \
    "topic_id INTEGER NOT NULL," \
    "timestamp INTEGER NOT NULL, " \
    "data BLOB NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();

  create_stmt = "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
  database_->prepare_statement(create_stmt)->execute_and_reset();

  std::string ros_distro = rcpputils::get_env_var("ROS_DISTRO");
  auto insert_db_schema =
    database_->prepare_statement(
    "INSERT INTO schema (schema_version, ros_distro) VALUES (?, ?)");
  insert_db_schema->bind(kDBSchemaVersion_, ros_distro);
  insert_db_schema->execute_and_reset();
}

void SqliteStorage::create_topic(
  const rosbag2_storage::TopicMetadata & topic,
  const rosbag2_storage::MessageDefinition & message_definition)
{
  std::lock_guard<std::mutex> db_lock(db_read_write_mutex_);
  if (topics_.find(topic.name) == std::end(topics_)) {
    auto insert_topic =
      database_->prepare_statement(
      "INSERT INTO topics"
      "(name, type, serialization_format, offered_qos_profiles, type_description_hash) "
      "VALUES (?, ?, ?, ?, ?)");
    insert_topic->bind(
      topic.name,
      topic.type,
      topic.serialization_format,
      rosbag2_storage::serialize_rclcpp_qos_vector(topic.offered_qos_profiles),
      topic.type_description_hash);
    insert_topic->execute_and_reset();
    topics_.emplace(topic.name, static_cast<int>(database_->get_last_insert_id()));
  }
  // TODO(morlov): Add topic.type_description_hash when it will be really calculated or getting
  //  from service. Currently dummy hashes causing tests failure
  std::string topic_type_and_hash = message_definition.topic_type;
  if (!topic_type_and_hash.empty() &&
    msg_definitions_.find(topic_type_and_hash) == std::end(msg_definitions_))
  {
    std::string type_description_hash = message_definition.type_hash.empty() ?
      topic.type_description_hash : message_definition.type_hash;
    auto insert_msg_definition =
      database_->prepare_statement(
      "INSERT INTO message_definitions (topic_type, encoding, encoded_message_definition, "
      "type_description_hash) VALUES (?, ?, ?, ?)");
    insert_msg_definition->bind(
      message_definition.topic_type, message_definition.encoding,
      message_definition.encoded_message_definition, type_description_hash);
    insert_msg_definition->execute_and_reset();
    msg_definitions_.emplace(
      topic_type_and_hash,
      static_cast<int>(database_->get_last_insert_id()));
  }
  db_file_size_ = db_page_size_ * read_total_page_count_locked();
}

void SqliteStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  std::lock_guard<std::mutex> db_lock(db_read_write_mutex_);
  if (topics_.find(topic.name) != std::end(topics_)) {
    auto delete_topic =
      database_->prepare_statement(
      "DELETE FROM topics where name = ? and type = ? and serialization_format = ?");
    delete_topic->bind(topic.name, topic.type, topic.serialization_format);
    delete_topic->execute_and_reset();
    topics_.erase(topic.name);
  }
  db_file_size_ = db_page_size_ * read_total_page_count_locked();
}

void SqliteStorage::prepare_for_writing()
{
  write_statement_ = database_->prepare_statement(
    "INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");
}

namespace
{
void prepare_included_topics_filter(
  const rosbag2_storage::StorageFilter & storage_filter,
  std::vector<std::string> & where_conditions)
{
  std::string included_topic_names_str;
  // Add topic names
  if (!storage_filter.topics.empty()) {
    for (auto & topic : storage_filter.topics) {
      included_topic_names_str += "'" + topic + "'";
      if (&topic != &storage_filter.topics.back()) {
        included_topic_names_str += ",";
      }
    }
  }

  // Add service event topic names
  if (!storage_filter.services_events.empty()) {
    if (!included_topic_names_str.empty()) {
      included_topic_names_str += ",";
    }
    for (auto & service : storage_filter.services_events) {
      included_topic_names_str += "'" + service + "'";
      if (&service != &storage_filter.services_events.back()) {
        included_topic_names_str += ",";
      }
    }
  }

  std::string topics_filter_str;
  if (!included_topic_names_str.empty()) {
    topics_filter_str.append("(topics.name IN (" + included_topic_names_str + "))");
  }

  std::string regex_filter_str;
  // Add topics filter based on regular expression
  if (!storage_filter.regex.empty()) {
    // Construct string for selected topics
    regex_filter_str = "(topics.name REGEXP '" + storage_filter.regex + "')";
  }

  static const char * regex_for_all_service_events_str = "(topics.name REGEXP '.*/_service_event')";

  if (!topics_filter_str.empty() && !regex_filter_str.empty()) {
    // Note: Inclusive filter conditions shall be joined with OR
    // Note: Even if services_events list or topics list is empty we shall not include regex for
    // all service events or for all topics, because storage_filter.regex is not empty and shall
    // dominate in this case.
    where_conditions.push_back("(" + topics_filter_str + " OR " + regex_filter_str + ")");
  } else if (!topics_filter_str.empty()) {  // Note: regex_filter_str is empty in this case
    if (!storage_filter.services_events.empty()) {
      if (!storage_filter.topics.empty()) {
        where_conditions.push_back(topics_filter_str);
      } else {  // if topics list empty and service_events not empty we shall include all topics
        where_conditions.push_back(
          "(" + topics_filter_str + " OR NOT " + regex_for_all_service_events_str + ")");
      }
    } else if (!storage_filter.topics.empty()) {
      where_conditions.push_back(
        "(" + topics_filter_str + " OR " + regex_for_all_service_events_str + ")");
    } else {
      // This shall never happen unless someone will make incorrect changes in logic
      throw std::logic_error("Either service_events list or topics list shall be not empty!");
    }
  } else if (!regex_filter_str.empty()) {
    where_conditions.push_back(regex_filter_str);
  }
}

void prepare_excluded_topics_filter(
  const rosbag2_storage::StorageFilter & storage_filter,
  std::vector<std::string> & where_conditions)
{
  std::string excluded_topic_names_str;
  // Add excluded topic name
  if (!storage_filter.exclude_topics.empty()) {
    for (auto & topic : storage_filter.exclude_topics) {
      excluded_topic_names_str += "'" + topic + "'";
      if (&topic != &storage_filter.exclude_topics.back()) {
        excluded_topic_names_str += ",";
      }
    }
  }

  // Add service event topic name
  if (!storage_filter.exclude_service_events.empty()) {
    excluded_topic_names_str += ",";
    for (auto & service : storage_filter.exclude_service_events) {
      excluded_topic_names_str += "'" + service + "'";
      if (&service != &storage_filter.exclude_service_events.back()) {
        excluded_topic_names_str += ",";
      }
    }
  }

  if (!excluded_topic_names_str.empty()) {
    where_conditions.push_back("(topics.name NOT IN (" + excluded_topic_names_str + "))");
  }

  // exclude topics based on regular expressions
  if (!storage_filter.regex_to_exclude.empty()) {
    // Construct string for selected topics
    where_conditions.push_back(
      "(topics.name NOT IN "
      "(SELECT topics.name FROM topics WHERE topics.name REGEXP '" +
      storage_filter.regex_to_exclude + "'))");
  }
}
}  // namespace

void SqliteStorage::prepare_for_reading()
{
  std::string statement_str = "SELECT data, timestamp, topics.name, messages.id "
    "FROM messages JOIN topics ON messages.topic_id = topics.id WHERE ";
  std::vector<std::string> where_conditions;

  prepare_included_topics_filter(storage_filter_, where_conditions);
  prepare_excluded_topics_filter(storage_filter_, where_conditions);

  const std::string direction_op = read_order_.reverse ? "<" : ">";
  const std::string order_direction = read_order_.reverse ? "DESC" : "ASC";

  // Add seek head filter
  // When doing timestamp ordering, we need a secondary ordering on message_id
  // Timestamp is not required to be unique, but message_id is, so for messages with the same
  // timestamp we order by the id to have a consistent and deterministic order.
  where_conditions.push_back(
    "(((timestamp = " + std::to_string(seek_time_) + ") "
    "AND (messages.id " + direction_op + "= " + std::to_string(seek_row_id_) + ")) "
    "OR (timestamp " + direction_op + " " + std::to_string(seek_time_) + ")) ");

  for (auto it = where_conditions.begin(); it != where_conditions.end(); ++it) {
    statement_str += *it;
    if (it != where_conditions.end() - 1) {
      statement_str += " AND ";
    }
  }

  // Add order by time then id
  statement_str += "ORDER BY messages.timestamp " + order_direction;
  statement_str += ", messages.id " + order_direction;
  statement_str += ";";

  read_statement_ = database_->prepare_statement(statement_str);
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  current_message_row_ = message_result_.begin();
}

void SqliteStorage::fill_topics_and_types()
{
  if (database_->field_exists("topics", "offered_qos_profiles")) {
    if (database_->field_exists("topics", "type_description_hash")) {
      auto statement = database_->prepare_statement(
        "SELECT id, name, type, serialization_format, offered_qos_profiles, type_description_hash"
        " FROM topics ORDER BY id;");
      auto query_results = statement->execute_query<
        int64_t, std::string, std::string, std::string, std::string, std::string>();

      for (const auto & [inner_topic_id, topic_name, topic_type, ser_format,
        offered_qos_profiles_str, type_hash] : query_results)
      {
        auto offered_qos_profiles = rosbag2_storage::to_rclcpp_qos_vector(
          // Before db_schema_version_ = 3 we didn't store metadata in the database and real
          // metadata_.version will be lower than 9
          offered_qos_profiles_str, (db_schema_version_ >= 3) ? metadata_.version : 8);
        all_topics_and_types_.push_back(
          {get_or_generate_extern_topic_id(inner_topic_id), topic_name, topic_type, ser_format,
            offered_qos_profiles, type_hash});
      }
    } else {  // Without type_hash
      auto statement = database_->prepare_statement(
        "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics "
        "ORDER BY id;");
      auto query_results = statement->execute_query<
        int64_t, std::string, std::string, std::string, std::string>();

      for (const auto & [inner_topic_id, topic_name, topic_type, ser_format,
        offered_qos_profiles_str] : query_results)
      {
        auto offered_qos_profiles = rosbag2_storage::to_rclcpp_qos_vector(
          // Before db_schema_version_ = 3 we didn't store metadata in the database and real
          // metadata_.version will be lower than 9
          offered_qos_profiles_str, (db_schema_version_ >= 3) ? metadata_.version : 8);
        all_topics_and_types_.push_back(
          {get_or_generate_extern_topic_id(inner_topic_id), topic_name, topic_type, ser_format,
            offered_qos_profiles, ""});
      }
    }
  } else {  // No offered_qos_profiles and no type_hash
    auto statement = database_->prepare_statement(
      "SELECT id, name, type, serialization_format FROM topics ORDER BY id;");
    auto query_results =
      statement->execute_query<int64_t, std::string, std::string, std::string>();

    for (const auto & [inner_topic_id, topic_name, topic_type, ser_format] : query_results) {
      all_topics_and_types_.push_back(
        {get_or_generate_extern_topic_id(inner_topic_id), topic_name, topic_type, ser_format,
          {}, ""});
    }
  }
}

std::string SqliteStorage::get_storage_identifier() const
{
  return "sqlite3";
}

std::string SqliteStorage::get_relative_file_path() const
{
  return relative_path_;
}

uint64_t SqliteStorage::get_minimum_split_file_size() const
{
  return MIN_SPLIT_FILE_SIZE;
}

void SqliteStorage::add_topic_to_metadata(
  int64_t inner_topic_id, std::string topic_name, std::string topic_type, std::string ser_format,
  int64_t msg_count, const std::string & offered_qos_profiles_str, const std::string & type_hash)
{
  auto offered_qos_profiles = rosbag2_storage::to_rclcpp_qos_vector(
    // Before db_schema_version_ = 3 we didn't store metadata in the database and real
    // metadata_.version will be lower than 9
    offered_qos_profiles_str, (db_schema_version_ >= 3) ? metadata_.version : 8);
  const rosbag2_storage::TopicMetadata topic_metadata {
    get_or_generate_extern_topic_id(inner_topic_id), topic_name, topic_type, ser_format,
    offered_qos_profiles, type_hash};
  auto & topics_list = metadata_.topics_with_message_count;
  auto it = std::find_if(
    topics_list.begin(), topics_list.end(),
    [&](const rosbag2_storage::TopicInformation & topic_info) {
      return topic_info.topic_metadata == topic_metadata;
    });
  if (it != topics_list.end()) {
    it->message_count = msg_count;
  } else {
    metadata_.topics_with_message_count.push_back(
      {
        {
          get_or_generate_extern_topic_id(inner_topic_id), topic_name, topic_type, ser_format,
          offered_qos_profiles, type_hash
        },
        static_cast<size_t>(msg_count)
      });
  }

  metadata_.message_count += msg_count;
}

void SqliteStorage::read_metadata()
{
  std::vector<rosbag2_storage::BagMetadata> metadata_from_table;
  if (db_schema_version_ >= 3) {
    auto statement = database_->prepare_statement(
      "SELECT metadata_version, metadata FROM metadata ORDER BY id;");
    auto query_results = statement->execute_query<std::string, std::string>();

    for (auto result : query_results) {
      std::string serialized_metadata = std::get<1>(result);
      YAML::Node metadata_node = YAML::Load(serialized_metadata);

      rosbag2_storage::BagMetadata current_metadata{};
      YAML::convert<rosbag2_storage::BagMetadata>::decode(metadata_node, current_metadata);
      metadata_from_table.push_back(current_metadata);
    }
  }
  if (!metadata_from_table.empty()) {
    metadata_ = metadata_from_table.back();  // Take latest saved metadata
  } else {
    metadata_.storage_identifier = get_storage_identifier();
    metadata_.relative_file_paths = {get_relative_file_path()};
  }
  // Update metadata according to the real data stored in file
  metadata_.message_count = 0;
  metadata_.topics_with_message_count = {};

  rcutils_time_point_value_t min_time = INT64_MAX;
  rcutils_time_point_value_t max_time = 0;

  // fill in all topics and types even those do not have any messages
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types();
  }
  metadata_.topics_with_message_count.reserve(all_topics_and_types_.size());
  for (const auto & topic_metadata : all_topics_and_types_) {
    metadata_.topics_with_message_count.push_back({topic_metadata, 0});
  }

  if (database_->field_exists("topics", "offered_qos_profiles")) {
    if (database_->field_exists("topics", "type_description_hash")) {
      std::string query =
        "SELECT messages.topic_id, name, type, serialization_format, COUNT(messages.id), "
        "MIN(messages.timestamp), MAX(messages.timestamp), offered_qos_profiles, "
        "type_description_hash FROM messages JOIN topics on topics.id = messages.topic_id "
        "GROUP BY topics.name;";

      auto statement = database_->prepare_statement(query);
      auto query_results = statement->execute_query<
        int64_t, std::string, std::string, std::string, int64_t, rcutils_time_point_value_t,
        rcutils_time_point_value_t, std::string, std::string>();

      for (const auto & [inner_topic_id, topic_name, topic_type, ser_format, msg_count,
        min_recv_timestamp, max_recv_timestamp, offered_qos_profiles_str, type_hash] :
        query_results)
      {
        add_topic_to_metadata(
          inner_topic_id, topic_name, topic_type, ser_format, msg_count,
          offered_qos_profiles_str, type_hash);

        min_time = min_recv_timestamp < min_time ? min_recv_timestamp : min_time;
        max_time = max_recv_timestamp > max_time ? max_recv_timestamp : max_time;
      }
    } else {  // Without type_hash
      std::string query =
        "SELECT messages.topic_id, name, type, serialization_format, COUNT(messages.id), "
        "MIN(messages.timestamp), MAX(messages.timestamp), offered_qos_profiles "
        "FROM messages JOIN topics on topics.id = messages.topic_id "
        "GROUP BY topics.name;";

      auto statement = database_->prepare_statement(query);
      auto query_results =
        statement->execute_query<int64_t, std::string, std::string, std::string, int64_t,
          rcutils_time_point_value_t, rcutils_time_point_value_t, std::string>();

      for (const auto & [inner_topic_id, topic_name, topic_type, ser_format, msg_count,
        min_recv_timestamp, max_recv_timestamp, offered_qos_profiles_str] : query_results)
      {
        add_topic_to_metadata(
          inner_topic_id, topic_name, topic_type, ser_format, msg_count,
          offered_qos_profiles_str, "");

        min_time = min_recv_timestamp < min_time ? min_recv_timestamp : min_time;
        max_time = max_recv_timestamp > max_time ? max_recv_timestamp : max_time;
      }
    }
  } else {  // No offered_qos_profiles and no type_hash
    std::string query =
      "SELECT messages.topic_id, name, type, serialization_format, COUNT(messages.id), "
      "MIN(messages.timestamp), MAX(messages.timestamp) "
      "FROM messages JOIN topics on topics.id = messages.topic_id "
      "GROUP BY topics.name;";
    auto statement = database_->prepare_statement(query);
    auto query_results = statement->execute_query<int64_t, std::string, std::string,
        std::string, int64_t, rcutils_time_point_value_t, rcutils_time_point_value_t>();

    for (const auto & [inner_topic_id, topic_name, topic_type, ser_format, msg_count,
      min_recv_timestamp, max_recv_timestamp] : query_results)
    {
      add_topic_to_metadata(inner_topic_id, topic_name, topic_type, ser_format, msg_count, "", "");

      min_time = min_recv_timestamp < min_time ? min_recv_timestamp : min_time;
      max_time = max_recv_timestamp > max_time ? max_recv_timestamp : max_time;
    }
  }

  if (metadata_.message_count == 0) {
    min_time = 0;
    max_time = 0;
  }

  metadata_.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(min_time));
  metadata_.duration = std::chrono::nanoseconds(max_time) - std::chrono::nanoseconds(min_time);
  metadata_.bag_size = get_bagfile_size();

  if (db_schema_version_ >= 3 && database_->table_exists("schema")) {
    // Read schema version
    auto statement = database_->prepare_statement("SELECT ros_distro from schema;");
    auto query_results = statement->execute_query<std::string>();
    metadata_.ros_distro = std::get<0>(*query_results.begin());
  }
}

rosbag2_storage::BagMetadata SqliteStorage::get_metadata()
{
  if (storage_mode_ != rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    read_metadata();
  }
  return metadata_;
}

void SqliteStorage::set_filter(
  const rosbag2_storage::StorageFilter & storage_filter)
{
  // keep current start time and start row_id
  // set topic filter and reset read statement for re-read
  storage_filter_ = storage_filter;
  read_statement_ = nullptr;
}

void SqliteStorage::reset_filter()
{
  set_filter(rosbag2_storage::StorageFilter());
}

void SqliteStorage::seek(const rcutils_time_point_value_t & timestamp)
{
  // reset row id and set start time to input
  // keep topic filter and reset read statement for re-read
  seek_row_id_ = read_order_.reverse ? get_last_rowid() : 0;
  seek_time_ = timestamp;
  read_statement_ = nullptr;
}

std::string SqliteStorage::get_storage_setting(const std::string & key)
{
  return database_->query_pragma_value(key);
}

SqliteWrapper & SqliteStorage::get_sqlite_database_wrapper()
{
  if (nullptr == database_) {
    throw std::runtime_error("database not open");
  }
  return *database_;
}

int SqliteStorage::get_db_schema_version() const
{
  return db_schema_version_;
}

int SqliteStorage::get_last_rowid()
{
  auto statement = database_->prepare_statement("SELECT max(rowid) from messages;");
  auto query_results = statement->execute_query<int>();
  return std::get<0>(*query_results.begin());
}

int SqliteStorage::read_db_schema_version()
{
  int schema_version = -1;
  if (database_->table_exists("schema")) {
    // Read schema version
    auto statement = database_->prepare_statement("SELECT schema_version from schema;");
    auto query_results = statement->execute_query<int>();
    schema_version = std::get<0>(*query_results.begin());
  } else {
    if (database_->field_exists("topics", "offered_qos_profiles")) {
      schema_version = 2;
    } else {
      schema_version = 1;
    }
  }

  return schema_version;
}

uint64_t SqliteStorage::read_total_page_count_locked() const
{
  auto scope_exit_reset_statement = rcpputils::make_scope_exit(
    [this]() {
      page_count_statement_->reset();
    });
  auto page_count_query_result = page_count_statement_->execute_query<int>();
  auto page_count_query_result_begin = page_count_query_result.begin();
  if (page_count_query_result_begin == page_count_query_result.end()) {
    throw SqliteException{"Error. PRAGMA page_count return no result."};
  }
  return std::get<0>(*page_count_query_result_begin);
}

uint64_t SqliteStorage::get_page_size() const
{
  if (!database_) {
    return 0;  // Trying to call get_page_size when SqliteStorage::open was not called or db
    // failed to open
  }
  SqliteStatement get_page_size_statement = database_->prepare_statement("PRAGMA page_size;");

  auto page_size_query_result = get_page_size_statement->execute_query<int>();
  auto page_size_query_result_begin = page_size_query_result.begin();
  if (page_size_query_result_begin == page_size_query_result.end()) {
    throw SqliteException{"Error. PRAGMA page_size return no result."};
  }
  return std::get<0>(*page_size_query_result_begin);
}

uint16_t SqliteStorage::get_extern_topic_id(int64_t inner_topic_id) const
{
  auto iter = inner_to_extern_topic_id_map_.find(inner_topic_id);
  if (iter != inner_to_extern_topic_id_map_.end()) {
    return iter->second;
  } else {
    return 0;  // 0 corresponds to the invalid topic_id
  }
}

uint16_t SqliteStorage::get_or_generate_extern_topic_id(int64_t inner_topic_id)
{
  uint16_t extern_topic_id = get_extern_topic_id(inner_topic_id);
  if (extern_topic_id == 0) {
    if (last_extern_topic_id_ == std::numeric_limits<uint16_t>::max()) {
      ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_ERROR_STREAM(
        "External topic_id reached maximum allowed value" <<
          std::to_string(std::numeric_limits<uint16_t>::max()));
      throw std::range_error("External topic_id reached maximum allowed value");
    }
    last_extern_topic_id_.fetch_add(1, std::memory_order_relaxed);
    extern_topic_id = last_extern_topic_id_;
    inner_to_extern_topic_id_map_[inner_topic_id] = extern_topic_id;
  }
  return extern_topic_id;
}

}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::SqliteStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
