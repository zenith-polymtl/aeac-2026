#ifndef WATER_WEB_SERVER_NODE_HPP_
#define WATER_WEB_SERVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <thread>
#include <fstream>
#include <string>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/string.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/file_body.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
// #include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include "custom_interfaces/msg/ui_message.hpp"
#include "custom_interfaces/msg/gimbal_state.hpp"
#include "custom_interfaces/msg/drone_health.hpp"
#include <memory>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

namespace fs = std::filesystem;

const std::string WEB_COMPONENT_FOLDER = "/web_components/";
const int SERVER_PORT = 8080;

// API routes
const std::string API_MISSION_GO = "/api/mission/go";
const std::string API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const std::string API_AUTO_APPROACH = "/api/mission/auto_approach";
const std::string API_AUTO_SHOOT = "/api/mission/auto_shoot";
const std::string API_SHOOT = "/api/mission/shoot";
const std::string TAKE_PICTURE = "/api/mission/take_picture";
const std::string API_ABORT_ALL = "/api/mission/abort_all";
const std::string API_GIMBAL_FOLLOW = "/api/gimbal_follow";
const std::string API_GIMBAL_LOCK = "/api/gimbal_lock";


namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using UiMessage = custom_interfaces::msg::UiMessage;
using GimbalState = custom_interfaces::msg::GimbalState;
// using Image = sensor_msgs::msg::Image;
using Image = sensor_msgs::msg::CompressedImage;
using DroneHealth = custom_interfaces::msg::DroneHealth;

class WaterWebServerNode : public rclcpp::Node
{
public:
    WaterWebServerNode();
    ~WaterWebServerNode();

    void run_server();

private:
    void initalize_parameters();
    void initalize_variables();
    void initialize_publisher();
    void initialize_subscriber();

    // Server Configuration
    boost::beast::string_view mime_type(boost::beast::string_view path);

    std::string path_cat(boost::beast::string_view base, boost::beast::string_view path);

    void do_session(boost::asio::ip::tcp::socket socket, std::string const &doc_root);

    // Request Handeling
    http::response<http::string_body> handle_request(std::string const &doc_root, http::request<http::string_body> &&req);

    std::optional<http::response<http::string_body>> try_handle_api(beast::string_view target, const http::request<http::string_body> &req);

    std::optional<http::response<http::string_body>> try_serve_static_file(std::string const &doc_root, beast::string_view target, const http::request<http::string_body> &req);

    http::response<http::string_body> generate_responce(std::string message, const http::request<http::string_body> &req);

    
    // Socket functions
    void broadcast_status();
    void broadcast_message(const UiMessage msg);
    void picture_callback(const Image msg);
    void heartbeat_timer_callback();
    void send_connection_notification(const bool ignore_log = false);
    void on_client_connection();

    void drone_heartbeat_callback(const DroneHealth);
    void send_log(bool is_success, std::string message);
    void send_notification(const nlohmann::json status_json);


    // Variables
    std::set<websocket::stream<tcp::socket> *> ws_sessions_;
    std::mutex ws_mutex_;
    std::atomic<bool> running_{true};

    std::string current_status_ = "IDLE";

    std::string package_share_dir_;
    std::thread server_thread_;

    std::string drone_heartbeat_topic_;
    double drone_heartbeat_frequency_;
    int heartbeat_drone_failure_threashold_;
    int missed_drone_heartbeat_ = 0;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    bool drone_is_connected_ = false;
    bool zed_is_connected_ = false;
    
	rclcpp::Subscription<UiMessage>::SharedPtr message_to_ui_subsciber_;
    rclcpp::Subscription<GimbalState>::SharedPtr gimbal_state_subscriber_;
    rclcpp::Subscription<Image>::SharedPtr picture_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_go_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_approach_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_shoot_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr take_picutre_publisher_;

    rclcpp::Subscription<DroneHealth>::SharedPtr drone_heartbeat_subsciber_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_to_scene_publisher_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_all_mission_publisher_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gimbal_mode_publisher_;

    boost::asio::io_context ioc_;
    boost::asio::ip::tcp::acceptor acceptor_;
};

#endif // WATER_WEB_SERVER_NODE_HPP_