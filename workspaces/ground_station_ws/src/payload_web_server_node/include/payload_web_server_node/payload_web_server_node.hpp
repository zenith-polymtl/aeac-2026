#ifndef WATER_WEB_SERVER_NODE_HPP_
#define WATER_WEB_SERVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <thread>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/string.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/file_body.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include "custom_interfaces/msg/ui_message.hpp"
#include "custom_interfaces/msg/drone_health.hpp"
#include "custom_interfaces/msg/servo_control.hpp"
#include "custom_interfaces/srv/servo_state.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <memory>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

namespace fs = std::filesystem;


const std::string WEB_COMPONENT_FOLDER = "/web_components/";
const int SERVER_PORT = 8080;

// API routes
const std::string API_MISSION_GO = "/api/mission/go";
const std::string API_START_LAP = "/api/mission/lap/start";
const std::string API_FINISH_LAP = "/api/mission/lap/finish";
const std::string API_STOP_LAP = "/api/mission/lap/stop";
const std::string API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const std::string API_SERVO = "/api/mission/servo";
const std::string API_TAKE_PICTURE = "/api/mission/take_picture";
const std::string API_CONFIRM_DESCRIPTION = "/api/confirm_description";
const std::string API_SITE_DESCRIPTION = "/api/site_description";
const std::string API_ABORT_ALL = "/api/mission/abort_all";


namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using UiMessage = custom_interfaces::msg::UiMessage;
using DroneHealth = custom_interfaces::msg::DroneHealth;
using ServoState = custom_interfaces::srv::ServoState;
using json = nlohmann::json;
using ServoControl = custom_interfaces::msg::ServoControl;
using Image = sensor_msgs::msg::CompressedImage;


struct ServoCommand
{
    int servo_num;
    int pwm;
};

class PayloadWebServerNode : public rclcpp::Node
{
public:
    PayloadWebServerNode();
    ~PayloadWebServerNode();

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

    // CallBack
    void ui_message_callback(const UiMessage msg);
    void drone_heartbeat_callback(const DroneHealth msg);
    void heartbeat_timer_callback();
    void lap_time_callback(const std_msgs::msg::Int32 msg);
    void time_left_callback(const std_msgs::msg::Int32 msg);
    void time_left_timer_callback();
    void picture_callback(const Image msg);

    // Socket functions
    void broadcast_status();
    void send_notification(const nlohmann::json status_json);
    void on_client_connection();
    void send_connection_notification(const bool ignore_log=false);
    void send_log(bool is_success, std::string message);
    void send_client_latest_picture();


    // Variables
    std::set<websocket::stream<tcp::socket> *> ws_sessions_;
    std::mutex ws_mutex_;
    std::atomic<bool> running_{true};

    ServoControl parse_servo_command(const std::string& body);

    std::string current_status_ = "IDLE";

    std::string package_share_dir_;
    std::thread server_thread_;

    std::string drone_heartbeat_topic_;
    double drone_heartbeat_frequency_;
    int heartbeat_drone_failure_threashold_;
    int missed_drone_heartbeat_ = 0;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr time_left_timer_;
    bool drone_is_connected_ = false;
    bool zed_is_connected_ = false;
    int time_left_ = 0;
    double mean_lap_time_ = 0.0;
    int completed_laps_ = 0;
    int last_lap_time_ = 0;
    std::queue<std::string> scene_images;
    std::ofstream notes_file_;
    std::mutex notes_mutex_;
    
	rclcpp::Subscription<UiMessage>::SharedPtr message_to_ui_subsciber_;
    rclcpp::Subscription<DroneHealth>::SharedPtr drone_heartbeat_subsciber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_time_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr time_left_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_go_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_lap_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_lap_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_stop_now_publisher_;
    rclcpp::Publisher<ServoControl>::SharedPtr servo_control_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_to_scene_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr describe_scene_publisher_;
    rclcpp::Subscription<Image>::SharedPtr picture_subscriber_;

    // rclcpp::Client<ServoState>::SharedPtr servo_client_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_all_mission_publisher_;

    boost::asio::io_context ioc_;
    boost::asio::ip::tcp::acceptor acceptor_;
};

#endif // WATER_WEB_SERVER_NODE_HPP_