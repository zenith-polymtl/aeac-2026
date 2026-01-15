#ifndef WEB_SERVER_NODE_HPP_
#define WEB_SERVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/bool.hpp"
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
#include <memory>

const std::string WEB_COMPONENT_FOLDER = "/web_components/";
const int SERVER_PORT = 8080;

// API routes
const std::string API_MISSION_GO = "/api/mission/go";
const std::string API_LAP = "/api/mission/lap";

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class WebServerNode : public rclcpp::Node
{
public:
    WebServerNode();
    ~WebServerNode();

    void initialize_publisher();
    void run_server();

private:
    // Server Configuration
    boost::beast::string_view mime_type(boost::beast::string_view path);

    std::string path_cat(boost::beast::string_view base, boost::beast::string_view path);

    void do_session(boost::asio::ip::tcp::socket socket, std::string const &doc_root);

    // Request Handeling
    http::response<http::string_body> handle_request(std::string const &doc_root, http::request<http::string_body> &&req);

    std::optional<http::response<http::string_body>> try_handle_api(beast::string_view target, const http::request<http::string_body> &req);

    std::optional<http::response<http::string_body>> try_serve_static_file(std::string const &doc_root, beast::string_view target, const http::request<http::string_body> &req);

    // Socket functions
    void broadcast_status();

    // Variables
    std::set<websocket::stream<tcp::socket> *> ws_sessions_;
    std::mutex ws_mutex_;
    std::atomic<bool> running_{true};

    std::string current_status_ = "IDLE";

    std::string package_share_dir_;
    std::thread server_thread_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_go_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lap_publisher_;

    boost::asio::io_context ioc_;
    boost::asio::ip::tcp::acceptor acceptor_;
};

#endif // WEB_SERVER_NODE_HPP_