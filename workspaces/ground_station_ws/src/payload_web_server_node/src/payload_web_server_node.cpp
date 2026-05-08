#include "payload_web_server_node/payload_web_server_node.hpp"

namespace /* anonymous */
{
    // These functions create responses and can be reused anywhere
    http::response<http::string_body> make_bad_request(
        const http::request<http::string_body> &req,
        beast::string_view why)
    {
        http::response<http::string_body> res{http::status::bad_request, req.version()};
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "text/html");
        res.keep_alive(req.keep_alive());
        res.body() = std::string(why);
        res.prepare_payload();
        return res;
    }

    http::response<http::string_body> make_not_found(
        const http::request<http::string_body> &req,
        beast::string_view target)
    {
        http::response<http::string_body> res{http::status::not_found, req.version()};
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "text/html");
        res.keep_alive(req.keep_alive());
        res.body() = "The resource '" + std::string(target) + "' was not found.";
        res.prepare_payload();
        return res;
    }

    http::response<http::string_body> make_server_error(
        const http::request<http::string_body> &req,
        beast::string_view what)
    {
        http::response<http::string_body> res{http::status::internal_server_error, req.version()};
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "text/html");
        res.keep_alive(req.keep_alive());
        res.body() = "An error occurred: '" + std::string(what) + "'";
        res.prepare_payload();
        return res;
    }
} // anonymous namespace

PayloadWebServerNode::PayloadWebServerNode() : Node("payload_web_server_node"),
                                 ioc_(1),
                                 acceptor_(ioc_, {net::ip::make_address("0.0.0.0"), SERVER_PORT})
{
    initalize_parameters();
    initalize_variables();
    initialize_publisher();
    initialize_subscriber();
    package_share_dir_ = ament_index_cpp::get_package_share_directory("payload_web_server_node");
    server_thread_ = std::thread([this]()
                                 { run_server(); });
}

PayloadWebServerNode::~PayloadWebServerNode()
{
    acceptor_.close();
    if (server_thread_.joinable())
    {
        server_thread_.join();
    }
}

void PayloadWebServerNode::initalize_parameters()
{
    // Declare parameters
    this->declare_parameter<double>("drone_heartbeat_rate", 1.0); //Hz
    this->declare_parameter<std::string>("drone_heartbeat_topic", "/aeac/external/drone_heartbeat");
    this->declare_parameter<int>("heartbeat_drone_failure_threashold", 3); // Number of missed heartbeat before flagging

    // // Get parameters
    drone_heartbeat_frequency_ = this->get_parameter("drone_heartbeat_rate").as_double();
    drone_heartbeat_topic_ = this->get_parameter("drone_heartbeat_topic").as_string();
    heartbeat_drone_failure_threashold_ = this->get_parameter("heartbeat_drone_failure_threashold").as_int();
}

void PayloadWebServerNode::initalize_variables()
{
    auto period = std::chrono::duration<double>(1.0 / drone_heartbeat_frequency_);
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&PayloadWebServerNode::heartbeat_timer_callback, this)
    );
}

void PayloadWebServerNode::initialize_publisher()
{
    rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    mission_go_publisher_ = create_publisher<std_msgs::msg::Bool>("/aeac/external/mission/go", reliable_qos);
    start_lap_publisher_ = create_publisher<std_msgs::msg::Bool>("/aeac/external/mission/control_nav/lap/start", reliable_qos);
    finish_lap_publisher_ = create_publisher<std_msgs::msg::Bool>("/aeac/external/mission/control_nav/lap/finish", reliable_qos);
    move_to_scene_publisher_ = create_publisher<std_msgs::msg::Bool>("/aeac/external/mission/control_nav/move_to_scene", reliable_qos);
    servo_control_publisher_ = create_publisher<ServoControl>("/aeac/external/payload/toggle_servo", reliable_qos);

    // servo_client_ = create_client<ServoState>("/aeac/external/payload/set_state");

    abort_all_mission_publisher_ = create_publisher<std_msgs::msg::Bool>("/aeac/external/mission/abort_all", reliable_qos);

    // while (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_INFO(this->get_logger(), "Waiting for servo client...");
    // }
}

void PayloadWebServerNode::initialize_subscriber() 
{
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    
    message_to_ui_subsciber_ = create_subscription<UiMessage>("/aeac/external/UI/display", reliable_qos, std::bind(&PayloadWebServerNode::ui_message_callback, this, std::placeholders::_1));
    drone_heartbeat_subsciber_ = create_subscription<DroneHealth>(drone_heartbeat_topic_, reliable_qos, std::bind(&PayloadWebServerNode::drone_heartbeat_callback, this, std::placeholders::_1));
}

void PayloadWebServerNode::heartbeat_timer_callback()
{
    if (!drone_is_connected_) return;

    missed_drone_heartbeat_++;
    if (missed_drone_heartbeat_ > heartbeat_drone_failure_threashold_)
    {
        drone_is_connected_ = false;
        zed_is_connected_ = false;
        send_connection_notification();
    }
}

void PayloadWebServerNode::ui_message_callback(const UiMessage msg)
{
    send_log(msg.is_success, msg.message);
}

void PayloadWebServerNode::drone_heartbeat_callback(const DroneHealth msg)
{
    bool should_send_notification = false;
    bool ignore_logging = false;
    if (zed_is_connected_ != msg.zed_healthy) 
    {
        should_send_notification = true;
        ignore_logging = true;
    }
    if (!drone_is_connected_)
    {
        should_send_notification = true;
        drone_is_connected_ = true;
        ignore_logging = false;
    }
    zed_is_connected_ = msg.zed_healthy;
    missed_drone_heartbeat_ = 0;

    if (should_send_notification) 
    {
        send_connection_notification(ignore_logging);
    }

}

void PayloadWebServerNode::send_log(bool is_success, std::string message)
{
    nlohmann::json status_json = {
        {"type", "message"},
        {"is_success", is_success},
        {"message", message},
    };

    send_notification(status_json);
}

void PayloadWebServerNode::send_connection_notification(const bool ignore_log)
{
    nlohmann::json status_json = {
        {"type", "connection"},
        {"drone_is_connected", drone_is_connected_},
        {"zed_is_connected", zed_is_connected_}
    };
    send_notification(status_json);
    if (!ignore_log)
        send_log(drone_is_connected_, drone_is_connected_ ? "Drone Connection established" : "Drone Connection lost");
}


void PayloadWebServerNode::send_notification(const nlohmann::json status_json) 
{
    std::string message = status_json.dump();

    std::lock_guard<std::mutex> lock(ws_mutex_);

    for (auto *ws_ptr : ws_sessions_)
    {
        try {
            beast::error_code ec;
            ws_ptr->text(true);
            ws_ptr->write(net::buffer(message), ec);
            if (ec)
            {
                RCLCPP_WARN(get_logger(), "Failed to send to one client: %s", ec.message().c_str());
            }
        }
        catch(...){
            RCLCPP_WARN(get_logger(), "Error Broadcasting");
        }

    }
}

void PayloadWebServerNode::run_server()
{
    std::string doc_root = package_share_dir_ + WEB_COMPONENT_FOLDER;
    std::cout << doc_root.c_str() << std::endl;
    RCLCPP_INFO_STREAM(this->get_logger(), "Starting web server on port http://localhost:" << SERVER_PORT);
    for (;;)
    {
        beast::error_code ec;
        tcp::socket socket(ioc_);
        acceptor_.accept(socket, ec);
        if (ec)
        {
            break;
        }
        std::thread(
            [this, doc_root, socket = std::move(socket)]() mutable
            {
                do_session(std::move(socket), doc_root);
            })
            .detach();
    }
}

beast::string_view PayloadWebServerNode::mime_type(beast::string_view path)
{
    using beast::iequals;
    auto const ext = [&path]
    {
        auto const pos = path.rfind(".");
        if (pos == beast::string_view::npos)
            return beast::string_view{};
        return path.substr(pos);
    }();
    if (iequals(ext, ".htm"))
        return "text/html";
    if (iequals(ext, ".html"))
        return "text/html";
    if (iequals(ext, ".php"))
        return "text/html";
    if (iequals(ext, ".css"))
        return "text/css";
    if (iequals(ext, ".txt"))
        return "text/plain";
    if (iequals(ext, ".js"))
        return "application/javascript";
    if (iequals(ext, ".json"))
        return "application/json";
    if (iequals(ext, ".png"))
        return "image/png";
    if (iequals(ext, ".jpe"))
        return "image/jpeg";
    if (iequals(ext, ".jpeg"))
        return "image/jpeg";
    if (iequals(ext, ".jpg"))
        return "image/jpeg";
    if (iequals(ext, ".gif"))
        return "image/gif";
    if (iequals(ext, ".ico"))
        return "image/vnd.microsoft.icon";
    return "application/text";
}

std::string PayloadWebServerNode::path_cat(beast::string_view base, beast::string_view path)
{
    if (base.empty())
    {
        return std::string(path);
    }
    std::string result(base);
    char constexpr path_separator = '/';
    if (result.back() == path_separator)
        result.resize(result.size() - 1);
    result.append(path.data(), path.size());

    return result;
}

http::response<http::string_body>
PayloadWebServerNode::handle_request(
    std::string const &doc_root,
    http::request<http::string_body> &&req)
{
    // 1. Basic validation
    if (req.method() != http::verb::get && req.method() != http::verb::post)
        return make_bad_request(req, "Only GET and POST methods is supported");

    beast::string_view target = req.target();

    if (target.empty() || target[0] != '/' || target.find("..") != beast::string_view::npos)
        return make_bad_request(req, "Illegal request-target");

    // 2. Try API endpoints first
    if (auto api_response = try_handle_api(target, req))
        return *api_response;

    // 3. Try to serve static file
    if (auto file_response = try_serve_static_file(doc_root, target, req))
        return *file_response;

    // 4. Fallback - not found
    return make_not_found(req, target);
}

ServoControl PayloadWebServerNode::parse_servo_command(const std::string& body)
{
    auto j = json::parse(body);
    ServoControl cmd;
    cmd.servo_num = j.at("servo_num").get<int>();
    cmd.pwm       = j.at("pwm").get<int>();
    return cmd;
}

std::optional<http::response<http::string_body>>
PayloadWebServerNode::try_handle_api(
    beast::string_view target,
    const http::request<http::string_body> &req)
{
    if (target == API_MISSION_GO)
    {
        RCLCPP_INFO(get_logger(), "Mission Go received!");
        std_msgs::msg::Bool msg;
        msg.data = true;
        mission_go_publisher_->publish(msg);

        return generate_responce("Request Mission Go received", req);
    }
    if (target == API_START_LAP)
    {
        RCLCPP_INFO(get_logger(), "Start Lap Received!");

        return generate_responce("Start Lap received", req);
    }
    if (target == API_FINISH_LAP)
    {
        RCLCPP_INFO(get_logger(), "Finish Lap Received!");

        return generate_responce("Finish Lap received", req);
    }
    if (target == API_STOP_LAP)
    {
        RCLCPP_INFO(get_logger(), "Stop Lap Received!");

        return generate_responce("Stop Lap received", req);
    }
    if (target == API_MOVE_TO_SCENE)
    {
        RCLCPP_INFO(get_logger(), "Received Scene movement command!");
        std_msgs::msg::Bool msg;
        msg.data = true;
        move_to_scene_publisher_->publish(msg);

        return generate_responce("Request Scene movement received", req);
    }
    if (target == API_SERVO) 
    {
        RCLCPP_INFO(get_logger(), "Received Servo command!");

        try
        {
            ServoControl cmd = parse_servo_command(req.body());

            RCLCPP_INFO(get_logger(), "Servo: %d, PWM: %d",
                        cmd.servo_num, cmd.pwm);
            servo_control_publisher_->publish(cmd);

            return generate_responce("Servo command received", req);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Invalid JSON: %s", e.what());
            return generate_responce("Invalid JSON", req);
        }
    }
    if (target == API_TAKE_PICTURE)
    {
        RCLCPP_INFO(get_logger(), "Take Picture Received!");

        return generate_responce("Take Picture received", req);
    }
    if (target == API_ABORT_ALL)
    {
        RCLCPP_INFO(get_logger(), "Abort Received!");
        std_msgs::msg::Bool msg;
        msg.data = true;
        abort_all_mission_publisher_->publish(msg);

        return generate_responce("Request Abort received", req);
    }
    return std::nullopt;
}

http::response<http::string_body> PayloadWebServerNode::generate_responce(std::string message, const http::request<http::string_body> &req) {
    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(http::field::content_type, "application/json");
    res.body() = "{\"message\": \"" + message + "\"}";
    res.prepare_payload();
    res.keep_alive(req.keep_alive());
    return res;
}

std::optional<http::response<http::string_body>> PayloadWebServerNode::try_serve_static_file(
    std::string const &doc_root,
    beast::string_view target,
    const http::request<http::string_body> &req)
{
    std::string path = path_cat(doc_root, target);
    if (target == "/")
    {
        path = path_cat(doc_root, "/html/index.html");
    }

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open())
    {
        return std::nullopt;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::string content(size, '\0');
    if (!file.read(&content[0], size))
    {
        return make_server_error(req, "Failed to read file content");
    }

    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(http::field::content_type, mime_type(path));
    res.body() = std::move(content);
    res.prepare_payload();
    res.keep_alive(req.keep_alive());

    return res;
}

void PayloadWebServerNode::on_client_connection()
{
    send_connection_notification(true);
}

void PayloadWebServerNode::do_session(tcp::socket socket, std::string const &doc_root)
{
    beast::error_code ec;
    beast::flat_buffer buffer;
    http::request<http::string_body> req;
    http::read(socket, buffer, req, ec);
    if (ec)
    {
        return;
    }
    if (websocket::is_upgrade(req))
    {
        websocket::stream<tcp::socket> ws{std::move(socket)};
        ws.set_option(websocket::stream_base::decorator([](websocket::response_type &res)
                                                        { res.set(http::field::server, std::string(BOOST_BEAST_VERSION_STRING) + " websocket-server-sync"); }));
        ws.accept(req, ec);
        if (ec)
        {
            return;
        }
        websocket::stream<tcp::socket> *ws_ptr = &ws;
        {
            std::lock_guard<std::mutex> lock(ws_mutex_);
            ws_sessions_.insert(ws_ptr);
        }
        auto cleanup = [&]() {
            std::lock_guard<std::mutex> lock(ws_mutex_);
            ws_sessions_.erase(ws_ptr);
            // RCLCPP_INFO(get_logger(), "WebSocket client disconnected. Total: %zu", ws_sessions_.size());
        };
        struct Guard {
            std::function<void()> fn;
            ~Guard() { if (fn) fn(); }
        } guard{cleanup};

        on_client_connection();

        // RCLCPP_INFO(get_logger(), "New WebSocket client connected. Total: %zu", ws_sessions_.size());
        for (;;)
        {
            beast::flat_buffer ws_buffer;
            ws.read(ws_buffer, ec);
            if (ec == websocket::error::closed)
                break;
            if (ec)
            {
                return;
            }
            ws.text(ws.got_text());
            ws.write(ws_buffer.data(), ec);
            if (ec)
            {
                return;
            }
        }
        ws.close(websocket::close_code::normal, ec);
        if (ec && ec != websocket::error::closed)
        {
            // RCLCPP_WARN(get_logger(), "WebSocket close failed: %s", ec.message().c_str());
        }
    }
    else
    {
        http::response<http::string_body> msg = handle_request(doc_root, std::move(req));
        msg.keep_alive();
        http::write(socket, std::move(msg), ec);
        if (ec)
        {
            return;
        }
    }
    socket.shutdown(tcp::socket::shutdown_send, ec);
}

void PayloadWebServerNode::broadcast_status()
{
    nlohmann::json status_json = {
        {"timestamp", rclcpp::Clock().now().seconds()},
        {"status", current_status_},
    };

    std::string message = status_json.dump();

    std::lock_guard<std::mutex> lock(ws_mutex_);

    for (auto *ws_ptr : ws_sessions_)
    {
        beast::error_code ec;
        ws_ptr->text(true);
        ws_ptr->write(net::buffer(message), ec);
        if (ec)
        {
            RCLCPP_WARN(get_logger(), "Failed to send to one client: %s", ec.message().c_str());
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PayloadWebServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
