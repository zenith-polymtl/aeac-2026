#include "water_web_server_node/water_web_server_node.hpp"

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

WaterWebServerNode::WaterWebServerNode() : Node("water_web_server_node"),
                                 ioc_(1),
                                 acceptor_(ioc_, {net::ip::make_address("0.0.0.0"), SERVER_PORT})
{
    initialize_publisher();
    initialize_subscriber();
    package_share_dir_ = ament_index_cpp::get_package_share_directory("water_web_server_node");
    server_thread_ = std::thread([this]()
                                 { run_server(); });
}

WaterWebServerNode::~WaterWebServerNode()
{
    acceptor_.close();
    if (server_thread_.joinable())
    {
        server_thread_.join();
    }
}

void WaterWebServerNode::initialize_publisher()
{
    mission_go_publisher_ = create_publisher<std_msgs::msg::Bool>("/mission/go", 10);
    start_lap_publisher_ = create_publisher<std_msgs::msg::Bool>("/mission/control_nav/lap/start", 10);
    finish_lap_publisher_ = create_publisher<std_msgs::msg::Bool>("/mission/control_nav/lap/finish", 10);
    move_to_scene_publisher_ = create_publisher<std_msgs::msg::Bool>("/mission/control_nav/move_to_scene", 10);
    abort_all_mission_publisher_ = create_publisher<std_msgs::msg::Bool>("/mission/abort_all", 10);
}

void WaterWebServerNode::initialize_subscriber() 
{
    message_to_ui_subsciber_ = create_subscription<UiMessage>(
		"/send_to_ui", 10, std::bind(&WaterWebServerNode::broadcast_message, this, std::placeholders::_1));
}

void WaterWebServerNode::broadcast_message(const UiMessage msg)
{
    nlohmann::json status_json = {
        {"is_success", msg.is_success},
        {"message", msg.message},
    };

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

void WaterWebServerNode::run_server()
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

beast::string_view WaterWebServerNode::mime_type(beast::string_view path)
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

std::string WaterWebServerNode::path_cat(beast::string_view base, beast::string_view path)
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
WaterWebServerNode::handle_request(
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

std::optional<http::response<http::string_body>>
WaterWebServerNode::try_handle_api(
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
    if (target == API_MOVE_TO_SCENE)
    {
        RCLCPP_INFO(get_logger(), "Received Scene movement command!");
        std_msgs::msg::Bool msg;
        msg.data = true;
        move_to_scene_publisher_->publish(msg);

        return generate_responce("Request Scene movement received", req);
    }
    if (target == API_AUTO_APPROACH)
    {
        RCLCPP_INFO(get_logger(), "Received Auto Approach!");

        // TODO: Add Auto Approach Logic

        return generate_responce("Request Auto Approach received", req);
    }
    if (target == API_AUTO_SHOOT)
    {
        RCLCPP_INFO(get_logger(), "Received Auto Shoot!");

        // TODO: Add Auto Shoot Logic

        return generate_responce("Request Auto Shoot received", req);
    }
    if (target == API_SHOOT)
    {
        RCLCPP_INFO(get_logger(), "Received shoot!");

        // TODO: Add Shoot Logic

        return generate_responce("Request Shoot received", req);
    }
    if (target == TAKE_PICTURE)
    {
        RCLCPP_INFO(get_logger(), "Received Take Picture!");
        
        // TODO: Add Toggle Gimbal Logic

        return generate_responce("Request Taje Picture received", req);

    }
    if (target == API_GIMBAL_TOGGLE)
    {
        RCLCPP_INFO(get_logger(), "Received Toggle Gimbal!");
        
        // TODO: Add Toggle Gimbal Logic

        return generate_responce("Request Toggle Gimbal received", req);
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

http::response<http::string_body> WaterWebServerNode::generate_responce(std::string message, const http::request<http::string_body> &req) {
    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(http::field::content_type, "application/json");
    res.body() = "{\"message\": \"" + message + "\"}";
    res.prepare_payload();
    res.keep_alive(req.keep_alive());
    return res;
}

std::optional<http::response<http::string_body>> WaterWebServerNode::try_serve_static_file(
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

void WaterWebServerNode::do_session(tcp::socket socket, std::string const &doc_root)
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

void WaterWebServerNode::broadcast_status()
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
    auto node = std::make_shared<WaterWebServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
