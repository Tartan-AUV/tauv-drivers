/**
 * @file   dvl-sensor.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 */

#include "dvl_a50/dvl-sensor.hpp"

#include <cerrno>
#include <cstring>

namespace dvl_sensor {

DVL_A50::DVL_A50() : Node("dvl_a50_node") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos =
        rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

    // Polls the socket without blocking; every complete line is handled each tick.
    timer_receive = this->create_wall_timer(std::chrono::milliseconds(5),
                                            std::bind(&DVL_A50::handle_receive, this));

    // Publishers
    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", qos);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", qos);
    dvl_pub_config_status =
        this->create_publisher<dvl_msgs::msg::ConfigStatus>("dvl/config/status", qos);
    dvl_pub_command_response =
        this->create_publisher<dvl_msgs::msg::CommandResponse>("dvl/command/response", qos);
    dvl_sub_config_command = this->create_subscription<
        dvl_msgs::msg::ConfigCommand>("dvl/config/command",
                                      qos,
                                      std::bind(&DVL_A50::command_subscriber, this, _1));

    this->declare_parameter<std::string>("dvl_ip_address", "192.168.194.95");
    this->declare_parameter<std::string>("velocity_frame_id", "dvl_A50/velocity_link");
    this->declare_parameter<std::string>("position_frame_id", "dvl_A50/position_link");
    this->declare_parameter<bool>("acoustic_enabled", false);
    // Stamp velocity reports at the ping (receive time minus the DVL's own
    // transmission - validity delay, ~110 ms) instead of at arrival. Only useful
    // with the EKF's smooth_lagged_data, so off by default.
    stamp_at_ping_ = this->declare_parameter<bool>("stamp_at_ping", false);

    velocity_frame_id = this->get_parameter("velocity_frame_id").as_string();
    position_frame_id = this->get_parameter("position_frame_id").as_string();
    ip_address = this->get_parameter("dvl_ip_address").as_string();
    acoustic_enabled_ = this->get_parameter("acoustic_enabled").as_bool();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());

    //--- TCP/IP SOCKET ----
    tcpSocket = new TCPSocket((char*)ip_address.c_str(), 16171);

    // If the DVL is not reachable yet, the receive timer keeps retrying.
    tryConnect();
}

DVL_A50::~DVL_A50() {
    if (connected_) {
        this->set_json_parameter("acoustic_enabled", "false");
    }
    tcpSocket->Close();
    delete tcpSocket;
}

bool DVL_A50::tryConnect() {
    last_connect_attempt_ = std::chrono::steady_clock::now();
    tcpSocket->Close();
    if (tcpSocket->Create() < 0) {
        RCLCPP_ERROR(get_logger(), "Error creating the socket");
        return false;
    }
    std::string error;
    int error_code = 0;
    if (tcpSocket->Connect(1000, error, error_code) != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "DVL not reachable at %s (%s), retrying every 2 s",
                             ip_address.c_str(), error.c_str());
        return false;
    }
    connected_ = true;
    fault = 0;
    rx_buffer_.clear();
    RCLCPP_INFO(get_logger(), "DVL-A50 connected!");
    this->set_json_parameter("acoustic_enabled", acoustic_enabled_ ? "true" : "false");
    return true;
}

void DVL_A50::dropConnection(const std::string& reason) {
    RCLCPP_ERROR(get_logger(), "DVL connection lost (%s), reconnecting", reason.c_str());
    connected_ = false;
    fault = 1;
    tcpSocket->Close();
}

void DVL_A50::handle_receive() {
    if (!connected_) {
        if (std::chrono::steady_clock::now() - last_connect_attempt_ >= std::chrono::seconds(2)) {
            tryConnect();
        }
        return;
    }

    char buf[4096];
    while (true) {
        const ssize_t n = tcpSocket->ReceiveAvailable(buf, sizeof(buf));
        if (n > 0) {
            rx_buffer_.append(buf, static_cast<size_t>(n));
            continue;
        }
        if (n == 0) {
            dropConnection("closed by the DVL");
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            break;
        }
        dropConnection(strerror(errno));
        return;
    }

    size_t pos;
    while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
        const std::string line = rx_buffer_.substr(0, pos);
        rx_buffer_.erase(0, pos + 1);
        if (!line.empty()) {
            handle_line(line);
        }
    }
    // A sane report is well under 4 kB; drop runaway garbage without a newline.
    if (rx_buffer_.size() > 65536) {
        rx_buffer_.clear();
    }
}

void DVL_A50::handle_line(const std::string& line) {
    try {
        json_data = json::parse(line);

        if (json_data.contains("altitude")) {
            this->publish_vel_trans_report();
        } else if (json_data.contains("pitch")) {
            this->publish_dead_reckoning_report();
        } else if (json_data.contains("response_to")) {
            if (json_data["response_to"] == "set_config" ||
                json_data["response_to"] == "calibrate_gyro" ||
                json_data["response_to"] == "reset_dead_reckoning")
                this->publish_command_response();
            else if (json_data["response_to"] == "get_config")
                this->publish_config_status();
        }
    } catch (std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Bad DVL message: %s", e.what());
    }
}

/*
 * Publish velocity and transductors report
 */
void DVL_A50::publish_vel_trans_report() {
    // DVL message struct
    dvl_msgs::msg::DVLBeam beam0;
    dvl_msgs::msg::DVLBeam beam1;
    dvl_msgs::msg::DVLBeam beam2;
    dvl_msgs::msg::DVLBeam beam3;
    dvl_msgs::msg::DVL dvl;

    const rclcpp::Time received = Node::now();
    dvl.header.stamp = received;
    dvl.header.frame_id = velocity_frame_id;

    dvl.time = double(json_data["time"]);
    dvl.time_of_validity = json_data["time_of_validity"].get<int64_t>();
    dvl.time_of_transmission = json_data["time_of_transmission"].get<int64_t>();
    if (stamp_at_ping_) {
        // Both fields come from the DVL clock, so their difference (~110 ms) is the
        // ping-to-report delay regardless of whether that clock is synced.
        const int64_t delay_us = dvl.time_of_transmission - dvl.time_of_validity;
        if (delay_us >= 0 && delay_us < 1000000) {
            dvl.header.stamp = received - rclcpp::Duration::from_nanoseconds(delay_us * 1000);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Implausible DVL report delay %ld us, stamping at receipt", delay_us);
        }
    }

    dvl.velocity.x = double(json_data["vx"]);
    dvl.velocity.y = double(json_data["vy"]);
    dvl.velocity.z = double(json_data["vz"]);
    dvl.fom = double(json_data["fom"]);
    double current_altitude = double(json_data["altitude"]);
    dvl.velocity_valid = json_data["velocity_valid"];

    // Removed logic to add old altitude if not valid. Does not report accurately what is coming
    // from the DVL. Logic should be implemented on a case by case basis.
    dvl.altitude = current_altitude;

    dvl.status = json_data["status"];
    dvl.form = json_data["format"];

    // Add covariance from message
    std::vector<double> twistCovariance;

    if (json_data.contains("covariance") && json_data["covariance"].is_array()) {
        const auto& matrix = json_data["covariance"];

        for (const auto& row : matrix) {
            if (!row.is_array()) continue;
            for (const auto& value : row) {
                twistCovariance.push_back(value.get<double>());
            }
        }
    }

    dvl.covariance = twistCovariance;

    beam0.id = json_data["transducers"][0]["id"];
    beam0.velocity = double(json_data["transducers"][0]["velocity"]);
    beam0.distance = double(json_data["transducers"][0]["distance"]);
    beam0.rssi = double(json_data["transducers"][0]["rssi"]);
    beam0.nsd = double(json_data["transducers"][0]["nsd"]);
    beam0.valid = json_data["transducers"][0]["beam_valid"];

    beam1.id = json_data["transducers"][1]["id"];
    beam1.velocity = double(json_data["transducers"][1]["velocity"]);
    beam1.distance = double(json_data["transducers"][1]["distance"]);
    beam1.rssi = double(json_data["transducers"][1]["rssi"]);
    beam1.nsd = double(json_data["transducers"][1]["nsd"]);
    beam1.valid = json_data["transducers"][1]["beam_valid"];

    beam2.id = json_data["transducers"][2]["id"];
    beam2.velocity = double(json_data["transducers"][2]["velocity"]);
    beam2.distance = double(json_data["transducers"][2]["distance"]);
    beam2.rssi = double(json_data["transducers"][2]["rssi"]);
    beam2.nsd = double(json_data["transducers"][2]["nsd"]);
    beam2.valid = json_data["transducers"][2]["beam_valid"];

    beam3.id = json_data["transducers"][3]["id"];
    beam3.velocity = double(json_data["transducers"][3]["velocity"]);
    beam3.distance = double(json_data["transducers"][3]["distance"]);
    beam3.rssi = double(json_data["transducers"][3]["rssi"]);
    beam3.nsd = double(json_data["transducers"][3]["nsd"]);
    beam3.valid = json_data["transducers"][3]["beam_valid"];

    dvl.beams = {beam0, beam1, beam2, beam3};
    dvl_pub_report->publish(dvl);
}

/*
 * Publish dead reckoning
 */
void DVL_A50::publish_dead_reckoning_report() {
    dvl_msgs::msg::DVLDR DVLDeadReckoning;
    // std::cout << std::setw(4) << json_data << std::endl;
    DVLDeadReckoning.header.stamp = Node::now();
    DVLDeadReckoning.header.frame_id = position_frame_id;
    DVLDeadReckoning.time = double(json_data["ts"]);
    DVLDeadReckoning.position.x = double(json_data["x"]);
    DVLDeadReckoning.position.y = double(json_data["y"]);
    DVLDeadReckoning.position.z = double(json_data["z"]);
    DVLDeadReckoning.pos_std = double(json_data["std"]);
    DVLDeadReckoning.roll = double(json_data["roll"]);
    DVLDeadReckoning.pitch = double(json_data["pitch"]);
    DVLDeadReckoning.yaw = double(json_data["yaw"]);
    DVLDeadReckoning.type = json_data["type"];
    DVLDeadReckoning.status = json_data["status"];
    DVLDeadReckoning.format = json_data["format"];
    dvl_pub_pos->publish(DVLDeadReckoning);
}

/*
 * Publish the command response
 */
void DVL_A50::publish_command_response() {
    dvl_msgs::msg::CommandResponse command_resp;
    command_resp.response_to = json_data["response_to"];
    command_resp.success = json_data["success"];
    command_resp.error_message = json_data["error_message"];
    command_resp.result = 0;
    command_resp.format = json_data["format"];
    command_resp.type = json_data["type"];
    dvl_pub_command_response->publish(command_resp);
}

/*
 * Publish the sensor current configuration
 */
void DVL_A50::publish_config_status() {
    dvl_msgs::msg::ConfigStatus status_msg;
    status_msg.response_to = json_data["response_to"];
    status_msg.success = json_data["success"];
    status_msg.error_message = json_data["error_message"];
    status_msg.speed_of_sound = json_data["result"]["speed_of_sound"];
    status_msg.acoustic_enabled = json_data["result"]["acoustic_enabled"];
    status_msg.dark_mode_enabled = json_data["result"]["dark_mode_enabled"];
    status_msg.mounting_rotation_offset = json_data["result"]["mounting_rotation_offset"];
    status_msg.range_mode = json_data["result"]["range_mode"];
    status_msg.format = json_data["format"];
    status_msg.type = json_data["type"];
    dvl_pub_config_status->publish(status_msg);
}

void DVL_A50::command_subscriber(const dvl_msgs::msg::ConfigCommand::SharedPtr msg) {
    if (msg->command == "set_config")
        this->set_json_parameter(msg->parameter_name, msg->parameter_value);
    else if (msg->command == "get_config") {
        json command = {{"command", "get_config"}};
        this->send_parameter_to_sensor(command);
    } else if (msg->command == "calibrate_gyro") {
        json command = {{"command", "calibrate_gyro"}};
        this->send_parameter_to_sensor(command);
    } else if (msg->command == "reset_dead_reckoning") {
        json command = {{"command", "reset_dead_reckoning"}};
        this->send_parameter_to_sensor(command);
    }
}

DVL_Parameters DVL_A50::resolveParameter(std::string param) {
    if (param == "speed_of_sound")
        return speed_of_sound;
    else if (param == "acoustic_enabled")
        return acoustic_enabled;
    else if (param == "dark_mode_enabled")
        return dark_mode_enabled;
    else if (param == "mountig_rotation_offset")
        return mountig_rotation_offset;
    else if (param == "range_mode")
        return range_mode;

    return invalid_param;
}

/*
 * Create JSON command message to set parameter in the DVL sensor
 */
void DVL_A50::set_json_parameter(const std::string name, const std::string value) {
    json message;
    message["command"] = "set_config";

    switch (resolveParameter(name)) {
        case speed_of_sound:
            try {
                message["parameters"]["speed_of_sound"] = (int)std::stoi(value);
                this->send_parameter_to_sensor(message);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case acoustic_enabled:
            try {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["acoustic_enabled"] = data;
                this->send_parameter_to_sensor(message);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case dark_mode_enabled:
            try {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["dark_mode_enabled"] = data;
                this->send_parameter_to_sensor(message);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case mountig_rotation_offset:
            try {
                message["parameters"]["mountig_rotation_offset"] = (double)std::stod(value);
                this->send_parameter_to_sensor(message);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case range_mode:
            try {
                message["parameters"]["range_mode"] = value;
                this->send_parameter_to_sensor(message);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Invalid parameter!");
            break;
    }
}

/*
 * Send parameter to the sensor using the JSON command
 */
void DVL_A50::send_parameter_to_sensor(const json& message) {
    std::string str = message.dump();
    char* c = &*str.begin();
    tcpSocket->Send(c);
}

}  // namespace dvl_sensor

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dvl_sensor::DVL_A50>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
