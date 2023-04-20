
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "mecanumbot_hardware/mecanumbot_hardware.hpp"

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::hardware::MecanumbotHardware,
    hardware_interface::SystemInterface
)

using namespace debict::mecanumbot::hardware;

hardware_interface::CallbackReturn MecanumbotHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    hardware_interface::CallbackReturn baseResult = hardware_interface::SystemInterface::on_init(hardware_info);
    if (baseResult != hardware_interface::CallbackReturn::SUCCESS) {
        return baseResult;
    }

    serial_port_name_ = info_.hardware_parameters["serial_port"];
    motor_ids_.resize(info_.joints.size());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    for (hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid number of state interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MecanumbotHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumbotHardware::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
            )
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumbotHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
            )
        );
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware starting ...");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

    serial_port_ = std::make_shared<MecanumbotSerialPort>();
    if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware failed to open serial port");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware started");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumbotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopping ...");

    if (serial_port_->is_open()) {
        serial_port_->close();
        serial_port_.reset();
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "Mecanumbot hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumbotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

    // We currently have an ack response, so readread_frames the frames
    char message[1024];
    serial_port_->read_frames(message);
    
    try {
        auto json = json::parse(message);
        // iterate over the json object
        for (json::iterator it = json.begin(); it != json.end(); ++it) {
            // print key and value
            std::cout << it.key() << " : " << it.value() << "\n";
        }
        // Put real velocity in there only for debuging
        // velocity_states_[0] = json.at("time");
    } catch (json::parse_error& e) {
        // output exception information
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), "Error parsing JSON: %s", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("MecanumbotHardware"), "Message received: %s", message);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumbotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Only send motor commands if the velocity changed
        if (velocity_commands_[i] != velocity_commands_saved_[i]) {

            // Create the motor command message as a JSON string 
            std::string message = "{\"right\":" + std::to_string(velocity_commands_[0]) + 
                                    ",\"left\":" + std::to_string(velocity_commands_[1])+ "}";

            // write velocity with only two decimal

            // string to char array
            char message_char[message.length() + 1];
            strcpy(message_char, message.c_str());
            message[message.length()] = '\0';
            
            // Send the motor command
            serial_port_->write_frame(message_char);

            // Store the current velocity
            velocity_commands_saved_[i] = velocity_commands_[i];
        }
    }
    std::cout << "Velocity commands saved: " << velocity_commands_saved_[0] << ", " << velocity_commands_saved_[1] << std::endl;
    
    return hardware_interface::return_type::OK;
}
