
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mecanumbot_controller/mecanumbot_drive_controller.hpp"

PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::controller::MecanumbotDriveController,
    controller_interface::ControllerInterface
)

using namespace debict::mecanumbot::controller;

MecanumbotDriveController::MecanumbotDriveController()
    : controller_interface::ControllerInterface()
    , velocity_command_subsciption_(nullptr)
    , velocity_command_ptr_(nullptr)
{

}

controller_interface::InterfaceConfiguration MecanumbotDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.push_back(r_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(l_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumbotDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.push_back(r_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(l_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_init()
{
    try {
        auto node = get_node();
        
    }
    catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumbotDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Get the last velocity command
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (!velocity_command || !(*velocity_command)) {
        return controller_interface::return_type::OK;
    }
    std::cout << "velocity_command angular z: " << (*velocity_command)->twist.angular.z << std::endl;
    std::cout << "velocity_command linear x: " << (*velocity_command)->twist.linear.x << std::endl;
    std::cout << "velocity_command linear y: " << (*velocity_command)->twist.linear.y << std::endl;

    // Calculate the wheel velocity
    // See: http://robotsforroboticists.com/drive-kinematics/
    const auto twist = (*velocity_command)->twist;
    double r_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double l_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);

    r_wheel_->set_velocity(r_wheel_velocity);
    l_wheel_->set_velocity(l_wheel_velocity);
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure MecanumbotDriverController");

    r_wheel_joint_name_ = get_node()->get_parameter("r_wheel_joint_name").as_string();
    l_wheel_joint_name_ = get_node()->get_parameter("l_wheel_joint_name").as_string();
    if (r_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'r_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (l_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'l_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
    wheel_distance_length_ = get_node()->get_parameter("wheel_distance.length").as_double();
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_radius' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_width_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.width' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_length_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.length' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    wheel_separation_width_ = wheel_distance_width_ / 2;
    wheel_separation_length_ = wheel_distance_length_ / 2;

    velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/cmd_vel", 10, [this](const Twist::SharedPtr twist)
    {
        velocity_command_ptr_.writeFromNonRT(twist);
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the wheels
    r_wheel_ = get_wheel(r_wheel_joint_name_);
    l_wheel_ = get_wheel(l_wheel_joint_name_);
    if (!r_wheel_ || !l_wheel_) {
        return controller_interface::CallbackReturn::ERROR;
    }
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
    // Uninitialize the wheels
    r_wheel_.reset();
    l_wheel_.reset();

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_error(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumbotDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

std::shared_ptr<MecanumbotWheel> MecanumbotDriveController::get_wheel(const std::string & wheel_joint_name)
{
    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        std::cout << "interface.get_name(): " << interface.get_name() << std::endl;
        return interface.get_name() == wheel_joint_name+"/velocity" && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Create the wheel instance
    return std::make_shared<MecanumbotWheel>(
        std::ref(*velocity_state),
        std::ref(*velocity_command)
        );
}
