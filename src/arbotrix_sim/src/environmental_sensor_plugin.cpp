#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>

namespace gazebo
{

class EnvironmentalSensorPlugin : public ModelPlugin
{
public:
  EnvironmentalSensorPlugin()
  : temperature_(24.0),
    humidity_(35.0)
  {
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    // Read optional values from the SDF.
    if (sdf->HasElement("temperature")) {
      temperature_ = sdf->Get<double>("temperature");
    }

    if (sdf->HasElement("humidity")) {
      humidity_ = sdf->Get<double>("humidity");
    }

    // Initialize ROS 2 if it has not already been initialized.
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>(
      model_->GetName() + "_environment_sensor");

    temperature_pub_ =
      node_->create_publisher<std_msgs::msg::Float64>(
        "/" + model_->GetName() + "/temperature", 10);

    humidity_pub_ =
      node_->create_publisher<std_msgs::msg::Float64>(
        "/" + model_->GetName() + "/humidity", 10);

    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
        std::bind(
          &EnvironmentalSensorPlugin::OnUpdate,
          this));

    RCLCPP_INFO(
      node_->get_logger(),
      "Environmental sensor started: %s",
      model_->GetName().c_str());

    RCLCPP_INFO(
      node_->get_logger(),
      "Temperature: %.2f C | Humidity: %.2f %%",
      temperature_,
      humidity_);
  }

private:
  void OnUpdate()
  {
    rclcpp::spin_some(node_);

    std_msgs::msg::Float64 temperature_msg;
    temperature_msg.data = temperature_;
    temperature_pub_->publish(temperature_msg);

    std_msgs::msg::Float64 humidity_msg;
    humidity_msg.data = humidity_;
    humidity_pub_->publish(humidity_msg);
  }

  physics::ModelPtr model_;

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    temperature_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    humidity_pub_;

  event::ConnectionPtr update_connection_;

  double temperature_;
  double humidity_;
};

GZ_REGISTER_MODEL_PLUGIN(EnvironmentalSensorPlugin)

}  // namespace gazebo
