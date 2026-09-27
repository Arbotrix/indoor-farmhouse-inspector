#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sdf/sdf.hh>

#include <memory>
#include <string>
#include <functional>
#include <cmath>

namespace gazebo
{

class PumpPlugin : public ModelPlugin
{
public:

  PumpPlugin()
  : pump_on_(false),
    previous_pump_state_(false)
  {
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    (void)sdf;

    model_ = model;

    if (!rclcpp::ok())
    {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("pump_controller");

    pump_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/pump_cmd",
      10,
      std::bind(
        &PumpPlugin::PumpCommandCallback,
        this,
        std::placeholders::_1));

    pump_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/pump_state",
      10);

    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
        std::bind(&PumpPlugin::OnUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "Pump controller started on model: %s",
      model_->GetName().c_str());

    pump_on_ = false;
    previous_pump_state_ = false;
  }


private:

  void PumpCommandCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    pump_on_ = msg->data;

    if (pump_on_)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "WATER PUMP: ON");
    }
    else
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "WATER PUMP: OFF");
    }
  }


  void CreateSpray()
  {
    auto world = model_->GetWorld();

    if (!world)
      return;

    if (world->ModelByName("water_spray"))
      return;

    sdf::SDF spray_sdf;

    spray_sdf.SetFromString(R"(
      <?xml version="1.0"?>
      <sdf version="1.6">

        <model name="water_spray">

          <static>true</static>

          <link name="spray_link">

            <!-- Main water stream -->
            <visual name="water_stream">

              <pose>0.25 0 0 0 1.5708 0</pose>

              <geometry>
                <cylinder>
                  <radius>0.035</radius>
                  <length>0.50</length>
                </cylinder>
              </geometry>

              <material>
                <ambient>0.05 0.55 1.0 0.8</ambient>
                <diffuse>0.05 0.55 1.0 0.8</diffuse>
                <specular>0.3 0.3 0.3 1</specular>
              </material>

            </visual>


            <!-- Droplet 1 -->
            <visual name="droplet_1">

              <pose>0.52 0.025 0.03 0 0 0</pose>

              <geometry>
                <sphere>
                  <radius>0.035</radius>
                </sphere>
              </geometry>

              <material>
                <ambient>0.05 0.55 1.0 0.8</ambient>
                <diffuse>0.05 0.55 1.0 0.8</diffuse>
              </material>

            </visual>


            <!-- Droplet 2 -->
            <visual name="droplet_2">

              <pose>0.62 -0.025 -0.02 0 0 0</pose>

              <geometry>
                <sphere>
                  <radius>0.028</radius>
                </sphere>
              </geometry>

              <material>
                <ambient>0.05 0.55 1.0 0.8</ambient>
                <diffuse>0.05 0.55 1.0 0.8</diffuse>
              </material>

            </visual>


            <!-- Droplet 3 -->
            <visual name="droplet_3">

              <pose>0.72 0.02 0.025 0 0 0</pose>

              <geometry>
                <sphere>
                  <radius>0.032</radius>
                </sphere>
              </geometry>

              <material>
                <ambient>0.05 0.55 1.0 0.8</ambient>
                <diffuse>0.05 0.55 1.0 0.8</diffuse>
              </material>

            </visual>


            <!-- Droplet 4 -->
            <visual name="droplet_4">

              <pose>0.82 -0.02 -0.015 0 0 0</pose>

              <geometry>
                <sphere>
                  <radius>0.025</radius>
                </sphere>
              </geometry>

              <material>
                <ambient>0.05 0.55 1.0 0.8</ambient>
                <diffuse>0.05 0.55 1.0 0.8</diffuse>
              </material>

            </visual>

          </link>

        </model>

    </sdf>
    )");


    UpdateSprayPose(spray_sdf);

    world->InsertModelSDF(spray_sdf);

    RCLCPP_INFO(
      node_->get_logger(),
      "WATER SPRAY: CREATED");
  }


  void UpdateSprayPose(sdf::SDF & spray_sdf)
  {
    ignition::math::Pose3d robot_pose =
      model_->WorldPose();

    ignition::math::Pose3d spray_pose =
      robot_pose;

    // Distance from robot center to spray origin
    const double offset = 0.40;

    const double yaw =
      robot_pose.Rot().Yaw();

    spray_pose.Pos().X() +=
      offset * std::cos(yaw);

    spray_pose.Pos().Y() +=
      offset * std::sin(yaw);

    // Height of nozzle
    spray_pose.Pos().Z() += 0.29;

    spray_sdf.Root()
      ->GetElement("model")
      ->GetElement("pose")
      ->Set(spray_pose);
  }


  void UpdateExistingSpray()
  {
    auto world = model_->GetWorld();

    if (!world)
      return;

    auto spray =
      world->ModelByName("water_spray");

    if (!spray)
      return;

    ignition::math::Pose3d robot_pose =
      model_->WorldPose();

    ignition::math::Pose3d spray_pose =
      robot_pose;

    // Spray follows the robot's front
    const double offset = 0.40;

    const double yaw =
      robot_pose.Rot().Yaw();

    spray_pose.Pos().X() +=
      offset * std::cos(yaw);

    spray_pose.Pos().Y() +=
      offset * std::sin(yaw);

    // Keep spray at nozzle height
    spray_pose.Pos().Z() += 0.29;

    spray->SetWorldPose(spray_pose);
  }


  void RemoveSpray()
  {
    auto world = model_->GetWorld();

    if (!world)
      return;

    if (world->ModelByName("water_spray"))
    {
      world->RemoveModel("water_spray");

      RCLCPP_INFO(
        node_->get_logger(),
        "WATER SPRAY: REMOVED");
    }
  }


  void OnUpdate()
  {
    rclcpp::spin_some(node_);

    /*
     * Detect pump state changes.
     */
    if (pump_on_ != previous_pump_state_)
    {
      if (pump_on_)
      {
        CreateSpray();
      }
      else
      {
        RemoveSpray();
      }

      previous_pump_state_ =
        pump_on_;
    }


    /*
     * While pump is ON,
     * continuously move the spray
     * with the robot.
     */
    if (pump_on_)
    {
      UpdateExistingSpray();
    }


    /*
     * Publish current pump state.
     */
    std_msgs::msg::Bool state_msg;

    state_msg.data =
      pump_on_;

    pump_state_pub_->publish(
      state_msg);
  }


  physics::ModelPtr model_;

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr pump_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr pump_state_pub_;

  event::ConnectionPtr update_connection_;

  bool pump_on_;

  bool previous_pump_state_;
};


GZ_REGISTER_MODEL_PLUGIN(PumpPlugin)

}  // namespace gazebo
