#ifndef _GAZEBO_HAND_OF_GOD_PLUGIN_HH_
#define _GAZEBO_HAND_OF_GOD_PLUGIN_HH_

// #include <functional>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>



// #include <gazebo/common/Plugin.hh>


#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

namespace gazebo
{

  /// \brief A visual plugin that draws a force published on a topic
  class GAZEBO_VISIBLE HandOfGodPlugin : public ModelPlugin
  {
    public:
      HandOfGodPlugin();
      virtual ~HandOfGodPlugin();

    protected:
      virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
      virtual void OnUpdate(const common::UpdateInfo&);
      physics::ModelPtr GetModelPtr(std::string model_name);

    private:
      // void TriggerCallback(const boost::shared_ptr<const msgs::Int> &_msg);
      // void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);

      std::string namespace_;
      physics::ModelPtr model_;
      physics::WorldPtr world_;
      physics::LinkPtr link_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      // Parameters from sdf
      std::string pose_file_name_;    // File name for the poses, File format {TIME (int_64), POSITION (double) {x_pos, y_pos, z_pos}, attitude_quaternion (double), {w, x, y,z}}
      bool rotate_on_drop;            // Force a rotation on drop


      common::Time last_time_us_;     // Last time (us) an update step was performed

      // LaunchStatus launch_status_ = VEHICLE_STANDBY;
      // common::Time trigger_time_;


      // Stored Parameters from the SDF
      bool physics_on_start               = true;     // Allow physics on bootup
      bool require_heartbeat_for_physics  = false;    // Require a heart from mavlink 
      bool require_ekf_good_for_physics   = false;    // Require an EFK good status before starting physics

      // double max_rot_velocity_ = 3500;
      // double ref_motor_rot_vel_ = 0.0;
      // double arm_rot_vel_ = 100;
      // double launch_duration_ = 0.01;
      // double force_magnitude_ = 1.0;

      // std::string trigger_sub_topic_ = "/gazebo/command/motor_speed";

      // transport::NodePtr node_handle_;
      // transport::SubscriberPtr trigger_sub_;

      // // Subscribe to Gazebo messages
      // transport::SubscriberPtr heartbeat_sub_{nullptr};
      // transport::SubscriberPtr handofgod_sub_{nullptr};

    };  // class GAZEBO_VISIBLE HandOfGodPlugin

}       // namespace gazebo

#endif  // _GAZEBO_HAND_OF_GOD_PLUGIN_HH_
