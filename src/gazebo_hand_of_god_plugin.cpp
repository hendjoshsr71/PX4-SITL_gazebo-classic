/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Hand of God Plugin
 *
 * This plugin allows a user to move a model
 *
 * @author Josh Henderson <hendjoshsr71@gmail.com>
 */

#include "gazebo_hand_of_god_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(HandOfGodPlugin)

HandOfGodPlugin::HandOfGodPlugin() : ModelPlugin()
{
}

HandOfGodPlugin::~HandOfGodPlugin()
{
  update_connection_->~Connection();
}

void HandOfGodPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_parachute_plugin] Please specify a robotNamespace.\n";
  }

  // Turn off physics to start with until the first 
  physics_on_start = true;
  model_->SetGravityMode(physics_on_start);

// WILL TURNING OFF GRAVITY here mean the accelerometers dont register gravity??

  // If we have a `pose_file` look for the given file
  if (sdf->HasElement("pose_file"))
  {
    pose_file_name_ = sdf->GetElement("pose_file")->Get<std::string>();

    // Check if the specified file exists
    // if (file_exists)
    // {
    //   pose_file_pointer = ;
    // } else {
    //   gzerr << "[gazebo_hand_of_god_plugin] File: " << pose_file_name_ << " could not be openned.\n";
    // }


  } else {
    pose_file_name_ = nullptr;
  }

  if (sdf->HasElement("rotation_on_drop")) {
    rotate_on_drop = sdf->GetElement("rotation_on_drop")->Get<bool>();
  } else {
    rotate_on_drop = false;
  }

  // getSdfParam<std::string>(sdf, "commandSubTopic", trigger_sub_topic_, trigger_sub_topic_);
  // getSdfParam<std::string>(sdf, "modelName", model_name_, model_name_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&HandOfGodPlugin::OnUpdate, this, _1));

  // node_handle_ = transport::NodePtr(new transport::Node());
  // node_handle_->Init(namespace_);

  // trigger_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + trigger_sub_topic_, &HandOfGodPlugin::VelocityCallback, this);

// ******************
// Important on initialization we need PX4 and Gazebo to start together
// IE Gazebo's physics (gravity) shouldn't apply until we are ready or for a few seconds

// Or pause PX4 and Gazebo
// IE if i tell gazebo to move the object Y meters high I don't want the EKF to get upset.
// So the PX4 state should be reset to where gazebo is now at....

/* How can we send a signal to PX4/gazebo to do actions?

  * Mouse click on a GUI element?
  * Mavlink command in the PX4 terminal?
  * ROS commnad
  * Joystick (ie through mavlink)?
*/

  // Subscribe to gazebo mavlink topics
  const std::string heartbeat_sub_topic_ = "mavlink/heartbeat";
  const std::string handofgod_sub_topic_ = "mavlink/handofgod";
  heartbeat_sub_ = node_handle_->Subscribe("~/" + heartbeat_sub_topic_, &GazeboMavlinkInterface::HeartBeatCallback, this);
  
  handofgod_sub_ = node_handle_->Subscribe("~/" + handofgod_sub_topic_, &GazeboMavlinkInterface::HandOfGodCallback, this);    // 

  
}

void HandOfGodPlugin::OnUpdate(const common::UpdateInfo&)
{

  // Wait until we have received a mavlink heartbeart to begin physics
  // Note: there are situtations like testings in-flight reboots where we would not be more careful about which heartbeat we care about
  // IE wait for first sim_related packet  
  // PX4 SITL: Simulator work starts then the PX4 process (similar to an actual autopilot occurs)
  // 

  if (!mavlink_interface_->ReceivedHeartbeats()) {
    return;
  }

  // Look for hand_of_god messages to set the state or spawn
  if (/* condition */)
  {
    /* code */
  }



#if GAZEBO_MAJOR_VERSION >= 9
    const ignition::math::Pose3d vehicle_pose = model_->WorldPose();
#else
    const ignition::math::Pose3d vehicle_pose = ignitionFromGazeboMath(model_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif

  ignition::math::Pose3d new_vehicle_pose = vehicle_pose;

  // // User GUI State
  // switch (gui_state)
  // {
  // case /* constant-expression */:
  //   /* code */
        // // Set vehicle pose to  10.0 m above current position
        // vehicle_pose -= Pose3d(0, 0, 10.0, 1, 0, 0, 0);
        // model_->SetWorldPose(vehicle_pose);
  //   break;

  // default:
  //   break;
  // }




// Example of a timer
  //get data depending on gazebo version
#if GAZEBO_MAJOR_VERSION >= 9
  const common::Time current_time_us = world_->SimTime();
#else
  const common::Time current_time_us = world_->GetSimTime();
#endif
  const double dt_us = (current_time_us - last_time_us_).Double();
    // std::cout << "dt = " << dt_us << "\n";

    const double TIME_TO_MOVE = 5.0;
    static bool flip = false;
    if (dt_us > (TIME_TO_MOVE)) {
      std::cout << "Vehicle_Pose X = " << vehicle_pose.Pos().X() << ", Z = " << vehicle_pose.Pos().Z() << "\n";

      if (!flip) {
        // Set vehicle pose to 10m above current position
        new_vehicle_pose += ignition::math::Pose3d(0, 0, 50.0, 1, 0, 0, 0);
        std::cout << "new Pose X = " << new_vehicle_pose.Pos().X() << ", Z = " << new_vehicle_pose.Pos().Z() << "\n";

        std::cout << "Model Name: " << model_->GetName() << "\n";

        model_->SetWorldPose(new_vehicle_pose);
        //Force the model to rotate during the drop. The current physics do not cause it to rotate during a drop
        // Apply a small rotational velocity

        model_->SetAngularVel(ignition::math::Vector3d(10* 3.14/180.0, 0.0, 0.0));

        flip = true;
      } else {
        //  If gravity is active we dont need this :)

        // // Set vehicle pose back to the ground
        // new_vehicle_pose -= ignition::math::Pose3d(0, 0, 10.0, 1, 0, 0, 0);
        // model_->SetWorldPose(new_vehicle_pose);

        // Set vehicle pose back to the ground
        new_vehicle_pose += ignition::math::Pose3d(0, 0, 100.0, 1, 0, 0, 0);
        model_->SetWorldPose(new_vehicle_pose);
        model_->SetAngularVel(ignition::math::Vector3d(100* 3.14/180, 80* 3.14/180, 0.0));

        flip = false;
      }

      last_time_us_ = current_time_us;
    }
}

// void HandOfGodPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
//   if(rot_velocities->motor_speed_size() < motor_number_) {
//     std::cout  << "You tried to access index " << motor_number_
//       << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
//   } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
// }

// void HandOfGodPlugin::LoadPackage(){
//   // Don't create duplicate the payload
//   physics::ModelPtr parachute_model = GetModelPtr(model_name_);
//   if(parachute_model) return;

//   // Insert parachute model
//   std::string model_uri = "model://" + model_name_;
//   world_->InsertModelFile(model_uri);

//   msgs::Int request;
//   request.set_data(0);

// }

physics::ModelPtr HandOfGodPlugin::GetModelPtr(std::string model_name){
  physics::ModelPtr model;

  #if GAZEBO_MAJOR_VERSION >= 9
    model = world_->ModelByName(model_name);
  #else
    model = world_->GetModel(model_name);
  #endif
  return model;
}

} // namespace gazebo
