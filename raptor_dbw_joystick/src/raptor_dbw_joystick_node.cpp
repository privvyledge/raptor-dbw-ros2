// Copyright (c) 2018-2021 New Eagle, Copyright (c) 2015-2018, Dataspeed Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "raptor_dbw_joystick/raptor_dbw_joystick.hpp"

#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  rclcpp::executors::SingleThreadedExecutor exec{};

  // Get parameter values
  auto temp = std::make_shared<rclcpp::Node>("get_joy_params_node", options);
  temp->declare_parameter("ignore", rclcpp::PARAMETER_BOOL);
  temp->declare_parameter("enable", rclcpp::PARAMETER_BOOL);
  temp->declare_parameter("svel", rclcpp::PARAMETER_DOUBLE);
  temp->declare_parameter("max_steer_angle", rclcpp::PARAMETER_DOUBLE);

  // parameters to setup speed or raw control
  temp->declare_parameter("raw_control", rclcpp::PARAMETER_BOOL);  // True: open_loop or raw, False:  closed_loop or speed
  temp->declare_parameter("max_accelerator_pedal", rclcpp::PARAMETER_DOUBLE);  // for raw mode
  temp->declare_parameter("max_speed", rclcpp::PARAMETER_DOUBLE);  // for speed mode
//  temp->declare_parameter("min_speed", rclcpp::PARAMETER_DOUBLE);  // for speed mode
  temp->declare_parameter("max_accel", rclcpp::PARAMETER_DOUBLE);  // for speed mode
  temp->declare_parameter("max_decel", rclcpp::PARAMETER_DOUBLE);  // for speed mode
  temp->declare_parameter("speed_increment", rclcpp::PARAMETER_DOUBLE);  // for speed mode

  bool n_ignore = temp->get_parameter("ignore").as_bool();
  bool n_enable = temp->get_parameter("enable").as_bool();
  double n_svel = temp->get_parameter("svel").as_double();
  float n_max_steer_angle = temp->get_parameter("max_steer_angle").as_double();
  // parameters to setup speed or raw control
  bool n_raw_control = temp->get_parameter("raw_control").as_bool();
  float n_max_accelerator_pedal = temp->get_parameter("max_accelerator_pedal").as_double();
  float n_max_speed = temp->get_parameter("max_speed").as_double();
//  float n_min_speed = temp->get_parameter("min_speed").as_double();
  float n_max_accel = temp->get_parameter("max_accel").as_double();
  float n_max_decel = temp->get_parameter("max_decel").as_double();
  float n_speed_increment = temp->get_parameter("speed_increment").as_double();

  // Create RaptorDbwJoystick class
  auto node = std::make_shared<raptor_dbw_joystick::RaptorDbwJoystick>(
    options,
    n_ignore,
    n_enable,
    n_svel,
    n_max_steer_angle,
    n_raw_control,
    n_max_accelerator_pedal,
    n_max_speed,
//    n_min_speed,
    n_max_accel,
    n_max_decel,
    n_speed_increment
  );

  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
