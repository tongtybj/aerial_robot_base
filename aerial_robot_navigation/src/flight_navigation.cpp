// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, DRAGON Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the DRAGON Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <aerial_robot_navigation/flight_navigation.h>
#include <aerial_robot_navigation/util/joy_parser.h>

using namespace aerial_robot_navigation;
static const rclcpp::Logger NAV_LOGGER = rclcpp::get_logger("Navigation");

BaseNavigator::BaseNavigator():
  target_pos_(0, 0, 0),
  target_vel_(0, 0, 0),
  target_acc_(0, 0, 0),
  target_rpy_(0, 0, 0),
  target_omega_(0, 0, 0),
  target_ang_acc_(0, 0, 0),
  init_height_(0), land_height_(0),
  force_att_control_flag_(false),
  trajectory_mode_(false),
  trajectory_reset_time_(0),
  teleop_reset_time_(0),
  low_voltage_flag_(false),
  high_voltage_flag_(false),
  prev_xy_control_mode_(ACC_CONTROL_MODE),
  xy_control_flag_(false),
  vel_based_waypoint_(false),
  gps_waypoint_(false),
  gps_waypoint_time_(0),
  joy_stick_heart_beat_(false),
  joy_stick_prev_time_(0),
  teleop_flag_(true),
  force_landing_flag_(false),
  land_check_start_time_(0) {
  setNaviState(ARM_OFF_STATE);
}

void BaseNavigator::initialize(rclcpp::Node::SharedPtr node,
                               RobotModelPtr robot_model,
                               StateEstimatorPtr estimator,
                               double loop_du) {
  node_ = node;
  rosParamInit();

  robot_model_ = robot_model;
  estimator_ = estimator;

  navi_sub_ = node_->create_subscription<aerial_robot_msgs::msg::FlightNav>
    ("uav/nav", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::naviCallback, this, std::placeholders::_1));

  single_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>
    ("target_pose", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::singleGoalCallback, this, std::placeholders::_1));

  simple_move_base_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>
    ("/move_base_simple/goal", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::simpleMoveBaseGoalCallback, this, std::placeholders::_1));

  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>
    ("target_path", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::pathCallback, this, std::placeholders::_1));

  battery_sub_ = node_->create_subscription<std_msgs::msg::Float32>
    ("battery_voltage_status", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::batteryCheckCallback, this, std::placeholders::_1));

  flight_status_ack_sub_ = node_->create_subscription<std_msgs::msg::UInt8>
    ("flight_config_ack", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::flightStatusAckCallback, this, std::placeholders::_1));

  takeoff_sub_ = node_->create_subscription<std_msgs::msg::Empty>
    ("teleop_command/takeoff", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::takeoffCallback, this, std::placeholders::_1));

  halt_sub_ = node_->create_subscription<std_msgs::msg::Empty>
    ("teleop_command/halt", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::haltCallback, this, std::placeholders::_1));

  force_landing_sub_ = node_->create_subscription<std_msgs::msg::Empty>
    ("teleop_command/force_landing", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::forceLandingCallback, this, std::placeholders::_1));

  land_sub_ = node_->create_subscription<std_msgs::msg::Empty>
    ("teleop_command/land", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::landCallback, this, std::placeholders::_1));

  start_sub_ = node_->create_subscription<std_msgs::msg::Empty>
    ("teleop_command/start", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::startCallback, this, std::placeholders::_1));

  joy_stick_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>
    ("joy", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::joyStickControl, this, std::placeholders::_1));

  stop_teleop_sub_ = node_->create_subscription<std_msgs::msg::UInt8>
    ("stop_teleop", rclcpp::SystemDefaultsQoS(),
     std::bind(&BaseNavigator::stopTeleopCallback, this, std::placeholders::_1));

  flight_config_pub_ = node_->create_publisher<spinal::msg::FlightConfigCmd>
    ("flight_config_cmd", rclcpp::SystemDefaultsQoS());

  flight_state_pub_ = node_->create_publisher<std_msgs::msg::UInt8>
    ("flight_state", rclcpp::SystemDefaultsQoS());

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>
    ("trajectory", rclcpp::SystemDefaultsQoS());

  waypoint_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>
    ("waypoints", rclcpp::SystemDefaultsQoS());

  loop_du_ = loop_du;
  estimate_mode_ = estimator_->getEstimateMode();
  force_landing_start_time_ =  node_->get_clock()->now().seconds();
}

void BaseNavigator::batteryCheckCallback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
  if(std::isnan(msg->data)) {
    throw std::runtime_error("the voltage from spinal is Nan, \
                             please re-calibrate the voltage scale using /set_adc_scale.");
  }

  if(bat_cell_ == 0) {
    RCLCPP_WARN(NAV_LOGGER, "No correct battery information, cell is 0");
    return;
  }

  float voltage = msg->data;
  /* consider the voltage drop */
  if(getNaviState() == TAKEOFF_STATE || getNaviState() == HOVER_STATE) {
    voltage += ( (bat_resistance_voltage_rate_ * voltage +  bat_resistance_) * hovering_current_);
  }

  float average_voltage = voltage / bat_cell_;
  float input_cell = voltage / VOL_100P;
  float rate = 0;
  if(average_voltage  > VOL_90P) rate = (average_voltage - VOL_90P) / (VOL_100P - VOL_90P) * 10 + 90;
  else if (average_voltage  > VOL_80P) rate = (average_voltage - VOL_80P) / (VOL_90P - VOL_80P) * 10 + 80;
  else if (average_voltage  > VOL_70P) rate = (average_voltage - VOL_70P) / (VOL_80P - VOL_70P) * 10 + 70;
  else if (average_voltage  > VOL_60P) rate = (average_voltage - VOL_60P) / (VOL_70P - VOL_60P) * 10 + 60;
  else if (average_voltage  > VOL_50P) rate = (average_voltage - VOL_50P) / (VOL_60P - VOL_50P) * 10 + 50;
  else if (average_voltage  > VOL_40P) rate = (average_voltage - VOL_40P) / (VOL_50P - VOL_40P) * 10 + 40;
  else if (average_voltage  > VOL_30P) rate = (average_voltage - VOL_30P) / (VOL_40P - VOL_30P) * 10 + 30;
  else if (average_voltage  > VOL_20P) rate = (average_voltage - VOL_20P) / (VOL_30P - VOL_20P) * 10 + 20;
  else if (average_voltage  > VOL_10P) rate = (average_voltage - VOL_10P) / (VOL_20P - VOL_10P) * 10 + 10;
  else rate = (average_voltage - VOL_0P) / (VOL_10P - VOL_0P) * 10;

  if (rate > 100) rate = 100;
  if(rate < 0) {
    /* can remove this information */
    RCLCPP_WARN(NAV_LOGGER, "no correct voltage information from spinal");
    return;
  }

  if(rate < low_voltage_thre_) {
      low_voltage_flag_  = true;
      RCLCPP_WARN_THROTTLE(NAV_LOGGER, *(node_->get_clock()), 1000,"low voltage!");
  } else {
    low_voltage_flag_  = false;
  }

  if(input_cell - bat_cell_ > high_voltage_cell_thre_) {
    high_voltage_flag_ = true;
  } else {
    high_voltage_flag_ = false;
  }
}

void BaseNavigator::pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg) {

  if(getNaviState() != HOVER_STATE) return;

  generateNewTrajectory(msg->poses);
}


void BaseNavigator::singleGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {

  if(getNaviState() != HOVER_STATE) return;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.push_back(*msg);

  generateNewTrajectory(path);
}

void BaseNavigator::simpleMoveBaseGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {

  if(getNaviState() != HOVER_STATE) return;

  geometry_msgs::msg::PoseStamped target_pose = *msg;
  target_pose.pose.position.z = getTargetPos().z();
  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.push_back(target_pose);
  generateNewTrajectory(path);
}

void BaseNavigator::naviCallback(const aerial_robot_msgs::msg::FlightNav::ConstSharedPtr msg) {

  if(getNaviState() != HOVER_STATE) return;

  gps_waypoint_ = false; // force reset gps mode

  if(force_att_control_flag_) return;

  /* yaw */
  if(msg->yaw_nav_mode == aerial_robot_msgs::msg::FlightNav::POS_MODE) {
    setTargetYaw(angles::normalize_angle(msg->target_yaw));
    setTargetOmegaZ(0);
  }

  if(msg->yaw_nav_mode == aerial_robot_msgs::msg::FlightNav::VEL_MODE) {
    setTargetOmegaZ(msg->target_omega_z);
    teleop_reset_time_ = node_->get_clock()->now().seconds() + teleop_reset_duration_;
  }

  if(msg->yaw_nav_mode == aerial_robot_msgs::msg::FlightNav::POS_VEL_MODE) {
    setTargetYaw(angles::normalize_angle(msg->target_yaw));
    setTargetOmegaZ(msg->target_omega_z);

    trajectory_mode_ = true;
    trajectory_reset_time_ = node_->get_clock()->now().seconds() + trajectory_reset_duration_;
  }

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::msg::FlightNav::VEL_MODE) {
    setTargetVelZ(msg->target_vel_z);
    teleop_reset_time_ = node_->get_clock()->now().seconds() + teleop_reset_duration_;
  } else if(msg->pos_z_nav_mode == aerial_robot_msgs::msg::FlightNav::POS_MODE) {
    setTargetPosZ(msg->target_pos_z);
    setTargetVelZ(0);
  } else if(msg->pos_z_nav_mode == aerial_robot_msgs::msg::FlightNav::POS_VEL_MODE) {
    setTargetPosZ(msg->target_pos_z);
    setTargetVelZ(msg->target_vel_z);

    trajectory_mode_ = true;
    trajectory_reset_time_ = node_->get_clock()->now().seconds() + trajectory_reset_duration_;
  }

  /* xy control */
  switch(msg->pos_xy_nav_mode) {
  case aerial_robot_msgs::msg::FlightNav::POS_MODE:
    {
      KDL::Vector target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);

      KDL::Vector target_delta = getTargetPos() - target_cog_pos;
      target_delta.z(0);

      if(target_delta.Norm() > vel_nav_threshold_) {
        RCLCPP_WARN(NAV_LOGGER, "start vel nav control for waypoint");
        vel_based_waypoint_ = true;
        xy_control_mode_ = VEL_CONTROL_MODE;
      }

      if(!vel_based_waypoint_) xy_control_mode_ = POS_CONTROL_MODE;

      setTargetPosX(target_cog_pos.x());
      setTargetPosY(target_cog_pos.y());
      setTargetVelX(0);
      setTargetVelY(0);

      break;
    }
  case aerial_robot_msgs::msg::FlightNav::VEL_MODE:
    {
      /* do not switch to pure vel mode */
      xy_control_mode_ = POS_CONTROL_MODE;

      teleop_reset_time_ = node_->get_clock()->now().seconds() + teleop_reset_duration_;

      switch(msg->control_frame) {
        case WORLD_FRAME:
          {
            setTargetVelX(msg->target_vel_x);
            setTargetVelY(msg->target_vel_y);
            break;
          }
        case LOCAL_FRAME:
          {
            double yaw_angle = estimator_->getCogEuler(estimate_mode_).z();
            KDL::Vector target_vel = frameConversion(KDL::Vector(msg->target_vel_x, msg->target_vel_y, 0), yaw_angle);
            setTargetVelX(target_vel.x());
            setTargetVelY(target_vel.y());
            break;
          }
        default:
          {
            break;
          }
        }
      break;
    }
  case aerial_robot_msgs::msg::FlightNav::POS_VEL_MODE:
    {
      xy_control_mode_ = POS_CONTROL_MODE;
      setTargetPosX(msg->target_pos_x);
      setTargetPosY(msg->target_pos_y);
      setTargetVelX(msg->target_vel_x);
      setTargetVelY(msg->target_vel_y);

      trajectory_mode_ = true;
      trajectory_reset_time_ = node_->get_clock()->now().seconds() + trajectory_reset_duration_;

      break;
    }
  case aerial_robot_msgs::msg::FlightNav::ACC_MODE:
    {
      /* should be in COG frame */
      xy_control_mode_ = ACC_CONTROL_MODE;
      prev_xy_control_mode_ = ACC_CONTROL_MODE;

      switch(msg->control_frame) {
        case WORLD_FRAME:
          {
            setTargetAccX(msg->target_acc_x);
            setTargetAccY(msg->target_acc_y);
            break;
          }
        case LOCAL_FRAME:
          {
            double yaw_angle = estimator_->getCogEuler(estimate_mode_).z();
            KDL::Vector target_acc = frameConversion(KDL::Vector(msg->target_acc_x, msg->target_acc_y, 0), yaw_angle);
            setTargetAccX(target_acc.x());
            setTargetAccY(target_acc.y());
            break;
          }
        default:
          {
            break;
          }
        }
      break;
    }
  case aerial_robot_msgs::msg::FlightNav::GPS_WAYPOINT_MODE:
    {
      target_wp_ = geodesy::toMsg(msg->target_pos_x, msg->target_pos_y);
      gps_waypoint_ = true;

      break;
    }
  }

  if(msg->pos_xy_nav_mode != aerial_robot_msgs::msg::FlightNav::ACC_MODE) {
      setTargetZeroAcc();
  }
}

void BaseNavigator::joyStickControl(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  sensor_msgs::msg::Joy joy_cmd = joyParse(*msg);
  if (joy_cmd.axes.size() == 0 || joy_cmd.buttons.size() == 0) {
      RCLCPP_WARN(NAV_LOGGER, "the joystick type is not supported (buttons: %d, axes: %d)",
                  (int)msg->buttons.size(), (int)msg->axes.size());
      return;
  }

  if(!joy_stick_heart_beat_) joy_stick_heart_beat_ = true;
  double joy_cmd_time = rclcpp::Time(joy_cmd.header.stamp).seconds();

  joy_stick_prev_time_ = node_->get_clock()->now().seconds();

  /* common command */
  /* start */
  if(joy_cmd.buttons[JOY_BUTTON_START] == 1 && getNaviState() == ARM_OFF_STATE) {
    motorArming();
    return;
  }

  /* force landing && halt */
  if(joy_cmd.buttons[JOY_BUTTON_STOP] == 1) {
    /* Force Landing in inflight mode: TAKEOFF_STATE/LAND_STATE/HOVER_STATE */
    if(!force_landing_flag_ && isInflightState()) {
      RCLCPP_WARN(NAV_LOGGER, "Joy Control: force landing state");
      spinal::msg::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::msg::FlightConfigCmd::FORCE_LANDING_CMD;
      flight_config_pub_->publish(flight_config_cmd);
      force_landing_flag_ = true;

      /* update the force landing stamp for the halt process*/
      force_landing_start_time_ = joy_cmd_time;
    }

    /* Halt mode */
    if(joy_cmd_time - force_landing_start_time_ > force_landing_to_halt_du_
       && getNaviState() > START_STATE) {
      //if(!teleop_flag_) return; /* can not do the process if other processs are running */

      RCLCPP_ERROR(NAV_LOGGER, "Joy Control: Halt!");

      setNaviState(STOP_STATE);

      /* update the target pos(maybe not necessary) */
      setTargetXyFromCurrentState();
      setTargetYawFromCurrentState();
    }

    return;

  } else {
    /* update the halt process */
    force_landing_start_time_ = joy_cmd_time;
  }

  /* takeoff */
  if(joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] == 1 && joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] == 1) {
    startTakeoff();
    return;
  }

  /* landing */
  if(joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] == 1 && joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] == 1) {
    if(force_att_control_flag_) return;

    if(getNaviState() == LAND_STATE) return;
    if(!teleop_flag_) return; /* can not do the process if other processs are running */

    setNaviState(LAND_STATE);
    //update
    RCLCPP_INFO(NAV_LOGGER, "Joy Control: Land state");

    return;
  }

  teleop_reset_time_ = node_->get_clock()->now().seconds() + teleop_reset_duration_;

  double raw_x_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];
  double raw_y_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];
  double raw_z_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];
  double raw_yaw_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];

  /* Motion: Z (Height) */
  if(getNaviState() == HOVER_STATE) {
    if(fabs(raw_z_cmd) > joy_stick_deadzone_) setTargetVelZ(raw_z_cmd * max_teleop_z_vel_);
    else setTargetVelZ(0);
  }

  /* Motion: Yaw */
  if(getNaviState() == HOVER_STATE) {
    if(fabs(raw_yaw_cmd) > joy_stick_deadzone_) setTargetOmegaZ(raw_yaw_cmd * max_teleop_yaw_vel_);
    else setTargetOmegaZ(0);
  }

  /* Motion: XY */
  /* mode selection */
  /* switch to acc model */
  if(joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] == 1) {
    if (xy_control_mode_ != ACC_CONTROL_MODE) {
      RCLCPP_WARN(NAV_LOGGER, "Force siwtch to attitude control mode");
      force_att_control_flag_ = true;
      estimator_->setForceAttControlFlag(force_att_control_flag_);
      xy_control_mode_ = ACC_CONTROL_MODE;
    }
  }

  /* switch to pure vel mode */
  if(joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] == 1) {
    if (xy_control_mode_ != VEL_CONTROL_MODE) {
      RCLCPP_INFO(NAV_LOGGER, "Switch to pure vel control mode");
      force_att_control_flag_ = false;
      setTargetZeroVel();
      setTargetZeroAcc();
      xy_control_mode_ = VEL_CONTROL_MODE;
    }
  }

  /* siwtch to pos mode */
  if(joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] == 1) {
    if (xy_control_mode_ != POS_CONTROL_MODE) {
      RCLCPP_INFO(NAV_LOGGER, "change to pos control");
      force_att_control_flag_ = false;
      setTargetXyFromCurrentState();
      xy_control_mode_ = POS_CONTROL_MODE;
    }
  }

  /* finish if teleop flag is not true */
  if(!teleop_flag_) return;

  /* mode oriented state */
  control_frame_ = WORLD_FRAME;
  KDL::Rotation local_frame_rot;
  if(joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2]) {
    control_frame_ = LOCAL_FRAME;

    /* convert the frame */
    const auto segments_tf = robot_model_->getSegmentsTf();
    if(segments_tf.find(teleop_local_frame_) == segments_tf.end()) {
      RCLCPP_ERROR(NAV_LOGGER, "can not find %s in kinematics model", teleop_local_frame_.c_str());
      setTargetZeroAcc();
      return;
    }

    std::string baselink = robot_model_->getBaselinkName();
    KDL::Frame teleop_local_frame_tf = segments_tf.at(baselink).Inverse() * segments_tf.at(teleop_local_frame_);

    double yaw_angle = estimator_->getCogEuler(estimate_mode_).z();

    local_frame_rot = KDL::Rotation::RPY(0, 0, yaw_angle) * teleop_local_frame_tf.M;
  }

  switch (xy_control_mode_) {
    case POS_CONTROL_MODE:
      {
        if(fabs(raw_x_cmd) < joy_stick_deadzone_) raw_x_cmd = 0;
        if(fabs(raw_y_cmd) < joy_stick_deadzone_) raw_y_cmd = 0;

        /* vel command */
        setTargetVelX(raw_x_cmd * max_teleop_xy_vel_);
        setTargetVelY(raw_y_cmd * max_teleop_xy_vel_);

        if(control_frame_ == LOCAL_FRAME) {
          KDL::Vector target_vel = frameConversion(getTargetVel(), local_frame_rot);
          setTargetVelX(target_vel.x());
          setTargetVelY(target_vel.y());
        }
        break;
      }
    case VEL_CONTROL_MODE:
      {
        /* vel command */
        setTargetVelX(raw_x_cmd * fabs(raw_x_cmd) * max_teleop_xy_vel_);
        setTargetVelY(raw_y_cmd * fabs(raw_y_cmd) * max_teleop_xy_vel_);

        if(control_frame_ == LOCAL_FRAME) {
          KDL::Vector target_vel = frameConversion(getTargetVel(), local_frame_rot);
          setTargetVelX(target_vel.x());
          setTargetVelY(target_vel.y());
        }

        break;
      }
    case ACC_CONTROL_MODE:
      {
        /* acc command */
        double acc_scale = max_teleop_rp_angle_ * aerial_robot_estimation::G;
        setTargetAccX(raw_x_cmd * acc_scale);
        setTargetAccY(raw_y_cmd * acc_scale);

        if(control_frame_ == LOCAL_FRAME) {
          KDL::Vector target_acc = frameConversion(getTargetAcc(), local_frame_rot);
          setTargetAccX(target_acc.x());
          setTargetAccY(target_acc.y());
        }
        break;
      }
    default:
      {
        break;
      }
    }
}

void BaseNavigator::update() {

  if(force_att_control_flag_) {
    if(getNaviState() == LAND_STATE) {
      // reset to hover state and do manually landing
      setTargetZFromCurrentState();
      setNaviState(HOVER_STATE);
    }
  } else {
    /* check the xy estimation status, if not ready, change to att_control_mode */
    if(!estimator_->getBasePosStateStatus(State::X, estimate_mode_) ||
       !estimator_->getBasePosStateStatus(State::Y, estimate_mode_)) {

      if(xy_control_mode_ == VEL_CONTROL_MODE ||
         xy_control_mode_ == POS_CONTROL_MODE) {

        RCLCPP_WARN(NAV_LOGGER, "No estimation for X, Y state, change to attitude control mode");
        prev_xy_control_mode_ = xy_control_mode_;
        xy_control_mode_ = ACC_CONTROL_MODE;
      }
    } else {
      if(xy_control_mode_ == ACC_CONTROL_MODE &&
         prev_xy_control_mode_ != ACC_CONTROL_MODE) {

        RCLCPP_INFO(NAV_LOGGER, "Estimation for X, Y state is established, siwtch back to the xy control mode");
        xy_control_mode_ = prev_xy_control_mode_;
      }

      RCLCPP_INFO_STREAM_ONCE
        (NAV_LOGGER,
         std::string("\033[32m\n \n ======================  \n Ready for takeoff !!! \n ====================== \n \033[0m"));
    }
  }

  /* sensor health check */
  if(estimator_->getUnhealthLevel() == Sensor::UNHEALTH_LEVEL3 && !force_landing_flag_) {

    if(isInflightState()) {
      RCLCPP_WARN(NAV_LOGGER, "Sensor Unhealth Level%d: force landing state", estimator_->getUnhealthLevel());

      spinal::msg::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::msg::FlightConfigCmd::FORCE_LANDING_CMD;
      flight_config_pub_->publish(flight_config_cmd);
      force_landing_flag_ = true;
    }
  }

  if(getNaviState() == TAKEOFF_STATE || getNaviState() == HOVER_STATE) {

    bool normal_land = false;

      /* joystick heartbeat check */
      if(check_joy_stick_heart_beat_ && joy_stick_heart_beat_ &&
         node_->get_clock()->now().seconds() - joy_stick_prev_time_ > joy_stick_heart_beat_du_) {
        normal_land = true;
        RCLCPP_ERROR(NAV_LOGGER, "Normal Landing: att control mode, because no joy control");
      }

      /* low voltage flag */
      if(low_voltage_flag_) {
        normal_land = true;
        RCLCPP_ERROR(NAV_LOGGER, "Normal Landing: low battery");
      }

      if(normal_land && !force_att_control_flag_) {
        setNaviState(LAND_STATE);
      }
    }

  /* update the target pos and velocity */
  if (trajectory_mode_) {
      updatePoseFromTrajectory();
  } else {

    /* force reset velocity in idling mode */
    if (node_->get_clock()->now().seconds() > teleop_reset_time_) {
      setTargetZeroVel();
      setTargetOmegaZ(0);
    }

    /* uniform linear motion */
    addTargetPos(getTargetVel() * loop_du_);
    addTargetYaw(getTargetOmega().z() * loop_du_);
  }

  KDL::Vector curr_pos = estimator_->getCogPos(estimate_mode_);
  KDL::Vector curr_vel = estimator_->getCogVel(estimate_mode_);
  KDL::Vector delta = target_pos_ - curr_pos;
  double now_time = node_->get_clock()->now().seconds();

  switch(getNaviState()) {
  case START_STATE:
    {
      /* low voltage */
      if(low_voltage_flag_) {
        setNaviState(ARM_OFF_STATE);
        break;
      }

      if(high_voltage_flag_) {
        setNaviState(ARM_OFF_STATE);
        RCLCPP_ERROR(NAV_LOGGER, "high voltage!");
        break;
      }

      estimator_->setSensorFusionFlag(true);
      force_landing_flag_ = false;

      spinal::msg::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::msg::FlightConfigCmd::ARM_ON_CMD;
      flight_config_pub_->publish(flight_config_cmd);

      break;
    }
  case TAKEOFF_STATE:
    { //Takeoff Phase

      /* set flying flag to true once */
      if(!estimator_->getFlyingFlag()) estimator_->setFlyingFlag(true);

      if(xy_control_mode_ == POS_CONTROL_MODE) {

        if (fabs(delta.z()) > z_convergent_thresh_ ||
            fabs(delta.x()) > xy_convergent_thresh_ ||
            fabs(delta.y()) > xy_convergent_thresh_) {
          hover_convergent_start_time_ = node_->get_clock()->now().seconds();
        }
      } else {
        /* only check the z action */
        if (fabs(delta.z()) > z_convergent_thresh_) {
          hover_convergent_start_time_ = node_->get_clock()->now().seconds();
        }
      }

      if (now_time - hover_convergent_start_time_ > hover_convergent_duration_) {

        hover_convergent_start_time_ = now_time;
          setNaviState(HOVER_STATE);
          RCLCPP_INFO_STREAM
            (NAV_LOGGER,
             std::string("\033[32m \n \n ======================  \n Hover!!! \n ====================== \n \033[0m"));
        }
      break;
    }
  case LAND_STATE:
    {
      updateLandCommand();

      if (now_time - land_check_start_time_ > land_check_duration_)
        {
          double delta = curr_pos.z() - land_height_;
          double vel = curr_vel.z();

          RCLCPP_INFO(NAV_LOGGER, "expected land height: %f (current height: %f), velocity: %f ", land_height_, curr_pos.z(), vel);

          if (fabs(delta) < land_pos_convergent_thresh_ &&
              vel > -land_vel_convergent_thresh_) {
            RCLCPP_INFO(NAV_LOGGER, "\n \n ======================  \n Land !!! \n ====================== \n");
            RCLCPP_INFO(NAV_LOGGER, "Start disarming motors");
            setNaviState(STOP_STATE);
          } else {
            // not staedy, update the land height
            land_height_ = curr_pos.z();
          }

          land_check_start_time_ = now_time;
        }
      break;
    }
  case HOVER_STATE:
    {
      if(force_att_control_flag_) break;

      if(gps_waypoint_) gpsWaypointTracking();

      if(vel_based_waypoint_) {
        delta.z(0); // we do not need z

        /* vel nav */
        if(delta.Norm() > vel_nav_threshold_) {
          KDL::Vector nav_vel = delta * vel_nav_gain_;

          double speed = nav_vel.Norm();
          if(speed  > nav_vel_limit_) nav_vel = nav_vel * (nav_vel_limit_ / speed);

          setTargetVelX(nav_vel.x());
          setTargetVelY(nav_vel.y());
        } else {

          if(gps_waypoint_) {
            KDL::Vector gps_waypoint_delta = getDeltaPosFromGpsWaypoint();
            RCLCPP_WARN(NAV_LOGGER, "back to pos nav control for GPS way point, gps waypoint delta: %f, %f",
                        gps_waypoint_delta.x(), gps_waypoint_delta.y());
            gps_waypoint_  = false;
          } else {
            RCLCPP_WARN(NAV_LOGGER, "back to pos nav control for way point");
          }

          xy_control_mode_ = POS_CONTROL_MODE;
          vel_based_waypoint_ = false;
          setTargetZeroVel();
        }
      }
      break;
    }
  case STOP_STATE:
    {
      reset();

      spinal::msg::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::msg::FlightConfigCmd::ARM_OFF_CMD;
      flight_config_pub_->publish(flight_config_cmd);

      if(force_landing_flag_) {
        halt();
        force_landing_flag_ = false;
      }
      break;
    }
  default:
    {
      break;
    }
  }

  /* publish the state */
  std_msgs::msg::UInt8 state_msg;
  state_msg.data = getNaviState();
  if(force_landing_flag_) state_msg.data = FORCE_LANDING_STATE;
  else if(low_voltage_flag_) state_msg.data = LOW_BATTERY_STATE;
  flight_state_pub_->publish(state_msg);
}

bool BaseNavigator::isInflightState() {

  if (getNaviState() == TAKEOFF_STATE ||
      getNaviState() == LAND_STATE ||
      getNaviState() == HOVER_STATE) {
    return true;
  }

  return false;
}

void BaseNavigator::updateLandCommand() {
  // update pos and vel for z
  KDL::Vector curr_pos = estimator_->getCogPos(estimate_mode_);

  addTargetPosZ(land_descend_vel_ * loop_du_);
  setTargetVelZ(land_descend_vel_);

  // update vel for other axes
  setTargetVelX(0);
  setTargetVelY(0);
  setTargetOmega(0, 0, 0);
}

void BaseNavigator::generateNewTrajectory(std::vector<geometry_msgs::msg::PoseStamped> path) {

  if (traj_generator_ptr_.get() != nullptr) {
    RCLCPP_WARN(NAV_LOGGER, "Force to finish the last trajectory following");
    traj_generator_ptr_.reset();
  }

  std::vector<agi::QuadState> states;

  agi::QuadState start_state;
  start_state.setZero();
  KDL::Vector current_pos = estimator_->getCogPos(estimate_mode_);
  start_state.p = agi::Vector<3>(current_pos.x(), current_pos.y(), current_pos.z());
  KDL::Vector current_vel = estimator_->getCogVel(estimate_mode_);
  start_state.v = agi::Vector<3>(current_vel.x(), current_vel.y(), current_vel.z());

  double yaw_angle = estimator_->getCogEuler(estimate_mode_).z();
  start_state.setYaw(yaw_angle);
  double last_target_omega_z = getTargetOmega().z();
  start_state.w(2) = last_target_omega_z; // use target omega z instead of the real omega to avoid the noise
  start_state.t = node_->get_clock()->now().seconds();
  states.push_back(start_state);

  // set waypoints
  for(auto& pose: path) {

    agi::QuadState state;
    state.setZero();
    state.p.x() = pose.pose.position.x;
    state.p.y() = pose.pose.position.y;
    state.p.z() = pose.pose.position.z;
    agi::Quaternion q;
    q.x() = pose.pose.orientation.x;
    q.y() = pose.pose.orientation.y;
    q.z() = pose.pose.orientation.z;
    q.w() = pose.pose.orientation.w;
    if (std::fabs(1 - q.squaredNorm())  < 1e-6) {
      state.q(q);
    } else {
      RCLCPP_WARN(NAV_LOGGER, "Target quaternion is invalid [%f, %f, %f, %f], reset as the start state",
                  q.x(), q.y(), q.z(), q.w());
      state.q(start_state.q());
    }

    double du = rclcpp::Time(pose.header.stamp).seconds() - start_state.t;
    if (du < 0.01) {

      // if the target time is older or closer to the current time, reset the du by using an average velocity
      RCLCPP_INFO(NAV_LOGGER, "recalcualte the du");
      double du_tran = (state.p - start_state.p).norm() / trajectory_mean_vel_;
      double delta_yaw = state.getYaw() - start_state.getYaw();
      if (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
      if (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;
      double du_rot = fabs(delta_yaw) / trajectory_mean_yaw_rate_;
      du = std::max(du_tran, trajectory_min_du_);
      if (!enable_latch_yaw_trajectory_) du = std::max(du_rot, du);
    }
    state.t = start_state.t + du;

    states.push_back(state);
  }

  agi::QuadState end_state = states.back();
  double du = end_state.t - start_state.t;
  RCLCPP_INFO_STREAM(NAV_LOGGER, "Revceive the new target pose of " << end_state.p.transpose()
                     << " (yaw: " << end_state.getYaw() << ")"
                     << " which starts with the last target pose: " << start_state.p.transpose()
                     << " (yaw: " << start_state.getYaw() << ")"
                     << " and target vel: " << start_state.v.transpose()
                     << " (omega z: " << start_state.w(2) << ")"
                     << " and target acc: " << start_state.a.transpose()
                     << " and flight duration: " << du);

  traj_generator_ptr_ = std::make_shared<agi::MinJerkTrajectory>(states);

  trajectory_mode_ = true;


  // visualize
  double dt = 0.02;
  nav_msgs::msg::Path msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(start_state.t * 1e9));
  msg.header.frame_id = "world";

  for (double t = 0; t <= du; t += dt) {
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header.stamp = rclcpp::Time(msg.header.stamp) + rclcpp::Duration::from_seconds(t);
    pose_stamp.header.frame_id = msg.header.frame_id;

    agi::QuadState state = traj_generator_ptr_->getState(start_state.t + t);

    pose_stamp.pose.position.x = state.p.x();
    pose_stamp.pose.position.y = state.p.y();
    pose_stamp.pose.position.z = state.p.z();

    pose_stamp.pose.orientation.x = state.q().x();
    pose_stamp.pose.orientation.y = state.q().y();
    pose_stamp.pose.orientation.z = state.q().z();
    pose_stamp.pose.orientation.w = state.q().w();

    msg.poses.push_back(pose_stamp);
  }
  path_pub_->publish(msg);

  visualization_msgs::msg::MarkerArray marker_array_msg;
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(start_state.t * 1e9));
  marker_msg.header.frame_id = "world";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
  for (int i = 0; i < states.size(); i++) {
    agi::QuadState state = states.at(i);
    marker_msg.id = i;
    marker_msg.pose.position.x = state.p(0);
    marker_msg.pose.position.y = state.p(1);
    marker_msg.pose.position.z = state.p(2);
    marker_msg.pose.orientation.w = 1;
    double r = 0.2;
    marker_msg.scale.x = r;
    marker_msg.scale.y = r;
    marker_msg.scale.z = r;
    marker_msg.color.r = 1.0;
    marker_msg.color.a = 1.0;
    marker_array_msg.markers.push_back(marker_msg);
  }
  waypoint_pub_->publish(marker_array_msg);
}

void BaseNavigator::updatePoseFromTrajectory() {

  if(getNaviState() != HOVER_STATE) return;

  double t = node_->get_clock()->now().seconds();

  if (traj_generator_ptr_.get() == nullptr) {

    if (t > trajectory_reset_time_) {
      setTargetZeroVel();
      setTargetZeroAcc();

      setTargetZeroOmega();
      setTargetZeroAngAcc();

      trajectory_mode_ = false;

      RCLCPP_INFO(NAV_LOGGER, "[Trajectory] Stop trajectory mode in POS-VEL mode");
    }
    return;
  }

  // trajectory following mode
  // asynchronous with generateNewTrajectory
  if (traj_generator_ptr_.get() == nullptr) {
    RCLCPP_WARN(NAV_LOGGER,
                "[Trajectory] Terminate in trajectory mode since traj_generator_ptr_ is empty");
    return;
  }

  double end_t = traj_generator_ptr_->getEndSetpoint().state.t;

  // terminate if reach the end of trajectory
  if (t > end_t) {
    RCLCPP_INFO(NAV_LOGGER, "Reach the end of trajectory");

    setTargetZeroVel();
    setTargetZeroAcc();

    setTargetZeroOmega();
    setTargetZeroAngAcc();

    trajectory_mode_ = false;

    traj_generator_ptr_.reset();

    return;
  }

  // asynchronous with generateNewTrajectory
  if (traj_generator_ptr_.get() == nullptr) {
    RCLCPP_WARN(NAV_LOGGER, "[Trajectory] Terminate in trajectory mode since traj_generator_ptr_ is empty");
    return;
  }

  // find the target pose at t from trajectory
  agi::QuadState target_state = traj_generator_ptr_->getState(t);
  setTargetPos(KDL::Vector(target_state.p(0), target_state.p(1), target_state.p(2)));
  setTargetVel(KDL::Vector(target_state.v(0), target_state.v(1), target_state.v(2)));
  setTargetAcc(KDL::Vector(target_state.a(0), target_state.a(1), target_state.a(2)));

  double target_yaw = target_state.getYaw();
  double target_omega_z = target_state.w(2);
  double target_ang_acc_z = target_state.tau(2);
  setTargetYaw(target_yaw);
  setTargetOmegaZ(target_omega_z);
  setTargetAngAccZ(target_ang_acc_z);

  KDL::Vector curr_pos = estimator_->getCogPos(estimate_mode_);
  double yaw_angle = estimator_->getCogEuler(estimate_mode_).z();
  RCLCPP_INFO_THROTTLE(NAV_LOGGER, *(node_->get_clock()), 500,
                       "Trajectory mode, target pos&yaw: [%f, %f, %f, %f], curr pos&yaw: [%f, %f, %f, %f]", \
                       target_state.p(0), target_state.p(1), target_state.p(2), target_yaw, \
                       curr_pos.x(), curr_pos.y(), curr_pos.z(), yaw_angle);

}

void BaseNavigator::reset() {

  estimator_->setSensorFusionFlag(false);
  estimator_->setFlyingFlag(false);

  trajectory_mode_ = false;
  init_height_ = 0;
  land_height_ = 0;
}

void BaseNavigator::startTakeoff() {

  if(getNaviState() == TAKEOFF_STATE) return;

  /* check xy position error in initial state */
  double pos_x_error = getTargetPos().x() - estimator_->getCogPos(estimate_mode_).x();
  double pos_y_error = getTargetPos().y() - estimator_->getCogPos(estimate_mode_).y();
  double pos_xy_error_dist = std::sqrt(pos_x_error * pos_x_error + pos_y_error * pos_y_error);

  if(pos_xy_error_dist > takeoff_xy_pos_tolerance_) {
    RCLCPP_ERROR_STREAM(NAV_LOGGER,
                        "initial xy error distance: " << pos_xy_error_dist <<
                        " is larger than threshold " << takeoff_xy_pos_tolerance_ <<
                        ". switch back to ARM_OFF_STATE");
    setNaviState(STOP_STATE);
  }

  /* check difference in height between arming and takeoff */
  if(fabs(init_height_ - estimator_->getCogPos(estimate_mode_).z()) > takeoff_z_pos_tolerance_) {
    RCLCPP_ERROR_STREAM(NAV_LOGGER, "difference between init height and current height: " <<
                        fabs(init_height_ - estimator_->getCogPos(estimate_mode_).z()) <<
                        " is larger than threshold " << takeoff_z_pos_tolerance_ << ". switch back to ARM_OFF_STATE");
    setNaviState(STOP_STATE);
  }

  if(getNaviState() == ARM_ON_STATE) {
    setNaviState(TAKEOFF_STATE);
    RCLCPP_INFO(NAV_LOGGER, "Takeoff state");
  }
}

void BaseNavigator::motorArming() {

  /* z(altitude) */
  /* check whether there is the fusion for the altitude */
  if(!estimator_->getBasePosStateStatus(State::Z, estimate_mode_)) {
    RCLCPP_ERROR(NAV_LOGGER, "No correct sensor fusion for z(altitude), can not fly");
    return;
  }

  for(const auto& handler: estimator_->getGpsHandlers()) {
    if(handler->getStatus() == Status::ACTIVE) {
      getParam<double>("outdoor_takeoff_height", takeoff_height_, 1.2);
      getParam<double>("outdoor_hover_convergent_duration", hover_convergent_duration_, 0.5);
      getParam<double>("outdoor_xy_convergent_thresh", xy_convergent_thresh_, 0.6);
      getParam<double>("outdoor_z_convergent_thresh", z_convergent_thresh_, 0.05);

      RCLCPP_INFO_STREAM(NAV_LOGGER,
                         "Update the navigation parameters for outdoor flight, takeoff height: " << takeoff_height_ <<
                         "; outdoor_hover_convergent_duration: " << hover_convergent_duration_ <<
                         "; outdoor_xy_convergent_thresh: " << xy_convergent_thresh_ <<
                         "; outdoor_z_convergent_thresh: " << z_convergent_thresh_);

      break;
    }
  }

  setNaviState(START_STATE);
  trajectory_mode_ = false;
  setTargetXyFromCurrentState();
  setTargetPosZ(takeoff_height_);
  setTargetVelZ(0);
  setTargetAccZ(0);
  setInitHeight(estimator_->getCogPos(estimate_mode_).z());
  setTargetYawFromCurrentState();

  RCLCPP_INFO_STREAM(NAV_LOGGER, "init height for takeoff: " << init_height_ << ", target height: " << getTargetPos().z());
  RCLCPP_INFO_STREAM(NAV_LOGGER, "target xy pos: " << "[" << getTargetPos().x() << ", " << getTargetPos().y() << "]");

  RCLCPP_INFO(NAV_LOGGER, "Start state");
}

KDL::Vector BaseNavigator::frameConversion(KDL::Vector origin_val, KDL::Rotation rot) {
  return rot * origin_val;
}

KDL::Vector BaseNavigator::frameConversion(KDL::Vector origin_val, float yaw) {
  return frameConversion(origin_val, KDL::Rotation::RPY(0, 0, yaw));
}

void BaseNavigator::flightStatusAckCallback(std_msgs::msg::UInt8::ConstSharedPtr msg) {

  if(msg->data == spinal::msg::FlightConfigCmd::ARM_OFF_CMD) {
    // arming off
    RCLCPP_INFO(NAV_LOGGER, "STOP RES From AERIAL ROBOT");
    setNaviState(ARM_OFF_STATE);
  }

  if(msg->data == spinal::msg::FlightConfigCmd::ARM_ON_CMD) {
    // arming on
    RCLCPP_INFO(NAV_LOGGER, "START RES From AERIAL ROBOT");
    setNaviState(ARM_ON_STATE);
  }

  if(msg->data == spinal::msg::FlightConfigCmd::FORCE_LANDING_CMD) {
    // get the first force landing message from spinal
    RCLCPP_INFO(NAV_LOGGER, "FORCE LANDING MSG From AERIAL ROBOT");
    force_landing_flag_ = true;
  }
}

void BaseNavigator::startCallback(std_msgs::msg::Empty::ConstSharedPtr msg) {
  motorArming();
}

void BaseNavigator::takeoffCallback(std_msgs::msg::Empty::ConstSharedPtr msg) {
  startTakeoff();
}

void BaseNavigator::landCallback(std_msgs::msg::Empty::ConstSharedPtr msg) {

  if(force_att_control_flag_) return;

  if(!teleop_flag_) return;

  setNaviState(LAND_STATE);
  RCLCPP_INFO(NAV_LOGGER, "Land state");
}

void BaseNavigator::haltCallback(const std_msgs::msg::Empty::ConstSharedPtr msg) {

  if(!teleop_flag_) return;

  force_landing_flag_ = true;
  setNaviState(STOP_STATE);

  RCLCPP_INFO(NAV_LOGGER, "Halt state");
}

void BaseNavigator::forceLandingCallback(std_msgs::msg::Empty::ConstSharedPtr msg) {

  spinal::msg::FlightConfigCmd flight_config_cmd;
  flight_config_cmd.cmd = spinal::msg::FlightConfigCmd::FORCE_LANDING_CMD;
  flight_config_pub_->publish(flight_config_cmd);
  force_landing_flag_ = true;

  RCLCPP_INFO(NAV_LOGGER, "Force Landing state");
}

void BaseNavigator::stopTeleopCallback(std_msgs::msg::UInt8::ConstSharedPtr msg) {

  if(msg->data == 1) {
    RCLCPP_WARN(NAV_LOGGER, "stop teleop control");
    teleop_flag_ = false;
  } else if(msg->data == 0) {
    RCLCPP_WARN(NAV_LOGGER, "start teleop control");
    teleop_flag_ = true;
  }
}

void BaseNavigator::setTargetXyFromCurrentState() {

  setXyControlMode(POS_CONTROL_MODE);
  KDL::Vector pos_cog = estimator_->getCogPos(estimate_mode_);
  setTargetPosX(pos_cog.x());
  setTargetPosY(pos_cog.y());

  // set the velocty to zero
  setTargetVelX(0);
  setTargetVelY(0);

  // set the acceleration to zero
  setTargetAccX(0);
  setTargetAccY(0);
}

void BaseNavigator::setTargetZFromCurrentState() {

  KDL::Vector pos_cog = estimator_->getCogPos(estimate_mode_);
  setTargetPosZ(pos_cog.z());

  // set the velocty to zero
  setTargetVelZ(0);

  // set the acceleration to zero
  setTargetAccZ(0);
}

void BaseNavigator::setTargetYawFromCurrentState() {

  double yaw = estimator_->getCogEuler(estimate_mode_).z();
  setTargetYaw(yaw);

  // set the velocty to zero
  setTargetOmegaZ(0);
}

KDL::Vector BaseNavigator::getDeltaPosFromGpsWaypoint() {

#if 0
  auto base_wp = estimator_->getCurrGpsPoint();
  KDL::Rotation convert_frame = KDL::Rotation::RPY(M_PI, 0, 0);
  KDL::Vector gps_waypoint_delta
    = convert_frame * sensor_plugin::Gps::wgs84ToNedLocalFrame(base_wp, target_wp_);
  return gps_waypoint_delta;
#else
  return KDL::Vector::Zero();
#endif
}

void BaseNavigator::gpsWaypointTracking() {

#if 0
  double now_time = node_->get_clock()->now().seconds();

  if(now_time - gps_waypoint_time_ < gps_waypoint_check_du_) return;

  KDL::Vector gps_waypoint_delta = getDeltaPosFromGpsWaypoint()
  if(gps_waypoint_delta.length() < gps_waypoint_threshold_) gps_waypoint_ = false;

  xy_control_mode_ = POS_CONTROL_MODE;

  if(gps_waypoint_delta.length() > vel_nav_threshold_) {
    vel_based_waypoint_ = true;
    xy_control_mode_ = VEL_CONTROL_MODE;
  }

  KDL::Vector target_cog_pos = estimator_->getCogPos(estimate_mode_) + gps_waypoint_delta;
  setTargetPosX(target_cog_pos.x());
  setTargetPosY(target_cog_pos.y());

  delta = gps_waypoint_delta;
  gps_waypoint_time_ = now_time;
#endif
}


void BaseNavigator::rosParamInit()
{
  getParam<bool>("param_verbose", param_verbose_, false);
  getParam<int>("xy_control_mode", xy_control_mode_, 0);
  getParam<double>("takeoff_height", takeoff_height_, 0.0);

  getParam<double>("land_descend_vel",land_descend_vel_, -0.3);
  if (land_descend_vel_ >= 0) {
    RCLCPP_WARN(NAV_LOGGER, "land_descend_vel_ (current value: %f) should be negative", land_descend_vel_);
    land_descend_vel_ = -0.3;
  }

  getParam<double>("takeoff_xy_pos_tolerance", takeoff_xy_pos_tolerance_, 0.3);
  getParam<double>("takeoff_z_pos_tolerance", takeoff_z_pos_tolerance_, 0.3);
  getParam<double>("hover_convergent_duration", hover_convergent_duration_, 1.0);
  getParam<double>("land_check_duration", land_check_duration_, 0.5);
  if (land_check_duration_ < 0.5) {
    RCLCPP_WARN(NAV_LOGGER, "land_check_duration_ (current value: %f) should be not smaller than 0.5", land_check_duration_);
    land_check_duration_ = 0.5;
  }

  getParam<double>("trajectory_reset_duration", trajectory_reset_duration_, 0.5);
  getParam<double>("teleop_reset_duration", teleop_reset_duration_, 0.5);
  getParam<double>("z_convergent_thresh", z_convergent_thresh_, 0.05);
  getParam<double>("xy_convergent_thresh", xy_convergent_thresh_, 0.15);
  getParam<double>("land_pos_convergent_thresh", land_pos_convergent_thresh_, 0.02);
  getParam<double>("land_vel_convergent_thresh", land_vel_convergent_thresh_, 0.05);

  //*** trajectory
  getParam<double>("trajectory_mean_vel", trajectory_mean_vel_, 0.5);
  getParam<double>("trajectory_mean_yaw_rate", trajectory_mean_yaw_rate_, 0.3);
  getParam<double>("trajectory_min_du", trajectory_min_du_, 2.0);
  getParam<bool>("enable_latch_yaw_trajectory", enable_latch_yaw_trajectory_, false);

  //*** auto vel nav
  getParam<double>("nav_vel_limit", nav_vel_limit_, 0.2);
  getParam<double>("vel_nav_threshold", vel_nav_threshold_, 0.4);
  getParam<double>("vel_nav_gain", vel_nav_gain_, 1.0);

  //*** gps waypoint
  getParam<double>("gps_waypoint_threshold", gps_waypoint_threshold_, 3.0);
  getParam<double>("gps_waypoint_check_du", gps_waypoint_check_du_, 1.0);

  //*** teleop navigation
  getParam<double>("max_teleop_xy_vel", max_teleop_xy_vel_, 0.5);
  getParam<double>("max_teleop_z_vel", max_teleop_z_vel_, 0.5);
  getParam<double>("max_teleop_yaw_vel", max_teleop_yaw_vel_, 0.5);
  getParam<double>("max_teleop_rp_angle", max_teleop_rp_angle_, 0.2);
  getParam<double>("joy_stick_deadzone", joy_stick_deadzone_, 0.2);
  getParam<double>("joy_stick_heart_beat_du", joy_stick_heart_beat_du_, 2.0);
  getParam<double>("force_landing_to_halt_du", force_landing_to_halt_du_, 1.0);
  getParam<bool>("check_joy_stick_heart_beat", check_joy_stick_heart_beat_, false);
  getParam<std::string>("teleop_local_frame", teleop_local_frame_, std::string("root"));

  getParam<int>("bat_info.bat_cell", bat_cell_, 0); // Lipo battery cell
  getParam<double>("bat_info.low_voltage_thre", low_voltage_thre_, 0.1); // Lipo battery cell
  getParam<double>("bat_info.high_voltage_cell_thre", high_voltage_cell_thre_, 1.0);
  getParam<double>("bat_info.bat_resistance", bat_resistance_, 0.0); //Battery internal resistance
  getParam<double>("bat_info.bat_resistance_voltage_rate", bat_resistance_voltage_rate_, 0.0); //Battery internal resistance_voltage_rate
  getParam<double>("bat_info.hovering_current", hovering_current_, 0.0); // current at hovering state
}
