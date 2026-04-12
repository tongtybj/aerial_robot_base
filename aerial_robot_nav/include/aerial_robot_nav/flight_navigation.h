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

#pragma once

/* common utility */
#include <angles/angles.h>

/* ros API */
#include <rclcpp/rclcpp.hpp>

/* aerial_robot_core API */
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <aerial_robot_nav/trajectory/trajectory_reference/polynomial_trajectory.hpp>
// #include <aerial_robot_estimation/sensor/gps.h>

/* ros messages */
#include <aerial_robot_msgs/msg/flight_nav.hpp>
#include <geodesy/utm.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <sensor_msgs/msg/joy.hpp>
#include <spinal/msg/flight_config_cmd.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


using RobotModelPtr = std::shared_ptr<aerial_robot_model::RobotModel>;
using StateEstimatorPtr = std::shared_ptr<aerial_robot_estimation::StateEstimator>;
using FlightConfigPubPtr = rclcpp::Publisher<spinal::msg::FlightConfigCmd>::SharedPtr;

namespace aerial_robot_nav {
  /* control mode */
  enum control_mode {
    POS_CONTROL_MODE,
    VEL_CONTROL_MODE,
    ACC_CONTROL_MODE
  };

  /* control frame */
  enum control_frame {
    WORLD_FRAME, /* global frame, e.g. ENU, mocap */
    LOCAL_FRAME /* head frame which is identical with imu head direction */
  };

  /* navi state */
  enum flight_state {
    ARM_OFF_STATE,
    START_STATE,
    ARM_ON_STATE,
    TAKEOFF_STATE,
    LAND_STATE,
    HOVER_STATE,
    STOP_STATE
  };

  class BaseNavigator {
  public:
    BaseNavigator();

    virtual ~BaseNavigator() = default;

    virtual void initialize(rclcpp::Node::SharedPtr node,
                            RobotModelPtr robot_model,
                            StateEstimatorPtr estimator,
                            double loop_du);
    virtual void update();

    FlightConfigPubPtr getFlightConfigPublisher() { return flight_config_pub_; }

    inline uint8_t getNaviState(){  return navi_state_;}
    inline void setNaviState(const uint8_t  state){ navi_state_ = state;}
    virtual bool isInflightState();

    inline uint8_t getXyControlMode(){  return (uint8_t)xy_control_mode_;}
    inline void setXyControlMode(uint8_t mode){  xy_control_mode_ = mode;}

    inline uint8_t getControlframe(){  return (uint8_t)control_frame_;}
    inline void setControlframe(uint8_t frame_type){  control_frame_ = frame_type;}

    inline bool getXyVelModePosCtrlTakeoff(){  return xy_vel_mode_pos_ctrl_takeoff_;}
    inline bool getForceLandingFlag() {return force_landing_flag_;}
    inline double getForceLandingStartTime() {return force_landing_start_time_;}

    inline KDL::Vector getTargetPos() {return target_pos_;}
    inline KDL::Vector getTargetVel() {return target_vel_;}
    inline KDL::Vector getTargetAcc() {return target_acc_;}
    inline KDL::Vector getTargetRPY() {return target_rpy_;}
    inline KDL::Vector getTargetOmega() {return target_omega_;}
    inline KDL::Vector getTargetAngAcc() {return target_ang_acc_;}

    inline void setTargetPos(KDL::Vector pos) { target_pos_ = pos; }
    inline void setTargetPos(double x, double y, double z) { setTargetPos(KDL::Vector(x, y, z)); }
    inline void addTargetPos(KDL::Vector diff_pos) { target_pos_ += diff_pos; }
    inline void addTargetPos(double x, double y, double z) { addTargetPos(KDL::Vector(x, y, z)); }
    inline void setTargetVel(KDL::Vector vel) { target_vel_ = vel; }
    inline void setTargetVel(double x, double y, double z) { setTargetVel(KDL::Vector(x, y, z)); }
    inline void setTargetZeroVel() { setTargetVel(0,0,0); }
    inline void setTargetAcc(KDL::Vector vel) { target_acc_ = vel; }
    inline void setTargetAcc(double x, double y, double z) { setTargetAcc(KDL::Vector(x, y, z)); }
    inline void setTargetZeroAcc() { setTargetAcc(KDL::Vector(0,0,0)); }

    inline void setTargetRoll(float value) { target_rpy_.x(value); }
    inline void setTargetOmega(KDL::Vector omega) { target_omega_ = omega; }
    inline void setTargetOmega(double x, double y, double z) { setTargetOmega(KDL::Vector(x, y, z)); }
    inline void setTargetZeroOmega() { setTargetOmega(0,0,0); }
    inline void setTargetOmegaX(float value) { target_omega_.x(value); }
    inline void setTargetPitch(float value) { target_rpy_.y(value); }
    inline void setTargetOmegaY(float value) { target_omega_.y(value); }
    inline void setTargetYaw(float value) { target_rpy_.x(value); }
    inline void addTargetYaw(float value) { setTargetYaw(angles::normalize_angle(target_rpy_.z() + value)); }
    inline void setTargetOmegaZ(float value) { target_omega_.z(value); }
    inline void setTargetRPY(KDL::Vector value) { target_rpy_ = value; }
    inline void setTargetAngAcc(KDL::Vector acc) { target_ang_acc_ = acc; }
    inline void setTargetAngAcc(double x, double y, double z) { setTargetAngAcc(KDL::Vector(x, y, z)); }
    inline void setTargetZeroAngAcc() { setTargetAngAcc(KDL::Vector(0,0,0)); }
    inline void setTargetAngAccX(double value) { target_ang_acc_.x(value); }
    inline void setTargetAngAccY(double value) { target_ang_acc_.y(value); }
    inline void setTargetAngAccZ(double value) { target_ang_acc_.z(value); }

    inline void setTargetPosX( float value){  target_pos_.x(value);}
    inline void setTargetVelX( float value){  target_vel_.x(value);}
    inline void setTargetAccX( float value){  target_acc_.x(value);}
    inline void setTargetPosY( float value){  target_pos_.y(value);}
    inline void setTargetVelY( float value){  target_vel_.y(value);}
    inline void setTargetAccY( float value){  target_acc_.y(value);}
    inline void setTargetPosZ( float value){  target_pos_.z(value);}
    inline void setTargetVelZ( float value){  target_vel_.z(value);}
    inline void setTargetAccZ( float value){  target_acc_.z(value);}
    inline void addTargetPosZ( float value){  target_pos_ += KDL::Vector(0, 0, value);}

    inline void setTeleopFlag(bool teleop_flag) { teleop_flag_ = teleop_flag; }
    inline bool getTeleopFlag() { return teleop_flag_; }

    inline const double getInitHeight() const { return init_height_; }
    inline void setInitHeight(double height) { init_height_ = height; }

    void tfPublish();

    uint8_t getEstimateMode(){ return estimate_mode_;}
    void setEstimateMode(uint8_t estimate_mode){ estimate_mode_ = estimate_mode;}

    void generateNewTrajectory(std::vector<geometry_msgs::msg::PoseStamped> path);

    static constexpr uint8_t POS_CONTROL_COMMAND = 0;
    static constexpr uint8_t VEL_CONTROL_COMMAND = 1;

    // abnormal state
    static constexpr uint8_t LOW_BATTERY_STATE = 0x10;
    static constexpr uint8_t FORCE_LANDING_STATE = 0x11;

    // battery check
    static constexpr float VOL_100P =  4.2;
    static constexpr float VOL_90P =  4.085;
    static constexpr float VOL_80P =  3.999;
    static constexpr float VOL_70P =  3.936;
    static constexpr float VOL_60P =  3.883;
    static constexpr float VOL_50P =  3.839;
    static constexpr float VOL_40P =  3.812;
    static constexpr float VOL_30P =  3.791;
    static constexpr float VOL_20P =  3.747;
    static constexpr float VOL_10P =  3.683;
    static constexpr float VOL_0P =  3.209;

  protected:
    /* node handle */
    rclcpp::Node::SharedPtr node_;

    /* publisher */
    rclcpp::Publisher<spinal::msg::FlightConfigCmd>::SharedPtr flight_config_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr flight_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;

    /* subscriber */
    rclcpp::Subscription<aerial_robot_msgs::msg::FlightNav>::SharedPtr navi_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr single_goal_sub_, simple_move_base_goal_sub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flight_status_ack_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_sub_, land_sub_, start_sub_, halt_sub_, force_landing_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr ctrl_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_stick_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr stop_teleop_sub_;

    RobotModelPtr robot_model_;
    StateEstimatorPtr estimator_;

    bool param_verbose_;

    uint8_t navi_state_;

    int  xy_control_mode_;
    int  prev_xy_control_mode_;
    bool xy_vel_mode_pos_ctrl_takeoff_;

    double loop_du_;
    int  control_frame_;
    int estimate_mode_;
    bool  force_att_control_flag_;
    bool trajectory_mode_;
    double force_landing_start_time_;

    double takeoff_xy_pos_tolerance_;
    double takeoff_z_pos_tolerance_;
    double hover_convergent_start_time_;
    double hover_convergent_duration_;
    double land_check_start_time_;
    double land_check_duration_;
    double trajectory_reset_time_;
    double trajectory_reset_duration_;
    double teleop_reset_time_;
    double teleop_reset_duration_;
    double z_convergent_thresh_;
    double xy_convergent_thresh_;
    double land_pos_convergent_thresh_;
    double land_vel_convergent_thresh_;

    /* target value */
    KDL::Vector target_pos_, target_vel_, target_acc_;
    KDL::Vector target_rpy_, target_omega_, target_ang_acc_;

    double takeoff_height_;
    double init_height_;
    double land_height_;
    double land_descend_vel_;

    /* auto vel nav */
    bool vel_based_waypoint_;
    double nav_vel_limit_; // the vel limitation
    double vel_nav_threshold_; // the range (board) to siwtch between vel_nav and pos_nav
    double vel_nav_gain_;

    /* gps waypoint */
    bool gps_waypoint_;
    geographic_msgs::msg::GeoPoint target_wp_;
    double gps_waypoint_time_;
    double gps_waypoint_check_du_;
    double gps_waypoint_threshold_;

    /* teleop */
    bool teleop_flag_;
    bool xy_control_flag_;
    bool force_landing_flag_;
    bool check_joy_stick_heart_beat_;
    bool joy_stick_heart_beat_;

    double max_teleop_xy_vel_;
    double max_teleop_z_vel_;
    double max_teleop_yaw_vel_;
    double max_teleop_rp_angle_;

    double joy_stick_deadzone_;
    double joy_stick_prev_time_;
    double joy_stick_heart_beat_du_;
    double force_landing_to_halt_du_;

    std::string teleop_local_frame_;

    /* trajectory */
    double trajectory_mean_vel_;
    double trajectory_mean_yaw_rate_;
    double trajectory_min_du_;
    bool enable_latch_yaw_trajectory_;
    std::shared_ptr<agi::MinJerkTrajectory> traj_generator_ptr_;

    /* battery info */
    double low_voltage_thre_;
    bool low_voltage_flag_;
    double high_voltage_cell_thre_;
    bool high_voltage_flag_;
    int bat_cell_;
    double bat_resistance_;
    double bat_resistance_voltage_rate_;
    double hovering_current_;

    /* basic navigation command */
    virtual void halt() {}
    virtual void reset();
    void startTakeoff();
    void motorArming();
    virtual void updateLandCommand();

    void setTargetXyFromCurrentState();
    void setTargetZFromCurrentState();
    void setTargetYawFromCurrentState();

    void updatePoseFromTrajectory();
    KDL::Vector frameConversion(KDL::Vector origin_val, KDL::Rotation rot);
    KDL::Vector frameConversion(KDL::Vector origin_val, float yaw);

    /* gps waypoint tracking */
    KDL::Vector getDeltaPosFromGpsWaypoint();
    void gpsWaypointTracking();

    /* ros API */
    virtual void rosParamInit();
    void simpleMoveBaseGoalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void singleGoalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);
    virtual void naviCallback(aerial_robot_msgs::msg::FlightNav::ConstSharedPtr msg);
    virtual void joyStickControl(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);
    void batteryCheckCallback(std_msgs::msg::Float32::ConstSharedPtr msg);
    void flightStatusAckCallback(std_msgs::msg::UInt8::ConstSharedPtr msg);
    void takeoffCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
    void startCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
    void landCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
    void haltCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
    void forceLandingCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
    void stopTeleopCallback(std_msgs::msg::UInt8::ConstSharedPtr stop_msg);

    template<class T> void getParam(std::string param_name, T& param, T default_value) {

      std::string prefix = "navigation";
      node_->get_parameter_or<T>(prefix + "." + param_name, param, default_value);
    }

  };
};
