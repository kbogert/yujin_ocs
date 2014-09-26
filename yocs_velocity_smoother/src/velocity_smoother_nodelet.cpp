/**
 * @file /src/velocity_smoother_nodelet.cpp
 *
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <dynamic_reconfigure/server.h>
#include <yocs_velocity_smoother/paramsConfig.h>

#include <ecl/threads/thread.hpp>

#include "yocs_velocity_smoother/velocity_smoother_nodelet.hpp"

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_velocity_smoother {

/*********************
** Implementation
**********************/

#define LINEAR_THRESHOLD 0.001

void VelocitySmoother::reconfigCB(yocs_velocity_smoother::paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %f %f %f %f %f",
           config.speed_lim_v, config.speed_lim_w, config.accel_lim_v, config.accel_lim_w, config.decel_factor);

  speed_lim_v  = config.speed_lim_v;
  speed_lim_w  = config.speed_lim_w;
  accel_lim_v  = config.accel_lim_v;
  accel_lim_w  = config.accel_lim_w;
  decel_factor = config.decel_factor;
  decel_lim_v  = decel_factor*accel_lim_v;
  decel_lim_w  = decel_factor*accel_lim_w;

  calculate_landing_coefficient();

}

// calculate the next pose given a velocity command and a length of time
void VelocitySmoother::dead_reckoning(const geometry_msgs::Twist & vel, geometry_msgs::Vector3 & pose, double time_in_secs) {

  if (vel.angular.z != 0) {
    // Equations 6, 7, and 8
    double prev_z = pose.z;
    pose.z = pose.z + vel.angular.z * time_in_secs;
    pose.x = pose.x + vel.linear.x / vel.angular.z * ( sin(pose.z) - sin(prev_z) );
    pose.y = pose.y + vel.linear.x / vel.angular.z * ( cos(pose.z) - cos(prev_z) );
  } else {
    // Equations 9, 10, and 11
    pose.x = pose.x + vel.linear.x * time_in_secs * cos(pose.z);
    pose.y = pose.y + vel.linear.x * time_in_secs * sin(pose.z);
  }

}

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back((ros::Time::now() - last_cb_time).toSec());
  }
  else
  {
    period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
  }

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = ros::Time::now();

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
    cb_avg_time = 0.1;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  input_active = true;

  // Bound speed with the maximum values
  target_vel.linear.x  =
      msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_v) : std::max(msg->linear.x,  -speed_lim_v);
  target_vel.angular.z =
      msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);

  target_pos = geometry_msgs::Vector3();
  current_pos = geometry_msgs::Vector3();

  calculate_landing_coefficient();
}

// recalculate Cx, equation 47
void VelocitySmoother::calculate_landing_coefficient() {
  // use 95% of the maximum value to allow for some error in the accelleration limits
  if (target_vel.linear.x != 0)
    landing_coef = clamp_abs(0.95 * accel_lim_w / (6 * target_vel.linear.x * target_vel.linear.x ), 0.4);
  else
    landing_coef = 0.4; // What should this be?

}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (robot_feedback == ODOMETRY)
    current_vel = msg->twist.twist;

  // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (robot_feedback == COMMANDS)
    current_vel = *msg;

  // ignore otherwise
}

void VelocitySmoother::spin()
{
  double period = 1.0/frequency;
  ros::Rate spin_rate(frequency);

  while (! shutdown_req && ros::ok())
  {
    if ((input_active == true) && (cb_avg_time > 0.0) &&
        ((ros::Time::now() - last_cb_time).toSec() > std::min(3.0*cb_avg_time, 0.5)))
    {
      // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
      // this, just in case something went wrong with our input, or he just forgot good manners...
      // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
      // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
      // several messages arrive with the same time and so lead to a zero median
      input_active = false;
      if (IS_ZERO_VEOCITY(target_vel) == false)
      {
        ROS_WARN_STREAM("Velocity Smoother : input got inactive leaving us a non-zero target velocity ("
              << target_vel.linear.x << ", " << target_vel.angular.z << "), zeroing...[" << name << "]");
        target_vel = ZERO_VEL_COMMAND;
      }
    }

    if ((robot_feedback != NONE) && (input_active == true) && (cb_avg_time > 0.0) &&
        (((ros::Time::now() - last_cb_time).toSec() > 5.0*cb_avg_time)     || // 5 missing msgs
          (std::abs(current_vel.linear.x  - last_cmd_vel.linear.x)  > 0.2) ||
          (std::abs(current_vel.angular.z - last_cmd_vel.angular.z) > 2.0)))
    {
      // If the publisher has been inactive for a while, or if our current commanding differs a lot
      // from robot velocity feedback, we cannot trust the former; relay on robot's feedback instead
      // This can happen mainly due to preemption of current controller on velocity multiplexer.
      // TODO: current command/feedback difference thresholds are 진짜 arbitrary; they should somehow
      // be proportional to max v and w...
      // The one for angular velocity is very big because is it's less necessary (for example the
      // reactive controller will never make the robot spin) and because the gyro has a 15 ms delay
      ROS_WARN("Using robot velocity feedback (%s) instead of last command: %f, %f, %f",
                robot_feedback == ODOMETRY ? "odometry" : "end commands",
               (ros::Time::now()      - last_cb_time).toSec(),
                current_vel.linear.x  - last_cmd_vel.linear.x,
                current_vel.angular.z - last_cmd_vel.angular.z);
      last_cmd_vel = current_vel;
    }

    geometry_msgs::TwistPtr cmd_vel;
    cmd_vel.reset(new geometry_msgs::Twist(target_vel));

    // Implementation of Koh, K., & Cho, H. (1999). A smooth path tracking algorithm for wheeled mobile robots with dynamic constraints. Journal of Intelligent and Robotic Systems, 367–385. Retrieved from http://link.springer.com/article/10.1023/A:1008045202113
    dead_reckoning(last_cmd_vel, current_pos, period);
    dead_reckoning(target_vel, target_pos, period);

    // reset the position of the vehicle so that we don't try to over-correct when the command is to stop
    if (target_vel.linear.x == 0) {
      current_pos.x = 0;
      current_pos.y = 0;
    }


    // find error in position and velocity (eqs 22-24)
    double err_th = target_pos.z - current_pos.z;

    // prevent micro oscilations when holding position
    if (! (IS_ZERO_VEOCITY(target_vel) && std::abs(current_vel.linear.x) < LINEAR_THRESHOLD && std::abs(current_vel.angular.z) < accel_lim_w * period * 0.6 && std::abs(err_th) < 0.01)) {

      double err_x = (target_pos.x - current_pos.x) * cos(target_pos.z) + (target_pos.y - current_pos.y) * sin(target_pos.z);
      double err_y = (target_pos.x - current_pos.x) * sin(target_pos.z) + (target_pos.y - current_pos.y) * cos(target_pos.z);

      // find new angular velocity command

      // angle and velocity of the landing curve (eqs 37-39)
      double th_p = target_pos.z + atan( 3 * landing_coef * ( std::pow(std::abs(err_y / landing_coef), 2/3.0)) * sign(err_y));
      double omega_p = target_vel.angular.z;

      // Paper convieniently left out this little case >:-|
      if (err_y != 0)
        omega_p += ((2 / std::pow(std::abs(err_y / landing_coef), 1/3.0)) / (1 + tan(th_p - target_pos.z)*tan(th_p - target_pos.z))) * (-target_vel.angular.z * err_x + last_cmd_vel.linear.x * sin(err_th)) * sign(err_y);

      // angle and velocity subject to acceleration constraints (eqs 40-42)
      // Note eq 40 has an error in it, we need to subtract omega_c from omega_p or else the target velocity is ignored and the angular velocity constantly increases
      double omega_s = omega_p - current_vel.angular.z + sqrt(2 * accel_lim_w * std::abs(th_p - current_pos.z)) * sign(th_p - current_pos.z);
      double ang_accel_limited = clamp_abs(omega_s / period, accel_lim_w);

      // find new linear velocity command (eqs 48-51)
      double v_s = target_vel.linear.x - current_vel.linear.x * cos(err_th) + target_vel.angular.z*err_y + sqrt(2 * accel_lim_v * std::abs(err_x)) * sign(err_x);
      double lin_accel_limited = clamp_abs(v_s / period, accel_lim_v);

      cmd_vel->angular.z = last_cmd_vel.angular.z + ang_accel_limited * period;
      cmd_vel->linear.x = last_cmd_vel.linear.x + lin_accel_limited * period;

      // make sure we don't exceed the velocity limits (shouldn't happen anyway, but just in case)
      cmd_vel->angular.z = clamp_abs(cmd_vel->angular.z, speed_lim_w);
      cmd_vel->linear.x = clamp_abs(cmd_vel->linear.x, speed_lim_v);

    }


    smooth_vel_pub.publish(cmd_vel);
    last_cmd_vel = *cmd_vel;

    spin_rate.sleep();
  }
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Dynamic Reconfigure
  dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<yocs_velocity_smoother::paramsConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

  // Optional parameters
  int feedback;
  nh.param("frequency",      frequency,     20.0);
  nh.param("decel_factor",   decel_factor,   1.0);
  nh.param("robot_feedback", feedback, (int)NONE);

  if ((int(feedback) < NONE) || (int(feedback) > COMMANDS))
  {
    ROS_WARN("Invalid robot feedback type (%d). Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)",
             feedback);
    feedback = NONE;
  }

  robot_feedback = static_cast<RobotFeedbackType>(feedback);

  // Mandatory parameters
  if ((nh.getParam("speed_lim_v", speed_lim_v) == false) ||
      (nh.getParam("speed_lim_w", speed_lim_w) == false))
  {
    ROS_ERROR("Missing velocity limit parameter(s)");
    return false;
  }

  if ((nh.getParam("accel_lim_v", accel_lim_v) == false) ||
      (nh.getParam("accel_lim_w", accel_lim_w) == false))
  {
    ROS_ERROR("Missing acceleration limit parameter(s)");
    return false;
  }

  // Deceleration can be more aggressive, if necessary
  decel_lim_v = decel_factor*accel_lim_v;
  decel_lim_w = decel_factor*accel_lim_w;

  target_pos = geometry_msgs::Vector3();
  target_vel = ZERO_VEL_COMMAND;
  current_pos = geometry_msgs::Vector3();
  current_vel = ZERO_VEL_COMMAND;
  landing_coef = 0.2; // This can never be zero, initialize to something sane

  // Publishers and subscribers
  odometry_sub    = nh.subscribe("odometry",      1, &VelocitySmoother::odometryCB, this);
  current_vel_sub = nh.subscribe("robot_cmd_vel", 1, &VelocitySmoother::robotVelCB, this);
  raw_in_vel_sub  = nh.subscribe("raw_cmd_vel",   1, &VelocitySmoother::velocityCB, this);
  smooth_vel_pub  = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  return true;
}


/*********************
** Nodelet
**********************/

class VelocitySmootherNodelet : public nodelet::Nodelet
{
public:
  VelocitySmootherNodelet()  { }
  ~VelocitySmootherNodelet()
  {
    NODELET_DEBUG("Velocity Smoother : waiting for worker thread to finish...");
    vel_smoother_->shutdown();
    worker_thread_.join();
  }

  std::string unresolvedName(const std::string &name) const {
    size_t pos = name.find_last_of('/');
    return name.substr(pos + 1);
  }


  virtual void onInit()
  {
    ros::NodeHandle ph = getPrivateNodeHandle();
    std::string resolved_name = ph.getUnresolvedNamespace(); // this always returns like /robosem/goo_arm - why not unresolved?
    std::string name = unresolvedName(resolved_name); // unresolve it ourselves
    NODELET_DEBUG_STREAM("Velocity Smoother : initialising nodelet...[" << name << "]");
    vel_smoother_.reset(new VelocitySmoother(name));
    if (vel_smoother_->init(ph))
    {
      NODELET_DEBUG_STREAM("Velocity Smoother : nodelet initialised [" << name << "]");
      worker_thread_.start(&VelocitySmoother::spin, *vel_smoother_);
    }
    else
    {
      NODELET_ERROR_STREAM("Velocity Smoother : nodelet initialisation failed [" << name << "]");
    }
  }

private:
  boost::shared_ptr<VelocitySmoother> vel_smoother_;
  ecl::Thread                        worker_thread_;
};

} // namespace yocs_velocity_smoother

PLUGINLIB_EXPORT_CLASS(yocs_velocity_smoother::VelocitySmootherNodelet, nodelet::Nodelet);
