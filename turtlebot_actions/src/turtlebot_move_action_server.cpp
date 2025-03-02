/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>

// Include Kobuki messages for hazard detection
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>

class MoveActionServer
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_actions::TurtlebotMoveAction> as_;
  std::string action_name_;

  turtlebot_actions::TurtlebotMoveFeedback feedback_;
  turtlebot_actions::TurtlebotMoveResult result_;
  turtlebot_actions::TurtlebotMoveGoalConstPtr goal_;
  
  ros::Publisher cmd_vel_pub_;
  tf::TransformListener listener_;
  
  // Parameters
  std::string base_frame;
  std::string odom_frame;
  double turn_rate;
  double forward_rate;
  
  // Hazard detection flag and subscribers
  bool hazard_detected_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber cliff_sub_;
  ros::Subscriber wheel_drop_sub_;
  
public:
  MoveActionServer(const std::string name) : 
    nh_("~"), as_(nh_, name, false), action_name_(name)
  {
    // Get parameters
    nh_.param<std::string>("base_frame", base_frame, "base_link");
    nh_.param<std::string>("odom_frame", odom_frame, "odom");
    nh_.param<double>("turn_rate", turn_rate, 0.75);
    nh_.param<double>("forward_rate", forward_rate, 0.25);
    
    // Initialize hazard flag
    hazard_detected_ = false;
    
    // Register the goal and preempt callbacks
    as_.registerGoalCallback(boost::bind(&MoveActionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveActionServer::preemptCB, this));
    
    as_.start();
    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to hazard topics (using the default TurtleBot topic names)
    bumper_sub_ = nh_.subscribe("/mobile_base/events/bumper", 10,
                                &MoveActionServer::bumperCallback, this);
    cliff_sub_ = nh_.subscribe("/mobile_base/events/cliff", 10,
                               &MoveActionServer::cliffCallback, this);
    wheel_drop_sub_ = nh_.subscribe("/mobile_base/events/wheel_drop", 10,
                                    &MoveActionServer::wheelDropCallback, this);
  }

  // Called when a new goal is received
  void goalCB()
  {
    // Reset hazard flag for a new goal.
    hazard_detected_ = false;

    // Accept the new goal
    feedback_.forward_distance = 0.0;
    feedback_.turn_distance = 0.0;
    
    result_.forward_distance = 0.0;
    result_.turn_distance = 0.0;
    
    goal_ = as_.acceptNewGoal();
    
    if (!turnOdom(goal_->turn_distance))
    { 
      as_.setAborted(result_);
      return;
    }
    
    if (driveForwardOdom(goal_->forward_distance))
      as_.setSucceeded(result_);
    else
      as_.setAborted(result_);
  }

  // Preempt callback
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // Publish a stop command
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(stop_cmd);
    as_.setPreempted();
  }

  // Hazard callbacks
  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      ROS_ERROR("Bumper pressed!");
      hazard_detected_ = true;
    }
  }
  
  void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg)
  {
    if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
    {
      ROS_ERROR("Cliff detected!");
      hazard_detected_ = true;
    }
  }
  
  void wheelDropCallback(const kobuki_msgs::WheelDropEvent::ConstPtr& msg)
  {
    if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
    {
      ROS_ERROR("Wheel drop detected!");
      hazard_detected_ = true;
    }
  }

  bool driveForwardOdom(double distance)
  {
    // If the distance to travel is negligible, don't even try.
    if (fabs(distance) < 0.01)
      return true;
    
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;
  
    try
    {
      listener_.waitForTransform(base_frame, odom_frame, 
                                 ros::Time::now(), ros::Duration(1.0));
      
      listener_.lookupTransform(base_frame, odom_frame, 
                                ros::Time(0), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.0;
    base_cmd.linear.x = forward_rate;
    
    if (distance < 0)
      base_cmd.linear.x = -base_cmd.linear.x;
    
    ros::Rate rate(25.0);
    bool done = false;
    while (!done && nh_.ok() && as_.isActive())
    {
      // Process any incoming callbacks.
      ros::spinOnce();

      // Check for preemption
      if (as_.isPreemptRequested())
      {
        ROS_INFO("%s: Preempt requested during driveForwardOdom, stopping.", action_name_.c_str());
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        as_.setPreempted();
        return false;
      }
      
      // Check for hazard
      if (hazard_detected_)
      {
        ROS_ERROR("Hazard detected! Aborting drive forward.");
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        as_.setAborted(result_);
        return false;
      }

      // Send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep(); 
      
      try
      {
        listener_.lookupTransform(base_frame, odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        break;
      }
      // Calculate how far we've moved
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      
      // Update feedback and result.
      feedback_.forward_distance = dist_moved;
      result_.forward_distance = dist_moved;
      as_.publishFeedback(feedback_);

      if (fabs(dist_moved) > fabs(distance))
      {
        done = true;
      }
    }
    // Publish a stop command after moving
    base_cmd.linear.x = 0.0;
    base_cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(base_cmd);

    return done;
  }

  bool turnOdom(double radians)
  {
    // If the angle is negligible, don't even try.
    if (fabs(radians) < 0.01)
      return true;
  
    while(radians < -M_PI) radians += 2 * M_PI;
    while(radians > M_PI) radians -= 2 * M_PI;

    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    try
    {
      listener_.waitForTransform(base_frame, odom_frame, 
                                 ros::Time::now(), ros::Duration(1.0));

      listener_.lookupTransform(base_frame, odom_frame, 
                                ros::Time(0), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = 0.0;
    base_cmd.linear.y = 0.0;
    base_cmd.angular.z = turn_rate;
    if (radians < 0)
      base_cmd.angular.z = -turn_rate;
    
    ros::Rate rate(25.0);
    bool done = false;
    while (!done && nh_.ok() && as_.isActive())
    {
      // Process incoming callbacks.
      ros::spinOnce();

      // Check for preemption
      if (as_.isPreemptRequested())
      {
        ROS_INFO("%s: Preempt requested during turnOdom, stopping.", action_name_.c_str());
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        as_.setPreempted();
        return false;
      }
      
      // Check for hazard
      if (hazard_detected_)
      {
        ROS_ERROR("Hazard detected! Aborting turn.");
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        as_.setAborted(result_);
        return false;
      }
      
      // Send the turn command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();

      try
      {
        listener_.lookupTransform(base_frame, odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        break;
      }
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      double angle_turned = relative_transform.getRotation().getAngle();
      
      // Update feedback and result.
      feedback_.turn_distance = angle_turned;
      result_.turn_distance = angle_turned;
      as_.publishFeedback(feedback_);
      
      if (fabs(angle_turned) > fabs(radians))
        done = true;
    }
    // Publish a stop command after turning
    base_cmd.linear.x = 0.0;
    base_cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(base_cmd);

    return done;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_move_action_server");

  MoveActionServer server("turtlebot_move");
  ros::spin();

  return 0;
}
