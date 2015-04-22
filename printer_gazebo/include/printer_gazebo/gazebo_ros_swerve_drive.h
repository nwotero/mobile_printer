//Adapted from gazebo_ros_skid_steer_drive.h

#ifndef GAZEBO_ROS_SWERVE_DRIVE_H_
#define GAZEBO_ROS_SWERVE_DRIVE_H_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosSwerveDrive : public ModelPlugin {

    public:
	  GazeboRosSwerveDrive();
      ~GazeboRosSwerveDrive();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void getVelocities();

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string left_front_swerve_joint_name_;
      std::string right_front_swerve_joint_name_;
      std::string left_rear_swerve_joint_name_;
      std::string right_rear_swerve_joint_name_;
      std::string left_front_joint_name_;
      std::string right_front_joint_name_;
      std::string left_rear_joint_name_;
      std::string right_rear_joint_name_;

      double wheel_separation_;
      double wheel_diameter_;
      double torque;
      double wheel_speed_[4];
      double swerve_speed_;

      physics::JointPtr swerveJoints[4];
      double swerveAngle;
      physics::JointPtr joints[4];

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      tf::TransformBroadcaster *transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;
      bool broadcast_tf_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // Swerve Drive stuff
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_;
      double y_;
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

  };

}


#endif