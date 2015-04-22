//Adapted from gazebo_ros_skid_steer_drive.cpp
#include <algorithm>
#include <assert.h>

#include <printer_gazebo/gazebo_ros_cylindrical_manipulator.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

  enum {
    SHOULDER=0,
    ELBOW=1,
    PRISM=2,
  };

  double cot(double i) { return(1 / tan(i)); }

  GazeboRosCylindricalManipulator::GazeboRosCylindricalManipulator() {
    a2 = 0.5;
    a3 = 0.5;
    d0 = 0.174;
    d1 = 0.2;
    d3 = 0.5723;
  }

  // Destructor
  GazeboRosCylindricalManipulator::~GazeboRosCylindricalManipulator() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  // Load the controller
  void GazeboRosCylindricalManipulator::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboRosCylindricalManipulator Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }
    
    this->broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
          
    } else {
      this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    this->shoulder_joint_name_ = "shoulder_joint";
    if (!_sdf->HasElement("shoulderJoint")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <shoulderJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->shoulder_joint_name_.c_str());
    } else {
      this->shoulder_joint_name_ = _sdf->GetElement("shoulderJoint")->Get<std::string>();
    }

    // TODO write error if joint doesn't exist!
    this->elbow_joint_name_ = "elbow_joint";
    if (!_sdf->HasElement("elbowJoint")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <elbowJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->elbow_joint_name_.c_str());
    } else {
      this->elbow_joint_name_ = _sdf->GetElement("elbowJoint")->Get<std::string>();
    }

    this->prism_joint_name_ = "prism_joint";
        if (!_sdf->HasElement("prismJoint")) {
          ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <prismJoint>, defaults to \"%s\"",
              this->robot_namespace_.c_str(), this->prism_joint_name_.c_str());
        } else {
          this->prism_joint_name_ = _sdf->GetElement("prismJoint")->Get<std::string>();
        }

    this->torque = 5000.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "mainpulator_cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "manipulator_odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_link";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->odometry_frame_ = "manipulator_odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("GazeboRosCylindricalManipulator Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->GetSimTime();

    // Initialize velocity stuff
    joint_speed_[SHOULDER] = 0;
    joint_speed_[ELBOW] = 0;
    joint_speed_[PRISM] = 0;

    alive_ = true;

    joints[SHOULDER] = this->parent->GetJoint(shoulder_joint_name_);
    joints[ELBOW] = this->parent->GetJoint(elbow_joint_name_);
    joints[PRISM] = this->parent->GetJoint(prism_joint_name_);

    if (!joints[SHOULDER]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosCylindricalManipulator Plugin (ns = %s) couldn't get shoulder hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->shoulder_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[ELBOW]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosCylindricalManipulator Plugin (ns = %s) couldn't get elbow hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->elbow_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[PRISM]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosCylindricalManipulator Plugin (ns = %s) couldn't get prismatic joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->prism_joint_name_.c_str());
	 gzthrow(error);
   }

    joints[SHOULDER]->SetMaxForce(0, torque);
    joints[ELBOW]->SetMaxForce(0, torque);
    joints[PRISM]->SetMaxForce(0, torque);

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO("Starting GazeboRosCylindricalManipulator Plugin (ns = %s)!", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosCylindricalManipulator::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosCylindricalManipulator::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosCylindricalManipulator::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosCylindricalManipulator::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getVelocities();
      joints[SHOULDER]->SetVelocity(0, joint_speed_[SHOULDER]);
      joints[ELBOW]->SetVelocity(0, joint_speed_[ELBOW]);
      joints[PRISM]->SetVelocity(0, joint_speed_[PRISM]);

      last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void GazeboRosCylindricalManipulator::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosCylindricalManipulator::getVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double shoulderAngle = joints[SHOULDER]->GetAngle(0).Radian();
    double elbowAngle = joints[ELBOW]->GetAngle(0).Radian();

    joint_speed_[SHOULDER] = ((cos(shoulderAngle)*cot(elbowAngle)/a2 - sin(shoulderAngle)/a2))*x_ + 
      (cos(shoulderAngle)/a2 + (cot(elbowAngle)*sin(shoulderAngle))/a2)*y_;

    joint_speed_[ELBOW] = (-(a2*cos(shoulderAngle) + a3*cos(shoulderAngle + elbowAngle))/(a2*a3*sin(elbowAngle)))*x_ + 
    (-(a2*sin(shoulderAngle) + a3*sin(shoulderAngle + elbowAngle))/(a2*a3*sin(elbowAngle)))*y_;

    joint_speed_[PRISM] = z_;

  }

  void GazeboRosCylindricalManipulator::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    z_ = cmd_msg->linear.z;
    ROS_INFO("Got new command! x = %f, y = %f, z = %f", x_, y_, z_);
  }

  void GazeboRosCylindricalManipulator::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosCylindricalManipulator::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame =
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    double x, y, z;
    double shoulderAngle = joints[SHOULDER]->GetAngle(0).Radian();
    double elbowAngle = joints[ELBOW]->GetAngle(0).Radian();
    double d2 = joints[PRISM]->GetAngle(0).Radian(); //Bad API, this actually returns meters.

    x = a2*cos(shoulderAngle) + a3*cos(shoulderAngle + elbowAngle); 
    y = a2*sin(shoulderAngle) + a3*sin(shoulderAngle + elbowAngle);
    z = d0 + d2 - d3;

    tf::Quaternion qt(0, 0, 0, 1);
    tf::Vector3 vt(x, y, z);

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {

    	transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            odom_frame, base_footprint_frame));

    }

    // publish odom topic
    odom_.pose.pose.position.x = x;
    odom_.pose.pose.position.y = y;
    odom_.pose.pose.position.z = z;

    odom_.pose.pose.orientation.x = 0;
    odom_.pose.pose.orientation.y = 0;
    odom_.pose.pose.orientation.z = 0;
    odom_.pose.pose.orientation.w = 1;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.00001;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    //float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = x_;
    odom_.twist.twist.linear.y = y_;
    odom_.twist.twist.linear.z = z_;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosCylindricalManipulator)
}