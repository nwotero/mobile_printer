//Adapted from gazebo_ros_skid_steer_drive.cpp
#include <algorithm>
#include <assert.h>

#include <printer_gazebo/gazebo_ros_swerve_drive.h>

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
    RIGHT_FRONT=0,
    LEFT_FRONT=1,
    RIGHT_REAR=2,
    LEFT_REAR=3,
  };

  GazeboRosSwerveDrive::GazeboRosSwerveDrive() {}

  // Destructor
  GazeboRosSwerveDrive::~GazeboRosSwerveDrive() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  // Load the controller
  void GazeboRosSwerveDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboRosSwerveDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }
    
    this->broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO("GazeboRosSwerveDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO("GazeboRosSwerveDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
          
    } else {
      this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    this->left_front_swerve_joint_name_ = "swerve_left_front_joint";
    if (!_sdf->HasElement("swerveLeftFrontJoint")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <swerveLeftFrontJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_swerve_joint_name_.c_str());
    } else {
      this->left_front_swerve_joint_name_ = _sdf->GetElement("swerveLeftFrontJoint")->Get<std::string>();
    }

    this->right_front_swerve_joint_name_ = "swerve_right_front_joint";
    if (!_sdf->HasElement("swerveRightFrontJoint")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <swerveRightFrontJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_front_swerve_joint_name_.c_str());
    } else {
      this->right_front_swerve_joint_name_ = _sdf->GetElement("swerveRightFrontJoint")->Get<std::string>();
    }

    this->left_rear_swerve_joint_name_ = "swerve_left_back_joint";
    if (!_sdf->HasElement("swerveLeftBackJoint")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <swerveLeftBackJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_rear_swerve_joint_name_.c_str());
    } else {
      this->left_rear_swerve_joint_name_ = _sdf->GetElement("swerveLeftBackJoint")->Get<std::string>();
    }

    this->right_rear_swerve_joint_name_ = "swerve_right_back_joint";
    if (!_sdf->HasElement("swerveRightBackJoint")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <swerveRightBackJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_rear_swerve_joint_name_.c_str());
    } else {
      this->right_rear_swerve_joint_name_ = _sdf->GetElement("swerveRightBackJoint")->Get<std::string>();
    }

    // TODO write error if joint doesn't exist!
    this->left_front_joint_name_ = "left_front_joint";
    if (!_sdf->HasElement("leftFrontJoint")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
    } else {
      this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();
    }

    this->right_front_joint_name_ = "right_front_joint";
        if (!_sdf->HasElement("rightFrontJoint")) {
          ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
              this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
        } else {
          this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();
        }

	this->left_rear_joint_name_ = "left_back_joint";
	if (!_sdf->HasElement("leftRearJoint")) {
	  ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
		  this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	} else {
	  this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();
	}

    this->right_rear_joint_name_ = "right_back_joint";
    if (!_sdf->HasElement("rightRearJoint")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    } else {
      this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();
    }


    // This assumes that front and rear wheel spacing is identical
    /*this->wheel_separation_ = this->parent->GetJoint(left_front_joint_name_)->GetAnchor(0).Distance(
    		this->parent->GetJoint(right_front_joint_name_)->GetAnchor(0));*/

    this->wheel_separation_ = 0.4;

    if (!_sdf->HasElement("wheelSeparation")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
          this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
      this->wheel_separation_ =
        _sdf->GetElement("wheelSeparation")->Get<double>();
    }

    // TODO get this from robot_description
    this->wheel_diameter_ = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
      this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5000.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("GazeboRosSwerveDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
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
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_REAR] = 0;
	  wheel_speed_[LEFT_REAR] = 0;
    swerve_speed_ = 0;

    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
    joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
    joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
    joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

    swerveJoints[LEFT_FRONT] = this->parent->GetJoint(left_front_swerve_joint_name_);
    swerveJoints[RIGHT_FRONT] = this->parent->GetJoint(right_front_swerve_joint_name_);
    swerveJoints[LEFT_REAR] = this->parent->GetJoint(left_rear_swerve_joint_name_);
    swerveJoints[RIGHT_REAR] = this->parent->GetJoint(right_rear_swerve_joint_name_);

    if (!swerveJoints[LEFT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get swerve joint named \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_swerve_joint_name_.c_str());
      gzthrow(error);
    }
    ROS_INFO("Got left front swerve!");

    if (!swerveJoints[RIGHT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get swerve joint named \"%s\"",
          this->robot_namespace_.c_str(), this->right_front_swerve_joint_name_.c_str());
      gzthrow(error);
    }
    ROS_INFO("Got right front swerve!");

    if (!swerveJoints[LEFT_REAR]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get swerve joint named \"%s\"",
          this->robot_namespace_.c_str(), this->left_rear_swerve_joint_name_.c_str());
      gzthrow(error);
    }
    ROS_INFO("Got left back swerve!");

    if (!swerveJoints[RIGHT_REAR]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get swerve joint named \"%s\"",
          this->robot_namespace_.c_str(), this->right_rear_swerve_joint_name_.c_str());
      gzthrow(error);
    }
    ROS_INFO("Got right back swerve!");

    if (!joints[LEFT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[RIGHT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[LEFT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	 gzthrow(error);
   }

   if (!joints[RIGHT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSwerveDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
	 gzthrow(error);
   }

    joints[LEFT_FRONT]->SetMaxForce(0, torque);
    joints[RIGHT_FRONT]->SetMaxForce(0, torque);
    joints[LEFT_REAR]->SetMaxForce(0, torque);
    joints[RIGHT_REAR]->SetMaxForce(0, torque);

    swerveJoints[LEFT_FRONT]->SetMaxForce(0, torque);
    swerveJoints[RIGHT_FRONT]->SetMaxForce(0, torque);
    swerveJoints[LEFT_REAR]->SetMaxForce(0, torque);
    swerveJoints[RIGHT_REAR]->SetMaxForce(0, torque);

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO("Starting GazeboRosSwerveDrive Plugin (ns = %s)!", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosSwerveDrive::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosSwerveDrive::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosSwerveDrive::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosSwerveDrive::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getVelocities();
      joints[LEFT_FRONT]->SetVelocity(0, wheel_speed_[LEFT_FRONT] / wheel_diameter_);
      joints[RIGHT_FRONT]->SetVelocity(0, wheel_speed_[RIGHT_FRONT] / wheel_diameter_);
      joints[LEFT_REAR]->SetVelocity(0, wheel_speed_[LEFT_REAR] / wheel_diameter_);
      joints[RIGHT_REAR]->SetVelocity(0, wheel_speed_[RIGHT_REAR] / wheel_diameter_);
      
      swerveJoints[LEFT_FRONT]->SetVelocity(0, swerve_speed_);
      swerveJoints[RIGHT_FRONT]->SetVelocity(0, swerve_speed_);
      swerveJoints[LEFT_REAR]->SetVelocity(0, swerve_speed_);
      swerveJoints[RIGHT_REAR]->SetVelocity(0, swerve_speed_);

      last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void GazeboRosSwerveDrive::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosSwerveDrive::getVelocities() {
    const double pi = 3.14159;
    boost::mutex::scoped_lock scoped_lock(lock);
    swerveAngle = swerveJoints[LEFT_FRONT]->GetAngle(0).Degree();

    double vr = sqrt(pow(x_, 2) + pow(y_, 2));
    double swerve_angle_d = (x_ == 0 && y_ == 0) ? swerveAngle : 
      (x_ != 0) ? atan2(y_, x_) : 
      (y_ > 0) ? pi/2 : -1*pi/2;
    ROS_INFO("Current swerve angle: %f", swerveAngle);
    ROS_INFO("Update swerve angle: %f", swerve_angle_d*180/pi);
    double va = rot_;

    const double kp = 0.5;
    const double tolerance = 0.0174532925;
    double error = swerve_angle_d - swerveAngle;
    ROS_INFO("Error = %f", error);

    if (error > tolerance){
      wheel_speed_[RIGHT_FRONT] = 0;
      wheel_speed_[RIGHT_REAR] = 0;
      wheel_speed_[LEFT_FRONT] = 0;
      wheel_speed_[LEFT_REAR] = 0;
    }

    swerve_speed_ = kp * error;

    wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_REAR] = vr + va * wheel_separation_ / 2.0;

    wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_REAR] = vr - va * wheel_separation_ / 2.0;

    // ROS_INFO("Updating joint velocities.\nDesired linear velocities = x: %f, y: %f, Angular = %f",
    //   x_, y_, rot_);

    // ROS_INFO("Wheel Velocities = RF: %f, RB: %f, LF: %f, LB: %f", 
      //wheel_speed_[RIGHT_FRONT], wheel_speed_[RIGHT_REAR], 
      //wheel_speed_[LEFT_FRONT], wheel_speed_[LEFT_REAR]);

    //ROS_INFO("Swerve speed = %f", swerve_speed_);

  }

  void GazeboRosSwerveDrive::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;

    ROS_INFO("Got command! x: %f, y: %f, rot: %f", x_, y_, rot_);
  }

  void GazeboRosSwerveDrive::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosSwerveDrive::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame =
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // TODO create some non-perfect odometry!
    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {

    	transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            odom_frame, base_footprint_frame));

    }

    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.01;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosSwerveDrive)
}