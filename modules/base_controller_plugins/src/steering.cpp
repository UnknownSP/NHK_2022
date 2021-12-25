/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace base_controller_plugins{

  class Steering : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  
  private:
  	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  	void TimerCallback(const ros::TimerEvent& event);
  	void CalcSteering(double actualDt);
  
  	double MaximumAcceleration;
  	double MaximumVelocity;
  	double RobotRadius;
  	double wheel_radius;
	double speed_coeff;
  
  	bool InvertX = false;
  	bool InvertY = false;
  	bool InvertZ = false;
  
  	bool LimitVelocity = true;
  	bool LimitAcceleration = true;
  
  	ros::NodeHandle nh;
  	ros::NodeHandle _nh;
  
  	ros::Subscriber cmdVel_sub;
  	ros::Timer control_tim;
  
  	ros::Publisher tire0CmdVel_pub;
  	ros::Publisher tire1CmdVel_pub;
  	ros::Publisher tire2CmdVel_pub;
  	ros::Publisher tire3CmdVel_pub;
	ros::Publisher steer0CmdPos_pub;
  	ros::Publisher steer1CmdPos_pub;
  	ros::Publisher steer2CmdPos_pub;
  	ros::Publisher steer3CmdPos_pub;
  
  	double targetVelX;
  	double targetVelY;
  	double targetRotZ;
  
  	double current_time = 0.0;
  	double last_time = 0.0;
  
  	double lastTarget_tire[4];
	double lastTarget_steer[4];
  	std_msgs::Float64 tireCmdVelmsg[4];
	std_msgs::Float64 steerCmdPosmsg[4];
	
	double root_1par2 = 0.70710678118;

    //nav_msgs::Odometry odom_twist;
  	//ros::Publisher odom_twist_pub;
  };
  
  void Steering::onInit(){
    nh = getNodeHandle();
    //constructor
    _nh = getPrivateNodeHandle();
  
  	_nh.param("tire_max_acc", this->MaximumAcceleration, 0.0);
  	_nh.param("tire_max_vel", this->MaximumVelocity, 0.0);
  	_nh.param("robot_radius", this->RobotRadius, 0.49);
  	_nh.param("wheel_radius", this->wheel_radius, 0.0400);
	_nh.param("speed_coeff", this->speed_coeff, 0.0);
  
  	NODELET_INFO("tire_max_acc : %f", this->MaximumAcceleration);
  	NODELET_INFO("tire_max_vel : %f", this->MaximumVelocity);
  	NODELET_INFO("robot_radius : %f", this->RobotRadius);
  
  	if(this->MaximumVelocity < 0)
  	{
  		this->LimitVelocity = false;
  	}
  
  	if(this->MaximumAcceleration < 0)
  	{
  		this->LimitAcceleration = false;
  	}
  
  	_nh.param("invert_x", this->InvertX, false);
  	_nh.param("invert_y", this->InvertY, false);
  	_nh.param("invert_z", this->InvertZ, false);
  
  	tire0CmdVel_pub = nh.advertise<std_msgs::Float64>("tire0_cmd_vel", 1);
  	tire1CmdVel_pub = nh.advertise<std_msgs::Float64>("tire1_cmd_vel", 1);
  	tire2CmdVel_pub = nh.advertise<std_msgs::Float64>("tire2_cmd_vel", 1);
  	tire3CmdVel_pub = nh.advertise<std_msgs::Float64>("tire3_cmd_vel", 1);
	steer0CmdPos_pub = nh.advertise<std_msgs::Float64>("steer0_cmd_pos", 1);
  	steer1CmdPos_pub = nh.advertise<std_msgs::Float64>("steer1_cmd_pos", 1);
  	steer2CmdPos_pub = nh.advertise<std_msgs::Float64>("steer2_cmd_pos", 1);
  	steer3CmdPos_pub = nh.advertise<std_msgs::Float64>("steer3_cmd_pos", 1);
  
  	targetVelX = targetVelY = targetRotZ = 0.0;
  
    int ctrl_freq;
    _nh.param("ctrl_freq", ctrl_freq, 20);

  	lastTarget_tire[0] = 0.0;
  	lastTarget_tire[1] = 0.0;
  	lastTarget_tire[2] = 0.0;
    lastTarget_tire[3] = 0.0;
	lastTarget_steer[0] = 0.0;
  	lastTarget_steer[1] = 0.0;
  	lastTarget_steer[2] = 0.0;
    lastTarget_steer[3] = 0.0;
  
  	tireCmdVelmsg[0].data = 0.0;
  	tireCmdVelmsg[1].data = 0.0;
  	tireCmdVelmsg[2].data = 0.0;
    tireCmdVelmsg[3].data = 0.0;
	steerCmdPosmsg[0].data = 0.0;
	steerCmdPosmsg[1].data = 0.0;
	steerCmdPosmsg[2].data = 0.0;
	steerCmdPosmsg[3].data = 0.0;
  
    //odom_twist = nav_msgs::Odometry();
    //odom_twist_pub = nh.advertise<nav_msgs::Odometry>("odom_twist", 10);

  	cmdVel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Steering::CmdVelCallback, this);
	control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq), &Steering::TimerCallback, this);
    //main
  	NODELET_INFO("base_controller node has started.");
  }

  void Steering::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
  	this->targetVelX = static_cast<double>(msg->linear.x);
  	this->targetVelY = static_cast<double>(msg->linear.y);
  	this->targetRotZ = static_cast<double>(msg->angular.z);
  
  	if(this->InvertX)
  	{
  		this->targetVelX *= -1;
  	}
  	if(this->InvertY)
  	{
  		this->targetVelY *= -1;
  	}
  	if(this->InvertZ)
  	{
  		this->targetRotZ *= -1;
  	}
  
   	//odom_twist.header.frame_id = "/steering/odom";
   	//odom_twist.header.stamp = ros::Time::now();
   	//odom_twist.child_frame_id = "/steering/odom_link";
   	//odom_twist.twist.covariance = {
   	//0.5, 0, 0, 0, 0, 0,  // covariance on gps_x
   	//0, 0.5, 0, 0, 0, 0,  // covariance on gps_y
   	//0, 0, 0.5, 0, 0, 0,  // covariance on gps_z
   	//0, 0, 0, 0.1, 0, 0,  // large covariance on rot x
   	//0, 0, 0, 0, 0.1, 0,  // large covariance on rot y
   	//0, 0, 0, 0, 0, 0.1}; // large covariance on rot z
   	//odom_twist.twist.twist = *msg;
   	//odom_twist_pub.publish(odom_twist);
  }

  void Steering::TimerCallback(const ros::TimerEvent& event)
  {
	CalcSteering(event.current_real.toSec() - event.last_real.toSec());
	tire0CmdVel_pub.publish(tireCmdVelmsg[0]);
	tire1CmdVel_pub.publish(tireCmdVelmsg[1]);
	tire2CmdVel_pub.publish(tireCmdVelmsg[2]);
    tire3CmdVel_pub.publish(tireCmdVelmsg[3]);
	steer0CmdPos_pub.publish(steerCmdPosmsg[0]);
	steer1CmdPos_pub.publish(steerCmdPosmsg[1]);
	steer2CmdPos_pub.publish(steerCmdPosmsg[2]);
    steer3CmdPos_pub.publish(steerCmdPosmsg[3]);
  }

  void Steering::CalcSteering(double actualDt){
  	double speed[4];
	double degree[4];
	double r_theta_sin = RobotRadius * targetRotZ * root_1par2;
	double r_theta_cos = RobotRadius * targetRotZ * root_1par2;
	double x[4] = {
		targetVelX - r_theta_sin,
		targetVelX - r_theta_cos,
		targetVelX + r_theta_sin,
		targetVelX + r_theta_cos,
	};
	double y[4] = {
		targetVelY + r_theta_cos,
		targetVelY - r_theta_sin,
		targetVelY - r_theta_cos,
		targetVelY + r_theta_sin,
	}; 

	for(int i=0;i<4;i++){
		speed[i] = sqrt( pow(x[i],2.0) + pow(y[i],2.0) );
		degree[i] = 2.4 * atan2( y[i], x[i] );
	}
	

  	//speed[0] = -((targetVelX * sin(1 * M_PI / 4))	+ (targetVelY * cos(1 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
  	//speed[1] = -((targetVelX * sin(3 * M_PI / 4))	+ (targetVelY * cos(3 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
  	//speed[2] = -((targetVelX * sin(5 * M_PI / 4))	+ (targetVelY * cos(5 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
    //speed[3] = -((targetVelX * sin(7 * M_PI / 4))	+ (targetVelY * cos(7 * M_PI / 4)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
  
  	double _k = this->speed_coeff;
  
  	if(this->LimitVelocity){
  		for(int i = 0; i < 4; i++){
  			auto _a = fabs(speed[i]);
  			if(_a * _k > this->MaximumVelocity){
  				_k = this->MaximumVelocity / _a;
                NODELET_WARN("An infeasible velocity command detected! You might want to look into it.");
  			}
  		}
  
  		for(int i = 0; i < 4; i++){
  			speed[i] *= _k;
  		}
  	}
  
  	if(this->LimitAcceleration){
  		float maxVelDelta = this->MaximumAcceleration * actualDt;
  
  		_k = 1.0;
  
  		for(int i = 0; i < 4; i++){
  			double diffabs = fabs(speed[i] - lastTarget_tire[i]);
  			if(diffabs * _k > maxVelDelta){
  				_k = maxVelDelta / diffabs;
  				NODELET_WARN("An infeasible acceleration detected! You might want to look into it.");
  			}
  		}
  
  		for(int i = 0; i < 4; i++){
  			speed[i] = _k*lastTarget_tire[i] + ((speed[i] - lastTarget_tire[i]) * _k);
  		}
  	}
  
	//double limit = 500.0;
  	for(int i = 0; i < 4; i++){
		//if(i != 2){
		//	t[i] *= 1.2;
		//}	  
  		this->lastTarget_tire[i] = speed[i];
		//if(fabs(t[i]) >= limit){
		//	if(t[i] > 0.0){
		//		t[i] = limit;
		//	}else{
		//		t[i] = -limit;
		//	}
		//}
  		this->tireCmdVelmsg[i].data = speed[i];
		this->steerCmdPosmsg[i].data = degree[i];
  	}
  }
  
}// namespace base_controller_plugins
PLUGINLIB_EXPORT_CLASS(base_controller_plugins::Steering, nodelet::Nodelet);