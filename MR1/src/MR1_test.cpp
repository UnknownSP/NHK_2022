#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <thread>     
#include <chrono>


constexpr double pi = 3.141592653589793238462643383279502884L;

namespace MR1{


enum class ControllerCommands : uint16_t
{
// change mode
    manual, 
    shutdown, // shutdown
    recover_current,
    recover_velocity, 
    recover_position,
    arm_home,
    clear_flag,
// collect the arrow

    Cyl_Catch_grab,
    Cyl_Catch_release,
    Cyl_Lift_up,
    Cyl_Lift_down,
    Cyl_Arm_grab,
    Cyl_Arm_release,

    arm_rotate,
    arm_rotate_to_grab_arrow,
    arm_rotate_to_grab_table,
    arm_rotate_to_load,
    ArmRotate_to_RotStart,

    ArmRotateToRotStart_Vel,
    ArmRotateToRotStart_Pos,
    ArmRotateToPickArrow,
    ArmRotateToAdjustArrow,
    ArmRotateToZeroDeg,
    ArmRotateToPickRackArrow,
    ArmRotateToAdjustRackArrow,
    ArmRotateToAdjustRackArrow_high,

    ArmRotateToAvoid,

    ArmRotateToRotStandby,

    WaitPickRackArrow,

    launch_start,
    launch_start_wait,
    launch_start_waitstart,

    adjust_arm_to_launch,
// launch_and_home the arrow
    launch_and_home,
    launch_short_start,
    launch_medium_start,
    launch_long_start,
// related to delay
    set_delay_10ms,
    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,
    delay,
    wait_next_pressed,

    Pitch_Homing,
    Pitch_MV_init,
    Pitch_MV_launch,
    Pitch_MV_fast_launch,
    Pitch_MV_Zero,
    Pitch_MV_avoid_arm,
    Pitch_recover,
};

enum class SolenoidValveCommands : uint8_t
{
    shutdown_cmd      = 0b000000,
    recover_cmd       = 0b000001,
    
    Cyl_Catch_release_cmd   = 0b0100000,//default open
    lift_arrow_cmd          = 0b1000000,//default down
    Cyl_Arm_cmd             = 0b0000001,//default release
    Cyl_GrabTable_cmd       = 0b0010000,

};

enum class MotorCommands : uint8_t
{
    shutdown_cmd      = 0x00,
    recover_cmd       = 0x01,
    homing_cmd        = 0x02,
    homing_pitch_cmd  = 0x10,
    get_status        = 0x03,
    recover_current   = 0x04,
	recover_velocity  = 0x05,
	recover_position  = 0x06,
    
};

enum class OpMode : uint8_t
{
    def,         
    full_op,   
};


class MR1_nodelet_main : public nodelet::Nodelet
{
public:
    virtual void onInit();
private:
    /***********************Function**************************/
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void PosCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void control_timer_callback(const ros::TimerEvent &event);
    void shutdown();
    void recover();
    void set_delay(double delay_s);
    void delay_start(double delay_s);
    /***********************Function**************************/
    
    
    ros::NodeHandle nh;
  	ros::NodeHandle _nh;
    ros::NodeHandle nh_MT;
    ros::Timer control_timer;
    
    /***********************Pub&Sub**************************/
    ros::Subscriber joy_sub;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;

    ros::Publisher SolenoidCmd_pub;
    ros::Publisher SolenoidOrder_pub;
	std_msgs::UInt8 solenoid_order_msg;
    
    std_msgs::UInt8 act_conf_cmd_msg;
	std_msgs::UInt8 shirasu_cmd_msg;

    ros::Publisher launch1_CmdPub;
    ros::Publisher launch2_CmdPub;
    ros::Publisher launch3_CmdPub;
    ros::Publisher launch1_VelPub;
    ros::Publisher launch2_VelPub;
    ros::Publisher launch3_VelPub;
    ros::Publisher pitch_CmdPub;
    ros::Publisher yaw_CmdPub;
    ros::Publisher pitch_PosPub;
    ros::Publisher yaw_PosPub;

	ros::Subscriber Mouse_sub;
    /***********************Pub&Sub**************************/

    bool _a = false;
    bool _b = false;
    bool _x = false;
    bool _y = false;
    bool _start = false;
    bool _back  = false;
    bool _rightthumb = false;
    bool _leftthumb = false;
    bool _righttrigger = false;
    bool _lefttrigger = false;
    static int _padx;
    static int _pady;
    static int _lb;
    static int _rb;
    static int ButtonA;
    static int ButtonB;
    static int ButtonX;
    static int ButtonY;
    static int ButtonLB;
    static int ButtonRB;
    static int ButtonStart;
    static int ButtonBack;
	static int ButtonLeftThumb;
    static int ButtonRightThumb;
    static int AxisDPadX;
    static int AxisDPadY;
    static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
    static int AxisRightThumbY;
    static int ButtonLeftTrigger;
    static int ButtonRightTrigger;

    int currentCommandIndex = 0;
    static const std::vector<ControllerCommands> SetLaunchPosi_commands;
    static const std::vector<ControllerCommands> manual_all;
    const std::vector<ControllerCommands> *command_list;


    /***********************Valiables**************************/
    int _delay_s = 0;
    bool _command_ongoing = false;
    bool _is_manual_enabled = true;

    double delay_time = 0.0;
    uint8_t lastSolenoidOrder = 0b0000000;
    double mouse_position_x;
    double mouse_position_y;

    std_msgs::Float64 launch_VelMsg[3];
    /***********************Valiables**************************/
};

int MR1_nodelet_main::_padx = 0;
int MR1_nodelet_main::_pady = 0;
int MR1_nodelet_main::_lb = 0;
int MR1_nodelet_main::_rb = 0;


int MR1_nodelet_main::ButtonA = 1;
int MR1_nodelet_main::ButtonB = 2;
int MR1_nodelet_main::ButtonX = 0;
int MR1_nodelet_main::ButtonY = 3;
int MR1_nodelet_main::ButtonLB = 4;
int MR1_nodelet_main::ButtonRB = 5;
int MR1_nodelet_main::ButtonBack = 8;
int MR1_nodelet_main::ButtonStart = 9;
int MR1_nodelet_main::ButtonLeftThumb = 6;
int MR1_nodelet_main::ButtonRightThumb = 7;

int MR1_nodelet_main::AxisDPadX = 4;
int MR1_nodelet_main::AxisDPadY = 5;
int MR1_nodelet_main::AxisLeftThumbX = 0;
int MR1_nodelet_main::AxisLeftThumbY = 1;
int MR1_nodelet_main::AxisRightThumbX = 2;
int MR1_nodelet_main::AxisRightThumbY = 3;
int MR1_nodelet_main::ButtonLeftTrigger = 10;
int MR1_nodelet_main::ButtonRightTrigger = 11;

const std::vector<ControllerCommands> MR1_nodelet_main::manual_all(
    {
        ControllerCommands::manual
    }
);

const std::vector<ControllerCommands> MR1_nodelet_main::SetLaunchPosi_commands(
    {
        ControllerCommands::Cyl_Arm_grab,
        ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::Pitch_MV_Zero,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::recover_velocity,
        ControllerCommands::ArmRotateToRotStart_Vel,
        ControllerCommands::recover_position,
        ControllerCommands::ArmRotateToRotStart_Pos,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Arm_release,
        ControllerCommands::Pitch_recover,
        //ControllerCommands::Pitch_MV_fast_launch,
        ControllerCommands::Pitch_MV_launch,
    }
);

void MR1_nodelet_main::onInit(void)
{
    nh = getNodeHandle();
    nh_MT = getMTNodeHandle();
    _nh = getPrivateNodeHandle();

    nh.getParam("ButtonA", ButtonA);
    nh.getParam("ButtonB", ButtonB);
    nh.getParam("ButtonX", ButtonX);
    nh.getParam("ButtonY", ButtonY);
    nh.getParam("ButtonLB", ButtonLB);
    nh.getParam("ButtonRB", ButtonRB);
    nh.getParam("ButtonStart", ButtonStart);
    nh.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh.getParam("ButtonRightThumb", ButtonRightThumb);
	nh.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh.getParam("AxisRightThumbX", AxisRightThumbX);
    nh.getParam("AxisRightThumbY", AxisRightThumbY);
    nh.getParam("AxisDPadX", AxisDPadX);
    nh.getParam("AxisDPadY", AxisDPadY);

    this->control_timer = nh.createTimer(ros::Duration(0.01), &MR1_nodelet_main::control_timer_callback, this);
    NODELET_INFO("MR1 node has started.");

    this->command_list = &MR1_nodelet_main::manual_all;


	/*******************pub & sub*****************/
    this->joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &MR1_nodelet_main::joyCallback, this);
    this->cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	this->SolenoidCmd_pub = nh.advertise<std_msgs::UInt8>("solenoid_cmd", 1);
    this->SolenoidOrder_pub = nh.advertise<std_msgs::UInt8>("solenoid_order", 1);
    this->launch1_CmdPub = nh.advertise<std_msgs::UInt8>("launch1_cmd", 1);
    this->launch2_CmdPub = nh.advertise<std_msgs::UInt8>("launch2_cmd", 1);
    this->launch3_CmdPub = nh.advertise<std_msgs::UInt8>("launch3_cmd", 1);
    this->launch1_VelPub = nh.advertise<std_msgs::Float64>("launch1_vel", 1);
    this->launch2_VelPub = nh.advertise<std_msgs::Float64>("launch2_vel", 1);
    this->launch3_VelPub = nh.advertise<std_msgs::Float64>("launch3_vel", 1);
    this->pitch_CmdPub = nh.advertise<std_msgs::UInt8>("pitch_cmd", 1);
    this->pitch_PosPub = nh.advertise<std_msgs::Float64>("pitch_pos", 1);
    this->yaw_CmdPub = nh.advertise<std_msgs::UInt8>("yaw_cmd", 1);
    this->yaw_PosPub = nh.advertise<std_msgs::Float64>("yaw_pos", 1);

    this->Mouse_sub = nh.subscribe<geometry_msgs::Twist>("mouse_vel", 10, &MR1_nodelet_main::PosCallback, this);
	/*******************pub & sub*****************/

	/*******************parameter*****************/
    //_nh.param("launch_long_vel", launch_long_vel, 0.0);
	/*******************parameter*****************/

    launch_VelMsg[0].data = 0.0;
    launch_VelMsg[1].data = 0.0;
    launch_VelMsg[2].data = 0.0;
}

/**************************************************************************************/
void MR1_nodelet_main::PosCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	this->mouse_position_x = msg->linear.x;
	this->mouse_position_y = msg->linear.y;
    NODELET_INFO("x : %f, y : %f", this->mouse_position_x, this->mouse_position_y);
}

//void MR1_nodelet_main::Cyl_Arm_grab_arrow(void){
//    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::Cyl_Arm_cmd;
//    this->solenoid_order_msg.data = this->lastSolenoidOrder;
//    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
//    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
//}
//
//void MR1_nodelet_main::Cyl_Arm_release_arrow(void){
//    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::Cyl_Arm_cmd;
//    this->solenoid_order_msg.data = this->lastSolenoidOrder;
//    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
//    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
//}
//void MR1_nodelet_main::ArmRotate_To_TargetPosi(double position){
//    this->arm_position_msg.data = position;
//    this->ArmVal_pub.publish(arm_position_msg);
//}

/**************************************************************************************/
void MR1_nodelet_main::shutdown(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    launch1_CmdPub.publish(act_conf_cmd_msg);
    launch2_CmdPub.publish(act_conf_cmd_msg);
    launch3_CmdPub.publish(act_conf_cmd_msg);
    pitch_CmdPub.publish(act_conf_cmd_msg);
    yaw_CmdPub.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
}

void MR1_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    launch1_CmdPub.publish(act_conf_cmd_msg);
    launch2_CmdPub.publish(act_conf_cmd_msg);
    launch3_CmdPub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    pitch_CmdPub.publish(act_conf_cmd_msg);
    yaw_CmdPub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
}

//void MR1_nodelet_main::homing(void){
//    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
//    ArmCmd_pub.publish(act_conf_cmd_msg);
//    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
//    ArmCmd_pub.publish(act_conf_cmd_msg);
//}

void MR1_nodelet_main::set_delay(double delay_s)
{
    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

void MR1_nodelet_main::delay_start(double delay_s)
{
    this->delay_time = ros::Time::now().toSec() + delay_s;
    while(this->delay_time > ros::Time::now().toSec());
}

void MR1_nodelet_main::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    static bool control_invert = false;
    static bool last_start;
    static bool last_back;
    static bool last_x;
    static bool last_y;
    static bool last_rb;
    static bool last_lb;
    static bool last_padx;
    static bool last_pady;
	/******/
	static bool last_rightthumb;
    static bool last_leftthumb;
	/******/
    static bool last_righttrigger;
    static bool last_leftrigger;
    //static int last_dpadXCmd = 0;

    static bool _a_enable = false;
    static bool _b_enable = false;
    static bool _Cyl_catch = false;
    static bool _x_enable = false;
    static bool _Cyl_arm = false;
    static bool _Cyl_table = false;

    this->_a = joy->buttons[ButtonA];
    this->_b = joy->buttons[ButtonB];
    this->_x = joy->buttons[ButtonX];
    this->_y = joy->buttons[ButtonY];
    this->_lb = joy->buttons[ButtonLB];
    this->_rb = joy->buttons[ButtonRB];
    this->_padx = joy->axes[AxisDPadX];
    this->_pady = joy->axes[AxisDPadY];
    this->_rightthumb = joy->buttons[ButtonRightThumb];
    this->_leftthumb = joy->buttons[ButtonLeftThumb];
    this->_righttrigger = joy->buttons[ButtonRightTrigger];
    this->_lefttrigger = joy->buttons[ButtonLeftTrigger];
    

    this->_start = joy->buttons[ButtonStart];
    this->_back  = joy->buttons[ButtonBack];

    //std::vector<double> throw_pos_fixed = { 0+this->throw_position_observed, 2*pi+this->throw_position_observed, -2*pi+this->throw_position_observed };

    if (_start)
    {
        this->recover();
    }
    if (_back)
    {
        this->shutdown();
        this->_command_ongoing = false;
    }
   
    if (_righttrigger && (_padx != -1))
    {   
       
    }
    if(_rb){
        if(_a && _a_enable){
            this->launch_VelMsg[0].data -= 100.0;
            _a_enable = false;
        }
        if(_b && _b_enable){
            this->launch_VelMsg[1].data -= 100.0;
            _b_enable = false;
        }
        if(_x && _x_enable){
            this->launch_VelMsg[2].data -= 100.0;
            _x_enable = false;
        }
    }else{
        if(_a && _a_enable){
            this->launch_VelMsg[0].data += 100.0;
            _a_enable = false;
        }

        if(_b && _b_enable){
            this->launch_VelMsg[1].data += 100.0;
            _b_enable = false;
        }

        if(_x && _x_enable){
            this->launch_VelMsg[2].data += 100.0;
            _x_enable = false;
        }
    }
    if(!_a){
        _a_enable = true;
    }
    if(!_b){
        _b_enable = true;
    }
    if(!_x){
        _x_enable = true;
    }
    
    launch1_VelPub.publish(this->launch_VelMsg[0]);
    launch2_VelPub.publish(this->launch_VelMsg[1]);
    launch3_VelPub.publish(this->launch_VelMsg[2]);


    if (this->_is_manual_enabled)
    {
        static double recent_vel_x = 0.0;
        static double recent_vel_y = 0.0;
        static double recent_vel_yaw = 0.0;
        double vel_x = joy->axes[AxisLeftThumbX];   
        double vel_y = joy->axes[AxisLeftThumbY];
        //double vel_yaw_l = (joy->buttons[ButtonLeftThumbX] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        //double vel_yaw_r = (joy->buttons[ButtonRightThumbX] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = joy->axes[AxisRightThumbX];//vel_yaw_l + vel_yaw_r;

        //if(vel_x == 0.0 && vel_y == 0.0 && vel_yaw ==0.0){
        //    vel_x = recent_vel_x;
        //    vel_y = recent_vel_y;
        //    vel_yaw = recent_vel_yaw;
        //}
        double vel_norm = hypot(vel_x, vel_y);
        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }
        if(control_invert){
            vel_x *= -1.0;
            vel_y *= -1.0;
        }
        if(joy->buttons[ButtonLeftThumb] >= 1.0){
            this->cmd_vel_msg.linear.x = vel_x * 0.3;
            this->cmd_vel_msg.linear.y = vel_y * 0.3;
            this->cmd_vel_msg.angular.z = -vel_yaw * 0.3;
        }else if(joy->buttons[ButtonRightThumb] >= 1.0){
            this->cmd_vel_msg.linear.x = vel_x * 5;
            this->cmd_vel_msg.linear.y = vel_y * 5;
            this->cmd_vel_msg.angular.z = -vel_yaw * 3.5;
        }else{
            this->cmd_vel_msg.linear.x = vel_x;
            this->cmd_vel_msg.linear.y = vel_y;
            this->cmd_vel_msg.angular.z = -vel_yaw;
        }
        this->cmd_vel_pub.publish(this->cmd_vel_msg);

        recent_vel_x = vel_x;
        recent_vel_y = vel_y;
        recent_vel_yaw = vel_yaw;
    }
    last_start = _start;
    last_back = _back;
    last_x = _x;
    last_y = _y;
    last_rb = _rb;
    last_lb = _lb;
    last_padx = _padx;
    last_pady = _pady;
	/******/
	last_rightthumb = _rightthumb;
    last_leftthumb = _leftthumb;
	/******/
    last_righttrigger= _righttrigger;
    last_leftrigger = _lefttrigger;

	
}

void MR1_nodelet_main::control_timer_callback(const ros::TimerEvent &event)
{ 
    //this->command_list->size() <= (int)this->currentCommandIndex || this->command_list == &this->manual_all
    if (!this->_command_ongoing)
    {
        //NODELET_INFO("control_time_return");
    }

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    //else if(currentCommand == ControllerCommands::recover_current)
    //{
    //    this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_current;
    //    this->ArmCmd_pub.publish(act_conf_cmd_msg);
    //    this->currentCommandIndex++;
    //    NODELET_INFO("home");
    //}
    //else if(currentCommand == ControllerCommands::recover_velocity)
    //{
    //    this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    //    this->ArmCmd_pub.publish(act_conf_cmd_msg);
    //    this->currentCommandIndex++;
    //    NODELET_INFO("velocity");
    //}
    //else if(currentCommand == ControllerCommands::recover_position)
    //{
    //    this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    //    this->ArmCmd_pub.publish(act_conf_cmd_msg);
    //    this->currentCommandIndex++;
    //    NODELET_INFO("position");
    //}
    if (currentCommand == ControllerCommands::set_delay_10ms)
    {
        //set_delay(0.010);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_250ms");
    }
    else if (currentCommand == ControllerCommands::set_delay_250ms)
    {
        set_delay(0.250);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_250ms");
    }
    else if (currentCommand == ControllerCommands::set_delay_500ms)
    {
        set_delay(0.500);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_500ms");
    }
    else if (currentCommand == ControllerCommands::set_delay_1s)
    {
        set_delay(1.000);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_1s");
    }
    else if (currentCommand == ControllerCommands::delay)
    {
        if (this->_delay_s == 0)
        {
            return;
        }

        if (this->_delay_s < ros::Time::now().toSec())
        {
            this->_delay_s = 0;
            this->currentCommandIndex++;
        }
    }
    
    if(this->command_list->size() <= (int)this->currentCommandIndex)
    {
        this->_command_ongoing = false;
        NODELET_INFO("command_list_finish");
        this->command_list = &this->manual_all;
        currentCommandIndex = 0;
    }


}

}
PLUGINLIB_EXPORT_CLASS(MR1::MR1_nodelet_main, nodelet::Nodelet);
