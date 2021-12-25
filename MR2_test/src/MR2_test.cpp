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

namespace dr{


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



class dr_nodelet_main : public nodelet::Nodelet
{
public:
    virtual void onInit();
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void control_timer_callback(const ros::TimerEvent &event);
    void reset_launcher();
    
    
    void reset_launcher_status();
    void Cyl_Arm_grab_arrow();
    void Cyl_Arm_release_arrow();

    void launch_ready();
    void adjust_arm_vel(float rotate_veloccity);
    void adjust_arm_pos(float rotate_position);
    void rotate_arm_to_grab_arrow();
    void rotate_arm_to_grab_table();
    void rotate_arm_to_load_arrow();
    void rotate_arm_to_first_posi();
    void ArmRotate_To_TargetPosi(double position);

    void Cyl_Catch_grab();
    void Cyl_Catch_release();
    void Cyl_Lift_up();
    void Cyl_Lift_down();
    void Cyl_GrabTable_grab();
    void Cyl_GrabTable_release();


    void next_OpMode();
    void back_OpMode();

    void shutdown();
    void recover();
    void homing();
    void pitch_homing();
    void clear_flags();
    void set_delay(double delay_s);

    void delay_start(double delay_s);
    double delay_time = 0.0;

    void change_OpMode();

    ros::NodeHandle nh;
  	ros::NodeHandle _nh;
    ros::NodeHandle nh_MT;

    //int linear_, angular_;
    ros::Subscriber joy_sub;

	/***********************/
	ros::Subscriber ThrowPos_sub;
	/***********************/

    ros::Publisher ArmCmd_pub;
    ros::Publisher ArmVal_pub;
    std_msgs::Float64 arm_vel_msg;
    std_msgs::Float64 arm_position_msg;
    std_msgs::Float64 arm_current_msg;

	/***********************/
    ros::Publisher PitchRightCmd_pub;
  	ros::Publisher PitchRightPos_pub;
    std_msgs::Float64 pitch_right_pos_msg;
    ros::Publisher PitchLeftCmd_pub;
  	ros::Publisher PitchLeftPos_pub;
    std_msgs::Float64 pitch_left_pos_msg;

    ros::Publisher foot_CmdPub0;
    ros::Publisher foot_CmdPub1;
    ros::Publisher foot_CmdPub2;
	ros::Publisher foot_CmdPub3;
    ros::Publisher steer_CmdPub0;
    ros::Publisher steer_ValPub0;
    ros::Publisher steer_CmdPub1;
    ros::Publisher steer_CmdPub2;
	ros::Publisher steer_CmdPub3;
	/**********************/
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;
	/**********************/
    ros::Publisher SolenoidCmd_pub;
    ros::Publisher SolenoidOrder_pub;
	std_msgs::UInt8 solenoid_order_msg;
    uint8_t lastSolenoidOrder = 0b0000000;
	/**********************/
    std_msgs::UInt8 act_conf_cmd_msg;

	/**********************/
	std_msgs::UInt8 shirasu_cmd_msg;
	/**********************/
    ros::Timer control_timer;

    int _delay_s = 0;

    double launch_long_vel;
    double launch_medium_vel;
    double launch_short_vel;
    double launch_long_pos;
    double launch_medium_pos;
    double launch_short_pos;

    double pitchright_init_deg;
    double pitchleft_init_deg;
    double pitchrightpos_1_deg;
    double pitchleftpos_1_deg;
    double pitchrightpos_2_deg;
    double pitchleftpos_2_deg;
    double pitchrightpos_3_deg;
    double pitchleftpos_3_deg;

    double pitchright_avoid_arm_deg;

    double pitchright_temp_deg;
    double pitchleft_temp_deg;

    double Pos_1_RotStart_deg;
    double Pos_2_RotStart_deg;
    double Pos_3_RotStart_deg;
    double Pos_1_RotStop_cur;
    double Pos_2_RotStop_cur;
    double Pos_3_RotStop_cur;

    double Arrow_Pick_Arm_deg;
    double Arrow_Adjust_Arm_deg;
    double Arrow_Pick_Rack_Arm_deg;
    double Arrow_Adjust_Rack_Arm_deg;
    double Arrow_Adjust_Hight_Rack_Arm_deg;
    double ArmAvoid_inner_deg;

	double throw_position_observed = 0;

    bool _lefttrigger_flag = false;

	int dr_mode = 0;
	// 0 : taiki_mode
	// 1 : genten_awase_mode
	// 2 : haji_mode
	// 3 : tohteki_mode
	// 4 : defence_mode

	/************************/
	/************************/

    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);
    bool _command_ongoing = false;
    bool _has_loaded = false;
    bool _initial_pose_finished = false;
    bool _launch_angle_reached = false;
    bool _rotating_to_launch = false;
    bool _is_manual_enabled = true;

    bool _LaunchSet_1 = false;
    bool _LaunchSet_2 = false;
    bool _LaunchSet_3 = false;

    bool _PitchSet_Right = false;
    bool _PitchSet_Left = false;

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
	/***************/
	static int ButtonLeftThumb;
    static int ButtonRightThumb;
	/***************/

    static int AxisDPadX;
    static int AxisDPadY;
    static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
    static int AxisRightThumbY;
    static int ButtonLeftTrigger;
    static int ButtonRightTrigger;

    int currentCommandIndex = 0;

    double last_arm_deg = 0.0;
    double adjust_arm_deg = 0.0;

    static const std::vector<OpMode> opmode;
    static const std::vector<ControllerCommands> launch_short_test_commands;
    static const std::vector<ControllerCommands> launch_medium_test_commands;
    static const std::vector<ControllerCommands> launch_long_test_commands;
    //-------------------------
    static const std::vector<ControllerCommands> launch_start_commands;

    double pitch_1_wall_deg;
    double pitch_2_wall_deg;
    double pitch_3_wall_deg;
    double pitch_4_wall_deg;
    double pitch_5_wall_deg;
    double pitch_2_free_deg;
    double pitch_3_free_deg;
    double pitch_4_free_deg;

    double Pos_1_wall_RotStart_deg;
    double Pos_2_wall_RotStart_deg;
    double Pos_3_wall_RotStart_deg;
    double Pos_4_wall_RotStart_deg;
    double Pos_5_wall_RotStart_deg;
    double Pos_2_free_RotStart_deg;
    double Pos_3_free_RotStart_deg;
    double Pos_4_free_RotStart_deg;
    double Pos_1_wall_RotStart_cur;
    double Pos_2_wall_RotStart_cur;
    double Pos_3_wall_RotStart_cur;
    double Pos_4_wall_RotStart_cur;
    double Pos_5_wall_RotStart_cur;
    double Pos_2_free_RotStart_cur;
    double Pos_3_free_RotStart_cur;
    double Pos_4_free_RotStart_cur;
    double Pos_1_wall_RotStop_deg;
    double Pos_2_wall_RotStop_deg;
    double Pos_3_wall_RotStop_deg;
    double Pos_4_wall_RotStop_deg;
    double Pos_5_wall_RotStop_deg;
    double Pos_2_free_RotStop_deg;
    double Pos_3_free_RotStop_deg;
    double Pos_4_free_RotStop_deg;
    double Pos_1_wall_RotStop_cur;
    double Pos_2_wall_RotStop_cur;
    double Pos_3_wall_RotStop_cur;
    double Pos_4_wall_RotStop_cur;
    double Pos_5_wall_RotStop_cur;
    double Pos_2_free_RotStop_cur;
    double Pos_3_free_RotStop_cur;
    double Pos_4_free_RotStop_cur;

    double RotStart_cur_adjust;
    double RotStart_deg_adjust;
    double RotStop_deg_adjust;
    double RotStart_deg_error;

    bool _LaunchSet_1_wall = false;
    bool _LaunchSet_2_wall = false;
    bool _LaunchSet_3_wall = false;
    bool _LaunchSet_4_wall = false;
    bool _LaunchSet_5_wall = false;
    bool _LaunchSet_2_free = false;
    bool _LaunchSet_3_free = false;
    bool _LaunchSet_4_free = false;
    //-------------------------
    static const std::vector<ControllerCommands> load_test_commands;
    static const std::vector<ControllerCommands> SetLaunchPosi_commands;
    static const std::vector<ControllerCommands> ArmPickup_commands;
    static const std::vector<ControllerCommands> ArmPickup_rack_commands;
    static const std::vector<ControllerCommands> ArmAvoid_commands;
    static const std::vector<ControllerCommands> initial_pose;
    static const std::vector<ControllerCommands> manual_all;
    const std::vector<ControllerCommands> *command_list;
};

int dr_nodelet_main::_padx = 0;
int dr_nodelet_main::_pady = 0;
int dr_nodelet_main::_lb = 0;
int dr_nodelet_main::_rb = 0;


int dr_nodelet_main::ButtonA = 1;
int dr_nodelet_main::ButtonB = 2;
int dr_nodelet_main::ButtonX = 0;
int dr_nodelet_main::ButtonY = 3;
int dr_nodelet_main::ButtonLB = 4;
int dr_nodelet_main::ButtonRB = 5;
int dr_nodelet_main::ButtonBack = 8;
int dr_nodelet_main::ButtonStart = 9;
int dr_nodelet_main::ButtonLeftThumb = 6;
int dr_nodelet_main::ButtonRightThumb = 7;

int dr_nodelet_main::AxisDPadX = 4;
int dr_nodelet_main::AxisDPadY = 5;
int dr_nodelet_main::AxisLeftThumbX = 0;
int dr_nodelet_main::AxisLeftThumbY = 1;
int dr_nodelet_main::AxisRightThumbX = 2;
int dr_nodelet_main::AxisRightThumbY = 3;
int dr_nodelet_main::ButtonLeftTrigger = 10;
int dr_nodelet_main::ButtonRightTrigger = 11;

const std::vector<ControllerCommands> dr_nodelet_main::launch_start_commands(
    {
        ControllerCommands::recover_velocity,
        ControllerCommands::launch_start_waitstart,
        ControllerCommands::launch_start_wait,
        ControllerCommands::recover_current,
        ControllerCommands::launch_start,
        ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::launch_and_home,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::launch_short_test_commands(
    {
        ControllerCommands::recover_current,
        ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::launch_short_start,
        //ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::launch_and_home,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::launch_medium_test_commands(
    {
        //ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::recover_current,
        ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::launch_medium_start,
        ControllerCommands::launch_and_home,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
    }
);
const std::vector<ControllerCommands> dr_nodelet_main::launch_long_test_commands(
    {
        ControllerCommands::recover_current,
        ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::launch_long_start,
        ControllerCommands::launch_and_home,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::load_test_commands(
    {
        //ControllerCommands::arm_home,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Catch_grab,
        //ControllerCommands::Cyl_Arm_release,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Lift_up,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Arm_grab,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Catch_release,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::recover_velocity,
        ControllerCommands::ArmRotate_to_RotStart,
        ControllerCommands::recover_position,
        ControllerCommands::arm_rotate_to_load,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        //ControllerCommands::Cyl_Lift_down,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::initial_pose(

    {
        //ControllerCommands::recover_position,
        //ControllerCommands::Cyl_Catch_release,
        //ControllerCommands::Cyl_Arm_release,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Lift_down,
        ControllerCommands::Pitch_Homing,
        ControllerCommands::arm_home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Pitch_recover,
        ControllerCommands::Pitch_recover,
        ControllerCommands::recover_position,
        //ControllerCommands::Pitch_MV_avoid_arm,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
    } 

);

const std::vector<ControllerCommands> dr_nodelet_main::manual_all(
    {
        ControllerCommands::manual
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::SetLaunchPosi_commands(
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

const std::vector<ControllerCommands> dr_nodelet_main::ArmPickup_commands(
    {
        ControllerCommands::Pitch_recover,
        ControllerCommands::Pitch_recover,
        ControllerCommands::Pitch_MV_avoid_arm,
        ControllerCommands::Cyl_Catch_grab,
        ControllerCommands::Cyl_Arm_release,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Lift_up,
        ControllerCommands::recover_position,
        ControllerCommands::ArmRotateToPickArrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Arm_grab,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Catch_release,
        ControllerCommands::recover_position,
        ControllerCommands::ArmRotateToAdjustArrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Arm_release,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Arm_grab,
        ControllerCommands::Cyl_Lift_down,
        ControllerCommands::recover_position,
        ControllerCommands::ArmRotateToZeroDeg,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //ControllerCommands::arm_home,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::arm_home,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::ArmPickup_rack_commands(
    {   
        ControllerCommands::Pitch_MV_Zero,
        ControllerCommands::Pitch_MV_Zero,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Cyl_Arm_release,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        //ControllerCommands::Pitch_Homing,
        ControllerCommands::recover_position,
        ControllerCommands::ArmRotateToPickRackArrow,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::WaitPickRackArrow,
        ControllerCommands::Cyl_Arm_grab,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        //ControllerCommands::recover_position,
        ControllerCommands::ArmRotateToAdjustRackArrow_high,
        //ControllerCommands::WaitPickRackArrow,
        //ControllerCommands::arm_home,
        //ControllerCommands::arm_home,
        //ControllerCommands::ArmRotateToAdjustRackArrow,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Arm_release,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Arm_grab,
        //ControllerCommands::recover_position,
        //ControllerCommands::ArmRotateToZeroDeg,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //ControllerCommands::arm_home,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::arm_home,
    }
);

const std::vector<ControllerCommands> dr_nodelet_main::ArmAvoid_commands(
    {
        ControllerCommands::recover_position,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::ArmRotateToAvoid,
    }
);

void dr_nodelet_main::onInit(void)
{
    nh = getNodeHandle();
    nh_MT = getMTNodeHandle();
    _nh = getPrivateNodeHandle();

    this->joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &dr_nodelet_main::joyCallback, this);

	/***************************************/
	this->ThrowPos_sub = nh_MT.subscribe<std_msgs::Float32>("motor4_current_val", 10, &dr_nodelet_main::PosCallback, this);
	/***************************************/

    this->ArmVal_pub = nh.advertise<std_msgs::Float64>("arm_val", 1);
    this->ArmCmd_pub = nh.advertise<std_msgs::UInt8>("arm_cmd", 1);

    this->PitchRightCmd_pub = nh.advertise<std_msgs::UInt8>("pitch_right_cmd", 1);
    this->PitchRightPos_pub = nh.advertise<std_msgs::Float64>("pitch_right_cmd_pos", 1);
    this->PitchLeftCmd_pub = nh.advertise<std_msgs::UInt8>("pitch_left_cmd", 1);
    this->PitchLeftPos_pub = nh.advertise<std_msgs::Float64>("pitch_left_cmd_pos", 1);

	/**************************************************/
	this->SolenoidCmd_pub = nh.advertise<std_msgs::UInt8>("solenoid_cmd", 1);
    this->SolenoidOrder_pub = nh.advertise<std_msgs::UInt8>("solenoid_order", 1);
	/**************************************************/
    this->cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    this->foot_CmdPub0 = nh.advertise<std_msgs::UInt8>("foot0_cmd", 1);
    this->foot_CmdPub1 = nh.advertise<std_msgs::UInt8>("foot1_cmd", 1);
    this->foot_CmdPub2 = nh.advertise<std_msgs::UInt8>("foot2_cmd", 1);
	this->foot_CmdPub3 = nh.advertise<std_msgs::UInt8>("foot3_cmd", 1);
    this->steer_CmdPub0 = nh.advertise<std_msgs::UInt8>("steer0_cmd", 1);
    this->steer_ValPub0 = nh.advertise<std_msgs::Float64>("steer0_val", 1);
    this->steer_CmdPub1 = nh.advertise<std_msgs::UInt8>("steer1_cmd", 1);
    this->steer_CmdPub2 = nh.advertise<std_msgs::UInt8>("steer2_cmd", 1);
	this->steer_CmdPub3 = nh.advertise<std_msgs::UInt8>("steer3_cmd", 1);
	/**************************************************/
    //this->hand_unchuck_thres_pub = _nh.advertise<std_msgs::UInt16>("hand/unchuck_thres", 1);


    /*nh_priv.getParam("lift_step_per_mm", this->steps_per_mm);

    std::vector<double> tmp;
    nh_priv.getParam("pick_position", tmp);
    if (tmp.size() == 5)
    {
        this->pick_position = tmp;
    }

    for (double& pos : this->pick_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("pick_pos: %f, %f, %f, %f, %f", this->pick_position[0], this->pick_position[1], this->pick_position[2],
            this->pick_position[3], this->pick_position[4]);*/

	/************************************************************
	std::vector<int> tmp2;
    nh_priv.getParam("solenoid_position", tmp2);
    if (tmp2.size() == 5)
    {
        this->solenoid_position = tmp2;
    }

    for (int& pos : this->solenoid_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("solenoid_pos: %d, %d, %d, %d, %d", this->solenoid_position[0], this->solenoid_position[1], this->solenoid_position[2],
            this->solenoid_position[3], this->solenoid_position[4]);
	************************************************************/
    _nh.param("launch_long_vel", launch_long_vel, 0.0);
    _nh.param("launch_medium_vel", launch_medium_vel, 0.0);
    _nh.param("launch_short_vel", launch_short_vel, 0.0);

    _nh.param("launch_long_pos", launch_long_pos, 0.0);
    _nh.param("launch_medium_pos", launch_medium_pos, 0.0);
    _nh.param("launch_short_pos", launch_short_pos, 0.0);

    _nh.param("pitchright_init_deg", pitchright_init_deg, 0.0);
    _nh.param("pitchleft_init_deg", pitchleft_init_deg, 0.0);
    _nh.param("pitchrightpos_3_deg", pitchrightpos_3_deg, 0.0);
    _nh.param("pitchleftpos_3_deg", pitchleftpos_3_deg, 0.0);
    _nh.param("pitchrightpos_2_deg", pitchrightpos_2_deg, 0.0);
    _nh.param("pitchleftpos_2_deg", pitchleftpos_2_deg, 0.0);
    _nh.param("pitchrightpos_1_deg", pitchrightpos_1_deg, 0.0);
    _nh.param("pitchleftpos_1_deg", pitchleftpos_1_deg, 0.0);

    _nh.param("pitchright_avoid_arm_deg", pitchright_avoid_arm_deg, 0.0);

    _nh.param("Arrow_Pick_Arm_deg", Arrow_Pick_Arm_deg, 0.0);
    _nh.param("Arrow_Adjust_Arm_deg", Arrow_Adjust_Arm_deg, 0.0);
    _nh.param("Arrow_Pick_Rack_Arm_deg", Arrow_Pick_Rack_Arm_deg, 0.0);
    _nh.param("Arrow_Adjust_Rack_Arm_deg", Arrow_Adjust_Rack_Arm_deg, 0.0);
    _nh.param("Arrow_Adjust_Hight_Rack_Arm_deg", Arrow_Adjust_Hight_Rack_Arm_deg, 0.0);
    _nh.param("ArmAvoid_inner_deg", ArmAvoid_inner_deg, 0.0);

    _nh.param("Pos_1_RotStart_deg", Pos_1_RotStart_deg, 0.0);
    _nh.param("Pos_2_RotStart_deg", Pos_2_RotStart_deg, 0.0);
    _nh.param("Pos_3_RotStart_deg", Pos_3_RotStart_deg, 0.0);

    _nh.param("Pos_1_RotStop_cur", Pos_1_RotStop_cur, 0.0);
    _nh.param("Pos_2_RotStop_cur", Pos_2_RotStop_cur, 0.0);
    _nh.param("Pos_3_RotStop_cur", Pos_3_RotStop_cur, 0.0);

    //----------------------------------------
    _nh.param("pitch_1_wall_deg", pitch_1_wall_deg, 0.0);
    _nh.param("pitch_2_wall_deg", pitch_2_wall_deg, 0.0);
    _nh.param("pitch_3_wall_deg", pitch_3_wall_deg, 0.0);
    _nh.param("pitch_4_wall_deg", pitch_4_wall_deg, 0.0);
    _nh.param("pitch_5_wall_deg", pitch_5_wall_deg, 0.0);
    _nh.param("pitch_2_free_deg", pitch_2_free_deg, 0.0);
    _nh.param("pitch_3_free_deg", pitch_3_free_deg, 0.0);
    _nh.param("pitch_4_free_deg", pitch_4_free_deg, 0.0);

    _nh.param("Pos_1_wall_RotStart_deg", Pos_1_wall_RotStart_deg, 0.0);
    _nh.param("Pos_1_wall_RotStart_cur", Pos_1_wall_RotStart_cur, 0.0);
    _nh.param("Pos_2_wall_RotStart_deg", Pos_2_wall_RotStart_deg, 0.0);
    _nh.param("Pos_2_wall_RotStart_cur", Pos_2_wall_RotStart_cur, 0.0);
    _nh.param("Pos_3_wall_RotStart_deg", Pos_3_wall_RotStart_deg, 0.0);
    _nh.param("Pos_3_wall_RotStart_cur", Pos_3_wall_RotStart_cur, 0.0);
    _nh.param("Pos_4_wall_RotStart_deg", Pos_4_wall_RotStart_deg, 0.0);
    _nh.param("Pos_4_wall_RotStart_cur", Pos_4_wall_RotStart_cur, 0.0);
    _nh.param("Pos_5_wall_RotStart_deg", Pos_5_wall_RotStart_deg, 0.0);
    _nh.param("Pos_5_wall_RotStart_cur", Pos_5_wall_RotStart_cur, 0.0);
    _nh.param("Pos_2_free_RotStart_deg", Pos_2_free_RotStart_deg, 0.0);
    _nh.param("Pos_2_free_RotStart_cur", Pos_2_free_RotStart_cur, 0.0);
    _nh.param("Pos_3_free_RotStart_deg", Pos_3_free_RotStart_deg, 0.0);
    _nh.param("Pos_3_free_RotStart_cur", Pos_3_free_RotStart_cur, 0.0);
    _nh.param("Pos_4_free_RotStart_deg", Pos_4_free_RotStart_deg, 0.0);
    _nh.param("Pos_4_free_RotStart_cur", Pos_4_free_RotStart_cur, 0.0);

    _nh.param("Pos_1_wall_RotStop_deg", Pos_1_wall_RotStop_deg, 0.0);
    _nh.param("Pos_1_wall_RotStop_cur", Pos_1_wall_RotStop_cur, 0.0);
    _nh.param("Pos_2_wall_RotStop_deg", Pos_2_wall_RotStop_deg, 0.0);
    _nh.param("Pos_2_wall_RotStop_cur", Pos_2_wall_RotStop_cur, 0.0);
    _nh.param("Pos_3_wall_RotStop_deg", Pos_3_wall_RotStop_deg, 0.0);
    _nh.param("Pos_3_wall_RotStop_cur", Pos_3_wall_RotStop_cur, 0.0);
    _nh.param("Pos_4_wall_RotStop_deg", Pos_4_wall_RotStop_deg, 0.0);
    _nh.param("Pos_4_wall_RotStop_cur", Pos_4_wall_RotStop_cur, 0.0);
    _nh.param("Pos_5_wall_RotStop_deg", Pos_5_wall_RotStop_deg, 0.0);
    _nh.param("Pos_5_wall_RotStop_cur", Pos_5_wall_RotStop_cur, 0.0);
    _nh.param("Pos_2_free_RotStop_deg", Pos_2_free_RotStop_deg, 0.0);
    _nh.param("Pos_2_free_RotStop_cur", Pos_2_free_RotStop_cur, 0.0);
    _nh.param("Pos_3_free_RotStop_deg", Pos_3_free_RotStop_deg, 0.0);
    _nh.param("Pos_3_free_RotStop_cur", Pos_3_free_RotStop_cur, 0.0);
    _nh.param("Pos_4_free_RotStop_deg", Pos_4_free_RotStop_deg, 0.0);
    _nh.param("Pos_4_free_RotStop_cur", Pos_4_free_RotStop_cur, 0.0);

    _nh.param("RotStart_cur_adjust", RotStart_cur_adjust, 0.0);
    _nh.param("RotStart_deg_adjust", RotStart_deg_adjust, 0.0);
    _nh.param("RotStop_deg_adjust", RotStop_deg_adjust, 0.0);
    _nh.param("RotStart_deg_error", RotStart_deg_error, 0.0);
    //----------------------------------------

    nh.getParam("ButtonA", ButtonA);
    nh.getParam("ButtonB", ButtonB);
    nh.getParam("ButtonX", ButtonX);
    nh.getParam("ButtonY", ButtonY);
    nh.getParam("ButtonLB", ButtonLB);
    nh.getParam("ButtonRB", ButtonRB);
    nh.getParam("ButtonStart", ButtonStart);
    nh.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh.getParam("ButtonRightThumb", ButtonRightThumb);
	/***************************/
	nh.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh.getParam("AxisRightThumbX", AxisRightThumbX);
    nh.getParam("AxisRightThumbY", AxisRightThumbY);
    nh.getParam("AxisDPadX", AxisDPadX);
    nh.getParam("AxisDPadY", AxisDPadY);

    this->control_timer = nh.createTimer(ros::Duration(0.05), &dr_nodelet_main::control_timer_callback, this);
    NODELET_INFO("dr node has started.");

    this->command_list = &dr_nodelet_main::manual_all;

}

/**************************************************************************************/
void dr_nodelet_main::PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->throw_position_observed = msg->data;
}

void dr_nodelet_main::Cyl_Arm_grab_arrow(void){
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::Cyl_Arm_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::Cyl_Arm_release_arrow(void){
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::Cyl_Arm_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::rotate_arm_to_grab_table(void){
    this->arm_position_msg.data = -pi/4;
    this->ArmVal_pub.publish(arm_position_msg);
}
void dr_nodelet_main::rotate_arm_to_load_arrow(void){
    //this->arm_position_msg.data = -3.2*pi/2;
    this->arm_position_msg.data = -3.2*pi/2;
    this->ArmVal_pub.publish(arm_position_msg);
}
void dr_nodelet_main::rotate_arm_to_first_posi(void){
    this->arm_position_msg.data = 0;
    this->ArmVal_pub.publish(arm_position_msg);
}
void dr_nodelet_main::ArmRotate_To_TargetPosi(double position){
    this->arm_position_msg.data = position;
    this->ArmVal_pub.publish(arm_position_msg);
}


//minas is load direction

void dr_nodelet_main::Cyl_Catch_grab(void)
{
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::Cyl_Catch_release_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::Cyl_Catch_release(void)
{
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::Cyl_Catch_release_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void dr_nodelet_main::Cyl_Lift_up(void)
{
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::lift_arrow_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}
void dr_nodelet_main::Cyl_Lift_down(void)
{
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::lift_arrow_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}
void dr_nodelet_main::Cyl_GrabTable_grab(void)
{
    this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::Cyl_GrabTable_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}
void dr_nodelet_main::Cyl_GrabTable_release(void)
{
    this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::Cyl_GrabTable_cmd;
    this->solenoid_order_msg.data = this->lastSolenoidOrder;
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
    this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

/**************************************************************************************/
void dr_nodelet_main::shutdown(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    ArmCmd_pub.publish(act_conf_cmd_msg);
    foot_CmdPub0.publish(act_conf_cmd_msg);
    foot_CmdPub1.publish(act_conf_cmd_msg);
    foot_CmdPub2.publish(act_conf_cmd_msg);
    foot_CmdPub3.publish(act_conf_cmd_msg);
    steer_CmdPub0.publish(act_conf_cmd_msg);
    steer_CmdPub1.publish(act_conf_cmd_msg);
    steer_CmdPub2.publish(act_conf_cmd_msg);
    steer_CmdPub3.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
    PitchRightCmd_pub.publish(act_conf_cmd_msg);
    PitchLeftCmd_pub.publish(act_conf_cmd_msg);
}



void dr_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    foot_CmdPub0.publish(act_conf_cmd_msg);
    foot_CmdPub1.publish(act_conf_cmd_msg);
    foot_CmdPub2.publish(act_conf_cmd_msg);
    foot_CmdPub3.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
    PitchRightCmd_pub.publish(act_conf_cmd_msg);
    PitchLeftCmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    ArmCmd_pub.publish(act_conf_cmd_msg);
    steer_CmdPub0.publish(act_conf_cmd_msg);
    steer_CmdPub1.publish(act_conf_cmd_msg);
    steer_CmdPub2.publish(act_conf_cmd_msg);
    steer_CmdPub3.publish(act_conf_cmd_msg);
    //act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    //foot_CmdPub2.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
}

void dr_nodelet_main::homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    ArmCmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    ArmCmd_pub.publish(act_conf_cmd_msg);
}

void dr_nodelet_main::pitch_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    PitchRightCmd_pub.publish(act_conf_cmd_msg);
    PitchLeftCmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_pitch_cmd;
    PitchRightCmd_pub.publish(act_conf_cmd_msg);
    PitchLeftCmd_pub.publish(act_conf_cmd_msg);
}

void dr_nodelet_main::set_delay(double delay_s)
{
    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

void dr_nodelet_main::delay_start(double delay_s)
{
    this->delay_time = ros::Time::now().toSec() + delay_s;
    while(this->delay_time > ros::Time::now().toSec());
}

void dr_nodelet_main::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
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
        Cyl_Lift_down();
        Cyl_Arm_release_arrow();
        Cyl_Catch_release();
        Cyl_GrabTable_release();
        this->last_arm_deg = 0.0;
        this->adjust_arm_deg = 0.0;
    }
    if (_back)
    {
        this->shutdown();
        this->_command_ongoing = false;
        this->_has_loaded = false;
    }
   
    if (_righttrigger && (_padx != -1))
    {   
        //this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
        //this->ArmCmd_pub.publish(act_conf_cmd_msg);
        //this->arm_vel_msg.data = 2;
        //this->ArmVal_pub.publish(arm_vel_msg);
        //this->homing();
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
        steer_CmdPub0.publish(act_conf_cmd_msg);
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
        steer_CmdPub0.publish(act_conf_cmd_msg);
    }
    if(_a && _a_enable){
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
        this->steer_CmdPub0.publish(act_conf_cmd_msg);
        _a_enable = false;
    }
    if(!_a){
        _a_enable = true;
    }

    if(_b && _b_enable){
        this->arm_position_msg.data = 7.0;
        this->steer_ValPub0.publish(arm_position_msg);
        _b_enable = false;
    }
    if(!_b){
        _b_enable = true;
    }

    if(_x && _x_enable){
        this->arm_position_msg.data = 0.0;
        this->steer_ValPub0.publish(arm_position_msg);
        _x_enable = false;
    }
    if(!_x){
        _x_enable = true;
    }
    
    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisLeftThumbX];   
        double vel_y = joy->axes[AxisLeftThumbY];
        //double vel_yaw_l = (joy->buttons[ButtonLeftThumbX] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        //double vel_yaw_r = (joy->buttons[ButtonRightThumbX] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = joy->axes[AxisRightThumbX];//vel_yaw_l + vel_yaw_r;
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

void dr_nodelet_main::control_timer_callback(const ros::TimerEvent &event)
{ 
    //this->command_list->size() <= (int)this->currentCommandIndex || this->command_list == &this->manual_all
    if (!this->_command_ongoing)
    {
        NODELET_INFO("control_time_return");
    }
    ROS_INFO("control_time_return");

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    if(currentCommand == ControllerCommands::arm_home)
    {
        this->homing();
        this->currentCommandIndex++;
        NODELET_INFO("home");
    }
    else if(currentCommand == ControllerCommands::recover_current)
    {
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_current;
        this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("home");
    }
    else if(currentCommand == ControllerCommands::recover_velocity)
    {
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
        this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("velocity");
    }
    else if(currentCommand == ControllerCommands::recover_position)
    {
        this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
        this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("position");
    }
    else if(currentCommand == ControllerCommands::Cyl_Catch_grab)
    {   
        this->Cyl_Catch_grab();
        this->currentCommandIndex++;
        NODELET_INFO("Cyl_Catch_grab");
    }
    else if(currentCommand == ControllerCommands::Cyl_Catch_release)
    {   
        this->Cyl_Catch_release();
        this->currentCommandIndex++;
        NODELET_INFO("Cyl_Catch_release");
    }
    else if(currentCommand == ControllerCommands::Cyl_Lift_up)
    { 
        this->Cyl_Lift_up();
        this->currentCommandIndex++;
        NODELET_INFO("Cyl_Lift_up");
    }
    else if(currentCommand == ControllerCommands::Cyl_Lift_down)
    {  
        this->Cyl_Lift_down();
        this->currentCommandIndex++;
        NODELET_INFO("Cyl_Lift_down");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_grab_arrow)
    { 
        this->rotate_arm_to_grab_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_grab");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_grab_table)
    {   
        this->rotate_arm_to_grab_table(); 
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_set");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_load)
    {   
        this->rotate_arm_to_load_arrow(); 
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_set");
    }
    else if(currentCommand == ControllerCommands::ArmRotate_to_RotStart)
    {
        this->arm_vel_msg.data = -2;
        this->ArmVal_pub.publish(arm_vel_msg);
        //while(this->throw_position_observed >= -3.2*pi/2);
        while(this->throw_position_observed >= -3.4*pi/2);
        this->arm_vel_msg.data = 0;
        this->ArmVal_pub.publish(arm_vel_msg);

        this->currentCommandIndex++;
        NODELET_INFO("ArmRotate_to_RotStart");
    }
    else if(currentCommand == ControllerCommands::ArmRotateToRotStart_Vel)
    {
        this->arm_vel_msg.data = 1;
        this->ArmVal_pub.publish(arm_vel_msg);
        //if(this->_LaunchSet_1_wall){
        //    while(this->throw_position_observed <= this->Pos_1_wall_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_2_wall){
        //    while(this->throw_position_observed <= this->Pos_2_wall_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_3_wall){
        //    while(this->throw_position_observed <= this->Pos_3_wall_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_4_wall){
        //    while(this->throw_position_observed <= this->Pos_4_wall_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_5_wall){
        //    while(this->throw_position_observed <= this->Pos_5_wall_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_2_free){
        //    while(this->throw_position_observed <= this->Pos_2_free_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_3_free){
        //    while(this->throw_position_observed <= this->Pos_3_free_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}else if(this->_LaunchSet_4_free){
        //    while(this->throw_position_observed <= this->Pos_4_free_RotStart_deg+this->RotStart_deg_adjust-0.5){
        //        if(_back){
        //            break;
        //        }
        //    };
        //}
        if(this->throw_position_observed >= 1.5+this->RotStart_deg_adjust-0.5){
            this->arm_vel_msg.data = 0;
            this->ArmVal_pub.publish(arm_vel_msg);

            this->currentCommandIndex++;  
        };
        NODELET_INFO("ArmRotate_to_RotStart");
    }
    else if(currentCommand == ControllerCommands::ArmRotateToRotStart_Pos)
    {   
        //if(this->_LaunchSet_1_wall){
        //    this->ArmRotate_To_TargetPosi(this->Pos_1_wall_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_2_wall){
        //    this->ArmRotate_To_TargetPosi(this->Pos_2_wall_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_3_wall){
        //    this->ArmRotate_To_TargetPosi(this->Pos_3_wall_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_4_wall){
        //    this->ArmRotate_To_TargetPosi(this->Pos_4_wall_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_5_wall){
        //    this->ArmRotate_To_TargetPosi(this->Pos_5_wall_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_2_free){
        //    this->ArmRotate_To_TargetPosi(this->Pos_2_free_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_3_free){
        //    this->ArmRotate_To_TargetPosi(this->Pos_3_free_RotStart_deg+this->RotStart_deg_adjust);
        //}else if(this->_LaunchSet_4_free){
        //    this->ArmRotate_To_TargetPosi(this->Pos_4_free_RotStart_deg+this->RotStart_deg_adjust);
        //}
        this->ArmRotate_To_TargetPosi(1.5+this->RotStart_deg_adjust);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_set");
    }
    else if(currentCommand == ControllerCommands::ArmRotateToPickArrow)
    {   
        this->ArmRotate_To_TargetPosi(this->Arrow_Pick_Arm_deg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::ArmRotateToAdjustArrow)
    {   
        this->ArmRotate_To_TargetPosi(this->Arrow_Adjust_Arm_deg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::ArmRotateToPickRackArrow)
    {   
        this->ArmRotate_To_TargetPosi(this->Arrow_Pick_Rack_Arm_deg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::ArmRotateToAdjustRackArrow)
    {   
        this->ArmRotate_To_TargetPosi(this->Arrow_Adjust_Rack_Arm_deg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::ArmRotateToAdjustRackArrow_high)
    {   
        this->ArmRotate_To_TargetPosi(this->Arrow_Adjust_Hight_Rack_Arm_deg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::ArmRotateToZeroDeg)
    {   
        this->ArmRotate_To_TargetPosi(0.0);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::ArmRotateToAvoid)
    {   
        this->ArmRotate_To_TargetPosi(this->ArmAvoid_inner_deg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::WaitPickRackArrow)
    {
        if(!this->_lefttrigger_flag){

        }else{
            this->_lefttrigger_flag = false;
            this->currentCommandIndex++;
        }
    }
    else if(currentCommand == ControllerCommands::launch_start)
    {
        if(_LaunchSet_1_wall){
            //while(this->throw_position_observed > Pos_1_wall_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_1_wall_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_1_wall_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_2_wall){
            //while(this->throw_position_observed > Pos_2_wall_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_2_wall_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_2_wall_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_3_wall){
            //while(this->throw_position_observed > Pos_3_wall_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_3_wall_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_3_wall_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_4_wall){
            //while(this->throw_position_observed > Pos_4_wall_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_4_wall_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_4_wall_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_5_wall){
            //while(this->throw_position_observed > Pos_5_wall_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_5_wall_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_5_wall_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_2_free){
            //while(this->throw_position_observed > Pos_2_free_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_2_free_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_2_free_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_3_free){
            //while(this->throw_position_observed > Pos_3_free_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_3_free_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_3_free_RotStart_cur + this->RotStart_cur_adjust;
        }else if(_LaunchSet_4_free){
            //while(this->throw_position_observed > Pos_4_free_RotStart_deg+RotStart_deg_error || this->throw_position_observed < Pos_4_free_RotStart_deg-RotStart_deg_error);
            this->arm_vel_msg.data = this->Pos_4_free_RotStart_cur + this->RotStart_cur_adjust;
        }
        //this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_current;
        //this->ArmCmd_pub.publish(act_conf_cmd_msg);
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_short");
    }
    else if(currentCommand == ControllerCommands::launch_start_wait)
    {
        if(_LaunchSet_1_wall){
            if(this->throw_position_observed > Pos_1_wall_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_2_wall){
            if(this->throw_position_observed > Pos_2_wall_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_3_wall){
            if(this->throw_position_observed > Pos_3_wall_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_4_wall){
            if(this->throw_position_observed > Pos_4_wall_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_5_wall){
            if(this->throw_position_observed > Pos_5_wall_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_2_free){
            if(this->throw_position_observed > Pos_2_free_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_3_free){
            if(this->throw_position_observed > Pos_3_free_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }else if(_LaunchSet_4_free){
            if(this->throw_position_observed > Pos_4_free_RotStart_deg+RotStart_deg_adjust){
                this->currentCommandIndex++;
            }
        }
        NODELET_INFO("adjust_launchers_velocity_short");
    }
    else if(currentCommand == ControllerCommands::launch_start_waitstart)
    {
        this->arm_vel_msg.data = 0.2;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::launch_short_start)
    {
        this->arm_vel_msg.data = this->launch_short_vel;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_short");
    }
    else if(currentCommand == ControllerCommands::launch_medium_start)
    {
        this->arm_vel_msg.data = this->launch_medium_vel;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_medium");
    }
    else if(currentCommand == ControllerCommands::launch_long_start)
    {
        this->arm_vel_msg.data = this->launch_long_vel;
        this->ArmVal_pub.publish(arm_vel_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_velocity_long");
    }
    else if(currentCommand == ControllerCommands::launch_and_home)
    {   
        //this->pitch_right_pos_msg.data = 0;
        //this->PitchRightPos_pub.publish(this->pitch_right_pos_msg);
        //this->pitch_left_pos_msg.data = 0;
        //this->PitchLeftPos_pub.publish(this->pitch_left_pos_msg);
        while (!this->_launch_angle_reached)
        {
            if(this->_LaunchSet_1_wall && this->throw_position_observed <= this->Pos_1_wall_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_1_wall_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_2_wall && this->throw_position_observed <= this->Pos_2_wall_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_2_wall_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_3_wall && this->throw_position_observed <= this->Pos_3_wall_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_3_wall_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_4_wall && this->throw_position_observed <= this->Pos_4_wall_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_4_wall_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_5_wall && this->throw_position_observed <= this->Pos_5_wall_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_5_wall_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_2_free && this->throw_position_observed <= this->Pos_2_free_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_2_free_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_3_free && this->throw_position_observed <= this->Pos_3_free_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_3_free_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }else if(this->_LaunchSet_4_free && this->throw_position_observed <= this->Pos_4_free_RotStop_deg+this->RotStop_deg_adjust)
            {
                this->_launch_angle_reached = true;
                this->arm_vel_msg.data = this->Pos_4_free_RotStop_cur;
                this->ArmVal_pub.publish(arm_vel_msg);
            }
            
            NODELET_INFO("%f",this->throw_position_observed);
        }
        
        //this->Cyl_Arm_release_arrow();
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //this->arm_vel_msg.data = 5;
        //this->ArmVal_pub.publish(arm_vel_msg);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //this->arm_vel_msg.data = 0;
        //this->ArmVal_pub.publish(arm_vel_msg);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        //this->act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
        //this->ArmCmd_pub.publish(act_conf_cmd_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        this->arm_vel_msg.data = 0;
        this->ArmVal_pub.publish(arm_vel_msg);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2300));
        //this->rotate_arm_to_first_posi();
        //this->rotate_arm_to_first_posi();
        //this->rotate_arm_to_first_posi();
        //this->homing();
        this->currentCommandIndex++;
        NODELET_INFO("launch_and_home");
        this->_launch_angle_reached = false;
    }
    else if(currentCommand == ControllerCommands::Pitch_MV_Zero)
    {
        this->pitch_right_pos_msg.data = -0.1;
        this->PitchRightPos_pub.publish(this->pitch_right_pos_msg);
        this->pitch_left_pos_msg.data = 0.0;
        this->PitchLeftPos_pub.publish(this->pitch_left_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("pitch zero");
    }
    else if(currentCommand == ControllerCommands::Pitch_MV_init)
    {   
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
        PitchRightCmd_pub.publish(act_conf_cmd_msg);
        PitchLeftCmd_pub.publish(act_conf_cmd_msg);
        this->pitch_right_pos_msg.data = this->pitchright_init_deg;
        this->PitchRightPos_pub.publish(this->pitch_right_pos_msg);
        this->pitch_left_pos_msg.data = this->pitchleft_init_deg;
        this->PitchLeftPos_pub.publish(this->pitch_left_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("pitch init");
    }
    else if(currentCommand == ControllerCommands::Pitch_MV_launch)
    {
        this->pitch_right_pos_msg.data = this->pitchright_init_deg;
        //this->pitch_left_pos_msg.data = this->pitchleft_init_deg;
        this->_PitchSet_Right = false;
        //this->_PitchSet_Left = false;
        while(!this->_PitchSet_Right){
            if(this->_LaunchSet_1_wall){
                if(this->pitch_right_pos_msg.data > this->pitch_1_wall_deg){
                    this->pitch_right_pos_msg.data = this->pitch_right_pos_msg.data - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_2_wall){
                if(this->pitch_right_pos_msg.data > this->pitch_2_wall_deg){
                    this->pitch_right_pos_msg.data = this->pitch_right_pos_msg.data - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_3_wall){
                if(this->pitch_right_pos_msg.data > this->pitch_3_wall_deg){
                    this->pitch_right_pos_msg.data = (double)(this->pitch_right_pos_msg.data) - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_4_wall){
                if(this->pitch_right_pos_msg.data > this->pitch_4_wall_deg){
                    this->pitch_right_pos_msg.data = (double)(this->pitch_right_pos_msg.data) - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_5_wall){
                if(this->pitch_right_pos_msg.data > this->pitch_5_wall_deg){
                    this->pitch_right_pos_msg.data = (double)(this->pitch_right_pos_msg.data) - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_2_free){
                if(this->pitch_right_pos_msg.data > this->pitch_2_free_deg){
                    this->pitch_right_pos_msg.data = (double)(this->pitch_right_pos_msg.data) - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_3_free){
                if(this->pitch_right_pos_msg.data > this->pitch_3_free_deg){
                    this->pitch_right_pos_msg.data = (double)(this->pitch_right_pos_msg.data) - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }else if(this->_LaunchSet_4_free){
                if(this->pitch_right_pos_msg.data > this->pitch_4_free_deg){
                    this->pitch_right_pos_msg.data = (double)(this->pitch_right_pos_msg.data) - 0.01;
                }else{
                    this->_PitchSet_Right = true;
                }
            }
            this->PitchRightPos_pub.publish(this->pitch_right_pos_msg);
            //this->PitchLeftPos_pub.publish(this->pitch_left_pos_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        this->_PitchSet_Right = false;
        //this->_PitchSet_Left = false;
        this->currentCommandIndex++;
        NODELET_INFO("pitch set");
    }
    else if(currentCommand == ControllerCommands::Pitch_MV_fast_launch)
    {
        if(this->_LaunchSet_1){
            this->pitch_right_pos_msg.data = this->pitchrightpos_1_deg;
        }else if(this->_LaunchSet_2){
            this->pitch_right_pos_msg.data = this->pitchrightpos_2_deg;
        }else if(this->_LaunchSet_3){
            this->pitch_right_pos_msg.data = this->pitchrightpos_3_deg;
        }
        this->PitchRightPos_pub.publish(this->pitch_right_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("pitch set");
    }
    else if(currentCommand == ControllerCommands::Pitch_MV_avoid_arm)
    {
        this->pitch_right_pos_msg.data = this->pitchright_avoid_arm_deg;
        this->PitchRightPos_pub.publish(this->pitch_right_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("pitch set");
    }
    else if(currentCommand == ControllerCommands::Pitch_recover)
    {
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
        PitchRightCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
    }
    else if(currentCommand == ControllerCommands::Pitch_Homing)
    {
        this->pitch_homing();
        this->currentCommandIndex++;
        NODELET_INFO("pitch homing");
    }
    else if(currentCommand == ControllerCommands::Cyl_Arm_grab)
    {   
        this->Cyl_Arm_grab_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("grab_arrow");
    }
    else if(currentCommand == ControllerCommands::Cyl_Arm_release)
    {   
        this->Cyl_Arm_release_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("release_arrow");
    }
    else if (currentCommand == ControllerCommands::set_delay_10ms)
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
PLUGINLIB_EXPORT_CLASS(dr::dr_nodelet_main, nodelet::Nodelet);
