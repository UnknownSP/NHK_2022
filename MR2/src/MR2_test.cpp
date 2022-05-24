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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <thread>     
#include <chrono>


constexpr double pi = 3.141592653589793238462643383279502884L;

namespace MR2{


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

// related to delay
    set_delay_10ms,
    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,
    delay,
    wait_next_pressed,

    defenceLift_mv_Upper,
    defenceLift_mv_Lower,
    defenceLift_mv_LagoriBase,
    defenceLift_mv_DodgeLagori_slowly,
    defenceLift_mv_Upper_slowly,
    defenceLift_mv_Lower_slowly,
    defenceLift_mv_LagoriBase_slowly,
    defenceRoll_mv_Horizontal,
    wait_defenceLift_Upper,
    wait_defenceLift_Lower,
    wait_defenceRoll_horizontal,
    Cyl_Defend_Grab_On,
    Cyl_Defend_Rise_On,
    Cyl_Defend_Press_On,
    Cyl_Defend_Grab_Off,
    Cyl_Defend_Rise_Off,
    Cyl_Defend_Press_Off,

};

enum class SolenoidValveCommands : uint8_t
{
    shutdown_cmd      = 0b000000,
    recover_cmd       = 0b000001,

    Cyl_R_Clutch_cmd        = 0b0000100,
    Cyl_R_Upper_Rotate_cmd  = 0b0000001,
    Cyl_R_Upper_Grab_cmd    = 0b0001000,
    Cyl_R_Upper_Deploy_cmd  = 0b0000010,
    Cyl_R_Lower_Grab_cmd    = 0b1000000,
    Cyl_R_Lower_Deploy_cmd  = 0b0100000,

    Cyl_L_Clutch_cmd        = 0b0001000,//ok
    Cyl_L_Upper_Rotate_cmd  = 0b0010000,//ok
    Cyl_L_Upper_Grab_cmd    = 0b0000100,//ok
    Cyl_L_Upper_Deploy_cmd  = 0b0100000,//ok
    Cyl_L_Lower_Grab_cmd    = 0b0000010,
    Cyl_L_Lower_Deploy_cmd  = 0b0000001,

    Cyl_Defend_Grab_cmd     = 0b0001000,
    Cyl_Defend_Rise_cmd     = 0b0010000,
    Cyl_Defend_Press_cmd    = 0b0100000,

    Cyl_Ball_Grab_cmd       = 0b0000001,
    Cyl_Ball_Gather_cmd     = 0b0000001,
    Cyl_Ball_Rise_cmd       = 0b0010000,

};
enum class SolenoidValveBoards : uint8_t
{
    Cyl_R_Clutch_board        = 1,
    Cyl_R_Upper_Rotate_board  = 1,
    Cyl_R_Upper_Grab_board    = 1,
    Cyl_R_Upper_Deploy_board  = 1,
    Cyl_R_Lower_Grab_board    = 1,
    Cyl_R_Lower_Deploy_board  = 1,

    Cyl_L_Clutch_board        = 0,
    Cyl_L_Upper_Rotate_board  = 0,
    Cyl_L_Upper_Grab_board    = 0,
    Cyl_L_Upper_Deploy_board  = 0,
    Cyl_L_Lower_Grab_board    = 0,
    Cyl_L_Lower_Deploy_board  = 0,

    Cyl_Defend_Grab_board     = 2,
    Cyl_Defend_Rise_board     = 2,
    Cyl_Defend_Press_board    = 2,

    Cyl_Ball_Grab_board       = 3,
    Cyl_Ball_Gather_board     = 3,
    Cyl_Ball_Rise_board       = 3,

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


class MR2_nodelet_main : public nodelet::Nodelet
{
public:
    virtual void onInit();
private:
    /***********************Function**************************/
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void control_timer_callback(const ros::TimerEvent &event);
	void MouseCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void KeyCallback(const std_msgs::String::ConstPtr& msg);
	void DefenceLift_PosCallback(const std_msgs::Float32::ConstPtr& msg);
	void DefenceRoll_PosCallback(const std_msgs::Float32::ConstPtr& msg);
	void Arm_R_PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void Arm_L_PosCallback(const std_msgs::Float32::ConstPtr& msg);

	void Odmetory_R_PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void Odmetory_L_PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void Odmetory_F_PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void Odmetory_B_PosCallback(const std_msgs::Float32::ConstPtr& msg);

    void shutdown();
    void recover();
    void set_delay(double delay_s);
    void delay_start(double delay_s);

    //void steer_homing();
    void Arm_R_homing(void);
    void Arm_L_homing(void);
    void Defence_Lift_homing(void);
    void Defence_Roll_homing(void);

    void send_steerAdjust(void);

    void Cylinder_Operation(std::string CylName, bool state);

    void Arm_R_move_Vel(double target);
    void Arm_L_move_Vel(double target);
    void Arm_R_move_Pos(double target);
    void Arm_L_move_Pos(double target);
    void Defence_Lift_move_Pos(double target);
    void Defence_Roll_move_Pos(double target);
    void Defence_Lift_move_Vel(double target);
    void Defence_Roll_move_Vel(double target);
    void R_mode_Count(int cound);
    void L_mode_Count(int cound);
    void Defend_mode_Count(int cound);

    //Odmetory
    void Odmetory_reset(int odm_num, bool _all);
    void Odmetory_position(bool reset);

    /***********************Function**************************/
    
    
    ros::NodeHandle nh;
  	ros::NodeHandle _nh;
    ros::NodeHandle nh_MT;
    ros::Timer control_timer;
    
    /***********************Pub&Sub**************************/
    ros::Subscriber joy_sub;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;

    ros::Publisher Solenoid_Cmd_pub[4];
    ros::Publisher Solenoid_Order_pub[4];
	std_msgs::UInt8 solenoid_order_msg[4];
    uint8_t lastSolenoid_Order[4] = {0b0000000};

    //ros::Publisher Solenoid1_Cmd_pub;
    //ros::Publisher Solenoid1_Order_pub;
    //std_msgs::UInt8 solenoid1_order_msg;
    //uint8_t lastSolenoid_1_Order = 0b0000000;
//
    //ros::Publisher Solenoid2_Cmd_pub;
    //ros::Publisher Solenoid2_Order_pub;
    //std_msgs::UInt8 solenoid2_order_msg;
    //uint8_t lastSolenoid_2_Order = 0b0000000;
    
    std_msgs::UInt8 act_conf_cmd_msg;
    std_msgs::UInt8 act_conf_cmd_msg2;
	std_msgs::UInt8 shirasu_cmd_msg;

    ros::Publisher foot_CmdPub0;
    ros::Publisher foot_CmdPub1;
    ros::Publisher foot_CmdPub2;
	ros::Publisher foot_CmdPub3;

    ros::Publisher Odmetory_R_CmdPub;
    ros::Publisher Odmetory_L_CmdPub;
    ros::Publisher Odmetory_F_CmdPub;
	ros::Publisher Odmetory_B_CmdPub;
	ros::Subscriber Odmetory_R_Pos_sub;
	ros::Subscriber Odmetory_L_Pos_sub;
	ros::Subscriber Odmetory_F_Pos_sub;
	ros::Subscriber Odmetory_B_Pos_sub;
    //ros::Publisher steer_CmdPub0;
    //ros::Publisher steer_CmdPub1;
    //ros::Publisher steer_CmdPub2;
	//ros::Publisher steer_CmdPub3;

    ros::Publisher Arm_R_Cmd_pub;
  	ros::Publisher Arm_R_Value_pub;
    std_msgs::Float64 Arm_R_Value_msg;

    ros::Publisher Arm_L_Cmd_pub;
  	ros::Publisher Arm_L_Value_pub;
    std_msgs::Float64 Arm_L_Value_msg;

    ros::Publisher Defence_Lift_Cmd_pub;
  	ros::Publisher Defence_Lift_Value_pub;
    std_msgs::Float64 Defence_Lift_Value_msg;

    ros::Publisher Defence_Roll_Cmd_pub;
  	ros::Publisher Defence_Roll_Value_pub;
    std_msgs::Float64 Defence_Roll_Value_msg;

	ros::Subscriber ThrowPos_sub;
	ros::Subscriber Mouse_sub;
	ros::Subscriber Key_sub;

	ros::Subscriber DefenceLift_Pos_sub;
	ros::Subscriber DefenceRoll_Pos_sub;
	ros::Subscriber Arm_R_Pos_sub;
	ros::Subscriber Arm_L_Pos_sub;

    ros::Publisher SteerAdjust_pub;
    std_msgs::Float64MultiArray adjust_pubData;
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
    static const std::vector<ControllerCommands> Defence_Up_commands;
    static const std::vector<ControllerCommands> Defence_Dodge_commands;
    static const std::vector<ControllerCommands> Defence_Enable_commands;
    static const std::vector<ControllerCommands> Defence_Down_commands;
    static const std::vector<ControllerCommands> Defence_Release_commands;
    static const std::vector<ControllerCommands> Defence_Unable_commands;
    static const std::vector<ControllerCommands> SetLaunchPosi_commands;
    static const std::vector<ControllerCommands> manual_all;
    const std::vector<ControllerCommands> *command_list;


    /***********************Valiables**************************/
    int _delay_s = 0;
    double _delay_s_r = 0.0;
    double _delay_s_l = 0.0;
    bool _command_ongoing = false;
    bool _is_manual_enabled = true;

    double delay_time = 0.0;
    //uint8_t lastSolenoidOrder = 0b0000000;
    double throw_position_observed;
    double mouse_position_x;
    double mouse_position_y;
    std::string key_press = "";
    double defenceLift_position = 0.0;
    double defenceRoll_position = 0.0;
    double defenceLift_target = 0.0;
    bool _defenceLift_target_init = false;
    double Arm_R_position = 0.0;
    double Arm_L_position = 0.0;

    double steer_adjust[4] = {0.0};
    bool _enable_steerAdjust = false;
    bool _ballPick_Mode = false;
    bool _piling_manual_Mode = true;
    bool _piling_auto_Mode = false;
    bool _defence_Mode = false;
    bool _homing_Mode = false;

    double defence_lift_upper_pos = 0.0;
    bool _defence_lift_upper_speeddown = false;
    double defence_lift_lower_pos = 0.0;
    double defence_lift_dodgelagori_pos = 0.0;
    double defence_lift_lagoribase_pos = 0.0;
    double defence_roll_horizontal_pos = 0.0;
    double arm_r_avoid1lagori_pos = 0.0;
    double arm_l_avoid1lagori_pos = 0.0;
    bool _arm_r_avoid1lagori = false;
    bool _arm_l_avoid1lagori = false;
    double arm_r_avoidlagoribase_pos = 0.0;
    double arm_l_avoidlagoribase_pos = 0.0;
    bool _arm_r_avoidlagoribase = false;
    bool _arm_l_avoidlagoribase = false;

    int _autoPile_R_mode_count = -1;
    int _autoPile_L_mode_count = -1;
    int _recent_R_mode_count = -1;
    int _recent_L_mode_count = -1;
    int R_mode_count_max = 17;
    int L_mode_count_max = 17;
    bool _R_mode_count_changed = false;
    bool _L_mode_count_changed = false;
    int _Defend_mode_count = -1;
    int _recent_Defend_mode_count = -1;
    int Defend_mode_count_max = 7;
    bool _Defend_mode_count_changed = false;
    bool _Arm_Deploy = false;

    bool _Arm_R_moving = false;
    bool _Arm_L_moving = false;

    bool _reverse_control = false;
    bool _swap_control = false;

    bool _odm_r_update = false;
    bool _odm_l_update = false;
    bool _odm_f_update = false;
    bool _odm_b_update = false;
    double odm_r_recent_pos = 0.0;
    double odm_r_diff = 0.0;
    double odm_r_now_pos = 0.0;
    double odm_l_recent_pos = 0.0;
    double odm_l_diff = 0.0;
    double odm_l_now_pos = 0.0;
    double odm_f_recent_pos = 0.0;
    double odm_f_diff = 0.0;
    double odm_f_now_pos = 0.0;
    double odm_b_recent_pos = 0.0;
    double odm_b_diff = 0.0;
    double odm_b_now_pos = 0.0;

    geometry_msgs::Twist odm_now_position;
    geometry_msgs::Twist odm_recent_position;
    /***********************Valiables**************************/
};

int MR2_nodelet_main::_padx = 0;
int MR2_nodelet_main::_pady = 0;
int MR2_nodelet_main::_lb = 0;
int MR2_nodelet_main::_rb = 0;


int MR2_nodelet_main::ButtonA = 1;
int MR2_nodelet_main::ButtonB = 2;
int MR2_nodelet_main::ButtonX = 0;
int MR2_nodelet_main::ButtonY = 3;
int MR2_nodelet_main::ButtonLB = 4;
int MR2_nodelet_main::ButtonRB = 5;
int MR2_nodelet_main::ButtonBack = 8;
int MR2_nodelet_main::ButtonStart = 9;
int MR2_nodelet_main::ButtonLeftThumb = 6;
int MR2_nodelet_main::ButtonRightThumb = 7;

int MR2_nodelet_main::AxisDPadX = 4;
int MR2_nodelet_main::AxisDPadY = 5;
int MR2_nodelet_main::AxisLeftThumbX = 0;
int MR2_nodelet_main::AxisLeftThumbY = 1;
int MR2_nodelet_main::AxisRightThumbX = 2;
int MR2_nodelet_main::AxisRightThumbY = 3;
int MR2_nodelet_main::ButtonLeftTrigger = 10;
int MR2_nodelet_main::ButtonRightTrigger = 11;

const std::vector<ControllerCommands> MR2_nodelet_main::manual_all(
    {
        ControllerCommands::manual
    }
);

const std::vector<ControllerCommands> MR2_nodelet_main::Defence_Enable_commands(
    {
        ControllerCommands::Cyl_Defend_Grab_On,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::defenceRoll_mv_Horizontal,
        ControllerCommands::wait_defenceRoll_horizontal,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        ControllerCommands::Cyl_Defend_Press_On,
        ControllerCommands::defenceLift_mv_Upper_slowly,
        ControllerCommands::wait_defenceLift_Upper,
        ControllerCommands::Cyl_Defend_Grab_Off,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Defend_Rise_On,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Defend_Grab_On,
    }
);
const std::vector<ControllerCommands> MR2_nodelet_main::Defence_Release_commands(
    {
        ControllerCommands::defenceLift_mv_LagoriBase_slowly,
    }
);
const std::vector<ControllerCommands> MR2_nodelet_main::Defence_Down_commands(
    {
        ControllerCommands::defenceLift_mv_Lower_slowly,
    }
);
const std::vector<ControllerCommands> MR2_nodelet_main::Defence_Unable_commands(
    {
        ControllerCommands::Cyl_Defend_Press_Off,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::Cyl_Defend_Grab_Off,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::defenceLift_mv_Upper_slowly,
    }
);
const std::vector<ControllerCommands> MR2_nodelet_main::Defence_Up_commands(
    {
        ControllerCommands::defenceLift_mv_Upper_slowly,
    }
);
const std::vector<ControllerCommands> MR2_nodelet_main::Defence_Dodge_commands(
    {
        ControllerCommands::defenceLift_mv_DodgeLagori_slowly,
    }
);
const std::vector<ControllerCommands> MR2_nodelet_main::SetLaunchPosi_commands(
    {
        //ControllerCommands::Cyl_Arm_grab,
        //ControllerCommands::Pitch_MV_Zero,
        //ControllerCommands::Pitch_MV_Zero,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::Pitch_Homing,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //ControllerCommands::recover_velocity,
        //ControllerCommands::ArmRotateToRotStart_Vel,
        //ControllerCommands::recover_position,
        //ControllerCommands::ArmRotateToRotStart_Pos,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::Cyl_Arm_release,
        //ControllerCommands::Pitch_recover,
        //ControllerCommands::Pitch_MV_fast_launch,
        //ControllerCommands::Pitch_MV_launch,
    }
);

void MR2_nodelet_main::onInit(void)
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

    this->command_list = &MR2_nodelet_main::manual_all;


	/*******************pub & sub*****************/
    this->joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &MR2_nodelet_main::joyCallback, this);
    this->cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //this->Solenoid1_Cmd_pub = nh.advertise<std_msgs::UInt8>("solenoid1_cmd", 1);
    //this->Solenoid1_Order_pub = nh.advertise<std_msgs::UInt8>("solenoid1_order", 1);
//
    //this->Solenoid2_Cmd_pub = nh.advertise<std_msgs::UInt8>("solenoid2_cmd", 1);
    //this->Solenoid2_Order_pub = nh.advertise<std_msgs::UInt8>("solenoid2_order", 1);

    for(int i=0;i<4;i++){
        this->Solenoid_Cmd_pub[i] = nh.advertise<std_msgs::UInt8>("solenoid"+std::to_string(i+1)+"_cmd", 1);
        this->Solenoid_Order_pub[i] = nh.advertise<std_msgs::UInt8>("solenoid"+std::to_string(i+1)+"_order", 1);
    }

	//this->SolenoidCmd_pub = nh.advertise<std_msgs::UInt8>("solenoid_cmd", 1);
    //this->SolenoidOrder_pub = nh.advertise<std_msgs::UInt8>("solenoid_order", 1);
    this->foot_CmdPub0 = nh.advertise<std_msgs::UInt8>("foot0_cmd", 1);
    this->foot_CmdPub1 = nh.advertise<std_msgs::UInt8>("foot1_cmd", 1);
    this->foot_CmdPub2 = nh.advertise<std_msgs::UInt8>("foot2_cmd", 1);
	this->foot_CmdPub3 = nh.advertise<std_msgs::UInt8>("foot3_cmd", 1);

    this->Odmetory_R_CmdPub = nh.advertise<std_msgs::UInt8>("odmetory_r_cmd", 1);
    this->Odmetory_L_CmdPub = nh.advertise<std_msgs::UInt8>("odmetory_l_cmd", 1);
    this->Odmetory_F_CmdPub = nh.advertise<std_msgs::UInt8>("odmetory_f_cmd", 1);
	this->Odmetory_B_CmdPub = nh.advertise<std_msgs::UInt8>("odmetory_b_cmd", 1);
    //this->steer_CmdPub0 = nh.advertise<std_msgs::UInt8>("steer0_cmd", 1);
    //this->steer_CmdPub1 = nh.advertise<std_msgs::UInt8>("steer1_cmd", 1);
    //this->steer_CmdPub2 = nh.advertise<std_msgs::UInt8>("steer2_cmd", 1);
	//this->steer_CmdPub3 = nh.advertise<std_msgs::UInt8>("steer3_cmd", 1);

    this->Arm_R_Cmd_pub = nh.advertise<std_msgs::UInt8>("arm_r_cmd", 1);
  	this->Arm_R_Value_pub = nh.advertise<std_msgs::Float64>("arm_r_value", 1);
    this->Arm_L_Cmd_pub = nh.advertise<std_msgs::UInt8>("arm_l_cmd", 1);
  	this->Arm_L_Value_pub = nh.advertise<std_msgs::Float64>("arm_l_value", 1);
    this->Defence_Lift_Cmd_pub = nh.advertise<std_msgs::UInt8>("defence_lift_cmd", 1);
  	this->Defence_Lift_Value_pub = nh.advertise<std_msgs::Float64>("defence_lift_value", 1);
    this->Defence_Roll_Cmd_pub = nh.advertise<std_msgs::UInt8>("defence_roll_cmd", 1);
  	this->Defence_Roll_Value_pub = nh.advertise<std_msgs::Float64>("defence_roll_value", 1);
    this->ThrowPos_sub = nh_MT.subscribe<std_msgs::Float32>("motor4_current_val", 10, &MR2_nodelet_main::PosCallback, this);
    this->Mouse_sub = nh.subscribe<geometry_msgs::Twist>("/mouse_vel", 10, &MR2_nodelet_main::MouseCallback, this);
    this->Key_sub = nh.subscribe<std_msgs::String>("/keypress", 10, &MR2_nodelet_main::KeyCallback, this);
    this->SteerAdjust_pub = nh.advertise<std_msgs::Float64MultiArray>("adjust_val", 1);

    this->DefenceLift_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor10_current_val", 10, &MR2_nodelet_main::DefenceLift_PosCallback, this);
    this->DefenceRoll_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor11_current_val", 10, &MR2_nodelet_main::DefenceRoll_PosCallback, this);
    this->Arm_R_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor8_current_val", 10, &MR2_nodelet_main::Arm_R_PosCallback, this);
    this->Arm_L_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor9_current_val", 10, &MR2_nodelet_main::Arm_L_PosCallback, this);

    this->Odmetory_R_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor20_current_val", 10, &MR2_nodelet_main::Odmetory_R_PosCallback, this);
    this->Odmetory_L_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor21_current_val", 10, &MR2_nodelet_main::Odmetory_L_PosCallback, this);
    this->Odmetory_F_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor22_current_val", 10, &MR2_nodelet_main::Odmetory_F_PosCallback, this);
    this->Odmetory_B_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor23_current_val", 10, &MR2_nodelet_main::Odmetory_B_PosCallback, this);

	/*******************pub & sub*****************/

	/*******************parameter*****************/
    //_nh.param("steer0_adjust", launch_long_vel, 0.0);
    //_nh.param("steer0_adjust_sub", this->steer_adjust[0], 0.0);
	//_nh.param("steer1_adjust_sub", this->steer_adjust[1], 0.0);
	//_nh.param("steer2_adjust_sub", this->steer_adjust[2], 0.0);
	//_nh.param("steer3_adjust_sub", this->steer_adjust[3], 0.0);
    _nh.param("defence_lift_upper_pos", this->defence_lift_upper_pos, 0.0);
    _nh.param("defence_lift_lower_pos", this->defence_lift_lower_pos, 0.0);
    _nh.param("defence_lift_dodgelagori_pos", this->defence_lift_dodgelagori_pos, 0.0);
    _nh.param("defence_lift_lagoribase_pos", this->defence_lift_lagoribase_pos, 0.0);
    _nh.param("defence_roll_horizontal_pos", this->defence_roll_horizontal_pos, 0.0);
    _nh.param("arm_r_avoid1lagori_pos", this->arm_r_avoid1lagori_pos, 0.0);
    _nh.param("arm_l_avoid1lagori_pos", this->arm_l_avoid1lagori_pos, 0.0);
    _nh.param("arm_r_avoidlagoribase_pos", this->arm_r_avoidlagoribase_pos, 0.0);
    _nh.param("arm_l_avoidlagoribase_pos", this->arm_l_avoidlagoribase_pos, 0.0);

    this->adjust_pubData.data.resize(4);
    for(int i=0;i<4;i++){
        this->adjust_pubData.data[i] = this->steer_adjust[i];
    }
	/*******************parameter*****************/

    odm_now_position.linear.x = 0.0;
    odm_now_position.linear.y = 0.0;
    odm_now_position.angular.z = 0.0;
    odm_recent_position.linear.x = 0.0;
    odm_recent_position.linear.y = 0.0;
    odm_recent_position.angular.z = 0.0;

    //this line must be placed here (last line of this function) because of some publisher will be called by this timer before it is declared
    this->control_timer = nh.createTimer(ros::Duration(0.01), &MR2_nodelet_main::control_timer_callback, this);
    NODELET_INFO("MR2 node has started.");

}

/**************************************************************************************/
void MR2_nodelet_main::MouseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	this->mouse_position_x = msg->linear.x;
	this->mouse_position_y = msg->linear.y;
    //NODELET_INFO("x : %f, y : %f", this->mouse_position_x, this->mouse_position_y);
}

void MR2_nodelet_main::KeyCallback(const std_msgs::String::ConstPtr& msg)
{   

	this->key_press = msg->data;
    if(this->key_press == "q"){
        Cylinder_Operation("L_Clutch",true);
    }else if(this->key_press == "w"){
        Cylinder_Operation("L_Clutch",false);
    }else if(this->key_press == "e"){
        Cylinder_Operation("L_Upper_Rotate",true);
    }else if(this->key_press == "r"){
        Cylinder_Operation("L_Upper_Rotate",false);
    }else if(this->key_press == "t"){
        Cylinder_Operation("L_Upper_Grab",true);
    }else if(this->key_press == "y"){
        Cylinder_Operation("L_Upper_Grab",false);
    }else if(this->key_press == "u"){
        Cylinder_Operation("L_Upper_Deploy",true);
    }else if(this->key_press == "i"){
        Cylinder_Operation("L_Upper_Deploy",false);
    }else if(this->key_press == "d"){
        Cylinder_Operation("L_Lower_Grab",true);
    }else if(this->key_press == "f"){
        Cylinder_Operation("L_Lower_Grab",false);
    }else if(this->key_press == "g"){
        Cylinder_Operation("L_Lower_Deploy",true);
    }else if(this->key_press == "h"){
        Cylinder_Operation("L_Lower_Deploy",false);
    }
    
    NODELET_INFO("keypress : %s", this->key_press.c_str());
}
void MR2_nodelet_main::PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->throw_position_observed = msg->data;
}
void MR2_nodelet_main::DefenceLift_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->defenceLift_position = msg->data;
}
void MR2_nodelet_main::DefenceRoll_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->defenceRoll_position = msg->data;
}
void MR2_nodelet_main::Arm_R_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->Arm_R_position = msg->data;
}
void MR2_nodelet_main::Arm_L_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->Arm_L_position = msg->data;
}
/**********************************************************************************************************/

void MR2_nodelet_main::Odmetory_R_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->odm_r_now_pos = msg->data;
    odm_r_diff = odm_r_now_pos - odm_r_recent_pos;
    _odm_r_update = true;
	if(_odm_r_update && _odm_l_update && _odm_f_update && _odm_b_update){
        _odm_r_update = false;
        _odm_l_update = false;
        _odm_f_update = false;
        _odm_b_update = false;
        odm_r_recent_pos = odm_r_now_pos;
        odm_l_recent_pos = odm_l_now_pos;
        odm_f_recent_pos = odm_f_now_pos;
        odm_b_recent_pos = odm_b_now_pos;
    }
}
void MR2_nodelet_main::Odmetory_L_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->odm_l_now_pos = msg->data;
    odm_l_diff = odm_l_now_pos - odm_l_recent_pos;
    _odm_l_update = true;
	if(_odm_r_update && _odm_l_update && _odm_f_update && _odm_b_update){
        _odm_r_update = false;
        _odm_l_update = false;
        _odm_f_update = false;
        _odm_b_update = false;
        odm_r_recent_pos = odm_r_now_pos;
        odm_l_recent_pos = odm_l_now_pos;
        odm_f_recent_pos = odm_f_now_pos;
        odm_b_recent_pos = odm_b_now_pos;
    }
}
void MR2_nodelet_main::Odmetory_F_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->odm_f_now_pos = msg->data;
    odm_f_diff = odm_f_now_pos - odm_f_recent_pos;
    _odm_f_update = true;
	if(_odm_r_update && _odm_l_update && _odm_f_update && _odm_b_update){
        _odm_r_update = false;
        _odm_l_update = false;
        _odm_f_update = false;
        _odm_b_update = false;
        odm_r_recent_pos = odm_r_now_pos;
        odm_l_recent_pos = odm_l_now_pos;
        odm_f_recent_pos = odm_f_now_pos;
        odm_b_recent_pos = odm_b_now_pos;
    }
}
void MR2_nodelet_main::Odmetory_B_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->odm_b_now_pos = msg->data;
    odm_b_diff = odm_b_now_pos - odm_b_recent_pos;
    _odm_b_update = true;
	if(_odm_r_update && _odm_l_update && _odm_f_update && _odm_b_update){
        _odm_r_update = false;
        _odm_l_update = false;
        _odm_f_update = false;
        _odm_b_update = false;
        odm_r_recent_pos = odm_r_now_pos;
        odm_l_recent_pos = odm_l_now_pos;
        odm_f_recent_pos = odm_f_now_pos;
        odm_b_recent_pos = odm_b_now_pos;
    }
}

void MR2_nodelet_main::Odmetory_position(bool reset){
    
}

void MR2_nodelet_main::Odmetory_reset(int odm_num, bool _all){
    std_msgs::UInt8 Cmd_Msg;
    Cmd_Msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    if(_all){
        this->Odmetory_R_CmdPub.publish(Cmd_Msg);
        this->Odmetory_L_CmdPub.publish(Cmd_Msg);
        this->Odmetory_F_CmdPub.publish(Cmd_Msg);
        this->Odmetory_B_CmdPub.publish(Cmd_Msg);
        Cmd_Msg.data = (uint8_t)MotorCommands::homing_cmd;
        this->Odmetory_R_CmdPub.publish(Cmd_Msg);
        this->Odmetory_L_CmdPub.publish(Cmd_Msg);
        this->Odmetory_F_CmdPub.publish(Cmd_Msg);
        this->Odmetory_B_CmdPub.publish(Cmd_Msg);
        return;
    }
    switch (odm_num)
    {
    case 0: //R
        this->Odmetory_R_CmdPub.publish(Cmd_Msg);
        Cmd_Msg.data = (uint8_t)MotorCommands::homing_cmd;
        this->Odmetory_R_CmdPub.publish(Cmd_Msg);
        break;
    case 1: //L
        this->Odmetory_L_CmdPub.publish(Cmd_Msg);
        Cmd_Msg.data = (uint8_t)MotorCommands::homing_cmd;
        this->Odmetory_L_CmdPub.publish(Cmd_Msg);
        break;
    case 2: //F
        this->Odmetory_F_CmdPub.publish(Cmd_Msg);
        Cmd_Msg.data = (uint8_t)MotorCommands::homing_cmd;
        this->Odmetory_F_CmdPub.publish(Cmd_Msg);
        break;
    case 3: //B
        this->Odmetory_B_CmdPub.publish(Cmd_Msg);
        Cmd_Msg.data = (uint8_t)MotorCommands::homing_cmd;
        this->Odmetory_B_CmdPub.publish(Cmd_Msg);
        break;
    default:
        break;
    }
}

/**********************************************************************************************************/

void MR2_nodelet_main::shutdown(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    for(int i=0; i<4;i++){
        Solenoid_Cmd_pub[i].publish(act_conf_cmd_msg);
    }
    foot_CmdPub0.publish(act_conf_cmd_msg);
    foot_CmdPub1.publish(act_conf_cmd_msg);
    foot_CmdPub2.publish(act_conf_cmd_msg);
    foot_CmdPub3.publish(act_conf_cmd_msg);
    Odmetory_R_CmdPub.publish(act_conf_cmd_msg);
    Odmetory_L_CmdPub.publish(act_conf_cmd_msg);
    Odmetory_F_CmdPub.publish(act_conf_cmd_msg);
    Odmetory_B_CmdPub.publish(act_conf_cmd_msg);
    //steer_CmdPub0.publish(act_conf_cmd_msg);
    //steer_CmdPub1.publish(act_conf_cmd_msg);
    //steer_CmdPub2.publish(act_conf_cmd_msg);
    //steer_CmdPub3.publish(act_conf_cmd_msg);
    Arm_R_Cmd_pub.publish(act_conf_cmd_msg);
    Arm_L_Cmd_pub.publish(act_conf_cmd_msg);
    Defence_Lift_Cmd_pub.publish(act_conf_cmd_msg);
    Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg);
}

void MR2_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    for(int i=0; i<4;i++){
        Solenoid_Cmd_pub[i].publish(act_conf_cmd_msg);
    }
    //act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    //steer_CmdPub0.publish(act_conf_cmd_msg);
    //steer_CmdPub1.publish(act_conf_cmd_msg);
    //steer_CmdPub2.publish(act_conf_cmd_msg);
    //steer_CmdPub3.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    foot_CmdPub0.publish(act_conf_cmd_msg);
    foot_CmdPub1.publish(act_conf_cmd_msg);
    foot_CmdPub2.publish(act_conf_cmd_msg);
    foot_CmdPub3.publish(act_conf_cmd_msg);
    Arm_R_Cmd_pub.publish(act_conf_cmd_msg);
    Arm_L_Cmd_pub.publish(act_conf_cmd_msg);
    Defence_Lift_Cmd_pub.publish(act_conf_cmd_msg);
    Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg);
}

//void MR2_nodelet_main::homing(void){
//    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
//    ArmCmd_pub.publish(act_conf_cmd_msg);
//    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
//    ArmCmd_pub.publish(act_conf_cmd_msg);
//}

void MR2_nodelet_main::set_delay(double delay_s)
{
    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

void MR2_nodelet_main::delay_start(double delay_s)
{
    this->delay_time = ros::Time::now().toSec() + delay_s;
    while(this->delay_time > ros::Time::now().toSec());
}

void MR2_nodelet_main::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
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
    static bool _x_enable = false;
    static bool _y_enable = false;

    //static bool _Cyl_Arm = false;
    //static bool _Cyl_Clutch = false;
    //static bool _Cyl_Shelf = false;
    //static bool _Cyl_Push = false;
    //static bool _Cyl_Pull = false;

    static bool _Cyl_R_Clutch        = false;
    static bool _Cyl_R_Upper_Rotate  = false;
    static bool _Cyl_R_Upper_Grab    = false;
    static bool _Cyl_R_Upper_Deploy  = false;
    static bool _Cyl_R_Lower_Grab    = false;
    static bool _Cyl_R_Lower_Deploy  = false;
    static bool _Cyl_L_Clutch        = false;
    static bool _Cyl_L_Upper_Rotate  = false;
    static bool _Cyl_L_Upper_Grab    = false;
    static bool _Cyl_L_Upper_Deploy  = false;
    static bool _Cyl_L_Lower_Grab    = false;
    static bool _Cyl_L_Lower_Deploy  = false;
    static bool _Cyl_Defend_Grab     = false;
    static bool _Cyl_Defend_Rise     = false;
    static bool _Cyl_Defend_Press    = false;
    static bool _Cyl_Ball_Grab       = false;
    static bool _Cyl_Ball_Gather     = false;
    static bool _Cyl_Ball_Rise       = false;

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
    //NODELET_INFO("Pad X : %d",_padx);
    //NODELET_INFO("Pad Y %d",_pady);
    //NODELET_INFO("Left Thumb X %f",joy->axes[AxisLeftThumbX]);
    //NODELET_INFO("Left Thumb Y %f",joy->axes[AxisLeftThumbY]);
    //NODELET_INFO("%d",_y);
    //NODELET_INFO("%d",_enable_steerAdjust);
    //if(_enable_steerAdjust){
    //    if(_start && _y)
    //    {
    //        _enable_steerAdjust = false;
    //        return;
    //    } 
    //    if(_pady == 1){
    //        if(_a) this->adjust_pubData.data[0] += 2.0*pi;
    //        if(_b) this->adjust_pubData.data[1] += 2.0*pi;
    //        if(_x) this->adjust_pubData.data[2] += 2.0*pi;
    //        if(_y) this->adjust_pubData.data[3] += 2.0*pi;
    //    }else if(_pady == -1){
    //        if(_a) this->adjust_pubData.data[0] -= 2.0*pi;
    //        if(_b) this->adjust_pubData.data[1] -= 2.0*pi;
    //        if(_x) this->adjust_pubData.data[2] -= 2.0*pi;
    //        if(_y) this->adjust_pubData.data[3] -= 2.0*pi;
    //    }
    //    SteerAdjust_pub.publish(this->adjust_pubData);
    //    return;
    //}

    //Defence_Lift MAX 39.4

    if (_start)
    {
        _arm_l_avoidlagoribase = false;
        _arm_l_avoid1lagori = false;
        _arm_r_avoidlagoribase = false;
        _arm_r_avoid1lagori = false;
        _Arm_Deploy = false;
        _Arm_R_moving = false;
        _Arm_L_moving = false;

        if(_b){
            this->recover();
        }else if(_y){
            this->Arm_R_homing();
            this->Arm_L_homing();
            this->Defence_Lift_homing();
            this->Defence_Roll_homing();
            NODELET_INFO("Complete Homing");
        }else if(_x){
            this->_homing_Mode = true;
            this-> _piling_manual_Mode = false;
            this-> _piling_auto_Mode = false;
            this-> _ballPick_Mode = false;
            this-> _defence_Mode = false;
        }else if(_pady == -1 && !_piling_manual_Mode){
            this-> _piling_manual_Mode = true;
            this-> _piling_auto_Mode = false;
            this-> _ballPick_Mode = false;
            this-> _defence_Mode = false;
            return;
        }else if(_pady == 1 && !_piling_auto_Mode){
            this-> _piling_manual_Mode = false;
            this-> _piling_auto_Mode = true;
            this-> _ballPick_Mode = false;
            this-> _defence_Mode = false;
            return;
        }else if(_padx == -1 && !_defence_Mode){
            this-> _piling_manual_Mode = false;
            this-> _piling_auto_Mode = false;
            this-> _ballPick_Mode = false;
            this-> _defence_Mode = true;
            return;
        }else if(_padx == 1 && !_ballPick_Mode){
            this-> _piling_manual_Mode = false;
            this-> _piling_auto_Mode = false;
            this-> _ballPick_Mode = true;
            this-> _defence_Mode = false;
            return;
        }
    }
    if (_back)
    {
        this->shutdown();
        this->_command_ongoing = false;
        _autoPile_L_mode_count = -1;
        _autoPile_R_mode_count = -1;
        _arm_r_avoidlagoribase = false;
        _arm_r_avoid1lagori = false;
        _arm_l_avoidlagoribase = false;
        _arm_l_avoid1lagori = false;
        _Arm_Deploy = false;
        _Defend_mode_count = -1;
        _defence_lift_upper_speeddown = false;
        _Arm_R_moving = false;
        _Arm_L_moving = false;
    }
   
    //if (_righttrigger && (_padx != -1))
    //{   
    //    this->steer_homing();
    //}


    //--------------------------------------------------------------------------------------------------------------------------------
    if(_piling_auto_Mode){
        NODELET_INFO("Piling Auto Mode");
        if(!(_autoPile_L_mode_count == 3 && !_arm_l_avoid1lagori) && !(_autoPile_L_mode_count == 6 && !_arm_l_avoidlagoribase)){
            if(joy->buttons[ButtonLeftThumb] != 0.0){
                //Cylinder_Operation("L_Clutch",true);
                this->Arm_L_move_Vel(joy->buttons[ButtonLeftThumb] * 15.0);
            }else if(_lb){
                //Cylinder_Operation("L_Clutch",true);
                this->Arm_L_move_Vel(-15.0);
            }else{
                //Cylinder_Operation("L_Clutch",false);
                this->Arm_L_move_Vel(0.0);
            }
        }
        if(!(_autoPile_R_mode_count == 3 && !_arm_r_avoid1lagori) && !(_autoPile_R_mode_count == 6 && !_arm_r_avoidlagoribase)){
            if(joy->buttons[ButtonRightThumb] != 0.0){
                //Cylinder_Operation("R_Clutch",true);
                this->Arm_R_move_Vel(joy->buttons[ButtonRightThumb] * -15.0);
            }else if(_rb){
                //Cylinder_Operation("R_Clutch",true);
                this->Arm_R_move_Vel(15.0);
            }else{
                //Cylinder_Operation("R_Clutch",false);
                this->Arm_R_move_Vel(0.0);
            }
        }
        //if(_pady != 1 && _padx != 1){
        //    if(!_command_ongoing){
        //        if(_y){
        //            this->Defence_Roll_move_Vel(3.0);
        //        }else if(_a){
        //            this->Defence_Roll_move_Vel(-3.0);
        //        }else{
        //            this->Defence_Roll_move_Vel(0);
        //        }
        //    }
        //}else if(_pady == 1){
        //    if(_y && _y_enable){
        //        if(_Cyl_Defend_Grab){
        //            Cylinder_Operation("Defend_Grab",true);
        //            _Cyl_Defend_Grab = false;
        //        }else{
        //            Cylinder_Operation("Defend_Grab",false);
        //            _Cyl_Defend_Grab = true;
        //        }
        //        _y_enable = false;
        //    }
        //}
        //if(_padx == 1 && !_command_ongoing){
        //    if(_y){
        //        this->Defence_Lift_move_Vel(3.0);
        //    }else if(_a){
        //        this->Defence_Lift_move_Vel(-3.0);
        //    }else{
        //        this->Defence_Lift_move_Vel(0);
        //    }
        //}
        //else if(_pady == -1){
        //    this->command_list = &Defence_Enable_commands;
        //    _command_ongoing = true;
        //}
        if(_autoPile_L_mode_count == -1 && _autoPile_R_mode_count == -1){
            if(!_start && ((_b && _b_enable) || (_x && _x_enable))){
                R_mode_Count(1);
                L_mode_Count(1);
                //_autoPile_R_mode_count = 0;
                //_autoPile_L_mode_count = 0;
            }
        }else{
            if(_b && _b_enable && !_start){
                if(_pady == 1){
                    if(_Cyl_R_Upper_Rotate){
                        Cylinder_Operation("R_Upper_Rotate",true);
                        _Cyl_R_Upper_Rotate = false;
                    }else{
                        Cylinder_Operation("R_Upper_Rotate",false);
                        _Cyl_R_Upper_Rotate = true;
                    }
                }else if(_padx == -1){
                    R_mode_Count(-1);
                }else{
                    R_mode_Count(1);
                }
                _b_enable = false;
            }
            if(_x && _x_enable && !_start){
                if(_pady == 1){
                    if(_Cyl_L_Upper_Rotate){
                        Cylinder_Operation("L_Upper_Rotate",true);
                        _Cyl_L_Upper_Rotate = false;
                    }else{
                        Cylinder_Operation("L_Upper_Rotate",false);
                        _Cyl_L_Upper_Rotate = true;
                    }
                }else if(_padx == -1){
                    L_mode_Count(-1);
                }else{
                    L_mode_Count(1);
                }
                _x_enable = false;
            }
        }
        if(_y && _y_enable && !_start){
            if(_padx == -1){
                Defend_mode_Count(-1);
            }else{
                Defend_mode_Count(1);
            }
            _y_enable = false;
        }
        if(_a && _a_enable && !_start){
            if(_padx == 1){
                if(!_reverse_control){
                    _reverse_control = true;
                    _swap_control = false;
                }else{
                    _reverse_control = false;
                    _swap_control = false;
                }
            }else if(_padx == -1){
                this->Defence_Lift_move_Vel(8.0);
            }else{
                this->Defence_Lift_move_Vel(-8.0);
            }
            _a_enable = false;
        }else if(!_command_ongoing && !_a){
            this->Defence_Lift_move_Vel(0);
        }
        
        if(_Defend_mode_count != _recent_Defend_mode_count){
            switch (_Defend_mode_count)
            {
            case 0:
                this->command_list = &Defence_Up_commands;
                _command_ongoing = true;   
                break;
            case 1:
                this->command_list = &Defence_Dodge_commands;
                _command_ongoing = true;   
                break;
            case 2:
                Cylinder_Operation("Defend_Grab",false);   
                break;
            case 3:
                Cylinder_Operation("Defend_Grab",true);   
                break;
            case 4:
                this->command_list = &Defence_Enable_commands;
                _command_ongoing = true;   
                _reverse_control = false;
                _swap_control = true;
                break;
            case 5:
                this->command_list = &Defence_Release_commands;
                _command_ongoing = true;   
                break;
            case 6:
                this->command_list = &Defence_Unable_commands;
                _command_ongoing = true;   
                break;
            case 7:
                this->command_list = &Defence_Down_commands;
                _command_ongoing = true;   
                break;
            
            default:
                break;
            }
        }
        _recent_Defend_mode_count = _Defend_mode_count;
    }else if(_piling_manual_Mode){
        NODELET_INFO("Piling Manual Mode");
        if(joy->buttons[ButtonRightThumb] != 0.0){
            Cylinder_Operation("R_Clutch",true);
            if(_rb){
                this->Arm_R_move_Vel(joy->buttons[ButtonRightThumb] * -15.0);
            }else{
                this->Arm_R_move_Vel(joy->buttons[ButtonRightThumb] * 15.0);
            }
        }else{
            //Cylinder_Operation("R_Clutch",false);
            this->Arm_R_move_Vel(0.0);
        }
        if(joy->buttons[ButtonLeftThumb] != 0.0){
            Cylinder_Operation("L_Clutch",true);
            if(_lb){
                this->Arm_L_move_Vel(joy->buttons[ButtonLeftThumb] * 15.0);
            }else{
                this->Arm_L_move_Vel(joy->buttons[ButtonLeftThumb] * -15.0);
            }
        }else{
            //Cylinder_Operation("L_Clutch",false);
            this->Arm_L_move_Vel(0.0);
        }
        if(_padx == -1 && _pady == 1){ //right upper
            if(_b && _b_enable){
                if(_Cyl_R_Upper_Grab){
                    Cylinder_Operation("R_Upper_Grab",true);
                    _Cyl_R_Upper_Grab = false;
                }else{
                    Cylinder_Operation("R_Upper_Grab",false);
                    _Cyl_R_Upper_Grab = true;
                }
                _b_enable = false;
            }
            if(_x && _x_enable){
                if(_Cyl_R_Upper_Rotate){
                    Cylinder_Operation("R_Upper_Rotate",true);
                    _Cyl_R_Upper_Rotate = false;
                }else{
                    Cylinder_Operation("R_Upper_Rotate",false);
                    _Cyl_R_Upper_Rotate = true;
                }
                _x_enable = false;
            }
            if(_y && _y_enable){
                Cylinder_Operation("R_Upper_Deploy",true);
                _y_enable = false;
            }
            if(_a && _a_enable){
                Cylinder_Operation("R_Upper_Deploy",false);
                _a_enable = false;
            }
        }else if(_padx == -1 && _pady == -1){ //right lower
            if(_b && _b_enable){
                if(_Cyl_R_Lower_Grab){
                    Cylinder_Operation("R_Lower_Grab",true);
                    _Cyl_R_Lower_Grab = false;
                }else{
                    Cylinder_Operation("R_Lower_Grab",false);
                    _Cyl_R_Lower_Grab = true;
                }
                _b_enable = false;
            }
            if(_y && _y_enable){
                Cylinder_Operation("R_Lower_Deploy",true);
                _y_enable = false;
            }
            if(_a && _a_enable){
                Cylinder_Operation("R_Lower_Deploy",false);
                _a_enable = false;
            }
        }else if(_padx == 1 && _pady == 1){ //left upper
            if(_b && _b_enable){
                if(_Cyl_L_Upper_Grab){
                    NODELET_INFO("True");
                    Cylinder_Operation("L_Upper_Grab",true);
                    _Cyl_L_Upper_Grab = false;
                }else{
                    NODELET_INFO("False");
                    Cylinder_Operation("L_Upper_Grab",false);
                    _Cyl_L_Upper_Grab = true;
                }
                _b_enable = false;
            }
            if(_x && _x_enable){
                if(_Cyl_L_Upper_Rotate){
                    Cylinder_Operation("L_Upper_Rotate",true);
                    _Cyl_L_Upper_Rotate = false;
                }else{
                    Cylinder_Operation("L_Upper_Rotate",false);
                    _Cyl_L_Upper_Rotate = true;
                }
                _x_enable = false;
            }
            if(_y && _y_enable){
                Cylinder_Operation("L_Upper_Deploy",true);
                _y_enable = false;
            }
            if(_a && _a_enable){
                Cylinder_Operation("L_Upper_Deploy",false);
                _a_enable = false;
            }
        }else if(_padx == 1 && _pady == -1){ //left lower
            if(_b && _b_enable){
                if(_Cyl_L_Lower_Grab){
                    Cylinder_Operation("L_Lower_Grab",true);
                    _Cyl_L_Lower_Grab = false;
                }else{
                    Cylinder_Operation("L_Lower_Grab",false);
                    _Cyl_L_Lower_Grab = true;
                }
                _b_enable = false;
            }
            if(_y && _y_enable){
                Cylinder_Operation("L_Lower_Deploy",true);
                _y_enable = false;
            }
            if(_a && _a_enable){
                Cylinder_Operation("L_Lower_Deploy",false);
                _a_enable = false;
            }
        }
    }else if(_defence_Mode){ //------------------------------------------------------------------------------------------------------
        NODELET_INFO("Defence Mode");
        //if (_start) //&& _padx == 1)
        //{
        //    this-> _piling_Mode = true;
        //    this-> _ballPick_Mode = false;
        //    this-> _defence_Mode = false;
        //    return;
        //}
        if(joy->buttons[ButtonRightThumb] != 0.0){
            this->Defence_Lift_move_Vel(joy->buttons[ButtonRightThumb] * -8.0);
        }else if(_rb){
            this->Defence_Lift_move_Vel(8.0);
        }else{
            this->Defence_Lift_move_Vel(0.0);
        }
        if(joy->buttons[ButtonLeftThumb] != 0.0){
            this->Defence_Roll_move_Vel(joy->buttons[ButtonLeftThumb] * -5.0);
        }else if(_lb){
            this->Defence_Roll_move_Vel(5.0);
        }else{
            this->Defence_Roll_move_Vel(0.0);
        }
        if(_b && _b_enable){
            if(_Cyl_Defend_Grab){
                Cylinder_Operation("Defend_Grab",true);
                _Cyl_Defend_Grab = false;
            }else{
                Cylinder_Operation("Defend_Grab",false);
                _Cyl_Defend_Grab = true;
            }
            _b_enable = false;
        }
        if(_y && _y_enable){
            if(_Cyl_Defend_Rise){
                Cylinder_Operation("Defend_Rise",true);
                _Cyl_Defend_Rise = false;
            }else{
                Cylinder_Operation("Defend_Rise",false);
                _Cyl_Defend_Rise = true;
            }
            _y_enable = false;
        }
        if(_a && _a_enable){
            if(_Cyl_Defend_Press){
                Cylinder_Operation("Defend_Press",true);
                _Cyl_Defend_Press = false;
            }else{
                Cylinder_Operation("Defend_Press",false);
                _Cyl_Defend_Press = true;
            }
            _a_enable = false;
        }
    }else if(_ballPick_Mode){ //------------------------------------------------------------------------------------------------------
        NODELET_INFO("Ball Pick Mode");
        //if (_start)// && _padx == -1)
        //{
        //    this-> _piling_Mode = true;
        //    this-> _ballPick_Mode = false;
        //    this-> _defence_Mode = false;
        //    return;
        //}
        if(_b && _b_enable){
            if(_Cyl_Ball_Grab){
                Cylinder_Operation("Ball_Grab",true);
                Cylinder_Operation("Ball_Gather",true);
                _Cyl_Ball_Grab = false;
            }else{
                Cylinder_Operation("Ball_Grab",false);
                Cylinder_Operation("Ball_Gather",false);
                _Cyl_Ball_Grab = true;
            }
            _b_enable = false;
        }
        if(_y && _y_enable){
            if(_Cyl_Ball_Rise){
                Cylinder_Operation("Ball_Rise",true);
                _Cyl_Ball_Rise = false;
            }else{
                Cylinder_Operation("Ball_Rise",false);
                _Cyl_Ball_Rise = true;
            }
            _y_enable = false;
        }
    }else if(_homing_Mode){ //------------------------------------------------------------------------------------------------------
        NODELET_INFO("Homing Mode");
        if(joy->buttons[ButtonRightThumb] != 0.0){
            this->Arm_R_move_Vel(joy->buttons[ButtonRightThumb] * -5.0);
            Cylinder_Operation("R_Clutch",true);
        }else if(_rb){
            this->Arm_R_move_Vel(5.0);
            Cylinder_Operation("R_Clutch",true);
        }else{
            this->Arm_R_move_Vel(0.0);
            Cylinder_Operation("R_Clutch",false);
        }
        if(joy->buttons[ButtonLeftThumb] != 0.0){
            this->Arm_L_move_Vel(joy->buttons[ButtonLeftThumb] * -5.0);
            Cylinder_Operation("L_Clutch",true);
        }else if(_lb){
            this->Arm_L_move_Vel(5.0);
            Cylinder_Operation("L_Clutch",true);
        }else{
            this->Arm_L_move_Vel(0.0);
            Cylinder_Operation("L_Clutch",false);
        }
        if(!_start){
            if(_y){
                this->Defence_Lift_move_Vel(5.0);
            }else if(_a){
                this->Defence_Lift_move_Vel(-5.0);
            }else{
                this->Defence_Lift_move_Vel(0.0);
            }
            if(_x){
                this->Defence_Roll_move_Vel(3.0);
            }else if(_b){
                this->Defence_Roll_move_Vel(-3.0);
            }else{
                this->Defence_Roll_move_Vel(0.0);
            }
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
    if(!_y){
        _y_enable = true;
    }
    
    ///--------------------------------------------------------------------------------------------------------------------------------
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
        //if(joy->buttons[ButtonLeftThumb] >= 1.0){
        //    this->cmd_vel_msg.linear.x = vel_x * 0.3;
        //    this->cmd_vel_msg.linear.y = vel_y * 0.3;
        //    this->cmd_vel_msg.angular.z = -vel_yaw * 0.3;
        //}else if(joy->buttons[ButtonRightThumb] >= 1.0){
        //    this->cmd_vel_msg.linear.x = vel_x * 5;
        //    this->cmd_vel_msg.linear.y = vel_y * 5;
        //    this->cmd_vel_msg.angular.z = -vel_yaw * 3.5;
        //}else{
        //    this->cmd_vel_msg.linear.x = vel_x;
        //    this->cmd_vel_msg.linear.y = vel_y;
        //    this->cmd_vel_msg.angular.z = -vel_yaw;
        //}
        if(_reverse_control){
            this->cmd_vel_msg.linear.x  = -1.0 * vel_x * 1;
            this->cmd_vel_msg.linear.y  = -1.0 * -1.0 * vel_y * 1;
            this->cmd_vel_msg.angular.z = -vel_yaw * 1;
        }else if(_swap_control){
            this->cmd_vel_msg.linear.x  = -1.0 * vel_y * 1;
            this->cmd_vel_msg.linear.y  = -1.0 * vel_x * 1;
            this->cmd_vel_msg.angular.z = -vel_yaw * 1;
        }else{
            this->cmd_vel_msg.linear.x  = vel_x * 1;
            this->cmd_vel_msg.linear.y  = -1.0 * vel_y * 1;
            this->cmd_vel_msg.angular.z = -vel_yaw * 1;
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

void MR2_nodelet_main::control_timer_callback(const ros::TimerEvent &event)
{ 
    if(_recent_R_mode_count != _autoPile_R_mode_count || _recent_L_mode_count != _autoPile_L_mode_count){
        if(_autoPile_R_mode_count == 0 && _autoPile_L_mode_count == 0){
            this->_delay_s_r = ros::Time::now().toSec() + 0.5;
        }    
    }
    if(_autoPile_R_mode_count == 0 && _autoPile_L_mode_count == 0 && !_Arm_Deploy){
        Cylinder_Operation("R_Clutch",true);
        Cylinder_Operation("R_Upper_Deploy",false);       
        Cylinder_Operation("L_Clutch",true);
        Cylinder_Operation("L_Upper_Deploy",false);       
        if(this->_delay_s_r < ros::Time::now().toSec()){
            this->_delay_s_r = 0.0;
            Cylinder_Operation("R_Upper_Grab",false);
            Cylinder_Operation("R_Upper_Rotate",false);
            Cylinder_Operation("R_Lower_Grab",false);
            Cylinder_Operation("R_Lower_Deploy",false);
            Cylinder_Operation("L_Upper_Grab",false);
            Cylinder_Operation("L_Upper_Rotate",false);
            Cylinder_Operation("L_Lower_Grab",false);
            Cylinder_Operation("L_Lower_Deploy",false);
            _Arm_Deploy = true;
        }
    }


    if(_recent_R_mode_count != _autoPile_R_mode_count || _autoPile_R_mode_count == 3 || _autoPile_R_mode_count == 6){
        switch (_autoPile_R_mode_count)
        {
        case 0:
            _arm_r_avoidlagoribase = false;
            _arm_r_avoid1lagori = false;
            Cylinder_Operation("R_Clutch",true);
            Cylinder_Operation("R_Upper_Deploy",false);
            //this->_delay_s_r = ros::Time::now().toSec() + 0.5;       
            //while(this->_delay_s_r > ros::Time::now().toSec());
            //this->_delay_s_r = 0.0;
            Cylinder_Operation("R_Upper_Grab",false);
            Cylinder_Operation("R_Upper_Rotate",false);
            Cylinder_Operation("R_Lower_Grab",false);
            Cylinder_Operation("R_Lower_Deploy",false);
            break;
        case 1:
            Cylinder_Operation("R_Upper_Grab",false);
            break;
        case 2:
            Cylinder_Operation("R_Upper_Grab",true);
            break;
        case 3:
            if(!_Arm_R_moving && !_arm_r_avoid1lagori){
                _Arm_R_moving = true;
                this->Arm_R_move_Vel(15.0);
            }else{
                if(Arm_R_position >= arm_r_avoid1lagori_pos && !_arm_r_avoid1lagori){
                    this->Arm_R_move_Vel(0.0);
                    _Arm_R_moving = false;
                    _arm_r_avoid1lagori = true;
                }
            }
            break;
        case 4:
            _Arm_R_moving = false;
            _arm_r_avoid1lagori = false;
            Cylinder_Operation("R_Lower_Grab",false);
            break;
        case 5:
            Cylinder_Operation("R_Lower_Grab",true);
            break;
        //case 7:
        //case 8:
        //    //L_LIFT MOVE
        //    break;
        case 6:
            if(!_Arm_R_moving && !_arm_r_avoidlagoribase){
                _Arm_R_moving = true;
                this->Arm_R_move_Vel(15.0);
            }else{
                if(Arm_R_position >= arm_r_avoidlagoribase_pos && !_arm_r_avoidlagoribase){
                    this->Arm_R_move_Vel(0.0);
                    _Arm_R_moving = false;
                    _arm_r_avoidlagoribase = true;
                }
            }
            break;
        case 7:
            _Arm_R_moving = false;
            _arm_r_avoidlagoribase = false;
            Cylinder_Operation("R_Lower_Grab",true);
            break;
        case 8:
            Cylinder_Operation("R_Lower_Grab",false);
            break;
        case 9:
            Cylinder_Operation("R_Lower_Deploy",false);
            break;
        case 10:
            Cylinder_Operation("R_Lower_Deploy",true);
            break;
        case 11:
            Cylinder_Operation("R_Upper_Grab",true);
            break;
        case 12:
            Cylinder_Operation("R_Upper_Grab",false);
            break;
        case 13:
            Cylinder_Operation("R_Upper_Deploy",false);
            break;
        case 14:
            if(fabs(Arm_R_position) >= 30.0 && fabs(Arm_R_position) <= 50.0){
                Cylinder_Operation("R_Upper_Deploy",true);
            }else{
                Cylinder_Operation("R_Lower_Deploy",false);
            }
            Cylinder_Operation("R_Upper_Rotate",false);
            break;
        case 15:
            Cylinder_Operation("R_Lower_Grab",false);
            Cylinder_Operation("R_Upper_Grab",false);
            break;
        case 16:
            Cylinder_Operation("R_Lower_Grab",true);
            Cylinder_Operation("R_Upper_Grab",true);
            break;
        case 17:
            Cylinder_Operation("R_Lower_Grab",false);
            Cylinder_Operation("R_Upper_Grab",false);
            break;
        default:
            break;
        }
    }
    if(_recent_L_mode_count != _autoPile_L_mode_count || _autoPile_L_mode_count == 3 || _autoPile_L_mode_count == 6){
        switch (_autoPile_L_mode_count)
        {
        case 0:
            _arm_l_avoidlagoribase = false;
            _arm_l_avoid1lagori = false;
            Cylinder_Operation("L_Clutch",true);
            Cylinder_Operation("L_Upper_Deploy",false);
            //this->_delay_s_l = ros::Time::now().toSec() + 0.5;       
            //while(this->_delay_s_l > ros::Time::now().toSec());
            //this->_delay_s_l = 0.0;
            Cylinder_Operation("L_Upper_Grab",false);
            Cylinder_Operation("L_Upper_Rotate",false);
            Cylinder_Operation("L_Lower_Grab",false);
            Cylinder_Operation("L_Lower_Deploy",false);
            break;
        case 1:
            Cylinder_Operation("L_Upper_Grab",false);
            break;
        case 2:
            Cylinder_Operation("L_Upper_Grab",true);
            break;
        case 3:
            if(!_Arm_L_moving && !_arm_l_avoid1lagori){
                _Arm_L_moving = true;
                this->Arm_L_move_Vel(-15.0);
            }else{
                if(Arm_L_position <= arm_l_avoid1lagori_pos && !_arm_l_avoid1lagori){
                    this->Arm_L_move_Vel(0.0);
                    _Arm_L_moving = false;
                    _arm_l_avoid1lagori = true;
                }
            }
            break;
        case 4:
            _Arm_L_moving = false;
            _arm_l_avoid1lagori = false;
            Cylinder_Operation("L_Lower_Grab",false);
            break;
        case 5:
            Cylinder_Operation("L_Lower_Grab",true);
            break;
        //case 7:
        //case 8:
        //    //L_LIFT MOVE
        //    break;
        case 6:
            if(!_Arm_L_moving && !_arm_l_avoidlagoribase){
                _Arm_L_moving = true;
                this->Arm_L_move_Vel(-15.0);
            }else{
                if(Arm_L_position <= arm_l_avoidlagoribase_pos && !_arm_l_avoidlagoribase){
                    this->Arm_L_move_Vel(0.0);
                    _Arm_L_moving = false;
                    _arm_l_avoidlagoribase = true;
                }
            }
            break;
        case 7:
            _Arm_L_moving = false;
            _arm_l_avoidlagoribase = false;
            Cylinder_Operation("L_Lower_Grab",true);
            break;
        case 8:
            Cylinder_Operation("L_Lower_Grab",false);
            break;
        case 9:
            Cylinder_Operation("L_Lower_Deploy",false);
            break;
        case 10:
            Cylinder_Operation("L_Lower_Deploy",true);
            break;
        case 11:
            Cylinder_Operation("L_Upper_Grab",true);
            break;
        case 12:
            Cylinder_Operation("L_Upper_Grab",false);
            break;
        case 13:
            Cylinder_Operation("L_Upper_Deploy",false);
            break;
        case 14:
            if(fabs(Arm_L_position) >= 30.0 && fabs(Arm_L_position) <= 50.0){
                Cylinder_Operation("L_Upper_Deploy",true);
            }else{
                Cylinder_Operation("L_Lower_Deploy",false);
            }
            Cylinder_Operation("L_Upper_Rotate",false);
            break;
        case 15:
            Cylinder_Operation("L_Lower_Grab",false);
            Cylinder_Operation("L_Upper_Grab",false);
            break;
        case 16:
            Cylinder_Operation("L_Lower_Grab",true);
            Cylinder_Operation("L_Upper_Grab",true);
            break;
        case 17:
            Cylinder_Operation("L_Lower_Grab",false);
            Cylinder_Operation("L_Upper_Grab",false);
            break;
        default:
            break;
        }
    }
    _recent_R_mode_count = _autoPile_R_mode_count;
    _recent_L_mode_count = _autoPile_L_mode_count;


    //this->command_list->size() <= (int)this->currentCommandIndex || this->command_list == &this->manual_all
    if (!this->_command_ongoing)
    {
        //NODELET_INFO("control_time_return");
        return;
    }
    //ROS_INFO("control_time_return");

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

    if(currentCommand == ControllerCommands::defenceLift_mv_Upper){
        Defence_Lift_move_Pos(this-> defence_lift_upper_pos);
        this->currentCommandIndex++;
        NODELET_INFO("Defence Lift Move to Upper");
    }else if(currentCommand == ControllerCommands::defenceLift_mv_Upper_slowly){
        if(defenceLift_position >= defence_lift_upper_pos-1.0){
            _defence_lift_upper_speeddown = false;
            Defence_Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Defence Lift Move to Upper Slowly");
        }else if(defenceLift_position >= defence_lift_upper_pos-10.0){
            Defence_Lift_move_Vel(8.0);
            _defence_lift_upper_speeddown = true;
        }else if(!_defence_lift_upper_speeddown){
            Defence_Lift_move_Vel(15.0);
        }
    }else if(currentCommand == ControllerCommands::defenceLift_mv_Lower){
        Defence_Lift_move_Pos(this-> defence_lift_lower_pos);
        this->currentCommandIndex++;
        NODELET_INFO("Defence Lift Move to Lower");
    }else if(currentCommand == ControllerCommands::defenceLift_mv_Lower_slowly){
        Defence_Lift_move_Vel(-8.0);
        if(defenceLift_position <= defence_lift_lower_pos+1.0){
            Defence_Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Defence Lift Move to Lower Slowly");
        }
    }else if(currentCommand == ControllerCommands::defenceLift_mv_LagoriBase){
        Defence_Lift_move_Pos(this-> defence_lift_lagoribase_pos);
        this->currentCommandIndex++;
        NODELET_INFO("Defence Lift Move to LagoriBase");
    }else if(currentCommand == ControllerCommands::defenceLift_mv_LagoriBase_slowly){
        Defence_Lift_move_Vel(-8.0);
        if(defenceLift_position <= defence_lift_lagoribase_pos+2.0){
            Defence_Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Defence Lift Move to LagoriBase Slowly");
        }
    }else if(currentCommand == ControllerCommands::defenceLift_mv_DodgeLagori_slowly){
        Defence_Lift_move_Vel(-8.0);
        if(defenceLift_position <= defence_lift_dodgelagori_pos+2.0){
            Defence_Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Defence Lift Move to DodgeLagori Slowly");
        }
    }else if(currentCommand == ControllerCommands::defenceRoll_mv_Horizontal){
        Defence_Roll_move_Pos(this-> defence_roll_horizontal_pos);
        this->currentCommandIndex++;
        NODELET_INFO("Defence Roll Move to Horizontal");
    }else if(currentCommand == ControllerCommands::Cyl_Defend_Grab_On){
        Cylinder_Operation("Defend_Grab", true);
        this->currentCommandIndex++;
        NODELET_INFO("Defende_Grab Enabled");
    }else if(currentCommand == ControllerCommands::Cyl_Defend_Grab_Off){
        Cylinder_Operation("Defend_Grab", false);
        this->currentCommandIndex++;
        NODELET_INFO("Defende_Grab Unabled");
    }else if(currentCommand == ControllerCommands::Cyl_Defend_Rise_On){
        Cylinder_Operation("Defend_Rise", true);
        this->currentCommandIndex++;
        NODELET_INFO("Defende_Rise Enabled");
    }else if(currentCommand == ControllerCommands::Cyl_Defend_Rise_Off){
        Cylinder_Operation("Defend_Rise", false);
        this->currentCommandIndex++;
        NODELET_INFO("Defende_Rise Unabled");
    }else if(currentCommand == ControllerCommands::Cyl_Defend_Press_On){
        Cylinder_Operation("Defend_Press", true);
        this->currentCommandIndex++;
        NODELET_INFO("Defende_Press Enabled");
    }else if(currentCommand == ControllerCommands::Cyl_Defend_Press_Off){
        Cylinder_Operation("Defend_Press", false);
        this->currentCommandIndex++;
        NODELET_INFO("Defende_Press Unabled");
    }else if(currentCommand == ControllerCommands::wait_defenceLift_Upper){
        if(defenceLift_position >= defence_lift_upper_pos - 1.0){
            this->currentCommandIndex++;
            NODELET_INFO("DefenceLift reached Upper");
        }
    }else if(currentCommand == ControllerCommands::wait_defenceLift_Lower){
        if(defenceLift_position <= defence_lift_lower_pos + 1.0){
            this->currentCommandIndex++;
            NODELET_INFO("DefenceLift reached Lower");
        }
    }else if(currentCommand == ControllerCommands::wait_defenceRoll_horizontal){
        if((defenceRoll_position >= defence_roll_horizontal_pos - 0.5) && (defenceRoll_position <= defence_roll_horizontal_pos + 0.5)){
            this->currentCommandIndex++;
            NODELET_INFO("DefenceRoll reached Horizontal");
        }
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

void MR2_nodelet_main::send_steerAdjust(void){
    SteerAdjust_pub.publish(this->adjust_pubData);
}

//void MR2_nodelet_main::steer_homing(void){
//    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
//    steer_CmdPub0.publish(act_conf_cmd_msg);
//    steer_CmdPub1.publish(act_conf_cmd_msg);
//    steer_CmdPub2.publish(act_conf_cmd_msg);
//    steer_CmdPub3.publish(act_conf_cmd_msg);
//    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
//    steer_CmdPub0.publish(act_conf_cmd_msg);
//    steer_CmdPub1.publish(act_conf_cmd_msg);
//    steer_CmdPub2.publish(act_conf_cmd_msg);
//    steer_CmdPub3.publish(act_conf_cmd_msg);
//}

void MR2_nodelet_main::Arm_R_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->Arm_R_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->Arm_R_Cmd_pub.publish(act_conf_cmd_msg);
}
void MR2_nodelet_main::Arm_L_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->Arm_L_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->Arm_L_Cmd_pub.publish(act_conf_cmd_msg);
}
void MR2_nodelet_main::Defence_Lift_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->Defence_Lift_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->Defence_Lift_Cmd_pub.publish(act_conf_cmd_msg);
}
void MR2_nodelet_main::Defence_Roll_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg);
}

void MR2_nodelet_main::Arm_R_move_Vel(double target){
    //act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    //Arm_R_Cmd_pub.publish(act_conf_cmd_msg);
    this->Arm_R_Value_msg.data = target;
    this->Arm_R_Value_pub.publish(this->Arm_R_Value_msg);
}
void MR2_nodelet_main::Arm_L_move_Vel(double target){
    //act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    //Arm_L_Cmd_pub.publish(act_conf_cmd_msg);
    this->Arm_L_Value_msg.data = target;
    this->Arm_L_Value_pub.publish(this->Arm_L_Value_msg);
}
void MR2_nodelet_main::Defence_Lift_move_Vel(double target){
    //act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    //Defence_Lift_Cmd_pub.publish(act_conf_cmd_msg);
    this->Defence_Lift_Value_msg.data = target;
    this->Defence_Lift_Value_pub.publish(this->Defence_Lift_Value_msg);
}
void MR2_nodelet_main::Defence_Lift_move_Pos(double target){
    this->_delay_s = ros::Time::now().toSec() + 0.5;
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    Defence_Lift_Cmd_pub.publish(act_conf_cmd_msg);

    while(this->_delay_s > ros::Time::now().toSec());
    this->_delay_s = 0.0;
    this->Defence_Lift_Value_msg.data = target;
    this->Defence_Lift_Value_pub.publish(this->Defence_Lift_Value_msg);
}
void MR2_nodelet_main::Defence_Roll_move_Vel(double target){
    act_conf_cmd_msg2.data = (uint8_t)MotorCommands::recover_velocity;
    Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg2);
    this->Defence_Roll_Value_msg.data = target;
    this->Defence_Roll_Value_pub.publish(this->Defence_Roll_Value_msg);
}
void MR2_nodelet_main::Defence_Roll_move_Pos(double target){
    this->_delay_s = ros::Time::now().toSec() + 0.5;
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_position;
    Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg);

    while(this->_delay_s > ros::Time::now().toSec());
    this->_delay_s = 0.0;
    this->Defence_Roll_Value_msg.data = target;
    this->Defence_Roll_Value_pub.publish(this->Defence_Roll_Value_msg);
}

void MR2_nodelet_main::R_mode_Count(int count){
    if(count == 0){
        this->_autoPile_R_mode_count = 0;
        NODELET_INFO("R_mode_count : %d",_autoPile_R_mode_count);
        return;    
    }
    this->_autoPile_R_mode_count += count;
    if(_autoPile_R_mode_count > R_mode_count_max){
        this->_autoPile_R_mode_count = 0;
    }else if(_autoPile_R_mode_count < 0){
        this->_autoPile_R_mode_count = R_mode_count_max;
    }
    NODELET_INFO("R_mode_count : %d",_autoPile_R_mode_count);
}
void MR2_nodelet_main::L_mode_Count(int count){
    if(count == 0){
        this->_autoPile_L_mode_count = 0;
        NODELET_INFO("L_mode_count : %d",_autoPile_L_mode_count);
        return;    
    }
    this->_autoPile_L_mode_count += count;
    if(_autoPile_L_mode_count > L_mode_count_max){
        this->_autoPile_L_mode_count = 0;
    }else if(_autoPile_L_mode_count < 0){
        this->_autoPile_L_mode_count = L_mode_count_max;
    }
    NODELET_INFO("L_mode_count : %d",_autoPile_L_mode_count);
}
void MR2_nodelet_main::Defend_mode_Count(int count){
    if(count == 0){
        this->_Defend_mode_count = 0;
        NODELET_INFO("Defend_mode_count : %d",_Defend_mode_count);
        return;    
    }
    this->_Defend_mode_count += count;
    if(_Defend_mode_count > Defend_mode_count_max){
        this->_Defend_mode_count = 0;
    }else if(_Defend_mode_count < 0){
        this->_Defend_mode_count = Defend_mode_count_max;
    }
    NODELET_INFO("Defend_mode_count : %d",_Defend_mode_count);
}

void MR2_nodelet_main::Cylinder_Operation(std::string CylName, bool state){
    uint8_t Ctrl_solenoidBoard = 0;
    uint8_t Ctrl_solenoidCommand = 0;
    
    if(CylName == "R_Clutch"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_R_Clutch_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_R_Clutch_cmd;
    }else if(CylName == "R_Upper_Rotate"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_R_Upper_Rotate_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_R_Upper_Rotate_cmd;
    }else if(CylName == "R_Upper_Grab"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_R_Upper_Grab_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_R_Upper_Grab_cmd;
    }else if(CylName == "R_Upper_Deploy"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_R_Upper_Deploy_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_R_Upper_Deploy_cmd;
    }else if(CylName == "R_Lower_Grab"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_R_Lower_Grab_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_R_Lower_Grab_cmd;
    }else if(CylName == "R_Lower_Deploy"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_R_Lower_Deploy_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_R_Lower_Deploy_cmd;
    }else if(CylName == "L_Clutch"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_L_Clutch_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_L_Clutch_cmd;
    }else if(CylName == "L_Upper_Rotate"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_L_Upper_Rotate_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_L_Upper_Rotate_cmd;
    }else if(CylName == "L_Upper_Grab"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_L_Upper_Grab_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_L_Upper_Grab_cmd;
    }else if(CylName == "L_Upper_Deploy"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_L_Upper_Deploy_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_L_Upper_Deploy_cmd;
    }else if(CylName == "L_Lower_Grab"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_L_Lower_Grab_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_L_Lower_Grab_cmd;
    }else if(CylName == "L_Lower_Deploy"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_L_Lower_Deploy_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_L_Lower_Deploy_cmd;
    }else if(CylName == "Defend_Grab"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_Defend_Grab_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Defend_Grab_cmd;
    }else if(CylName == "Defend_Rise"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_Defend_Rise_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Defend_Rise_cmd;
    }else if(CylName == "Defend_Press"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_Defend_Press_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Defend_Press_cmd;
    }else if(CylName == "Ball_Grab"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_Ball_Grab_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Ball_Grab_cmd;
    }else if(CylName == "Ball_Gather"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_Ball_Gather_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Ball_Gather_cmd;
    }else if(CylName == "Ball_Rise"){
        Ctrl_solenoidBoard = (uint8_t)SolenoidValveBoards::Cyl_Ball_Rise_board;
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Ball_Rise_cmd;
    }else{
        NODELET_INFO("error control solenoid");
        return;
    }
    if(state){
        this->lastSolenoid_Order[Ctrl_solenoidBoard] |= (uint8_t)Ctrl_solenoidCommand;
        this->solenoid_order_msg[Ctrl_solenoidBoard].data = this->lastSolenoid_Order[Ctrl_solenoidBoard];
        this->Solenoid_Order_pub[Ctrl_solenoidBoard].publish(this->solenoid_order_msg[Ctrl_solenoidBoard]);
    }else{
        this->lastSolenoid_Order[Ctrl_solenoidBoard] &= ~(uint8_t)Ctrl_solenoidCommand;
        this->solenoid_order_msg[Ctrl_solenoidBoard].data = this->lastSolenoid_Order[Ctrl_solenoidBoard];
        this->Solenoid_Order_pub[Ctrl_solenoidBoard].publish(this->solenoid_order_msg[Ctrl_solenoidBoard]);
    }
    //NODELET_INFO("send solenoid order");
}
}
PLUGINLIB_EXPORT_CLASS(MR2::MR2_nodelet_main, nodelet::Nodelet);
