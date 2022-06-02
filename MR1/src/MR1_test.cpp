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
#include <math.h>
#include <cmath>


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
// related to delay
    set_delay_10ms,
    set_delay_100ms,
    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,
    delay,
    wait_next_pressed,

    Cyl_R_load_ON,
    Cyl_R_load_OFF,
    Cyl_L_load_ON,
    Cyl_L_load_OFF,
    Cyl_Launch_ON,
    Cyl_Launch_OFF,
    Lift_mv_Upper,
    Lift_mv_Lower,
    Lift_mv_Lower_start,
    Lift_mv_R_Loadwait,
    Lift_mv_R_Loading,
    Lift_mv_R_Loading_start,
    Lift_mv_L_Loadwait,
    Lift_mv_L_Loading,
    Launcher_SpeedUp,
    Launcher_SpeedDown,

    Launcher_SpeedUp_BreakPos_1,
    Launcher_Move_BreakPos_1,
    Launcher_SpeedUp_BreakPos_2,
    Launcher_Move_BreakPos_2,
};

enum class SolenoidValveCommands : uint8_t
{
    shutdown_cmd      = 0b000000,
    recover_cmd       = 0b000001,
    
    Cyl_Right_load_cmd   = 0b0010000,//default open
    Cyl_Left_load_cmd    = 0b1000000,//default down
    Cyl_Launch_cmd       = 0b0001000,//default release

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
	void MouseCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void KeyPressCallback(const std_msgs::String::ConstPtr& msg);
    void KeyReleaseCallback(const std_msgs::String::ConstPtr& msg);
    void control_timer_callback(const ros::TimerEvent &event);
    void Lift_PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void shutdown();
    void recover();
    void set_delay(double delay_s);
    void delay_start(double delay_s);
    void Lift_move_Vel(double target);
    void Lift_homing(void);
    void Pitch_homing(void);
    void Yaw_homing(void);
    void Cylinder_Operation(std::string CylName, bool state);
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
    ros::Publisher Solenoid_Order_pub;
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
    ros::Publisher lift_CmdPub;
    ros::Publisher lift_VelPub;

	ros::Subscriber Mouse_sub;
	ros::Subscriber Key_press_sub;
	ros::Subscriber Key_release_sub;
	ros::Subscriber Lift_sub;
    /***********************Pub&Sub**************************/

    /***********************Valiables**************************/
    double _delay_s = 0;
    bool _command_ongoing = false;
    bool _is_manual_enabled = true;

    double delay_time = 0.0;
    uint8_t lastSolenoid_Order = 0b0000000;
    double mouse_position_x;
    double mouse_position_y;
    std::string key_press = "";
    std::string key_release = "";

    std_msgs::Float64 launch_VelMsg[3];
    std_msgs::Float64 launch_VelMsg_inverse[3];
    std_msgs::Float64 pitch_PosMsg;
    std_msgs::Float64 yaw_PosMsg;
    std_msgs::Float64 lift_VelMsg;

    double pitch_deg = 0.0;
    double yaw_deg = 0.0;

    bool _enableAim_fromJoy = false;
    bool _enableAim_fromKey = false;

    double displaysize_x;
    double displaysize_y;

    double Lift_position = 0.0;
    double lift_upper_pos;
    double lift_lower_pos;
    double lift_r_loadwait_pos;
    double lift_r_loading_pos;
    double lift_l_loadwait_pos;
    double lift_l_loading_pos;
    double launcher_first_speed;
    bool _lift_mv_lower_start = false;
    bool _lift_mv_r_loading_start = false;
    bool _enable_homing = false;
    bool _enable_autoMove = false;
    bool _enable_autoAim = false;

    double LagoriBrake_pos_1_yaw;
    double LagoriBrake_pos_1_pitch;
    double LagoriBrake_pos_1_speed;
    double LagoriBrake_pos_2_yaw;
    double LagoriBrake_pos_2_pitch;
    double LagoriBrake_pos_2_speed;

    double Aim_maxSpeed;
    double Aim_minSpeed;
    double Aim_maxPitch;
    double Aim_minPitch;
    double Aim_maxYaw_abs;
    double Aim_horizontal_range;
    /***********************Valiables**************************/

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
    static const std::vector<ControllerCommands> AutoLoading_commands;
    static const std::vector<ControllerCommands> Right_Load_commands;
    static const std::vector<ControllerCommands> Left_Load_commands;
    static const std::vector<ControllerCommands> Shot_commands;
    static const std::vector<ControllerCommands> LaunchSpeedUp_commands;
    static const std::vector<ControllerCommands> LaunchSpeedDown_commands;
    static const std::vector<ControllerCommands> LiftUp_commands;
    static const std::vector<ControllerCommands> LiftDown_commands;
    static const std::vector<ControllerCommands> LiftMove_R_Loadwait_commands;
    static const std::vector<ControllerCommands> LiftMove_R_Loading_commands;
    static const std::vector<ControllerCommands> LiftMove_L_Loadwait_commands;
    static const std::vector<ControllerCommands> LiftMove_L_Loading_commands;
    static const std::vector<ControllerCommands> AutoLagoriBreak_commands;

    static const std::vector<ControllerCommands> SetLaunchPosi_commands;
    static const std::vector<ControllerCommands> manual_all;
    const std::vector<ControllerCommands> *command_list;
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

const std::vector<ControllerCommands> MR1_nodelet_main::Right_Load_commands({
    ControllerCommands::Cyl_R_load_OFF,
    ControllerCommands::set_delay_1s,
    ControllerCommands::delay,
    ControllerCommands::Cyl_R_load_ON,
});
const std::vector<ControllerCommands> MR1_nodelet_main::Left_Load_commands({
    ControllerCommands::Cyl_L_load_OFF,
    ControllerCommands::set_delay_1s,
    ControllerCommands::delay,
    ControllerCommands::Cyl_L_load_ON,
});
const std::vector<ControllerCommands> MR1_nodelet_main::Shot_commands({
    ControllerCommands::Cyl_Launch_ON,
    ControllerCommands::set_delay_1s,
    ControllerCommands::delay,
    ControllerCommands::Cyl_Launch_OFF,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LaunchSpeedUp_commands({
    ControllerCommands::Launcher_SpeedUp,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LaunchSpeedDown_commands({
    ControllerCommands::Launcher_SpeedDown,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LiftUp_commands({
    ControllerCommands::Lift_mv_Upper,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LiftDown_commands({
    ControllerCommands::Lift_mv_Lower,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LiftMove_R_Loadwait_commands({
    ControllerCommands::Lift_mv_R_Loadwait,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LiftMove_R_Loading_commands({
    ControllerCommands::Lift_mv_R_Loading,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LiftMove_L_Loadwait_commands({
    ControllerCommands::Lift_mv_L_Loadwait,
});
const std::vector<ControllerCommands> MR1_nodelet_main::LiftMove_L_Loading_commands({
    ControllerCommands::Lift_mv_L_Loading,
});
const std::vector<ControllerCommands> MR1_nodelet_main::AutoLoading_commands({
    ControllerCommands::Cyl_Launch_OFF,
    ControllerCommands::Cyl_L_load_ON,
    ControllerCommands::Cyl_R_load_ON,
    ControllerCommands::Lift_mv_Upper,
    ControllerCommands::set_delay_1s,
    ControllerCommands::delay,
    //ControllerCommands::set_delay_500ms,
    //ControllerCommands::delay,
    ControllerCommands::Cyl_L_load_OFF,
    ControllerCommands::Cyl_R_load_OFF,
    ControllerCommands::Lift_mv_Lower_start,
    ControllerCommands::set_delay_500ms,
    ControllerCommands::delay,
    ControllerCommands::Cyl_L_load_ON,
    ControllerCommands::Cyl_R_load_ON,
});
const std::vector<ControllerCommands> MR1_nodelet_main::AutoLagoriBreak_commands({
    ControllerCommands::Launcher_Move_BreakPos_1,
    ControllerCommands::Launcher_SpeedUp_BreakPos_1,
    ControllerCommands::set_delay_100ms,
    ControllerCommands::delay,
    ControllerCommands::Cyl_Launch_ON,

    ControllerCommands::Lift_mv_R_Loading_start,

    ControllerCommands::set_delay_250ms,
    ControllerCommands::delay,
    ControllerCommands::Launcher_Move_BreakPos_2,
    ControllerCommands::set_delay_100ms,
    ControllerCommands::delay,
    ControllerCommands::Cyl_Launch_OFF,
    ControllerCommands::Launcher_SpeedUp_BreakPos_2,
    ControllerCommands::set_delay_500ms,
    ControllerCommands::delay,
    //ControllerCommands::set_delay_100ms,
    //ControllerCommands::delay,
    ControllerCommands::set_delay_250ms,
    ControllerCommands::delay,
    ControllerCommands::Cyl_Launch_ON,
    ControllerCommands::set_delay_250ms,
    ControllerCommands::delay,
    ControllerCommands::set_delay_250ms,
    ControllerCommands::delay,
    ControllerCommands::Cyl_Launch_OFF,
});

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

    this->command_list = &MR1_nodelet_main::manual_all;

	/*******************pub & sub*****************/
    this->joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &MR1_nodelet_main::joyCallback, this);
    this->cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	this->SolenoidCmd_pub = nh.advertise<std_msgs::UInt8>("solenoid_cmd", 1);
    this->Solenoid_Order_pub = nh.advertise<std_msgs::UInt8>("solenoid_order", 1);
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
    this->lift_CmdPub = nh.advertise<std_msgs::UInt8>("lift_cmd", 1);
    this->lift_VelPub = nh.advertise<std_msgs::Float64>("lift_vel", 1);

    this->Mouse_sub = nh.subscribe<geometry_msgs::Twist>("/mouse_vel", 10, &MR1_nodelet_main::MouseCallback, this);
    this->Key_press_sub = nh.subscribe<std_msgs::String>("/keypress", 10, &MR1_nodelet_main::KeyPressCallback, this);
    this->Key_release_sub = nh.subscribe<std_msgs::String>("/keyrelease", 10, &MR1_nodelet_main::KeyReleaseCallback, this);
    this->Lift_sub = nh_MT.subscribe<std_msgs::Float32>("motor6_current_val", 10, &MR1_nodelet_main::Lift_PosCallback, this);

	/*******************pub & sub*****************/

	/*******************parameter*****************/
    _nh.param("displaysize_x", displaysize_x, 0.0);
    _nh.param("displaysize_y", displaysize_y, 0.0);
    _nh.param("lift_upper_pos", lift_upper_pos, 0.0);
    _nh.param("lift_lower_pos", lift_lower_pos, 0.0);
    _nh.param("lift_r_loadwait_pos", lift_r_loadwait_pos, 0.0);
    _nh.param("lift_r_loading_pos", lift_r_loading_pos, 0.0);
    _nh.param("lift_l_loadwait_pos", lift_l_loadwait_pos, 0.0);
    _nh.param("lift_l_loading_pos", lift_l_loading_pos, 0.0);
    _nh.param("launcher_first_speed", launcher_first_speed, 0.0);
    _nh.param("LagoriBrake_pos_1_yaw", LagoriBrake_pos_1_yaw, 0.0);
    _nh.param("LagoriBrake_pos_1_pitch", LagoriBrake_pos_1_pitch, 0.0);
    _nh.param("LagoriBrake_pos_1_speed", LagoriBrake_pos_1_speed, 0.0);
    _nh.param("LagoriBrake_pos_2_yaw", LagoriBrake_pos_2_yaw, 0.0); 
    _nh.param("LagoriBrake_pos_2_pitch", LagoriBrake_pos_2_pitch, 0.0);
    _nh.param("LagoriBrake_pos_2_speed", LagoriBrake_pos_2_speed, 0.0);

    _nh.param("Aim_maxSpeed", Aim_maxSpeed, 0.0);
    _nh.param("Aim_minSpeed", Aim_minSpeed, 0.0);
    _nh.param("Aim_maxPitch", Aim_maxPitch, 0.0);
    _nh.param("Aim_minPitch", Aim_minPitch, 0.0);
    _nh.param("Aim_maxYaw_abs", Aim_maxYaw_abs, 0.0);
    _nh.param("Aim_horizontal_range", Aim_horizontal_range, 0.0);
	/*******************parameter*****************/

    launch_VelMsg[0].data = 0.0;
    launch_VelMsg[1].data = 0.0;
    launch_VelMsg[2].data = 0.0;
    yaw_PosMsg.data = 0.0;
    pitch_PosMsg.data = 0.0;

    this->control_timer = nh.createTimer(ros::Duration(0.01), &MR1_nodelet_main::control_timer_callback, this);
    NODELET_INFO("MR1 node has started.");
    
}

/**************************************************************************************/
void MR1_nodelet_main::MouseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    static int _debug_count = 0;
	this->mouse_position_x = msg->linear.x;
	this->mouse_position_y = msg->linear.y;

    double offset_position_x;
    double offset_position_y;
    double distance_to_target;
    double cal_degree;
    double cal_pitch;
    double cal_yaw;
    double cal_speed;
    double max_distance = hypot(displaysize_x/2.0,displaysize_y);

    //NODELET_INFO("x : %f, y : %f", this->mouse_position_x, this->mouse_position_y);

    _debug_count ++;
    if(_debug_count >= 100){
        NODELET_INFO("x : %f, y : %f", this->mouse_position_x, this->mouse_position_y);
        _debug_count = 0;
    }

    if(!_enable_homing && !_enable_autoMove && !_enable_autoAim){
        if(_enableAim_fromKey){
            yaw_PosMsg.data = (this->mouse_position_x-(displaysize_x/2.0)) / 1000.0;
            pitch_PosMsg.data = (this->mouse_position_y-(displaysize_y/2.0)) / 1200.0;;
            pitch_PosPub.publish(this->pitch_PosMsg);
            yaw_PosPub.publish(this->yaw_PosMsg);
        }else{
            yaw_PosMsg.data = 0.0;
            pitch_PosMsg.data = 0.0;
            pitch_PosPub.publish(this->pitch_PosMsg);
            yaw_PosPub.publish(this->yaw_PosMsg);
        }
    }else if(_enable_autoAim && !_enable_homing && !_enable_autoMove){
        offset_position_x = (this->mouse_position_x-(displaysize_x/2.0)); //*0.857
        offset_position_y = displaysize_y - this->mouse_position_y;
        offset_position_y =  pow((offset_position_y/displaysize_y),1.2/2.0) * displaysize_y;
        distance_to_target = hypot(offset_position_x,offset_position_y);
        cal_degree = -(atan2(offset_position_y,offset_position_x) - pi/2.0);
        if(distance_to_target <= Aim_horizontal_range){
            pitch_PosMsg.data = Aim_minPitch;
            if(fabs(cal_degree) <= Aim_maxYaw_abs){
                yaw_PosMsg.data = cal_degree;
            }else{
                if(cal_degree < 0){
                    yaw_PosMsg.data = -Aim_maxYaw_abs;
                }else{
                    yaw_PosMsg.data = Aim_maxYaw_abs;
                }
            }
            this->launch_VelMsg[0].data = Aim_minSpeed;
            this->launch_VelMsg[1].data = Aim_minSpeed;
            this->launch_VelMsg[2].data = Aim_minSpeed;
        }else{
            cal_pitch = ((distance_to_target-Aim_horizontal_range)/(max_distance-Aim_horizontal_range))*(Aim_maxPitch-Aim_minPitch) + Aim_minPitch;
            cal_speed = ((distance_to_target-Aim_horizontal_range)/(max_distance-Aim_horizontal_range))*(Aim_maxSpeed-Aim_minSpeed) + Aim_minSpeed;
            cal_speed = pow(cal_speed/Aim_maxSpeed,1.2/2.0) * Aim_maxSpeed;
            cal_yaw = -(atan2(offset_position_y/cos(cal_pitch),offset_position_x) - pi/2.0);
            //motometa_X * (cos(pitch)) = imaaru_X
            //motometa_X = imaaru_X / cos(pitch); cos(pitch) != 0
            if(fabs(cal_yaw) > Aim_maxYaw_abs){
                if(cal_yaw < 0){
                    cal_yaw = -Aim_maxYaw_abs;
                }else{
                    cal_yaw = Aim_maxYaw_abs;
                }
            }
            pitch_PosMsg.data = cal_pitch;
            yaw_PosMsg.data = cal_yaw;
            this->launch_VelMsg[0].data = cal_speed;
            this->launch_VelMsg[1].data = cal_speed;
            this->launch_VelMsg[2].data = cal_speed;
        }
        this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
        launch1_VelPub.publish(this->launch_VelMsg[0]);
        launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
        launch3_VelPub.publish(this->launch_VelMsg[2]);

        pitch_PosPub.publish(this->pitch_PosMsg);
        yaw_PosPub.publish(this->yaw_PosMsg);

        NODELET_INFO("pitch:%2.3f, yaw:%2.3f, speed:%5.1f", this->pitch_PosMsg.data, this->yaw_PosMsg.data, this->launch_VelMsg[0].data);
    }
}
void MR1_nodelet_main::KeyReleaseCallback(const std_msgs::String::ConstPtr& msg){
    this->key_release = msg->data;
    if(this->key_release == "u" || this->key_release == "j" ){
        Lift_move_Vel(0);
    }
    NODELET_INFO("keyrelease : %s", this->key_release.c_str());
}
void MR1_nodelet_main::KeyPressCallback(const std_msgs::String::ConstPtr& msg)
{
    static bool _Cyl_Right = false;
    static bool _Cyl_Left = false;
	this->key_press = msg->data;
    //if(this->key_press == "a"){
    //    this->_enableAim_fromKey = true;
    //}else if(this->key_press == "s"){
    //    this->_enableAim_fromKey = false;
    //}else 
    if(this->key_press == "Right"){
        _enable_homing = true;
        yaw_PosMsg.data += 0.0125;
        yaw_PosPub.publish(this->yaw_PosMsg);
    }else if(this->key_press == "Left"){
        _enable_homing = true;
        yaw_PosMsg.data -= 0.0125;
        yaw_PosPub.publish(this->yaw_PosMsg);
    }else if(this->key_press == "Up"){
        _enable_homing = true;
        pitch_PosMsg.data -= 0.0125;
        pitch_PosPub.publish(this->pitch_PosMsg);
    }else if(this->key_press == "Down"){
        _enable_homing = true;
        pitch_PosMsg.data += 0.0125;
        pitch_PosPub.publish(this->pitch_PosMsg);
    }

    if(this->key_press == "p"){
        this->recover();
    }else if(this->key_press == "o"){
        this->shutdown();
    }else if(this->key_press == "w"){
        if(!_command_ongoing){
            this->command_list = &Shot_commands;
            _command_ongoing = true;   
        }
    }else if(this->key_press == "a"){
        if(!_command_ongoing){
            //this->command_list = &Left_Load_commands;
            this->command_list = &LiftMove_L_Loadwait_commands;
            _command_ongoing = true;      
        }
    }else if(this->key_press == "d"){
        if(!_command_ongoing){
            //this->command_list = &Right_Load_commands;
            this->command_list = &LiftMove_R_Loadwait_commands;
            _command_ongoing = true;   
        }
    }else if(this->key_press == "t"){
        if(!_lift_mv_lower_start && !_lift_mv_r_loading_start && !_command_ongoing){
            this->command_list = &LiftUp_commands;
            _command_ongoing = true;
        }   
    }else if(this->key_press == "g"){
        if(!_lift_mv_lower_start && !_lift_mv_r_loading_start && !_command_ongoing){
            this->command_list = &LiftDown_commands;
            _command_ongoing = true;
        }   
    }else if(this->key_press == "r"){
        if(!_command_ongoing){
            this->command_list = &LaunchSpeedUp_commands;
            _command_ongoing = true;
        }   
    }else if(this->key_press == "f"){
        if(!_command_ongoing){
            this->command_list = &LaunchSpeedDown_commands;
            _command_ongoing = true;   
        }
        _enable_autoAim = false;
    }else if(this->key_press == "z"){
        //if(!_lift_mv_lower_start && !_lift_mv_r_loading_start && !_command_ongoing){
        //    this->command_list = &AutoLoading_commands;
        //    _command_ongoing = true;   
        //}
        if(_enable_autoAim){
            _enable_autoAim = false;
        }else if(!_enable_homing && !_enable_autoMove){
            _enable_autoAim = true;
        }
    }else if(this->key_press == "v"){
        if(!_command_ongoing){
            this->command_list = &AutoLagoriBreak_commands;
            _command_ongoing = true;
        }   
    }else if(this->key_press == "h"){
        Lift_homing();
        Pitch_homing();
        Yaw_homing();
        yaw_PosMsg.data = 0.0;
        pitch_PosMsg.data = 0.0;
        _enable_homing = false;
    }else if(this->key_press == "m"){
        this->_enableAim_fromKey = true;
        _enable_homing = false;
        _enable_autoMove = false;
    }else if(this->key_press == "n"){
        this->_enableAim_fromKey = false;
    }else if(this->key_press == "1"){
        this->launch_VelMsg[0].data += 100.0;
        this->launch_VelMsg[1].data += 100.0;
        this->launch_VelMsg[2].data += 100.0;
    }else if(this->key_press == "2"){
        this->launch_VelMsg[0].data -= 100.0;
        this->launch_VelMsg[1].data -= 100.0;
        this->launch_VelMsg[2].data -= 100.0;
    }else if(this->key_press == "u"){
        Lift_move_Vel(-10);
    }else if(this->key_press == "j"){
        Lift_move_Vel(10);
    }else if(this->key_press == "q"){
        if(!_command_ongoing){
            this->command_list = &LiftMove_L_Loading_commands;
            _command_ongoing = true;      
        }
    }else if(this->key_press == "e"){
       if(!_command_ongoing){
            this->command_list = &LiftMove_R_Loading_commands;
            _command_ongoing = true;      
        }
    }

    if(!_enable_autoAim){
        this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
        launch1_VelPub.publish(this->launch_VelMsg[0]);
        launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
        launch3_VelPub.publish(this->launch_VelMsg[2]);
    }

    NODELET_INFO("keypress : %s", this->key_press.c_str());
}
void MR1_nodelet_main::Lift_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->Lift_position = msg->data;
    //NODELET_INFO("Lift_position : %f", this->Lift_position);
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
    lift_CmdPub.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
}

void MR1_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_velocity;
    launch1_CmdPub.publish(act_conf_cmd_msg);
    launch2_CmdPub.publish(act_conf_cmd_msg);
    launch3_CmdPub.publish(act_conf_cmd_msg);
    lift_CmdPub.publish(act_conf_cmd_msg);
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

    //NODELET_INFO("%d",joy->buttons[ButtonA]);
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
    static bool _Cyl_catch = false;
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

    if(_pady == 1){
        Lift_move_Vel(6);
    }else if(_pady == -1){
        Lift_move_Vel(-6);
    }else{
        Lift_move_Vel(0);
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
    if(_padx == 1){
        yaw_deg += 0.5;
    }else if(_padx == -1){
        yaw_deg -= 0.5;
    }
    if(_pady == 1){
        pitch_deg += 0.5;
    }else if(_pady == -1){
        pitch_deg -= 0.5;
    }

    this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
    //launch1_VelPub.publish(this->launch_VelMsg[0]);
    //launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
    //launch3_VelPub.publish(this->launch_VelMsg[2]);

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
        //this->cmd_vel_pub.publish(this->cmd_vel_msg);

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
    if(_lift_mv_lower_start){
        if(Lift_position >= lift_lower_pos-0.5){
            Lift_move_Vel(0.0);
            NODELET_INFO("Lift Move to Lower end");
            _lift_mv_lower_start = false;
        }else if(Lift_position >= lift_lower_pos-10.0){
            Lift_move_Vel(8.0);
        }else if(Lift_position >= lift_lower_pos-20.0){
            Lift_move_Vel(15.0);
        }else if(Lift_position >= lift_lower_pos-30.0){
            Lift_move_Vel(30.0);
        }else{
            Lift_move_Vel(40.0);
        }
    }else if(_lift_mv_r_loading_start){
        if(Lift_position <= lift_r_loading_pos+0.5){
            Lift_move_Vel(0.0);
            NODELET_INFO("Lift Move to R Loading end");
            _lift_mv_r_loading_start = false;
        }else if(Lift_position <= lift_lower_pos+10.0){
            Lift_move_Vel(-8.0);
        }else if(Lift_position <= lift_lower_pos+20.0){
            Lift_move_Vel(-15.0);
        }else if(Lift_position <= lift_lower_pos+30.0){
            Lift_move_Vel(-30.0);
        }else{
            Lift_move_Vel(-40.0);
        }
    }

    if(currentCommand == ControllerCommands::Cyl_R_load_ON){
        Cylinder_Operation("Right_load",true);
        this->currentCommandIndex++;
        NODELET_INFO("R_Load_ON");
    }else if(currentCommand == ControllerCommands::Cyl_R_load_OFF){
        Cylinder_Operation("Right_load",false);
        this->currentCommandIndex++;
        NODELET_INFO("R_Load_OFF");
    }else if(currentCommand == ControllerCommands::Cyl_L_load_ON){
        Cylinder_Operation("Left_load",true);
        this->currentCommandIndex++;
        NODELET_INFO("L_Load_ON");
    }else if(currentCommand == ControllerCommands::Cyl_L_load_OFF){
        Cylinder_Operation("Left_load",false);
        this->currentCommandIndex++;
        NODELET_INFO("L_Load_OFF");
    }else if(currentCommand == ControllerCommands::Cyl_Launch_ON){
        Cylinder_Operation("Launch",true);
        this->currentCommandIndex++;
        NODELET_INFO("Launch_ON");
    }else if(currentCommand == ControllerCommands::Cyl_Launch_OFF){
        Cylinder_Operation("Launch",false);
        this->currentCommandIndex++;
        NODELET_INFO("Launch_OFF");
    }else if(currentCommand == ControllerCommands::Lift_mv_Upper){
        if(Lift_position <= lift_upper_pos+1.0){
            Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Lift Move to Upper");
        }else if(Lift_position <= lift_upper_pos+10.0){
            Lift_move_Vel(-8.0);
        }else if(Lift_position <= lift_upper_pos+20.0){
            Lift_move_Vel(-15.0);
        }else if(Lift_position <= lift_upper_pos+30.0){
            Lift_move_Vel(-30.0);
        }else{
            Lift_move_Vel(-40.0);
        }
    }else if(currentCommand == ControllerCommands::Lift_mv_Lower){
        if(Lift_position >= lift_lower_pos-0.5){
            Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Lift Move to Lower");
        }else if(Lift_position >= lift_lower_pos-10.0){
            Lift_move_Vel(8.0);
        }else if(Lift_position >= lift_lower_pos-20.0){
            Lift_move_Vel(15.0);
        }else if(Lift_position >= lift_lower_pos-30.0){
            Lift_move_Vel(30.0);
        }else{
            Lift_move_Vel(40.0);
        }
    }else if(currentCommand == ControllerCommands::Lift_mv_Lower_start){
        if(Lift_position >= lift_lower_pos-1.0){
            Lift_move_Vel(0.0);
            _lift_mv_lower_start = false;
        }else{
            _lift_mv_lower_start = true;
            //Lift_move_Vel(15.0);
        }
        this->currentCommandIndex++;
        NODELET_INFO("Lift Move to Lower start");
    }else if(currentCommand == ControllerCommands::Lift_mv_R_Loading_start){
        if(Lift_position <= lift_r_loading_pos+1.0){
            Lift_move_Vel(0.0);
            _lift_mv_r_loading_start = false;
        }else{
            _lift_mv_r_loading_start = true;
            //Lift_move_Vel(15.0);
        }
        this->currentCommandIndex++;
        NODELET_INFO("Lift Move to R Loading start");
    }else if(currentCommand == ControllerCommands::Lift_mv_R_Loadwait || currentCommand == ControllerCommands::Lift_mv_R_Loading || currentCommand == ControllerCommands::Lift_mv_L_Loadwait || currentCommand == ControllerCommands::Lift_mv_L_Loading){
        double diff = 0.0;
        if(currentCommand == ControllerCommands::Lift_mv_R_Loadwait){
            diff = Lift_position - lift_r_loadwait_pos;
        }else if(currentCommand == ControllerCommands::Lift_mv_R_Loading){
            diff = Lift_position - lift_r_loading_pos;
        }else if(currentCommand == ControllerCommands::Lift_mv_L_Loadwait){
            diff = Lift_position - lift_l_loadwait_pos;
        }else if(currentCommand == ControllerCommands::Lift_mv_L_Loading){
            diff = Lift_position - lift_l_loading_pos;
        }  
        double diff_abs = fabs(diff);
        if(diff_abs <= 0.2){
            Lift_move_Vel(0.0);
            this->currentCommandIndex++;
            NODELET_INFO("Lift Move");
        }else if(diff > 0.0){ // lift is positioning at under pos
            if(diff_abs < 10.0){
                Lift_move_Vel(-20.0);
            }else if(diff_abs < 20.0){
                Lift_move_Vel(-30.0);
            }else if(diff_abs < 30.0){
                Lift_move_Vel(-40.0);
            }else{
                Lift_move_Vel(-60.0);
            }
        }else if(diff < 0.0){ // lift is positioning at upper pos
            if(diff_abs < 10.0){
                Lift_move_Vel(20.0);
            }else if(diff_abs < 20.0){
                Lift_move_Vel(30.0);
            }else if(diff_abs < 30.0){
                Lift_move_Vel(40.0);
            }else{
                Lift_move_Vel(60.0);
            }
        }else{
            Lift_move_Vel(0.0);
        }
    }else if(currentCommand == ControllerCommands::Launcher_SpeedUp){
        if(this->launch_VelMsg[0].data < launcher_first_speed){
            this->launch_VelMsg[0].data += 10.0;
            this->launch_VelMsg[1].data += 10.0;
            this->launch_VelMsg[2].data += 10.0;

            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
        }else{
            this->currentCommandIndex++;
            NODELET_INFO("SpeedUp");
        }
    }else if(currentCommand == ControllerCommands::Launcher_SpeedDown){
        if(this->launch_VelMsg[0].data > 0.0){
            this->launch_VelMsg[0].data -= 10.0;
            this->launch_VelMsg[1].data -= 10.0;
            this->launch_VelMsg[2].data -= 10.0;

            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
        }else{
            this->launch_VelMsg[0].data = 0.0;
            this->launch_VelMsg[1].data = 0.0;
            this->launch_VelMsg[2].data = 0.0;

            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
            this->currentCommandIndex++;
            NODELET_INFO("SpeedDown");
        }
    }
    else if(currentCommand == ControllerCommands::Launcher_SpeedUp_BreakPos_1){
        if(this->launch_VelMsg[0].data < LagoriBrake_pos_1_speed - 0.1){
            this->launch_VelMsg[0].data += 20.0;
            this->launch_VelMsg[1].data += 20.0;
            this->launch_VelMsg[2].data += 20.0;
            
            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
        }else if(this->launch_VelMsg[0].data > LagoriBrake_pos_1_speed + 0.1){
            this->launch_VelMsg[0].data -= 20.0;
            this->launch_VelMsg[1].data -= 20.0;
            this->launch_VelMsg[2].data -= 20.0;

            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
        }else{
            this->currentCommandIndex++;
            NODELET_INFO("SpeedUp");
        }
    }
    else if(currentCommand == ControllerCommands::Launcher_SpeedUp_BreakPos_2){
        if(this->launch_VelMsg[0].data < LagoriBrake_pos_2_speed - 0.1){
            this->launch_VelMsg[0].data += 20.0;
            this->launch_VelMsg[1].data += 20.0;
            this->launch_VelMsg[2].data += 20.0;

            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
        }else if(this->launch_VelMsg[0].data > LagoriBrake_pos_2_speed + 0.1){
            this->launch_VelMsg[0].data -= 20.0;
            this->launch_VelMsg[1].data -= 20.0;
            this->launch_VelMsg[2].data -= 20.0;

            this->launch_VelMsg_inverse[1].data = - this->launch_VelMsg[1].data;
            launch1_VelPub.publish(this->launch_VelMsg[0]);
            launch2_VelPub.publish(this->launch_VelMsg_inverse[1]);
            launch3_VelPub.publish(this->launch_VelMsg[2]);
        }else{
            this->currentCommandIndex++;
            NODELET_INFO("SpeedUp");
        }
    }
    else if(currentCommand == ControllerCommands::Launcher_Move_BreakPos_1){
        _enable_autoMove = true;
        yaw_PosMsg.data = LagoriBrake_pos_1_yaw;
        pitch_PosMsg.data = LagoriBrake_pos_1_pitch;
        pitch_PosPub.publish(this->pitch_PosMsg);
        yaw_PosPub.publish(this->yaw_PosMsg);
        this->currentCommandIndex++;
        NODELET_INFO("Move Launcher to Break Pos 1");
    }
    else if(currentCommand == ControllerCommands::Launcher_Move_BreakPos_2){
        _enable_autoMove = true;
        yaw_PosMsg.data = LagoriBrake_pos_2_yaw;
        pitch_PosMsg.data = LagoriBrake_pos_2_pitch;
        pitch_PosPub.publish(this->pitch_PosMsg);
        yaw_PosPub.publish(this->yaw_PosMsg);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::set_delay_10ms)
    {
        //set_delay(0.010);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_250ms");
    }
    else if (currentCommand == ControllerCommands::set_delay_100ms)
    {
        set_delay(0.100);
        this->currentCommandIndex++;
        NODELET_INFO("set_delay_100ms");
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

void MR1_nodelet_main::Lift_move_Vel(double target){
    //act_conf_cmd_msg2.data = (uint8_t)MotorCommands::recover_velocity;
    //Defence_Roll_Cmd_pub.publish(act_conf_cmd_msg2);
    this->lift_VelMsg.data = target;
    this->lift_VelPub.publish(this->lift_VelMsg);
}

void MR1_nodelet_main::Lift_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->lift_CmdPub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->lift_CmdPub.publish(act_conf_cmd_msg);
}
void MR1_nodelet_main::Pitch_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->pitch_CmdPub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->pitch_CmdPub.publish(act_conf_cmd_msg);
}
void MR1_nodelet_main::Yaw_homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    this->yaw_CmdPub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    this->yaw_CmdPub.publish(act_conf_cmd_msg);
}


void MR1_nodelet_main::Cylinder_Operation(std::string CylName, bool state){
    uint8_t Ctrl_solenoidCommand = 0;
    
    if(CylName == "Right_load"){
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Right_load_cmd;
    }else if(CylName == "Left_load"){
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Left_load_cmd;
    }else if(CylName == "Launch"){
        Ctrl_solenoidCommand = (uint8_t)SolenoidValveCommands::Cyl_Launch_cmd;
    }else{
        NODELET_INFO("error control solenoid");
        return;
    }
    if(state){
        this->lastSolenoid_Order |= (uint8_t)Ctrl_solenoidCommand;
        this->solenoid_order_msg.data = this->lastSolenoid_Order;
        this->Solenoid_Order_pub.publish(this->solenoid_order_msg);
    }else{
        this->lastSolenoid_Order &= ~(uint8_t)Ctrl_solenoidCommand;
        this->solenoid_order_msg.data = this->lastSolenoid_Order;
        this->Solenoid_Order_pub.publish(this->solenoid_order_msg);
    }
    NODELET_INFO("send solenoid order");
}

}
PLUGINLIB_EXPORT_CLASS(MR1::MR1_nodelet_main, nodelet::Nodelet);
