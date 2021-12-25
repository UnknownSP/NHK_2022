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

namespace tr{
enum class ControllerCommands : uint16_t
{
    shutdown, // shutdown
    standby,  
    manual,
    home,
    clear_flag,
// collect the arrow 
    adjust_arm,
    adjust_arm_to_grab,
    adjust_arm_to_set,
    grab_arrow,
    release_arrow,
    table_move_to_launcher,
    table_move_to_base,
    table_rotate_to_launcher,
    table_rotate_to_base,
// launch the arrow
    launcher_move_top,
    launcher_move_bottom,
    launcher_reset,
    adjust_launchers_force_short,
    adjust_launchers_force_medium,
    adjust_launchers_force_long,
    adjust_launchers_angular_right,
    adjust_launchers_angular_short,
    adjust_launchers_angular_medium,
    adjust_launchers_angular_long,
    grab_shooter,
    release_shooter,
// related to delay
    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,
    delay,
    wait_next_pressed,
    //-------------------------------------------
    shooter_release,
    shooter_grab,
    shooter_init,
    shooter_move_load,
    angle_init,

    riseflag_shooter_pos_load,
    //-------------------------------------------
};

enum class OpMode : uint8_t
{
    def,         
    full_op,   
};

//Value passed to solenoid valve board
//Please refer to https://github.com/chibarobotstuidonhk/solenoid_drive_f103
enum class SolenoidValveCommands : uint8_t
{
    shutdown_cmd    = 0b000000,
    recover_cmd     = 0b000001,
    
    shooter_cmd     = 0b0000010,//default close
    R_hand_1_cmd    = 0b0010000,//default open
    L_hand_1_cmd    = 0b1000000,//default open
    move_base_cmd   = 0b0100000,//default right
    
    R_hand_roll_cmd = 0b0000001,//default open
    L_hand_roll_cmd = 0b0000100,//default open
    R_hand_2_cmd    = 0b0000010,//default open
    L_hand_2_cmd    = 0b0100000,//default open
    
};

enum class MotorCommands : uint8_t
{
    shutdown_cmd      = 0x00,
    recover_cmd       = 0x01,
    
    homing_cmd        = 0x10,
    
};


class tr_nodelet_main : public nodelet::Nodelet
  {
  public:
    virtual void onInit();


  private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void control_timer_callback(const ros::TimerEvent &event);



    //double LaunchMaximumradian;
    //double LaunchMinimumradian;
    //double Launchradian;

    //double LaunchInitiallength;
    //double BallScrewLead;
    //double PropLength;
    //double LaunchPartLength;
    //double toothperrotate;
    //double lengthperbit;
    double launcherlength;
    
    //double LaunchMaximumPullDistance;
    //double LaunchPullDistance;
    

    //double ArmRollRadian;
    //double ArmMaxHeight;
    //double ArmHeight;
    //double ArmMaximumRadian;
    //double ArmRadian;
    

    ros::NodeHandle nh;
  	ros::NodeHandle _nh;

    ros::Subscriber joy_sub;

    
    std_msgs::UInt8 act_conf_cmd_msg;

    ros::Publisher Shot_Power_Cmd_pub;
    ros::Publisher Shot_Power_Pos_pub;
    std_msgs::Float64 shot_power_pos_msg;

    ros::Publisher Shot_Angle_Cmd_pub;
  	ros::Publisher Shot_Angle_Pos_pub;
    std_msgs::Float64 shot_angle_pos_msg;
    
    ros::Publisher Pick_R_Base_Cmd_pub;
  	ros::Publisher Pick_R_Base_Pos_pub;
    std_msgs::Float64 Pick_R_base_pos_msg;

    ros::Publisher Pick_R_Height_Cmd_pub;
    ros::Publisher Pick_R_Height_Pos_pub;
    std_msgs::Float64 Pick_R_height_pos_msg;

    ros::Publisher Pick_L_Base_Cmd_pub;
  	ros::Publisher Pick_L_Base_Pos_pub;
    std_msgs::Float64 Pick_L_base_pos_msg;

    ros::Publisher Pick_L_Height_Cmd_pub;
    ros::Publisher Pick_L_Height_Pos_pub;
    std_msgs::Float64 Pick_L_height_pos_msg;

    ros::Publisher Solenoid1_Cmd_pub;
    ros::Publisher Solenoid1_Order_pub;
    std_msgs::UInt8 solenoid1_order_msg;
    uint8_t lastSolenoid_1_Order = 0b0000000;

    ros::Publisher Solenoid2_Cmd_pub;
    ros::Publisher Solenoid2_Order_pub;
    std_msgs::UInt8 solenoid2_order_msg;
    uint8_t lastSolenoid_2_Order = 0b0000000;

    ros::Publisher act_enable_pub0;
    ros::Publisher act_enable_pub1;
    ros::Publisher act_enable_pub2;
    ros::Publisher act_enable_pub3;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;

    ros::Timer control_timer;

    //action_modules
    void release_shooter();
    void grab_shooter();
    void reset_launcher();
    
    void launcher_move(float movelength);
    void adjust_launcher_force(float pulllength);
    void adjust_launcher_radian(float LaunchAngle);
    void reset_launcher_status();
    void grab_arrow();
    void release_arrow();

    void base_rotate(float rotateangle);
    void table_move_to_launcher();
    void table_move_to_base();
    void launch_ready();
    void adjust_arm(float movelength);
    void adjust_arm_to_grab();
    void adjust_arm_to_lift();
    void adjust_arm_to_set();

    void next_OpMode();
    void back_OpMode();

    int OpModecurrentindex = -1;
    OpMode current_OpMode;

    int _delay_s = 0;

    void shutdown();
    void recover();
    void homing();
    void clear_flags();
    void set_delay(double delay_s);

    void change_OpMode();

    //----------------------------------------
    void delay_start(double delay_s);
    double delay_time = 0.0;

    void Shot_Power_Homing();
    void Shot_Angle_Homing();
    void Pick_R_Base_Homing();
    void Pick_R_Height_Homing();
    void Pick_L_Base_Homing();
    void Pick_L_Height_Homing();

    void R_base_move_pick_standby_1_arrow();
    void R_base_move_pick_1_arrow();
    void R_base_move_pick_standby_2_arrow();
    void R_base_move_pick_2_arrow();
    void R_base_move_load_arrow();
    void R_base_move_startzone();
    void R_height_move_pick_arrow();
    void R_height_move_load_standby_arrow();
    void R_height_move_load_arrow();
    void R_height_move_target(double target);
    void L_base_move_pick_standby_1_arrow();
    void L_base_move_pick_1_arrow();
    void L_base_move_pick_standby_2_arrow();
    void L_base_move_pick_2_arrow();
    void L_base_move_load_arrow();
    void L_base_move_startzone();
    void L_height_move_pick_arrow();
    void L_height_move_load_standby_arrow();
    void L_height_move_load_arrow();
    void L_height_move_target(double target);

    void Shot_Power_move_shooterinit();
    void Shot_Power_move_target(double target);
    void Shot_Angle_move_initial();
    void Shot_Angle_move_target(double target);

    void Cyl_base_pick();
    void Cyl_base_load();
    void Cyl_R_hand_1_grab();
    void Cyl_R_hand_2_grab();
    void Cyl_R_hand_1_release();
    void Cyl_R_hand_2_release();
    void Cyl_R_hand_roll_pick();
    void Cyl_R_hand_roll_load();
    void Cyl_L_hand_1_grab();
    void Cyl_L_hand_2_grab();
    void Cyl_L_hand_1_release();
    void Cyl_L_hand_2_release();
    void Cyl_L_hand_roll_pick();
    void Cyl_L_hand_roll_load();
    void Cyl_shooter_grab();
    void Cyl_shooter_release();
    //----------------------------------------
    double R_base_pick_standby_1;
    double R_base_pick_1;
    double R_base_pick_standby_2;
    double R_base_pick_2;
    double R_base_load;
    double R_base_startzone;
    double R_height_pick;
    double R_height_load_standby;
    double R_height_load;

    double L_base_pick_standby_1;
    double L_base_pick_1;
    double L_base_pick_standby_2;
    double L_base_pick_2;
    double L_base_load;
    double L_base_startzone;
    double L_height_pick;
    double L_height_load_standby;
    double L_height_load;

    double shot_power_shooter_init;
    double shot_power_load;
    double shot_power_launch_pos_1;
    double shot_power_launch_pos_2;
    double shot_power_launch_pos_3;
    double shot_power_launch_pos_4;
    double shot_angle_launch_pos_1;
    double shot_angle_launch_pos_2;
    double shot_angle_launch_pos_3;
    double shot_angle_launch_pos_4;

    double shot_power_launch_pos_1_wall;
    double shot_power_launch_pos_2_wall;
    double shot_power_launch_pos_3_wall;
    double shot_power_launch_pos_4_wall;
    double shot_angle_launch_pos_1_wall;
    double shot_angle_launch_pos_2_wall;
    double shot_angle_launch_pos_3_wall;
    double shot_angle_launch_pos_4_wall;

    int Pick_mode = -1;
    int R_load_mode = -1;
    int L_load_mode = -1;

    bool _loaded = false;
    bool _R_loading = false;
    bool _L_loading = false;

    bool _shooter_pos_load = false;

    double shot_power_adjust = 0.0;

    double R_height_adjust = 0.0;
    double L_height_adjust = 0.0;

    bool _R_hand_1_pick = false;
    bool _L_hand_1_pick = false;
    //----------------------------------------

    // flags
    bool _is_operating = false;
    bool _is_standing_by = false;
    bool _launch_enable = false;
    bool _arm_reached = false;
    bool _launch_point_reached = false;
    bool _rack_point_reached = false;
    bool _has_table_restarted = false;
    bool _has_table_rotated = false;
    bool _has_table_moved = false;
    bool _next_pressed = false;
    bool _abort_pressed = false;
    bool _is_manual_enabled = true;
    bool _list_change_able = false;
    bool _command_ongoing = false;
    bool _has_loaded = false;
    bool _initial_pose_finished = false;

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
    static int arm_angle_plus;

    static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonBack;
	static int ButtonStart;
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

    static float arm_movelength;
    static float base_rotate_angle;
    static float arm_rotete_angle;
    static float launch_movelength;

    int currentCommandIndex = 0;
    
    static const std::vector<OpMode> opmode;
    static const std::vector<ControllerCommands> manual_all;
    //------------------------------------------------------------------------
    static const std::vector<ControllerCommands> Shot_and_SetLoadPos_commands;
    //------------------------------------------------------------------------
    const std::vector<ControllerCommands> *command_list;
};

int tr_nodelet_main::_padx = 0;
int tr_nodelet_main::_pady = 0;
int tr_nodelet_main::_lb = 0;
int tr_nodelet_main::_rb = 0;
int tr_nodelet_main::arm_angle_plus = 0;

int tr_nodelet_main::ButtonA = 1;
int tr_nodelet_main::ButtonB = 2;
int tr_nodelet_main::ButtonX = 0;
int tr_nodelet_main::ButtonY = 3;
int tr_nodelet_main::ButtonLB = 4;
int tr_nodelet_main::ButtonRB = 5;
int tr_nodelet_main::ButtonBack = 8;
int tr_nodelet_main::ButtonStart = 9;
int tr_nodelet_main::ButtonLeftThumb = 6;
int tr_nodelet_main::ButtonRightThumb = 7;

int tr_nodelet_main::AxisDPadX = 4;
int tr_nodelet_main::AxisDPadY = 5;
int tr_nodelet_main::AxisLeftThumbX = 0;
int tr_nodelet_main::AxisLeftThumbY = 1;
int tr_nodelet_main::AxisRightThumbX = 2;
int tr_nodelet_main::AxisRightThumbY = 3;
int tr_nodelet_main::ButtonLeftTrigger = 10;
int tr_nodelet_main::ButtonRightTrigger = 11;

float tr_nodelet_main::arm_movelength = 0;
float tr_nodelet_main::base_rotate_angle = 0;
float tr_nodelet_main::arm_rotete_angle = 0;
float tr_nodelet_main::launch_movelength = 0;

const std::vector<OpMode> tr_nodelet_main::opmode{
    {
        OpMode::def,
        OpMode::full_op
    }
};

const std::vector<ControllerCommands> tr_nodelet_main::manual_all(
    {
        ControllerCommands::manual,
    }
);

const std::vector<ControllerCommands> tr_nodelet_main::Shot_and_SetLoadPos_commands(
    {
        ControllerCommands::shooter_release,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::shooter_init,
        ControllerCommands::angle_init,
        //ControllerCommands::r_hand_homing,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::shooter_grab,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::shooter_move_load,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        //ControllerCommands::r_hand_homing,
        //ControllerCommands::l_hand_homing,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_1s,
        //ControllerCommands::delay,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //ControllerCommands::r_hand_recover,
        //ControllerCommands::l_hand_recover,
        ControllerCommands::riseflag_shooter_pos_load,
    }
);

void tr_nodelet_main::onInit(){
    nh = getNodeHandle();
    //constructor
    _nh = getPrivateNodeHandle();

    // related to launch
    //_nh.param("launch_max_radian", this->LaunchMaximumradian, 0.0);
    //_nh.param("launch_min_radian", this->LaunchMinimumradian, 0.0);
    //_nh.param("launch_radian", this->Launchradian, 0.0);
    //_nh.param("launch_initial_length", this->LaunchInitiallength, 0.0);
    //_nh.param("ball_screw_lead_", this->BallScrewLead, 0.0);//(mm)
    //_nh.param("prop_length", this->PropLength, 0.0);//(mm)
    //_nh.param("launch_part_length", this->LaunchPartLength, 0.0);//(mm)
    //_nh.param("launch_max_pull_distance",this->LaunchMaximumPullDistance,0.0); //490mm
    //_nh.param("launch_pull_distance",this->LaunchPullDistance,0.0);
    //_nh.param("tooth_per_rotate",this->toothperrotate); 
    //_nh.param("length_per_bit",this->lengthperbit);//(mm)
    _nh.param("R_base_pick_standby_1", this->R_base_pick_standby_1, 0.0);
    _nh.param("R_base_pick_1", this->R_base_pick_1, 0.0);
    _nh.param("R_base_pick_standby_2", this->R_base_pick_standby_2, 0.0);
    _nh.param("R_base_pick_2", this->R_base_pick_2, 0.0);
    _nh.param("R_base_load", this->R_base_load, 0.0);
    _nh.param("R_base_startzone", this->R_base_startzone, 0.0);
    _nh.param("R_height_pick", this->R_height_pick, 0.0);
    _nh.param("R_height_load_standby", this->R_height_load_standby, 0.0);
    _nh.param("R_height_load", this->R_height_load, 0.0);
    _nh.param("L_base_pick_standby_1", this->L_base_pick_standby_1, 0.0);
    _nh.param("L_base_pick_1", this->L_base_pick_1, 0.0);
    _nh.param("L_base_pick_standby_2", this->L_base_pick_standby_2, 0.0);
    _nh.param("L_base_pick_2", this->L_base_pick_2, 0.0);
    _nh.param("L_base_load", this->L_base_load, 0.0);
    _nh.param("L_base_startzone", this->L_base_startzone, 0.0);
    _nh.param("L_height_pick", this->L_height_pick, 0.0);
    _nh.param("L_height_load_standby", this->L_height_load_standby, 0.0);
    _nh.param("L_height_load", this->L_height_load, 0.0);
    _nh.param("shot_power_shooter_init", this->shot_power_shooter_init, 0.0);
    _nh.param("shot_power_load", this->shot_power_load, 0.0);
    _nh.param("shot_power_launch_pos_1", this->shot_power_launch_pos_1, 0.0);
    _nh.param("shot_power_launch_pos_2", this->shot_power_launch_pos_2, 0.0);
    _nh.param("shot_power_launch_pos_3", this->shot_power_launch_pos_3, 0.0);
    _nh.param("shot_power_launch_pos_4", this->shot_power_launch_pos_4, 0.0);
    _nh.param("shot_angle_launch_pos_1", this->shot_angle_launch_pos_1, 0.0);
    _nh.param("shot_angle_launch_pos_2", this->shot_angle_launch_pos_2, 0.0);
    _nh.param("shot_angle_launch_pos_3", this->shot_angle_launch_pos_3, 0.0);
    _nh.param("shot_angle_launch_pos_4", this->shot_angle_launch_pos_4, 0.0);

    _nh.param("shot_power_launch_pos_1_wall", this->shot_power_launch_pos_1_wall, 0.0);
    _nh.param("shot_power_launch_pos_2_wall", this->shot_power_launch_pos_2_wall, 0.0);
    _nh.param("shot_power_launch_pos_3_wall", this->shot_power_launch_pos_3_wall, 0.0);
    _nh.param("shot_power_launch_pos_4_wall", this->shot_power_launch_pos_4_wall, 0.0);
    _nh.param("shot_angle_launch_pos_1_wall", this->shot_angle_launch_pos_1_wall, 0.0);
    _nh.param("shot_angle_launch_pos_2_wall", this->shot_angle_launch_pos_2_wall, 0.0);
    _nh.param("shot_angle_launch_pos_3_wall", this->shot_angle_launch_pos_3_wall, 0.0);
    _nh.param("shot_angle_launch_pos_4_wall", this->shot_angle_launch_pos_4_wall, 0.0);

  	// related to grab and load the arrow 
    //_nh.param("roll_arm_radian", this->ArmRollRadian, 0.0);
  	//_nh.param("arm_max_height", this->ArmMaxHeight, 0.0);
    //_nh.param("arm_height", this->ArmHeight, 0.0);
    //_nh.param("base_max_radian", this->ArmMaximumRadian, 0.0);
    //_nh.param("base_radian", this->ArmRadian, 0.0);
    

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

    this->joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &tr_nodelet_main::joyCallback, this);

    this->Shot_Power_Cmd_pub = nh.advertise<std_msgs::UInt8>("Shot_Power_cmd", 1);
    this->Shot_Power_Pos_pub = nh.advertise<std_msgs::Float64>("Shot_Power_cmd_pos", 1);

    this->Shot_Angle_Cmd_pub = nh.advertise<std_msgs::UInt8>("Shot_Angle_cmd", 1);
  	this->Shot_Angle_Pos_pub = nh.advertise<std_msgs::Float64>("Shot_Angle_cmd_pos", 1);

    this->Pick_R_Base_Cmd_pub = nh.advertise<std_msgs::UInt8>("Pick_R_Base_cmd", 1);
  	this->Pick_R_Base_Pos_pub = nh.advertise<std_msgs::Float64>("Pick_R_Base_cmd_pos", 1);

    this->Pick_R_Height_Cmd_pub = nh.advertise<std_msgs::UInt8>("Pick_R_Height_cmd", 1);
    this->Pick_R_Height_Pos_pub = nh.advertise<std_msgs::Float64>("Pick_R_Height_cmd_pos", 1);

    this->Pick_L_Base_Cmd_pub = nh.advertise<std_msgs::UInt8>("Pick_L_Base_cmd", 1);
  	this->Pick_L_Base_Pos_pub = nh.advertise<std_msgs::Float64>("Pick_L_Base_cmd_pos", 1);

    this->Pick_L_Height_Cmd_pub = nh.advertise<std_msgs::UInt8>("Pick_L_Height_cmd", 1);
    this->Pick_L_Height_Pos_pub = nh.advertise<std_msgs::Float64>("Pick_L_Height_cmd_pos", 1);
  
    this->Solenoid1_Cmd_pub = nh.advertise<std_msgs::UInt8>("solenoid1_cmd", 1);
    this->Solenoid1_Order_pub = nh.advertise<std_msgs::UInt8>("solenoid1_order", 1);

    this->Solenoid2_Cmd_pub = nh.advertise<std_msgs::UInt8>("solenoid2_cmd", 1);
    this->Solenoid2_Order_pub = nh.advertise<std_msgs::UInt8>("solenoid2_order", 1);

    this->act_enable_pub0 = nh.advertise<std_msgs::UInt8>("foot0_cmd", 1);
    this->act_enable_pub1 = nh.advertise<std_msgs::UInt8>("foot1_cmd", 1);
    this->act_enable_pub2 = nh.advertise<std_msgs::UInt8>("foot2_cmd", 1);
    this->act_enable_pub3 = nh.advertise<std_msgs::UInt8>("foot3_cmd", 1);
    
    this->cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    this->control_timer = nh.createTimer(ros::Duration(0.05), &tr_nodelet_main::control_timer_callback, this);
    NODELET_INFO("tr node has started.");

    this->command_list = &tr_nodelet_main::manual_all;
    this->current_OpMode = this->opmode[0];

}


void tr_nodelet_main::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    static bool last_a = false;
    static bool last_b = false;
    static bool last_x = false;
    static bool last_y = false;
    static bool _lefttrigger_enable = false;
    static bool _Cyl_shooter = false;
    static bool _lb_enable = false;
    static bool _rb_enable = false;
    static bool _Cyl_R_hand_1 = false;
    static bool _Cyl_L_hand_1 = false;
    static bool _Cyl_R_hand_2 = false;
    static bool _Cyl_L_hand_2 = false;
    static bool _R_hand_pos_change = false;
    static bool _L_hand_pos_change = false;
    static bool _y_enable = false;
    static bool _b_enable = false;
    static bool _a_enable = false;
    static bool _x_enable = false;
    
    static double shooter_pull_length = 0.0;

    this->_a = joy->buttons[ButtonA];
    this->_b = joy->buttons[ButtonB];
    this->_x = joy->buttons[ButtonX];
    this->_y = joy->buttons[ButtonY];
    this->_lb = joy->buttons[ButtonLB];
    this->_rb = joy->buttons[ButtonRB];
    this->_padx = joy->axes[AxisDPadX];
    this->_pady = joy->axes[AxisDPadY];
    this->arm_angle_plus = _rb - _lb;
    this->_rightthumb = joy->buttons[ButtonRightThumb];
    this->_leftthumb = joy->buttons[ButtonLeftThumb];
    this->_righttrigger = joy->buttons[ButtonRightTrigger];
    this->_lefttrigger = joy->buttons[ButtonLeftTrigger];
    

    this->_start = joy->buttons[ButtonStart];
    this->_back  = joy->buttons[ButtonBack];


    if (_start)
    {
        this->recover();
        _R_loading = false;
        _L_loading = false;
        _loaded = false;
        _shooter_pos_load = false;
        _R_hand_pos_change = false;
        _L_hand_pos_change = false;
        Pick_mode = -1;
        R_load_mode = -1;
        L_load_mode = -1;
        shot_power_adjust = 0.0;
    }
    if (_back)
    {
        this->shutdown();
    }
    if(!this->_command_ongoing)
    {   
        //------------------------------------------------------------
        if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0) && _righttrigger){
            Shot_Angle_move_target(50);
            Cyl_R_hand_1_grab();
            Cyl_R_hand_2_grab();
            Cyl_L_hand_1_grab();
            Cyl_L_hand_2_grab();
            Pick_R_Base_Homing();
            Pick_R_Height_Homing();
            Pick_L_Base_Homing();
            Pick_L_Height_Homing();
        }else if((joy->buttons[ButtonRightThumb] != 1.0) && (joy->buttons[ButtonLeftThumb] != 1.0) && _righttrigger){
            Shot_Angle_Homing();
            Shot_Power_Homing();
        }
        if(_rb && _lb && !_R_loading && !_L_loading){
            Cyl_shooter_release();
        }
        //R_height_adjust -= joy->buttons[ButtonRightThumb];
        //R_height_adjust += joy->buttons[ButtonLeftThumb];
        //R_height_move_target(R_height_adjust);
        if(_lb){
            if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
                if(_Cyl_R_hand_1){
                    Cyl_R_hand_1_grab();
                    _Cyl_R_hand_1 = false;
                }else{
                    Cyl_R_hand_1_release();
                    _Cyl_R_hand_1 = true;
                }
            }else if((joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                if(_Cyl_R_hand_2){
                    Cyl_R_hand_2_grab();
                    _Cyl_R_hand_2 = false;
                }else{
                    Cyl_R_hand_2_release();
                    _Cyl_R_hand_2 = true;
                }
            }
        }
        if(_rb){
            if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
                if(_Cyl_L_hand_1){
                    Cyl_L_hand_1_grab();
                    _Cyl_L_hand_1 = false;
                }else{
                    Cyl_L_hand_1_release();
                    _Cyl_L_hand_1 = true;
                }
            }else if((joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                if(_Cyl_L_hand_2){
                    Cyl_L_hand_2_grab();
                    _Cyl_L_hand_2 = false;
                }else{
                    Cyl_L_hand_2_release();
                    _Cyl_L_hand_2 = true;
                }
            }
        }
        if(!_R_loading && !_L_loading && _lefttrigger){
            _loaded = false;
            _shooter_pos_load = false;
            this->command_list = &Shot_and_SetLoadPos_commands;
            _command_ongoing = true;
        }
        if(_y && _y_enable){            
            if(Pick_mode == 8){
                Pick_mode = 0;
            }else{
                Pick_mode++;
            }
            _y_enable = false;
        }else{
            _y_enable = true;
        }
        if(_a && _a_enable){
            if((joy->buttons[ButtonRightThumb] == 1.0)){
                if(_L_hand_pos_change){
                    Cyl_L_hand_roll_load();
                    _L_hand_pos_change = false;
                }else{
                    Cyl_L_hand_roll_pick();
                    _L_hand_pos_change = true;
                }
            }else if((joy->buttons[ButtonLeftThumb] == 1.0)){
                if(_R_hand_pos_change){
                    Cyl_R_hand_roll_load();
                    _R_hand_pos_change = false;
                }else{
                    Cyl_R_hand_roll_pick();
                    _R_hand_pos_change = true;
                }
            }
            _a_enable = false;
        }else{
            _a_enable = true;
        }
        if(_x && _x_enable){
            if(_shooter_pos_load){
                if(R_load_mode == 1){
                    R_load_mode = 0;
                }else{
                    R_load_mode++;
                }
            }
            _x_enable = false;
        }else{
            _x_enable = true;
        }
        if(_b && _b_enable){
            if(_shooter_pos_load){
                if(L_load_mode == 1){
                    L_load_mode = 0;
                }else{
                    L_load_mode++;
                }
            }
            _b_enable = false;
        }else{
            _b_enable = true;
        }
        if(_loaded && !_R_loading && !_L_loading ){
            //shot_power_adjust += joy->buttons[ButtonLeftThumb] * 2;
            //shot_power_adjust -= joy->buttons[ButtonRightThumb] * 2;
            if(_pady == 1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
                Shot_Power_move_target(shot_power_launch_pos_1 /*+ shot_power_adjust*/);
                Shot_Angle_move_target(shot_angle_launch_pos_1);
            }else if(_pady == 1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Shot_Power_move_target(shot_power_launch_pos_1_wall);
                Shot_Angle_move_target(shot_angle_launch_pos_1_wall);
            }
            if(_padx == 1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
                Shot_Power_move_target(shot_power_launch_pos_2 /*+ shot_power_adjust*/);
                Shot_Angle_move_target(shot_angle_launch_pos_2);
            }else if(_padx == 1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Shot_Power_move_target(shot_power_launch_pos_2_wall);
                Shot_Angle_move_target(shot_angle_launch_pos_2_wall);
            }
            if(_padx == -1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
                Shot_Power_move_target(shot_power_launch_pos_3 /*+ shot_power_adjust*/);
                Shot_Angle_move_target(shot_angle_launch_pos_3);
            }else if(_padx == -1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Shot_Power_move_target(shot_power_launch_pos_3_wall);
                Shot_Angle_move_target(shot_angle_launch_pos_3_wall);
            }
            if(_pady == -1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
                Shot_Power_move_target(shot_power_launch_pos_4 /*+ shot_power_adjust*/);
                Shot_Angle_move_target(shot_angle_launch_pos_4);
            }else if(_pady == -1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Shot_Power_move_target(shot_power_launch_pos_4_wall);
                Shot_Angle_move_target(shot_angle_launch_pos_4_wall);
            }
            _shooter_pos_load = false;
        }
        if(!_loaded && _shooter_pos_load && (_lb || _rb || _x || _b || (joy->buttons[ButtonRightThumb] != 0.0) || (joy->buttons[ButtonLeftThumb] != 0.0))){
            if(_x){
                if(!_L_loading){
                    _R_loading = true;
                }
                R_height_adjust = 0.0;
                Pick_mode = -1;
                shot_power_adjust = 0.0;
            }
            if(_b){
                if(!_R_loading){
                    _L_loading = true;
                }
                L_height_adjust = 0.0;
                Pick_mode = -1;
                shot_power_adjust = 0.0;
            }
            if(_R_loading){
                switch (R_load_mode)
                {
                case 0:
                    Cyl_base_load();
                    R_base_move_load_arrow();
                    R_height_adjust += joy->buttons[ButtonLeftThumb] * 2;
                    R_height_adjust -= joy->buttons[ButtonRightThumb] * 2;
                    R_height_move_target(this->R_height_load_standby + R_height_adjust);
                    break;
                case 1:
                    Cyl_base_pick();
                    R_base_move_pick_1_arrow();
                    R_height_move_pick_arrow();
                    if(_R_hand_1_pick){
                        Cyl_R_hand_2_release();
                    }else{
                        Cyl_R_hand_1_release();
                    }
                    R_height_adjust = 0.0;
                    _R_loading = false;
                    _loaded = true;
                    shot_power_adjust = 0.0;
                    break;
                }
                if(_lb){
                    if(_R_hand_1_pick){
                        if(_Cyl_R_hand_2){
                            Cyl_R_hand_2_grab();
                            _Cyl_R_hand_2 = false;
                        }else{
                            Cyl_R_hand_2_release();
                            _Cyl_R_hand_2 = true;
                        }
                    }else{
                        if(_Cyl_R_hand_1){
                            Cyl_R_hand_1_grab();
                            _Cyl_R_hand_1 = false;
                        }else{
                            Cyl_R_hand_1_release();
                            _Cyl_R_hand_1 = true;
                        }
                    }
                }
            }
            if(_L_loading){
                switch (L_load_mode)
                {
                case 0:
                    Cyl_base_load();
                    L_base_move_load_arrow();
                    L_height_adjust += joy->buttons[ButtonLeftThumb] * 2;
                    L_height_adjust -= joy->buttons[ButtonRightThumb] * 2;
                    L_height_move_target(this->L_height_load_standby + L_height_adjust);
                    break;
                case 1:
                    Cyl_base_pick();
                    L_base_move_pick_1_arrow();
                    L_height_move_pick_arrow();
                    if(_L_hand_1_pick){
                        Cyl_L_hand_2_release();
                    }else{
                        Cyl_L_hand_1_release();
                    }
                    L_height_adjust = 0.0;
                    _L_loading = false;
                    _loaded = true;
                    shot_power_adjust = 0.0;
                    break;
                }
                if(_rb){
                    if(_L_hand_1_pick){
                        if(_Cyl_L_hand_2){
                            Cyl_L_hand_2_grab();
                            _Cyl_L_hand_2 = false;
                        }else{
                            Cyl_L_hand_2_release();
                            _Cyl_L_hand_2 = true;
                        }
                    }else{
                        if(_Cyl_L_hand_1){
                            Cyl_L_hand_1_grab();
                            _Cyl_L_hand_1 = false;
                        }else{
                            Cyl_L_hand_1_release();
                            _Cyl_L_hand_1 = true;
                        }
                    }
                }
            }
        }
        if(!_R_loading && !_L_loading && (_y)){
            R_load_mode = -1;
            L_load_mode = -1;
            if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                R_base_move_startzone();
                L_base_move_startzone();
                Pick_mode = -1;
            }else{
                switch (Pick_mode)
                {
                case 0:
                    Cyl_base_pick();
                    Cyl_R_hand_roll_pick();
                    Cyl_L_hand_roll_pick();
                    R_base_move_pick_standby_1_arrow();
                    R_height_move_pick_arrow();
                    L_base_move_pick_standby_1_arrow();
                    L_height_move_pick_arrow();
                    break;

                case 1:
                    R_base_move_pick_1_arrow();
                    L_base_move_pick_1_arrow();

                    //----------
                    R_height_move_pick_arrow();
                    L_height_move_pick_arrow();
                    //----------
                    break;

                case 2:
                    Cyl_R_hand_1_grab();
                    Cyl_L_hand_1_grab();

                    //----------
                    R_height_move_pick_arrow();
                    L_height_move_pick_arrow();
                    //----------
                    break;

                case 3:
                    R_height_move_load_standby_arrow();
                    L_height_move_load_standby_arrow();
                    break;

                case 4:
                    Cyl_R_hand_roll_load();
                    Cyl_L_hand_roll_load();

                    //----------
                    R_height_move_load_standby_arrow();
                    L_height_move_load_standby_arrow();
                    //----------
                    break;

                case 5:
                    R_height_move_pick_arrow();
                    L_height_move_pick_arrow();
                    R_base_move_pick_standby_2_arrow();
                    L_base_move_pick_standby_2_arrow();
                    break;

                case 6:
                    R_base_move_pick_2_arrow();
                    L_base_move_pick_2_arrow();

                    //----------
                    R_height_move_pick_arrow();
                    L_height_move_pick_arrow();
                    //----------
                    break;

                case 7:
                    Cyl_R_hand_2_grab();
                    Cyl_L_hand_2_grab();

                    //----------
                    R_height_move_pick_arrow();
                    L_height_move_pick_arrow();
                    //----------
                    break;

                case 8:
                    R_height_move_load_standby_arrow();
                    L_height_move_load_standby_arrow();
                    break;
                }
            }
        }
        //------------------------------------------------------------

        /*
        if(_padx == -1){
            Shot_Angle_move_initial();
        }else if(_padx == 1){
            Shot_Angle_move_target(100.0);
        }
        if(_pady == 1){
            //shooter_pull_length += 0.1;
            //this->shot_power_pos_msg.data = shooter_pull_length;
            //this->Shot_Power_Pos_pub.publish(this->shot_power_pos_msg);
            act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
            Pick_R_hand_1_cmd_pub.publish(act_conf_cmd_msg);
            Shot_Power_move_shooterinit();
        }else if(_pady == -1){
            act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
            Pick_R_hand_1_cmd_pub.publish(act_conf_cmd_msg);
            Shot_Power_move_target(-10);
        }
        if(_lefttrigger && _lefttrigger_enable){
            if(_Cyl_shooter){
                Cyl_shooter_grab();
                _Cyl_shooter = false;
            }else{
                Cyl_shooter_release();
                _Cyl_shooter = true;
            }
            _lefttrigger_enable = false;
        }else{
            _lefttrigger_enable = true;
        }
        if(_lb && _lb_enable){
            if(_Cyl_R_hand_1){
                Cyl_R_hand_1_grab();
                _Cyl_R_hand_1 = false;
            }else{
                Cyl_R_hand_1_release();
                _Cyl_R_hand_1 = true;
            }
            _lb_enable = false;
        }else{
            _lb_enable = true;
        }
        if(_rb && _lb){
            Pick_R_Hand_Homing();
        }
        if(_rb){
            R_height_move_load_arrow();
        }
        if(_x){
            R_base_move_pick_1_arrow();
            R_height_move_load_standby_arrow();
            R_hand_move_pick_arrow();
        }
        if(_b){
            //Pick_R_Height_Homing();
            //Pick_R_Base_Homing();
            //Shot_Angle_Homing();
            //Shot_Power_Homing();
            Cyl_R_hand_1_release();
            Cyl_base_pick();
            R_base_move_pick_standby_1_arrow();
            R_height_move_pick_arrow();
            R_hand_move_pick_arrow();
        }
        if(_righttrigger){
            Pick_R_Hand_Homing();
            Pick_R_Base_Homing();
            Pick_R_Height_Homing();
            Shot_Angle_Homing();
            Shot_Power_Homing();
            //this->Pick_R_hand_pos_msg.data = -4.65;
            //this->Pick_R_Hand_Pos_pub.publish(this->Pick_R_hand_pos_msg);
            //R_base_move_load_arrow();
            //R_height_move_load_arrow();
            //R_hand_move_load_arrow();
        }
        if(_y){
            //Pick_R_Base_Homing();
            Cyl_base_load();
            R_base_move_load_arrow();
            R_height_move_load_standby_arrow();
            R_hand_move_load_arrow();
            //this->shot_angle_pos_msg.data = 100.0;
            //this->Shot_Angle_Pos_pub.publish(this->shot_angle_pos_msg);
            //R_hand_move_pick_arrow();
            //R_height_move_pick_arrow();
            //R_base_move_pick_1_arrow();
            //this->Pick_R_height_pos_msg.data = 38;
            //this->Pick_R_Height_Pos_pub.publish(this->Pick_R_height_pos_msg);
            //this->Pick_R_base_pos_msg.data = 3.0;
            //this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
            //this->shot_angle_pos_msg.data = 1.0;
            //this->Shot_Angle_Pos_pub.publish(this->shot_angle_pos_msg);
            //this->shot_power_pos_msg.data = -10.0;
            //this->Shot_Power_Pos_pub.publish(this->shot_power_pos_msg);
        }
        if(_a){
            R_base_move_pick_1_arrow();
            R_height_move_pick_arrow();
            R_hand_move_pick_arrow();
            //R_height_move_load_standby_arrow();
            //R_base_move_pick_standby_1_arrow();
            //Cyl_shooter_grab();
        }else{
            //Cyl_shooter_release();
        }
        */

        //if(!this->_has_loaded)
        //{
        //    if (_y)
        //    {
        //        this->command_list = &load_test_commands;
        //        _command_ongoing = true;
        //        _has_loaded = true;
        //    }
        //}   
        //else if (_a&&(_pady == 1))
        //{      
        //    this->command_list = &launch_long_test_commands;
        //    _command_ongoing = true;
        //    _has_loaded = false;
        //}
        //
        //if (_righttrigger)
        //{
        //    this->command_list = &initial_pose;
        //    _command_ongoing = true;
        //    _initial_pose_finished = true;
        //}
    }


    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisLeftThumbX];   
        double vel_y = joy->axes[AxisLeftThumbY];
        //double vel_yaw_l = (joy->buttons[ButtonLeftThumb] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        //double vel_yaw_r = (joy->buttons[ButtonRightThumb] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = joy->axes[AxisRightThumbX];//vel_yaw_l + vel_yaw_r;
        double vel_norm = hypot(vel_x, vel_y);


        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }

        //if (_x && !last_x)
        //{
        //    
        //    if ((((this->lastSolenoid_1_Order) >> (4)) & 1) == 1)
        //    {
        //        
        //        this->grab_arrow();
        //        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        //        this->adjust_arm_to_lift();
        //        NODELET_INFO("release_arrow");
        //    }
        //    else{
        //        this->adjust_arm_to_grab();
        //        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        //        this->release_arrow();
        //        NODELET_INFO("grab_arrow");
        //    }
        //    
        //}
        //if (_b && !last_b)
        //{            
        //        this->adjust_arm_to_set();
        //        NODELET_INFO("homing");
        //}

        if(joy->buttons[ButtonLeftThumb] >= 1.0){
            this->cmd_vel_msg.linear.x = -vel_x * 0.25;
            this->cmd_vel_msg.linear.y = -vel_y * 0.25;
            this->cmd_vel_msg.angular.z = -vel_yaw * 0.25;
        }else if(joy->buttons[ButtonRightThumb] >= 1.0){
            this->cmd_vel_msg.linear.x = -vel_x * 5;
            this->cmd_vel_msg.linear.y = -vel_y * 5;
            this->cmd_vel_msg.angular.z = -vel_yaw * 2.5;
        }else{
            this->cmd_vel_msg.linear.x = -vel_x;
            this->cmd_vel_msg.linear.y = -vel_y;
            this->cmd_vel_msg.angular.z = -vel_yaw;
        }
        this->cmd_vel_pub.publish(this->cmd_vel_msg);
    }
    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;  
}



//void tr_nodelet_main::launcher_move(float movelength){
   //double init_radian = movelength*2*pi/(this->toothperrotate*this->lengthperbit); 
   //this->shot_power_pos_msg.data = init_radian;
   //this->Shot_Power_Pos_pub.publish(this->shot_power_pos_msg);
//}

//void tr_nodelet_main::adjust_launcher_radian(float launchangle){
   // calculate the launch angle from the law of cosines
   //float conf_angle = 2*pi*(this->LaunchPartLength*(sqrtf32(pow((this->PropLength/this->LaunchPartLength),2)-pow(sinf32(launchangle),2)))-this->LaunchInitiallength)/this->BallScrewLead;
   //this->shot_angle_pos_msg.data = conf_angle;
   //this->Shot_Angle_Pos_pub.publish(this->shot_angle_pos_msg);
//}


void tr_nodelet_main::clear_flags(void){
        this->_is_operating = false;
        this->_is_standing_by = false;
        this->_launch_enable=false;
        this->_arm_reached = false;
        this->_launch_point_reached = false;
        this->_has_table_rotated = false;
        this->_has_table_moved = false;
        this->_rack_point_reached = false;
        this->_next_pressed = false;
        this->_abort_pressed = false;
        this->_command_ongoing = false;
}

void tr_nodelet_main::set_delay(double delay_s)
{
    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

void tr_nodelet_main::shutdown(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Shot_Power_Cmd_pub.publish(act_conf_cmd_msg);
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_R_Base_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_R_Height_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_L_Base_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_L_Height_Cmd_pub.publish(act_conf_cmd_msg);
    act_enable_pub0.publish(act_conf_cmd_msg);
    act_enable_pub1.publish(act_conf_cmd_msg);
    act_enable_pub2.publish(act_conf_cmd_msg);
    act_enable_pub3.publish(act_conf_cmd_msg);
    Solenoid1_Cmd_pub.publish(act_conf_cmd_msg);
    Solenoid2_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    Shot_Power_Cmd_pub.publish(act_conf_cmd_msg);
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_R_Base_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_R_Height_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_L_Base_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_L_Height_Cmd_pub.publish(act_conf_cmd_msg);
    act_enable_pub0.publish(act_conf_cmd_msg);
    act_enable_pub1.publish(act_conf_cmd_msg);
    act_enable_pub2.publish(act_conf_cmd_msg);
    act_enable_pub3.publish(act_conf_cmd_msg);
    Solenoid1_Cmd_pub.publish(act_conf_cmd_msg);
    Solenoid2_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Shot_Power_Cmd_pub.publish(act_conf_cmd_msg);
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Shot_Power_Cmd_pub.publish(act_conf_cmd_msg);
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
}    

void tr_nodelet_main::change_OpMode(void){
    this->OpModecurrentindex++;
    if(OpModecurrentindex >= (int)this->opmode.size()){
        OpModecurrentindex  = 0; 
    }
    this->current_OpMode = opmode.at(OpModecurrentindex);
}

void tr_nodelet_main::Shot_Power_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Shot_Power_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Shot_Power_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::Shot_Angle_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::Pick_R_Base_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Pick_R_Base_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Pick_R_Base_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::Pick_R_Height_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Pick_R_Height_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Pick_R_Height_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::Pick_L_Base_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Pick_L_Base_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Pick_L_Base_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::Pick_L_Height_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Pick_L_Height_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Pick_L_Height_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::R_base_move_pick_standby_1_arrow(void){
    this->Pick_R_base_pos_msg.data = this->R_base_pick_standby_1;
    this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
}

void tr_nodelet_main::R_base_move_pick_1_arrow(void){
    this->Pick_R_base_pos_msg.data = this->R_base_pick_1;
    this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
}

void tr_nodelet_main::R_base_move_pick_standby_2_arrow(void){
    this->Pick_R_base_pos_msg.data = this->R_base_pick_standby_2;
    this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
}

void tr_nodelet_main::R_base_move_pick_2_arrow(void){
    this->Pick_R_base_pos_msg.data = this->R_base_pick_2;
    this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
}

void tr_nodelet_main::R_base_move_load_arrow(void){
    this->Pick_R_base_pos_msg.data = this->R_base_load;
    this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
}

void tr_nodelet_main::R_base_move_startzone(void){
    this->Pick_R_base_pos_msg.data = this->R_base_startzone;
    this->Pick_R_Base_Pos_pub.publish(this->Pick_R_base_pos_msg);
}

void tr_nodelet_main::R_height_move_pick_arrow(void){
    this->Pick_R_height_pos_msg.data = this->R_height_pick;
    this->Pick_R_Height_Pos_pub.publish(this->Pick_R_height_pos_msg);
}

void tr_nodelet_main::R_height_move_load_standby_arrow(void){
    this->Pick_R_height_pos_msg.data = this->R_height_load_standby;
    this->Pick_R_Height_Pos_pub.publish(this->Pick_R_height_pos_msg);
}

void tr_nodelet_main::R_height_move_load_arrow(void){
    this->Pick_R_height_pos_msg.data = this->R_height_load;
    this->Pick_R_Height_Pos_pub.publish(this->Pick_R_height_pos_msg);
}

void tr_nodelet_main::R_height_move_target(double target){
    this->Pick_R_height_pos_msg.data = target;
    this->Pick_R_Height_Pos_pub.publish(this->Pick_R_height_pos_msg);
}

void tr_nodelet_main::L_base_move_pick_standby_1_arrow(void){
    this->Pick_L_base_pos_msg.data = this->L_base_pick_standby_1;
    this->Pick_L_Base_Pos_pub.publish(this->Pick_L_base_pos_msg);
}

void tr_nodelet_main::L_base_move_pick_1_arrow(void){
    this->Pick_L_base_pos_msg.data = this->L_base_pick_1;
    this->Pick_L_Base_Pos_pub.publish(this->Pick_L_base_pos_msg);
}

void tr_nodelet_main::L_base_move_pick_standby_2_arrow(void){
    this->Pick_L_base_pos_msg.data = this->L_base_pick_standby_2;
    this->Pick_L_Base_Pos_pub.publish(this->Pick_L_base_pos_msg);
}

void tr_nodelet_main::L_base_move_pick_2_arrow(void){
    this->Pick_L_base_pos_msg.data = this->L_base_pick_2;
    this->Pick_L_Base_Pos_pub.publish(this->Pick_L_base_pos_msg);
}

void tr_nodelet_main::L_base_move_load_arrow(void){
    this->Pick_L_base_pos_msg.data = this->L_base_load;
    this->Pick_L_Base_Pos_pub.publish(this->Pick_L_base_pos_msg);
}

void tr_nodelet_main::L_base_move_startzone(void){
    this->Pick_L_base_pos_msg.data = this->L_base_startzone;
    this->Pick_L_Base_Pos_pub.publish(this->Pick_L_base_pos_msg);
}

void tr_nodelet_main::L_height_move_pick_arrow(void){
    this->Pick_L_height_pos_msg.data = this->L_height_pick;
    this->Pick_L_Height_Pos_pub.publish(this->Pick_L_height_pos_msg);
}

void tr_nodelet_main::L_height_move_load_standby_arrow(void){
    this->Pick_L_height_pos_msg.data = this->L_height_load_standby;
    this->Pick_L_Height_Pos_pub.publish(this->Pick_L_height_pos_msg);
}

void tr_nodelet_main::L_height_move_load_arrow(void){
    this->Pick_L_height_pos_msg.data = this->L_height_load;
    this->Pick_L_Height_Pos_pub.publish(this->Pick_L_height_pos_msg);
}

void tr_nodelet_main::L_height_move_target(double target){
    this->Pick_L_height_pos_msg.data = target;
    this->Pick_L_Height_Pos_pub.publish(this->Pick_L_height_pos_msg);
}

void tr_nodelet_main::Shot_Power_move_shooterinit(void){
    this->shot_power_pos_msg.data = this->shot_power_shooter_init;
    this->Shot_Power_Pos_pub.publish(this->shot_power_pos_msg);
}

void tr_nodelet_main::Shot_Power_move_target(double target){
    this->shot_power_pos_msg.data = target;
    this->Shot_Power_Pos_pub.publish(this->shot_power_pos_msg);
}

void tr_nodelet_main::Shot_Angle_move_initial(void){
    this->shot_angle_pos_msg.data = 0.0;
    this->Shot_Angle_Pos_pub.publish(this->shot_angle_pos_msg);
}

void tr_nodelet_main::Shot_Angle_move_target(double target){
    this->shot_angle_pos_msg.data = target;
    this->Shot_Angle_Pos_pub.publish(this->shot_angle_pos_msg);
}

void tr_nodelet_main::Cyl_base_pick(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::move_base_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_base_load(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::move_base_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_R_hand_1_grab(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::R_hand_1_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_R_hand_2_grab(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::R_hand_2_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_R_hand_1_release(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::R_hand_1_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_R_hand_2_release(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::R_hand_2_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_L_hand_1_grab(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::L_hand_1_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_L_hand_2_grab(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::L_hand_2_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_L_hand_1_release(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::L_hand_1_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_L_hand_2_release(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::L_hand_2_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_shooter_grab(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::shooter_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_shooter_release(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::shooter_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_R_hand_roll_pick(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::R_hand_roll_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
    this->_R_hand_1_pick = true;
}

void tr_nodelet_main::Cyl_R_hand_roll_load(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::R_hand_roll_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
    this->_R_hand_1_pick = false;
}

void tr_nodelet_main::Cyl_L_hand_roll_pick(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::L_hand_roll_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
    this->_L_hand_1_pick = true;
}

void tr_nodelet_main::Cyl_L_hand_roll_load(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::L_hand_roll_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
    this->_L_hand_1_pick = false;
}

void tr_nodelet_main::delay_start(double delay_s)
{
    this->delay_time = ros::Time::now().toSec() + delay_s;
    while(this->delay_time > ros::Time::now().toSec());
}

void tr_nodelet_main::control_timer_callback(const ros::TimerEvent &event)
{ 
    //this->command_list->size() <= (int)this->currentCommandIndex || this->command_list == &this->manual_all
    if (!this->_command_ongoing)
    {

        //if (!((this->base_rotate_angle > 3.0 && this->_padx > 0) || (this->base_rotate_angle < -3.0 && this->_padx < 0))&&abs(_padx))
        //{
        //    this->base_rotate_angle += this->_padx * 0.025;
        //    this->base_rotate(this->base_rotate_angle);
        //}
        //if (abs(_pady))
        //{
        //    this->arm_movelength += this->_pady * 0.087;
        //    this->adjust_arm(this->arm_movelength);
        //}
        //if (!((this->arm_rotete_angle > pi && this->arm_angle_plus > 0) || (this->arm_rotete_angle < 0 && this->arm_angle_plus < 0))&&abs(arm_angle_plus))
        //{
        //    this->arm_rotete_angle += this->arm_angle_plus * 0.025;
        //    this->arm_rotate(this->arm_rotete_angle);
        //}
        
        
        NODELET_INFO("control_time_return");
        
    }

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    if(currentCommand == ControllerCommands::home)
    {
        this->homing();
        this->currentCommandIndex++;
        NODELET_INFO("home");
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
    else if (currentCommand == ControllerCommands::shooter_release)
    {
        this->Cyl_shooter_release();
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::shooter_grab)
    {
        this->Cyl_shooter_grab();
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::shooter_init)
    {   
        this->Shot_Power_move_shooterinit();
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::shooter_move_load)
    {
        this->Shot_Power_move_target(this->shot_power_load);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::angle_init)
    {
        this->Shot_Angle_move_initial();
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::riseflag_shooter_pos_load)
    {
        this->_shooter_pos_load = true;
        this->currentCommandIndex++;
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
PLUGINLIB_EXPORT_CLASS(tr::tr_nodelet_main, nodelet::Nodelet);