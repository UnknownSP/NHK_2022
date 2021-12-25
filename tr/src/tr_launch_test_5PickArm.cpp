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
    set_delay_100ms,
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
    shooter_move_load_wait,
    angle_init,
    angle_load_wait,
    angle_load,
    angle_avoid_loader,
    pick_slide_next_load,
    pick_slide_next_adjust,
    pick_slide_now_hand,
    pick_slide_release,
    pick_slide_adjust,
    pick_cyl_release,
    pick_cyl_grab,
    arrowadjust_grab,
    arrowadjust_release,

    riseflag_shooter_pos_load,

    shooter_init_reachcheck,
    shooter_load_wait_reachcheck,
    shooter_load_reachcheck,
    angle_load_reachcheck,
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
    
    shooter_cmd             = 0b0000010,//default close
    rotate_hands_push_cmd   = 0b0010000,//default open
    rotate_hands_pull_cmd   = 0b1000000,//default open
    hand_5_cmd              = 0b0100000,//default right
    arrowadjust_cmd         = 0b0001000,//default right
    
    hand_2_cmd      = 0b0000001,//default open
    hand_1_cmd      = 0b0000100,//default open
    hand_3_cmd      = 0b0000010,//default open
    hand_4_cmd      = 0b0100000,//default open
    
};

enum class MotorCommands : uint8_t
{
    shutdown_cmd      = 0x00,
    recover_cmd       = 0x01,
    homing_shirasu_cmd= 0x02,
    get_status        = 0x03,
    recover_current   = 0x04,
	recover_velocity  = 0x05,
	recover_position  = 0x06,
    homing_cmd        = 0x10,
    
};

class tr_nodelet_main : public nodelet::Nodelet
  {
  public:
    virtual void onInit();


  private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void control_timer_callback(const ros::TimerEvent &event);

    void ShotPower_PosCallback(const std_msgs::Float32::ConstPtr& msg);
    void ShotAngle_PosCallback(const std_msgs::Float32::ConstPtr& msg);

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
  	ros::NodeHandle nh_MT;

    ros::Subscriber joy_sub;
	ros::Subscriber ShotPower_Pos_sub;
	ros::Subscriber ShotAngle_Pos_sub;
    
    std_msgs::UInt8 act_conf_cmd_msg;

    ros::Publisher Shot_Power_Cmd_pub;
    ros::Publisher Shot_Power_Pos_pub;
    std_msgs::Float64 shot_power_pos_msg;

    ros::Publisher Shot_Angle_Cmd_pub;
  	ros::Publisher Shot_Angle_Pos_pub;
    std_msgs::Float64 shot_angle_pos_msg;
    
    ros::Publisher Pick_Slide_Cmd_pub;
    ros::Publisher Pick_Slide_Pos_pub;
    std_msgs::Float64 Pick_Slide_pos_msg;

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
   
    void Pick_Slide_Homing();

    void PickSlide_mv_1_load();
    void PickSlide_mv_1_release();
    void PickSlide_mv_2_load();
    void PickSlide_mv_2_release();
    void PickSlide_mv_3_load();
    void PickSlide_mv_3_release();
    void PickSlide_mv_4_load();
    void PickSlide_mv_4_release();
    void PickSlide_mv_5_load();
    void PickSlide_mv_5_release();
    void PickSlide_mv_1_adjust();
    void PickSlide_mv_2_adjust();
    void PickSlide_mv_3_adjust();
    void PickSlide_mv_4_adjust();
    void PickSlide_mv_5_adjust();
    void PickSlide_mv_startzone();
    void PickSlide_mv_avoidlauncher();
    void PickSlide_mv_picking();
    void PickSlide_mv_target(double target);

    void Shot_Power_move_shooterinit();
    void Shot_Power_move_target(double target);
    void Shot_Angle_move_initial();
    void Shot_Angle_move_target(double target);
    void Shot_Angle_move_load_wait();

    void Cyl_hand_1_grab();
    void Cyl_hand_1_release();
    void Cyl_hand_2_grab();
    void Cyl_hand_2_release();
    void Cyl_hand_3_grab();
    void Cyl_hand_3_release();
    void Cyl_hand_4_grab();
    void Cyl_hand_4_release();
    void Cyl_hand_5_grab();
    void Cyl_hand_5_release();
    void Cyl_rotate_hands_push_on();
    void Cyl_rotate_hands_pull_on();
    void Cyl_rotate_hands_push_off();
    void Cyl_rotate_hands_pull_off();
    void Cyl_rotate_hands_load();
    void Cyl_rotate_hands_pick();
    void Cyl_rotate_hands_stop();
    void Cyl_rotate_hands_free();

    void Cyl_arrowadjust_grab();
    void Cyl_arrowadjust_release();

    void Cyl_shooter_grab();
    void Cyl_shooter_release();
    //----------------------------------------
    double PickSlide_startzone;
    double PickSlide_picking;
    double PickSlide_avoidlauncher;
    double PickSlide_1_load;
    double PickSlide_1_release;
    double PickSlide_2_load;
    double PickSlide_2_release;
    double PickSlide_3_load;
    double PickSlide_3_release;
    double PickSlide_4_load;
    double PickSlide_4_release;
    double PickSlide_5_load;
    double PickSlide_5_release;

    double PickSlide_1_adjust;
    double PickSlide_2_adjust;
    double PickSlide_3_adjust;
    double PickSlide_4_adjust;
    double PickSlide_5_adjust;

    double shot_power_shooter_init;
    double shot_power_load;
    double shot_power_load_wait;
    double shot_power_launch_pos_1;
    double shot_power_launch_pos_2;
    double shot_power_launch_pos_3;
    double shot_power_launch_pos_4;
    double shot_power_launch_pos_5;
    double shot_angle_launch_pos_1;
    double shot_angle_launch_pos_2;
    double shot_angle_launch_pos_3;
    double shot_angle_launch_pos_4;
    double shot_angle_launch_pos_5;
    double shot_angle_avoid_loader;
    double shot_angle_avoid_loader_arrow;
    double shot_angle_load_wait;
    double shot_angle_load;

    double shot_angle_1_load;
    double shot_angle_2_load;
    double shot_angle_3_load;
    double shot_angle_4_load;
    double shot_angle_5_load;
    double shot_angle_load_adjust;

    double shot_power_launch_pos_1_wall;
    double shot_power_launch_pos_2_wall;
    double shot_power_launch_pos_3_wall;
    double shot_power_launch_pos_4_wall;
    double shot_angle_launch_pos_1_wall;
    double shot_angle_launch_pos_2_wall;
    double shot_angle_launch_pos_3_wall;
    double shot_angle_launch_pos_4_wall;

    double shot_power_launch_pos_1_sz;
    double shot_power_launch_pos_2_sz;
    double shot_power_launch_pos_3_sz;
    double shot_power_launch_pos_4_sz;
    double shot_power_launch_pos_5_sz;
    double shot_angle_launch_pos_1_sz;
    double shot_angle_launch_pos_2_sz;
    double shot_angle_launch_pos_3_sz;
    double shot_angle_launch_pos_4_sz;
    double shot_angle_launch_pos_5_sz;

    int Pick_mode = -1;
    int Load_mode = -1;

    int Load_position = 0;
    bool _load_end = false;

    int Load_hand = 0;

    bool _loaded = false;
    double Load_Height_adjust = 0.0;
    double Load_Angle_adjust = 0.0;

    bool _shooter_pos_load = false;

    double shot_power_adjust = 0.0;

    double shot_power_position_observed = 0.0;
    double shot_angle_position_observed = 0.0;

    double shot_power_allowableerror = 0.0;
    double shot_angle_allowableerror = 0.0;
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
    static const std::vector<ControllerCommands> Shot_and_Load_commands;
    static const std::vector<ControllerCommands> Shot_and_Load_Fast_commands;
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
        ControllerCommands::angle_load_wait,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::shooter_grab,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::shooter_move_load_wait,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::riseflag_shooter_pos_load,
    }
);

const std::vector<ControllerCommands> tr_nodelet_main::Shot_and_Load_commands(
    {
        ControllerCommands::shooter_release,
        //
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        //
        ControllerCommands::pick_slide_adjust,
        ControllerCommands::shooter_init,
        ControllerCommands::angle_load_wait,
        //
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        //
        ControllerCommands::pick_cyl_release,
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        ControllerCommands::pick_cyl_grab,
        //
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        //
        ControllerCommands::shooter_grab,
        //
        //ControllerCommands::set_delay_100ms,
        //ControllerCommands::delay,
        //
        ControllerCommands::pick_cyl_release,
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        ControllerCommands::pick_cyl_grab,
        //
        ControllerCommands::pick_slide_now_hand,
        ControllerCommands::shooter_move_load_wait,
        //
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        //
        ControllerCommands::angle_load,
        ControllerCommands::arrowadjust_grab,
        //
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        //
        ControllerCommands::shooter_move_load,
        //
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        //
        ControllerCommands::arrowadjust_release,
        ControllerCommands::pick_cyl_release,
        ControllerCommands::pick_slide_release,
        //
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        //
        ControllerCommands::angle_avoid_loader,
        //
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        //
        ControllerCommands::pick_slide_next_adjust,
    }
);

const std::vector<ControllerCommands> tr_nodelet_main::Shot_and_Load_Fast_commands(
    {
        ControllerCommands::shooter_release,
        //
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        //
        ControllerCommands::pick_slide_adjust,
        ControllerCommands::shooter_init,
        ControllerCommands::angle_load_wait,
        //
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        //ControllerCommands::set_delay_500ms,
        //ControllerCommands::delay,
        //
        ControllerCommands::pick_cyl_release,
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        ControllerCommands::pick_cyl_grab,
        //
        ControllerCommands::shooter_init_reachcheck,
        //
        ControllerCommands::shooter_grab,
        ControllerCommands::shooter_move_load_wait,
        //
        //ControllerCommands::set_delay_100ms,
        //ControllerCommands::delay,
        //
        ControllerCommands::pick_cyl_release,
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        ControllerCommands::pick_cyl_grab,
        //
        ControllerCommands::pick_slide_now_hand,
        //
        ControllerCommands::shooter_load_wait_reachcheck,
        //
        ControllerCommands::angle_load,
        ControllerCommands::arrowadjust_grab,
        //
        ControllerCommands::angle_load_reachcheck,
        //
        ControllerCommands::shooter_move_load,
        //
        ControllerCommands::shooter_load_reachcheck,
        //
        ControllerCommands::arrowadjust_release,
        ControllerCommands::pick_cyl_release,
        ControllerCommands::pick_slide_release,
        //
        ControllerCommands::set_delay_100ms,
        ControllerCommands::delay,
        //
        ControllerCommands::angle_avoid_loader,
        //
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        //
        ControllerCommands::pick_slide_next_adjust,
    }
);

void tr_nodelet_main::onInit(){
    nh = getNodeHandle();
    nh_MT = getMTNodeHandle();
    //constructor
    _nh = getPrivateNodeHandle();

    this->ShotPower_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor4_current_val", 10, &tr_nodelet_main::ShotPower_PosCallback, this);
    this->ShotAngle_Pos_sub = nh_MT.subscribe<std_msgs::Float32>("motor5_current_val", 10, &tr_nodelet_main::ShotAngle_PosCallback, this);

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
    _nh.param("PickSlide_startzone", this->PickSlide_startzone, 0.0);
    _nh.param("PickSlide_picking", this->PickSlide_picking, 0.0);
    _nh.param("PickSlide_avoidlauncher", this->PickSlide_avoidlauncher, 0.0);
    _nh.param("PickSlide_1_load", this->PickSlide_1_load, 0.0);
    _nh.param("PickSlide_1_release", this->PickSlide_1_release, 0.0);
    _nh.param("PickSlide_2_load", this->PickSlide_2_load, 0.0);
    _nh.param("PickSlide_2_release", this->PickSlide_2_release, 0.0);
    _nh.param("PickSlide_3_load", this->PickSlide_3_load, 0.0);
    _nh.param("PickSlide_3_release", this->PickSlide_3_release, 0.0);
    _nh.param("PickSlide_4_load", this->PickSlide_4_load, 0.0);
    _nh.param("PickSlide_4_release", this->PickSlide_4_release, 0.0);
    _nh.param("PickSlide_5_load", this->PickSlide_5_load, 0.0);
    _nh.param("PickSlide_5_release", this->PickSlide_5_release, 0.0);

    _nh.param("PickSlide_1_adjust", this->PickSlide_1_adjust, 0.0);
    _nh.param("PickSlide_2_adjust", this->PickSlide_2_adjust, 0.0);
    _nh.param("PickSlide_3_adjust", this->PickSlide_3_adjust, 0.0);
    _nh.param("PickSlide_4_adjust", this->PickSlide_4_adjust, 0.0);
    _nh.param("PickSlide_5_adjust", this->PickSlide_5_adjust, 0.0);

    _nh.param("shot_power_shooter_init", this->shot_power_shooter_init, 0.0);
    _nh.param("shot_power_load", this->shot_power_load, 0.0);
    _nh.param("shot_power_load_wait", this->shot_power_load_wait, 0.0);
    _nh.param("shot_power_launch_pos_1", this->shot_power_launch_pos_1, 0.0);
    _nh.param("shot_power_launch_pos_2", this->shot_power_launch_pos_2, 0.0);
    _nh.param("shot_power_launch_pos_3", this->shot_power_launch_pos_3, 0.0);
    _nh.param("shot_power_launch_pos_4", this->shot_power_launch_pos_4, 0.0);
    _nh.param("shot_power_launch_pos_5", this->shot_power_launch_pos_5, 0.0);
    _nh.param("shot_angle_launch_pos_1", this->shot_angle_launch_pos_1, 0.0);
    _nh.param("shot_angle_launch_pos_2", this->shot_angle_launch_pos_2, 0.0);
    _nh.param("shot_angle_launch_pos_3", this->shot_angle_launch_pos_3, 0.0);
    _nh.param("shot_angle_launch_pos_4", this->shot_angle_launch_pos_4, 0.0);
    _nh.param("shot_angle_launch_pos_5", this->shot_angle_launch_pos_5, 0.0);
    _nh.param("shot_angle_avoid_loader", this->shot_angle_avoid_loader, 0.0);
    _nh.param("shot_angle_avoid_loader_arrow", this->shot_angle_avoid_loader_arrow, 0.0);
    _nh.param("shot_angle_load_wait", this->shot_angle_load_wait, 0.0);
    _nh.param("shot_angle_load", this->shot_angle_load, 0.0);

    _nh.param("shot_angle_1_load", this->shot_angle_1_load, 0.0);
    _nh.param("shot_angle_2_load", this->shot_angle_2_load, 0.0);
    _nh.param("shot_angle_3_load", this->shot_angle_3_load, 0.0);
    _nh.param("shot_angle_4_load", this->shot_angle_4_load, 0.0);
    _nh.param("shot_angle_5_load", this->shot_angle_5_load, 0.0);
    _nh.param("shot_angle_load_adjust", this->shot_angle_load_adjust, 0.0);

    _nh.param("shot_power_allowableerror", this->shot_power_allowableerror, 0.0);
    _nh.param("shot_angle_allowableerror", this->shot_angle_allowableerror, 0.0);

    _nh.param("shot_power_launch_pos_1_wall", this->shot_power_launch_pos_1_wall, 0.0);
    _nh.param("shot_power_launch_pos_2_wall", this->shot_power_launch_pos_2_wall, 0.0);
    _nh.param("shot_power_launch_pos_3_wall", this->shot_power_launch_pos_3_wall, 0.0);
    _nh.param("shot_power_launch_pos_4_wall", this->shot_power_launch_pos_4_wall, 0.0);
    _nh.param("shot_angle_launch_pos_1_wall", this->shot_angle_launch_pos_1_wall, 0.0);
    _nh.param("shot_angle_launch_pos_2_wall", this->shot_angle_launch_pos_2_wall, 0.0);
    _nh.param("shot_angle_launch_pos_3_wall", this->shot_angle_launch_pos_3_wall, 0.0);
    _nh.param("shot_angle_launch_pos_4_wall", this->shot_angle_launch_pos_4_wall, 0.0);

    _nh.param("shot_power_launch_pos_1_sz", this->shot_power_launch_pos_1_sz, 0.0);
    _nh.param("shot_power_launch_pos_2_sz", this->shot_power_launch_pos_2_sz, 0.0);
    _nh.param("shot_power_launch_pos_3_sz", this->shot_power_launch_pos_3_sz, 0.0);
    _nh.param("shot_power_launch_pos_4_sz", this->shot_power_launch_pos_4_sz, 0.0);
    _nh.param("shot_power_launch_pos_5_sz", this->shot_power_launch_pos_5_sz, 0.0);
    _nh.param("shot_angle_launch_pos_1_sz", this->shot_angle_launch_pos_1_sz, 0.0);
    _nh.param("shot_angle_launch_pos_2_sz", this->shot_angle_launch_pos_2_sz, 0.0);
    _nh.param("shot_angle_launch_pos_3_sz", this->shot_angle_launch_pos_3_sz, 0.0);
    _nh.param("shot_angle_launch_pos_4_sz", this->shot_angle_launch_pos_4_sz, 0.0);
    _nh.param("shot_angle_launch_pos_5_sz", this->shot_angle_launch_pos_5_sz, 0.0);

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

    this->Pick_Slide_Cmd_pub = nh.advertise<std_msgs::UInt8>("Pick_Slide_cmd", 1);
    this->Pick_Slide_Pos_pub = nh.advertise<std_msgs::Float64>("Pick_Slide_cmd_pos", 1);
  
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

void tr_nodelet_main::ShotPower_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->shot_power_position_observed = msg->data;
}

void tr_nodelet_main::ShotAngle_PosCallback(const std_msgs::Float32::ConstPtr& msg)
{
	this->shot_angle_position_observed = msg->data;
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
    static bool _Cyl_hand_1 = false;
    static bool _Cyl_hand_2 = false;
    static bool _Cyl_hand_3 = false;
    static bool _Cyl_hand_4 = false;
    static bool _Cyl_hand_5 = false;
    static bool _loading = false;
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
        _loaded = false;
        _shooter_pos_load = false;
        Pick_mode = -1;
        Load_mode = -1;
        _loading = false;
        Load_hand = 0;
        Load_position = 1;
        shot_power_adjust = 0.0;
        _load_end = false;
    }
    if (_back)
    {
        this->shutdown();
    }
    if(!this->_command_ongoing)
    {   
        if(_rb){
        }
        if(_lb){
        }
        if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0) && _rb){
            //Cyl_hand_1_grab();
            //Cyl_hand_2_grab();
            //Cyl_hand_3_grab();
            //Cyl_hand_4_grab();
            //Cyl_hand_5_grab();
        }
        if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0) && _lb){
            //Cyl_hand_1_release();
            //Cyl_hand_2_release();
            //Cyl_hand_3_release();
            //Cyl_hand_4_release();
            //Cyl_hand_5_release();
        }
        //------------------------------------------------------------
        if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0) && _righttrigger){
            Shot_Angle_move_target(100);
            Cyl_rotate_hands_free();
            Pick_Slide_Homing();
        }else if((joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0) && _righttrigger){
            Shot_Angle_Homing();
            Shot_Power_Homing();
        }

        if(/*!_loading &&*/ _lefttrigger){
            if(_load_end){
                Cyl_shooter_release();
                Load_position = 1;
                PickSlide_mv_picking();
            }else{
                this->command_list = &Shot_and_Load_Fast_commands;
                _command_ongoing = true;
                if(Load_position == 5){
                    _load_end = true;
                }
            }
            //_loaded = false;
            //_shooter_pos_load = false;
        }
        if(_y){
            if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Cyl_rotate_hands_load();
            }else if((joy->buttons[ButtonRightThumb] != 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Cyl_rotate_hands_stop();
            }else if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] != 1.0)){
                Cyl_rotate_hands_free();
            }else{
                Cyl_rotate_hands_pick();
            }
            _y_enable = false;
        }else{
            _y_enable = true;
        }

        if(_a){
            if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Cyl_hand_1_grab();
                Cyl_hand_2_grab();
                Cyl_hand_3_grab();
                Cyl_hand_4_grab();
                Cyl_hand_5_grab();
            }else if(_rb && _lb){
                Cyl_hand_1_release();
                Cyl_hand_2_release();
                Cyl_hand_3_release();
                Cyl_hand_4_release();
                Cyl_hand_5_release();
            }else if((joy->buttons[ButtonRightThumb] == 1.0)){
                if(_Cyl_hand_5){
                    Cyl_hand_5_grab();
                    _Cyl_hand_5 = false;
                }else{
                    Cyl_hand_5_release();
                    _Cyl_hand_5 = true;
                }
            }else if((joy->buttons[ButtonLeftThumb] == 1.0)){
                if(_Cyl_hand_1){
                    Cyl_hand_1_grab();
                    _Cyl_hand_1 = false;
                }else{
                    Cyl_hand_1_release();
                    _Cyl_hand_1 = true;
                }
            }else if(_lb){
                if(_Cyl_hand_2){
                    Cyl_hand_2_grab();
                    _Cyl_hand_2 = false;
                }else{
                    Cyl_hand_2_release();
                    _Cyl_hand_2 = true;
                }
            }else if(_rb){
                if(_Cyl_hand_4){
                    Cyl_hand_4_grab();
                    _Cyl_hand_4 = false;
                }else{
                    Cyl_hand_4_release();
                    _Cyl_hand_4 = true;
                }
            }else{
                if(_Cyl_hand_3){
                    Cyl_hand_3_grab();
                    _Cyl_hand_3 = false;
                }else{
                    Cyl_hand_3_release();
                    _Cyl_hand_3 = true;
                }
            }
            _a_enable = false;
        }else{
            _a_enable = true;
        }

        //if(_loaded && !_loading ){
            //shot_power_adjust += joy->buttons[ButtonLeftThumb] * 2;
            //shot_power_adjust -= joy->buttons[ButtonRightThumb] * 2;
        if(/*(joy->buttons[ButtonRightThumb] == 0.0) &&*/ (joy->buttons[ButtonLeftThumb] == 0.0)){
            if(_lb){
                if(_pady == 1){
                    Shot_Power_move_target(shot_power_launch_pos_2);
                    Shot_Angle_move_target(shot_angle_launch_pos_2);
                }else if(_padx == 1 || _padx == -1){
                    Shot_Power_move_target(shot_power_launch_pos_3);
                    Shot_Angle_move_target(shot_angle_launch_pos_3);
                }else if(_pady == -1){
                    Shot_Power_move_target(shot_power_launch_pos_4);
                    Shot_Angle_move_target(shot_angle_launch_pos_4);
                }
            }else{
                if(_padx == 1){
                    Shot_Power_move_target(shot_power_launch_pos_1);
                    Shot_Angle_move_target(shot_angle_launch_pos_1);
                }else if(_padx == -1){
                    Shot_Power_move_target(shot_power_launch_pos_5);
                    Shot_Angle_move_target(shot_angle_launch_pos_5);
                }
            }
        }else if(/*(joy->buttons[ButtonRightThumb] == 1.0) && */(joy->buttons[ButtonLeftThumb] == 1.0)){
            if(_lb && _rb){
                if(_pady == 1){
                    Shot_Power_move_target(shot_power_shooter_init);
                }else if(_pady == -1){
                    Cyl_shooter_release();
                }
            }else if(_lb){
                if(_pady == 1){
                    Shot_Power_move_target(shot_power_launch_pos_2_sz);
                    Shot_Angle_move_target(shot_angle_launch_pos_2_sz);
                }else if(_padx == 1 || _padx == -1){
                    Shot_Power_move_target(shot_power_launch_pos_3_sz);
                    Shot_Angle_move_target(shot_angle_launch_pos_3_sz);
                }else if(_pady == -1){
                    Shot_Power_move_target(shot_power_launch_pos_4_sz);
                    Shot_Angle_move_target(shot_angle_launch_pos_4_sz);
                }
            }else{
                if(_padx == 1){
                    Shot_Power_move_target(shot_power_launch_pos_1_sz);
                    Shot_Angle_move_target(shot_angle_launch_pos_1_sz);
                }else if(_padx == -1){
                    Shot_Power_move_target(shot_power_launch_pos_5_sz);
                    Shot_Angle_move_target(shot_angle_launch_pos_5_sz);
                }
            }
        }
        //if(_pady == 1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_1);
        //    Shot_Angle_move_target(shot_angle_launch_pos_1);
        //}else if(_pady == 1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_1_wall);
        //    Shot_Angle_move_target(shot_angle_launch_pos_1_wall);
        //}
        //if(_padx == 1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_2);
        //    Shot_Angle_move_target(shot_angle_launch_pos_2);
        //}else if(_padx == 1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_2_wall);
        //    Shot_Angle_move_target(shot_angle_launch_pos_2_wall);
        //}
        //if(_padx == -1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_3);
        //    Shot_Angle_move_target(shot_angle_launch_pos_3);
        //}else if(_padx == -1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_3_wall);
        //    Shot_Angle_move_target(shot_angle_launch_pos_3_wall);
        //}
        //if(_pady == -1 && (joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_4);
        //    Shot_Angle_move_target(shot_angle_launch_pos_4);
        //}else if(_pady == -1 && (joy->buttons[ButtonRightThumb] == 0.0) && (joy->buttons[ButtonLeftThumb] == 0.0)){
        //    Shot_Power_move_target(shot_power_launch_pos_4_wall);
        //    Shot_Angle_move_target(shot_angle_launch_pos_4_wall);
        //}
            //_shooter_pos_load = false;
        //}

        if(_x){
            _load_end = false;
            Pick_mode = -1;
            shot_power_adjust = 0.0;
            if((joy->buttons[ButtonLeftThumb] == 1.0)){
                Load_position = 1;
            }else if(_lb){
                Load_position = 2;
            }else if(_rb){
                Load_position = 4;
            }else if((joy->buttons[ButtonRightThumb] == 1.0)){
                Load_position = 5;
            }else{
                Load_position = 3;
            }
            switch (Load_position)
            {
            case 1:
                PickSlide_mv_1_adjust();
                break;

            case 2:
                PickSlide_mv_2_adjust();
                break;

            case 3:
                PickSlide_mv_3_adjust();
                break;

            case 4:
                PickSlide_mv_4_adjust();
                break;

            case 5:
                PickSlide_mv_5_adjust();
                break;
            }
        }

        /*
        if(!_loaded && _shooter_pos_load && (_lb || _rb || _x || (joy->buttons[ButtonRightThumb] != 0.0) || (joy->buttons[ButtonLeftThumb] != 0.0))){
            if(_x){
                Load_Height_adjust = 0.0;
                Load_Angle_adjust = 0.0;
                Pick_mode = -1;
                shot_power_adjust = 0.0;
                if(!_loading){
                    if((joy->buttons[ButtonLeftThumb] == 1.0)){
                        Load_hand = 1;
                    }else if(_lb){
                        Load_hand = 2;
                    }else if(_rb){
                        Load_hand = 4;
                    }else if((joy->buttons[ButtonRightThumb] == 1.0)){
                        Load_hand = 5;
                    }else{
                        Load_hand = 3;
                    }
                }
                if(Load_mode == 4){
                    Load_mode = 0;
                }else{
                    Load_mode++;
                }
                _loading = true;
                _x_enable = false;
            }else{
                _x_enable = true;
            }
            if(_loading){
                switch (Load_mode)
                {
                case 0:
                    switch (Load_hand)
                    {
                    case 0:
                        break;
                    case 1:
                        PickSlide_mv_1_load();
                        break;
                    case 2:
                        PickSlide_mv_2_load();
                        break;
                    case 3:
                        PickSlide_mv_3_load();
                        break;
                    case 4:
                        PickSlide_mv_4_load();
                        break;
                    case 5:
                        PickSlide_mv_5_load();
                        break;
                    }
                    Shot_Angle_move_load_wait();
                    break;
                case 1:
                    //Shot_Angle_move_initial();
                    Load_Height_adjust += joy->buttons[ButtonLeftThumb] * 1;
                    Load_Height_adjust -= joy->buttons[ButtonRightThumb] * 1;
                    if(_lb){
                        Load_Angle_adjust += 1;
                    }
                    if(_rb){
                        Load_Angle_adjust -= 1;
                    }
                    Shot_Power_move_target(this->shot_power_load_wait + Load_Height_adjust);
                    switch (Load_hand)
                    {
                    case 0:
                        break;
                    case 1:
                        Shot_Angle_move_target(this->shot_angle_1_load + Load_Angle_adjust);
                        break;
                    case 2:
                        Shot_Angle_move_target(this->shot_angle_2_load + Load_Angle_adjust);
                        break;
                    case 3:
                        Shot_Angle_move_target(this->shot_angle_3_load + Load_Angle_adjust);
                        break;
                    case 4:
                        Shot_Angle_move_target(this->shot_angle_4_load + Load_Angle_adjust);
                        break;
                    case 5:
                        Shot_Angle_move_target(this->shot_angle_5_load + Load_Angle_adjust);
                        break;
                    }
                    break;
                case 2:
                    switch (Load_hand)
                    {
                    case 0:
                        break;
                    case 1:
                        Cyl_hand_1_release();
                        PickSlide_mv_1_release();
                        break;
                    case 2:
                        Cyl_hand_2_release();
                        PickSlide_mv_2_release();
                        break;
                    case 3:
                        Cyl_hand_3_release();
                        PickSlide_mv_3_release();
                        break;
                    case 4:
                        Cyl_hand_4_release();
                        PickSlide_mv_4_release();
                        break;
                    case 5:
                        Cyl_hand_5_release();
                        PickSlide_mv_5_release();
                        break;
                    }
                    break;
                case 3:
                    Shot_Angle_move_target(this->shot_angle_avoid_loader_arrow);
                    break;
                case 4:
                    PickSlide_mv_avoidlauncher();
                    Load_Height_adjust = 0.0;
                    Load_Angle_adjust = 0.0;
                    _loading = false;
                    _loaded = true;
                    shot_power_adjust = 0.0;
                    Load_hand = 0;
                    break;
                }
                //if(_lb){
                //    switch (Load_hand)
                //    {
                //    case 0:
                //        break;
                //    case 1:
                //        Cyl_hand_1_release();
                //        break;
                //    case 2:
                //        Cyl_hand_2_release();
                //        break;
                //    case 3:
                //        Cyl_hand_3_release();
                //        break;
                //    case 4:
                //        Cyl_hand_4_release();
                //        break;
                //    case 5:
                //        Cyl_hand_5_release();
                //        break;
                //    }
                //}else if(_rb){
                //    switch (Load_hand)
                //    {
                //    case 0:
                //        break;
                //    case 1:
                //        Cyl_hand_1_grab();
                //        break;
                //    case 2:
                //        Cyl_hand_2_grab();
                //        break;
                //    case 3:
                //        Cyl_hand_3_grab();
                //        break;
                //    case 4:
                //        Cyl_hand_4_grab();
                //        break;
                //    case 5:
                //        Cyl_hand_5_grab();
                //        break;
                //    }
                //}
            }
        }
        */

        if(/*!_loading &&*/ (_b) && _b_enable){
            _load_end = false;
            Load_mode = -1;
            if(_b){           
                if(Pick_mode == 3){
                    Pick_mode = 0;
                }else{
                    Pick_mode++;
                }
                _b_enable = false;
            }else{
                _b_enable = true;
            }

            if((joy->buttons[ButtonRightThumb] == 1.0) && (joy->buttons[ButtonLeftThumb] == 1.0)){
                Cyl_rotate_hands_load();
                PickSlide_mv_startzone();
                Load_position = 3;
                Pick_mode = -1;
            }else{
                switch (Pick_mode)
                {
                case 0:
                    Cyl_rotate_hands_pick();
                    PickSlide_mv_picking();
                    break;

                case 1:
                    Cyl_hand_1_grab();
                    Cyl_hand_2_grab();
                    Cyl_hand_3_grab();
                    Cyl_hand_4_grab();
                    Cyl_hand_5_grab();
                    break;

                case 2:
                    PickSlide_mv_1_adjust();
                    Cyl_rotate_hands_load();
                    Load_position = 1;
                    break;

                case 3:
                    PickSlide_mv_1_adjust();
                    Cyl_rotate_hands_load();
                    Load_position = 1;
                    break;
                }
            }
        }else if(!_b){
            _b_enable = true;
        }
        //------------------------------------------------------------

    }


    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisLeftThumbX];   
        double vel_y = joy->axes[AxisLeftThumbY];
        double vel_yaw = joy->axes[AxisRightThumbX];//vel_yaw_l + vel_yaw_r;
        double vel_norm = hypot(vel_x, vel_y);


        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }

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
    Pick_Slide_Cmd_pub.publish(act_conf_cmd_msg);
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
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    Shot_Angle_Cmd_pub.publish(act_conf_cmd_msg);
    Pick_Slide_Cmd_pub.publish(act_conf_cmd_msg);
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

void tr_nodelet_main::Pick_Slide_Homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    Pick_Slide_Cmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    Pick_Slide_Cmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::PickSlide_mv_startzone(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_startzone;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_picking(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_picking;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_avoidlauncher(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_avoidlauncher;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_1_load(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_1_load;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_1_release(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_1_release;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_2_load(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_2_load;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_2_release(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_2_release;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_3_load(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_3_load;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_3_release(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_3_release;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_4_load(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_4_load;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_4_release(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_4_release;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_5_load(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_5_load;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_5_release(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_5_release;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_1_adjust(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_1_adjust;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_2_adjust(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_2_adjust;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_3_adjust(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_3_adjust;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_4_adjust(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_4_adjust;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_5_adjust(void){
    this->Pick_Slide_pos_msg.data = this->PickSlide_5_adjust;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
}

void tr_nodelet_main::PickSlide_mv_target(double target){
    this->Pick_Slide_pos_msg.data = target;
    this->Pick_Slide_Pos_pub.publish(this->Pick_Slide_pos_msg);
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

void tr_nodelet_main::Shot_Angle_move_load_wait(void){
    this->shot_angle_pos_msg.data = this->shot_angle_load_wait;
    this->Shot_Angle_Pos_pub.publish(this->shot_angle_pos_msg);
}

void tr_nodelet_main::Cyl_hand_1_grab(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::hand_1_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_1_release(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::hand_1_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_2_grab(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::hand_2_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_2_release(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::hand_2_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_3_grab(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::hand_3_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_3_release(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::hand_3_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_4_grab(void){
    this->lastSolenoid_2_Order |= (uint8_t)SolenoidValveCommands::hand_4_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_4_release(void){
    this->lastSolenoid_2_Order &= ~(uint8_t)SolenoidValveCommands::hand_4_cmd;
    this->solenoid2_order_msg.data = this->lastSolenoid_2_Order;
    this->Solenoid2_Order_pub.publish(this->solenoid2_order_msg);
}

void tr_nodelet_main::Cyl_hand_5_grab(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::hand_5_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_hand_5_release(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::hand_5_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_arrowadjust_grab(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::arrowadjust_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_arrowadjust_release(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::arrowadjust_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_rotate_hands_push_on(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::rotate_hands_push_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_rotate_hands_push_off(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::rotate_hands_push_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_rotate_hands_pull_on(void){
    this->lastSolenoid_1_Order |= (uint8_t)SolenoidValveCommands::rotate_hands_pull_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
}

void tr_nodelet_main::Cyl_rotate_hands_pull_off(void){
    this->lastSolenoid_1_Order &= ~(uint8_t)SolenoidValveCommands::rotate_hands_pull_cmd;
    this->solenoid1_order_msg.data = this->lastSolenoid_1_Order;
    this->Solenoid1_Order_pub.publish(this->solenoid1_order_msg);
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

void tr_nodelet_main::Cyl_rotate_hands_load(void){
    Cyl_rotate_hands_push_on();
    Cyl_rotate_hands_pull_off();
}

void tr_nodelet_main::Cyl_rotate_hands_pick(void){
    Cyl_rotate_hands_push_off();
    Cyl_rotate_hands_pull_on();
}

void tr_nodelet_main::Cyl_rotate_hands_free(void){
    Cyl_rotate_hands_push_on();
    Cyl_rotate_hands_pull_on();
}

void tr_nodelet_main::Cyl_rotate_hands_stop(void){
    Cyl_rotate_hands_push_off();
    Cyl_rotate_hands_pull_off();
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
    else if (currentCommand == ControllerCommands::set_delay_100ms)
    {
        set_delay(0.100);
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
    else if (currentCommand == ControllerCommands::shooter_move_load_wait)
    {
        this->Shot_Power_move_target(this->shot_power_load_wait);
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
    else if (currentCommand == ControllerCommands::angle_load_wait)
    {
        this->Shot_Angle_move_target(this->shot_angle_load_wait);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::angle_avoid_loader)
    {
        this->Shot_Angle_move_target(this->shot_angle_avoid_loader);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::angle_load)
    {
        switch (Load_position)
        {
        case 1:
            Shot_Angle_move_target(this->shot_angle_1_load + this->shot_angle_load_adjust);
            break;
        
        case 2:
            Shot_Angle_move_target(this->shot_angle_2_load + this->shot_angle_load_adjust);
            break;

        case 3:
            Shot_Angle_move_target(this->shot_angle_3_load + this->shot_angle_load_adjust);
            break;

        case 4:
            Shot_Angle_move_target(this->shot_angle_4_load + this->shot_angle_load_adjust);
            break;

        case 5:
            Shot_Angle_move_target(this->shot_angle_5_load + this->shot_angle_load_adjust);
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_slide_next_load)
    {
        if(Load_position==5){
            Load_position = 1;
        }else{
            Load_position++;
        }
        switch (Load_position)
        {
        case 1:
            PickSlide_mv_1_load();
            break;
        
        case 2:
            PickSlide_mv_2_load();
            break;

        case 3:
            PickSlide_mv_3_load();
            break;

        case 4:
            PickSlide_mv_4_load();
            break;

        case 5:
            PickSlide_mv_5_load();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_slide_next_adjust)
    {
        if(Load_position==5){
            Load_position = 1;
        }else{
            Load_position++;
        }
        switch (Load_position)
        {
        case 1:
            PickSlide_mv_1_adjust();
            break;
        
        case 2:
            PickSlide_mv_2_adjust();
            break;

        case 3:
            PickSlide_mv_3_adjust();
            break;

        case 4:
            PickSlide_mv_4_adjust();
            break;

        case 5:
            PickSlide_mv_5_adjust();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_slide_now_hand)
    {
        switch (Load_position)
        {
        case 1:
            PickSlide_mv_1_load();
            break;
        
        case 2:
            PickSlide_mv_2_load();
            break;

        case 3:
            PickSlide_mv_3_load();
            break;

        case 4:
            PickSlide_mv_4_load();
            break;

        case 5:
            PickSlide_mv_5_load();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_slide_release)
    {
        switch (Load_position)
        {
        case 1:
            PickSlide_mv_1_release();
            break;
        
        case 2:
            PickSlide_mv_2_release();
            break;

        case 3:
            PickSlide_mv_3_release();
            break;

        case 4:
            PickSlide_mv_4_release();
            break;

        case 5:
            PickSlide_mv_5_release();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_slide_adjust)
    {
        switch (Load_position)
        {
        case 1:
            PickSlide_mv_1_adjust();
            break;
        
        case 2:
            PickSlide_mv_2_adjust();
            break;

        case 3:
            PickSlide_mv_3_adjust();
            break;

        case 4:
            PickSlide_mv_4_adjust();
            break;

        case 5:
            PickSlide_mv_5_adjust();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_cyl_release)
    {
        switch (Load_position)
        {
        case 1:
            Cyl_hand_1_release();
            break;
        
        case 2:
            Cyl_hand_2_release();
            break;

        case 3:
            Cyl_hand_3_release();
            break;

        case 4:
            Cyl_hand_4_release();
            break;

        case 5:
            Cyl_hand_5_release();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::pick_cyl_grab)
    {
        switch (Load_position)
        {
        case 1:
            Cyl_hand_1_grab();
            break;
        
        case 2:
            Cyl_hand_2_grab();
            break;

        case 3:
            Cyl_hand_3_grab();
            break;

        case 4:
            Cyl_hand_4_grab();
            break;

        case 5:
            Cyl_hand_5_grab();
            break;
        }
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::arrowadjust_grab)
    {
        Cyl_arrowadjust_grab();
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::arrowadjust_release)
    {
        Cyl_arrowadjust_release();
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::riseflag_shooter_pos_load)
    {
        this->_shooter_pos_load = true;
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::shooter_init_reachcheck)
    {
        if(this->shot_power_position_observed < shot_power_shooter_init + shot_power_allowableerror){
            this->currentCommandIndex++;
        }
    }
    else if (currentCommand == ControllerCommands::shooter_load_reachcheck)
    {
        if(this->shot_power_position_observed < shot_power_load + shot_power_allowableerror){
            this->currentCommandIndex++;
        }
    }
    else if (currentCommand == ControllerCommands::shooter_load_wait_reachcheck)
    {
        if(this->shot_power_position_observed > shot_power_load_wait - shot_power_allowableerror){
            this->currentCommandIndex++;
        }
    }
    else if (currentCommand == ControllerCommands::angle_load_reachcheck)
    {
        if(this->shot_angle_position_observed < shot_angle_1_load + shot_angle_allowableerror){
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
PLUGINLIB_EXPORT_CLASS(tr::tr_nodelet_main, nodelet::Nodelet);