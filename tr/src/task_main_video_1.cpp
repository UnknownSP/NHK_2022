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
    arm_rotate_to_PI,
    arm_rotate_to_0,
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
    grab_launcher,
    release_launcher,
// related to delay
    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,
    delay,
    wait_next_pressed,
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
    shutdown_cmd      = 0b000000,
    recover_cmd         = 0b000001,
    
    launch_cmd         = 0b000010,//default close
    release_arrow_cmd     = 0b010000,//default open
    move_table_cmd     = 0b100000,//default right
    
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

    ros::Publisher LaunchForceConfCmd_pub;
    ros::Publisher LaunchForceCmdPos_pub;
    std_msgs::Float64 launch_force_pos_msg;    
    
    ros::Publisher ArmMoveCmd_pub;
    ros::Publisher ArmMoveCmdPos_pub;
    std_msgs::Float64 arm_move_pos_msg;

    ros::Publisher SolenoidCmd_pub;
    ros::Publisher SolenoidOrder_pub;
    std_msgs::UInt8 solenoid_order_msg;
    uint8_t lastSolenoidOrder = 0b0000000;
  
    ros::Publisher LaunchangleConfCmd_pub;
  	ros::Publisher LaunchanguleCmdPos_pub;
    std_msgs::Float64 launcher_angle_pos_msg;


  	ros::Publisher BaseanguleCmdPos_pub;
    ros::Publisher BaseangleConfCmd_pub;
    std_msgs::Float64 base_angle_pos_msg;

    ros::Publisher ArmangleConfCmd_pub;
  	ros::Publisher ArmanguleCmdPos_pub;
    std_msgs::Float64 arm_angle_pos_msg;

    ros::Publisher act_enable_pub0;
    ros::Publisher act_enable_pub1;
    ros::Publisher act_enable_pub2;
    ros::Publisher act_enable_pub3;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;

    ros::Timer control_timer;

    //action_modules
    void release_launcher();
    void grab_launcher();
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
    void arm_rotate(float rotateangle);
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
    static const std::vector<ControllerCommands> launch_short_test_commands;
    static const std::vector<ControllerCommands> launch_medium_test_commands;
    static const std::vector<ControllerCommands> launch_long_test_commands;
    static const std::vector<ControllerCommands> load_test_commands;
    static const std::vector<ControllerCommands> initial_pose;
    static const std::vector<ControllerCommands> manual_all;
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

const std::vector<ControllerCommands> tr_nodelet_main::launch_short_test_commands(
    {
        ControllerCommands::adjust_launchers_force_short,
        ControllerCommands::adjust_launchers_angular_short,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::release_launcher,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_top,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::adjust_launchers_angular_right,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::grab_launcher,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_bottom,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        
    }
);

const std::vector<ControllerCommands> tr_nodelet_main::launch_medium_test_commands(
    {
        ControllerCommands::adjust_launchers_force_medium,
        ControllerCommands::adjust_launchers_angular_medium,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::release_launcher,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_top,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::adjust_launchers_angular_right,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::grab_launcher,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_bottom,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
    }
);
const std::vector<ControllerCommands> tr_nodelet_main::launch_long_test_commands(
    {
        ControllerCommands::adjust_launchers_force_long,
        ControllerCommands::adjust_launchers_angular_long,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::release_launcher,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_top,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::adjust_launchers_angular_right,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::grab_launcher,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_bottom,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
    }
);

const std::vector<ControllerCommands> tr_nodelet_main::load_test_commands(
    {
        ControllerCommands::arm_rotate_to_PI,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::table_move_to_launcher,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::table_rotate_to_launcher,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::adjust_arm_to_set,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::release_arrow,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::table_move_to_base,
        ControllerCommands::adjust_arm_to_grab,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::table_rotate_to_base,
        ControllerCommands::arm_rotate_to_0,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        
    }
);

const std::vector<ControllerCommands> tr_nodelet_main::initial_pose(

    {
        ControllerCommands::release_arrow,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::table_move_to_base,
        ControllerCommands::set_delay_250ms,
        ControllerCommands::delay,
        ControllerCommands::table_rotate_to_base,
        ControllerCommands::adjust_arm_to_grab,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_bottom,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::release_launcher,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::launcher_move_top,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::grab_launcher,
        ControllerCommands::set_delay_500ms,
        ControllerCommands::delay,
        ControllerCommands::home,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,

    } 

);

const std::vector<ControllerCommands> tr_nodelet_main::manual_all(
    {
        ControllerCommands::manual,
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
    _nh.param("launcher_length", this->launcherlength, 0.0);//(mm)

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

    this->LaunchForceConfCmd_pub = nh.advertise<std_msgs::UInt8>("LaunchForce_cmd", 1);
    this->LaunchForceCmdPos_pub = nh.advertise<std_msgs::Float64>("LaunchForce_cmd_pos", 1);

    this->LaunchangleConfCmd_pub = nh.advertise<std_msgs::UInt8>("Launchangle_cmd", 1);
  	this->LaunchanguleCmdPos_pub = nh.advertise<std_msgs::Float64>("Launchangle_cmd_pos", 1);

    this->SolenoidCmd_pub = nh.advertise<std_msgs::UInt8>("solenoid_cmd", 1);
    this->SolenoidOrder_pub = nh.advertise<std_msgs::UInt8>("solenoid_order", 1);

    this->BaseangleConfCmd_pub = nh.advertise<std_msgs::UInt8>("Baseangle_cmd", 1);
  	this->BaseanguleCmdPos_pub = nh.advertise<std_msgs::Float64>("Baseangle_cmd_pos", 1);

    this->ArmangleConfCmd_pub = nh.advertise<std_msgs::UInt8>("Armangle_cmd", 1);
  	this->ArmanguleCmdPos_pub = nh.advertise<std_msgs::Float64>("Armangle_cmd_pos", 1);
  
    this->ArmMoveCmd_pub = nh.advertise<std_msgs::UInt8>("ArmMove_cmd", 1);
    this->ArmMoveCmdPos_pub = nh.advertise<std_msgs::Float64>("ArmMove_cmd_pos", 1);

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
    }
    if (_back)
    {
        this->shutdown();
    }
    if(!this->_command_ongoing)
    {   
        if(_initial_pose_finished){

            if(!this->_has_loaded)
            {
                if (_y)
                {
                    this->command_list = &load_test_commands;
                    _command_ongoing = true;
                    _has_loaded = true;
                }
            }   
            else if (_a&&(_pady == 1))
            {      
                this->command_list = &launch_long_test_commands;
                _command_ongoing = true;
                _has_loaded = false;
            }
        }

        if (_righttrigger)
        {
            this->command_list = &initial_pose;
            _command_ongoing = true;
            _initial_pose_finished = true;
        }
    }


    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisRightThumbX];   
        double vel_y = joy->axes[AxisRightThumbY];
        double vel_yaw_l = (joy->buttons[ButtonLeftThumb] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw_r = (joy->buttons[ButtonRightThumb] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = vel_yaw_l + vel_yaw_r;
        double vel_norm = hypot(vel_x, vel_y);


        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }

        if (_x && !last_x)
        {
            
            if ((((this->lastSolenoidOrder) >> (4)) & 1) == 1)
            {
                
                this->grab_arrow();
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                this->adjust_arm_to_lift();
                NODELET_INFO("release_arrow");
            }
            else{
                this->adjust_arm_to_grab();
                std::this_thread::sleep_for(std::chrono::milliseconds(750));
                this->release_arrow();
                NODELET_INFO("grab_arrow");
            }
            
        }
        if (_b && !last_b)
        {            
                this->adjust_arm_to_set();
                NODELET_INFO("homing");
        }

        this->cmd_vel_msg.linear.x = -vel_x;
        this->cmd_vel_msg.linear.y = vel_y;
        this->cmd_vel_msg.angular.z = vel_yaw;
        this->cmd_vel_pub.publish(this->cmd_vel_msg);
    }
    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;  
}


void tr_nodelet_main::release_launcher(void){
  this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::launch_cmd;
  this->solenoid_order_msg.data = this->lastSolenoidOrder;
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void tr_nodelet_main::grab_launcher(void){
  this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::launch_cmd;
  this->solenoid_order_msg.data = this->lastSolenoidOrder;
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void tr_nodelet_main::adjust_arm(float movelength){
  this->arm_movelength = movelength;
  this->arm_move_pos_msg.data = movelength;
  this->ArmMoveCmdPos_pub.publish(this->arm_move_pos_msg);
}
void tr_nodelet_main::adjust_arm_to_grab(void){
  this->adjust_arm(9.6);
}
void tr_nodelet_main::adjust_arm_to_set(void){
  this->adjust_arm(6.9);
}
void tr_nodelet_main::adjust_arm_to_lift(void){
  this->adjust_arm(30.5);
}
void tr_nodelet_main::launcher_move(float movelength){
    this->launch_force_pos_msg.data = movelength;
    this->LaunchForceCmdPos_pub.publish(launch_force_pos_msg);
}


void tr_nodelet_main::reset_launcher(void){
  this->grab_launcher();
  this->launcher_move(0);
  this->launcher_angle_pos_msg.data = 0;
  this->LaunchanguleCmdPos_pub.publish(this->launcher_angle_pos_msg);
}

//void tr_nodelet_main::launcher_move(float movelength){
   //double init_radian = movelength*2*pi/(this->toothperrotate*this->lengthperbit); 
   //this->launch_force_pos_msg.data = init_radian;
   //this->LaunchForceCmdPos_pub.publish(this->launch_force_pos_msg);
//}

void tr_nodelet_main::adjust_launcher_force(float pulllength){
   this->launcher_move(this->launcherlength-pulllength);                                       
}

//void tr_nodelet_main::adjust_launcher_radian(float launchangle){
   // calculate the launch angle from the law of cosines
   //float conf_angle = 2*pi*(this->LaunchPartLength*(sqrtf32(pow((this->PropLength/this->LaunchPartLength),2)-pow(sinf32(launchangle),2)))-this->LaunchInitiallength)/this->BallScrewLead;
   //this->launcher_angle_pos_msg.data = conf_angle;
   //this->LaunchanguleCmdPos_pub.publish(this->launcher_angle_pos_msg);
//}

void tr_nodelet_main::reset_launcher_status(void){
   this->act_conf_cmd_msg.data= (uint8_t)SolenoidValveCommands::shutdown_cmd;
   this->SolenoidCmd_pub.publish(this->act_conf_cmd_msg);
   this->act_conf_cmd_msg.data= (uint8_t)SolenoidValveCommands::recover_cmd;
   this->SolenoidCmd_pub.publish(this->act_conf_cmd_msg);
}

void tr_nodelet_main::grab_arrow(void){
  this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::release_arrow_cmd;
  this->solenoid_order_msg.data = this->lastSolenoidOrder;
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void tr_nodelet_main::release_arrow(void){
  this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::release_arrow_cmd;
  this->solenoid_order_msg.data = this->lastSolenoidOrder;
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void tr_nodelet_main::base_rotate(float rotate_angle){ 
  this->base_rotate_angle = rotate_angle; 
  this->base_angle_pos_msg.data = rotate_angle;
  this->BaseanguleCmdPos_pub.publish(this->base_angle_pos_msg);
}

void tr_nodelet_main::table_move_to_launcher(void){
  this->lastSolenoidOrder |= (uint8_t)SolenoidValveCommands::move_table_cmd;
  this->solenoid_order_msg.data = this->lastSolenoidOrder;
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void tr_nodelet_main::table_move_to_base(void){
  this->lastSolenoidOrder &= ~(uint8_t)SolenoidValveCommands::move_table_cmd;
  this->solenoid_order_msg.data = this->lastSolenoidOrder;
  this->SolenoidOrder_pub.publish(this->solenoid_order_msg);
}

void tr_nodelet_main::arm_rotate(float rotate_angle){
  this->arm_rotete_angle = rotate_angle;
  this->arm_angle_pos_msg.data = rotate_angle;
  this->ArmanguleCmdPos_pub.publish(this->arm_angle_pos_msg);
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
    LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
    BaseangleConfCmd_pub.publish(act_conf_cmd_msg);
    ArmangleConfCmd_pub.publish(act_conf_cmd_msg);
    LaunchangleConfCmd_pub.publish(act_conf_cmd_msg);
    ArmMoveCmd_pub.publish(act_conf_cmd_msg);
    act_enable_pub0.publish(act_conf_cmd_msg);
    act_enable_pub1.publish(act_conf_cmd_msg);
    act_enable_pub2.publish(act_conf_cmd_msg);
    act_enable_pub3.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::recover(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::recover_cmd;
    LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
    BaseangleConfCmd_pub.publish(act_conf_cmd_msg);
    ArmangleConfCmd_pub.publish(act_conf_cmd_msg);
    LaunchangleConfCmd_pub.publish(act_conf_cmd_msg);
    ArmMoveCmd_pub.publish(act_conf_cmd_msg);
    act_enable_pub0.publish(act_conf_cmd_msg);
    act_enable_pub1.publish(act_conf_cmd_msg);
    act_enable_pub2.publish(act_conf_cmd_msg);
    act_enable_pub3.publish(act_conf_cmd_msg);
    SolenoidCmd_pub.publish(act_conf_cmd_msg);
}

void tr_nodelet_main::homing(void){
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
    LaunchangleConfCmd_pub.publish(act_conf_cmd_msg);
    act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
    LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
    LaunchangleConfCmd_pub.publish(act_conf_cmd_msg);
}    

void tr_nodelet_main::change_OpMode(void){
    this->OpModecurrentindex++;
    if(OpModecurrentindex >= (int)this->opmode.size()){
        OpModecurrentindex  = 0; 
    }
    this->current_OpMode = opmode.at(OpModecurrentindex);
}

void tr_nodelet_main::control_timer_callback(const ros::TimerEvent &event)
{ 
    //this->command_list->size() <= (int)this->currentCommandIndex || this->command_list == &this->manual_all
    if (!this->_command_ongoing)
    {

        if (!((this->base_rotate_angle > 3.0 && this->_padx > 0) || (this->base_rotate_angle < -3.0 && this->_padx < 0))&&abs(_padx))
        {
            this->base_rotate_angle += this->_padx * 0.025;
            this->base_rotate(this->base_rotate_angle);
        }
        if (abs(_pady))
        {
            this->arm_movelength += this->_pady * 0.087;
            this->adjust_arm(this->arm_movelength);
        }
        if (!((this->arm_rotete_angle > pi && this->arm_angle_plus > 0) || (this->arm_rotete_angle < 0 && this->arm_angle_plus < 0))&&abs(arm_angle_plus))
        {
            this->arm_rotete_angle += this->arm_angle_plus * 0.025;
            this->arm_rotate(this->arm_rotete_angle);
        }
        
        
        NODELET_INFO("control_time_return");
        
    }

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    if(currentCommand == ControllerCommands::home)
    {
        this->homing();
        this->currentCommandIndex++;
        NODELET_INFO("home");
    }
    else if(currentCommand == ControllerCommands::adjust_arm_to_grab)
    {   //arm_movelength = 35;
        this->adjust_arm_to_grab();
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_grab");
    }
    else if(currentCommand == ControllerCommands::adjust_arm_to_set)
    {   //arm_movelength = 35;
        this->adjust_arm_to_set(); 
        this->currentCommandIndex++;
        NODELET_INFO("adjust_arm_to_set");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_force_short)
    {
        this->launch_force_pos_msg.data = -20;
        this->LaunchForceCmdPos_pub.publish(launch_force_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_force_short");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_force_medium)
    {
        this->launch_force_pos_msg.data = -8;
        this->LaunchForceCmdPos_pub.publish(launch_force_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_force_medium");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_force_long)
    {
        this->launch_force_pos_msg.data = 0;
        this->LaunchForceCmdPos_pub.publish(launch_force_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_force_long");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_angular_short)
    {
        this->launcher_angle_pos_msg.data = 80;
        this->LaunchanguleCmdPos_pub.publish(launcher_angle_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_angular_short");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_angular_medium)
    {
        this->launcher_angle_pos_msg.data = 160;
        this->LaunchanguleCmdPos_pub.publish(launcher_angle_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_angular_medium");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_angular_long)
    {
        this->launcher_angle_pos_msg.data = 190;
        this->LaunchanguleCmdPos_pub.publish(launcher_angle_pos_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_angular_long");
    }
    else if(currentCommand == ControllerCommands::adjust_launchers_angular_right)
    {
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
        LaunchangleConfCmd_pub.publish(act_conf_cmd_msg);
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
        LaunchangleConfCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("adjust_launchers_angular_right");
    }
    else if(currentCommand == ControllerCommands::grab_arrow)
    {   
        this->grab_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("grab_arrow");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_PI)
    {   
        this->arm_rotate(pi);
        this->currentCommandIndex++;
        NODELET_INFO("arm_rotate_to_PI");
    }
    else if(currentCommand == ControllerCommands::arm_rotate_to_0)
    {   
        if(!this->_has_table_moved){
        this->arm_rotate(0);
        this->currentCommandIndex++;
        NODELET_INFO("arm_rotate_to_0");
        }

    }
    else if(currentCommand == ControllerCommands::release_arrow)
    {   
        this->release_arrow();
        this->currentCommandIndex++;
        NODELET_INFO("release_arrow");
    }
    else if(currentCommand == ControllerCommands::table_move_to_base)
    {   
        this->table_move_to_base();
        this->currentCommandIndex++;
        NODELET_INFO("table_move_to_base");
    }
    else if(currentCommand == ControllerCommands::table_move_to_launcher)
    {   
        this->table_move_to_launcher();
        this->currentCommandIndex++;
        NODELET_INFO("table_move_to_launcher");
    }
    else if(currentCommand == ControllerCommands::table_rotate_to_launcher)
    {   
        this->base_rotate(-1.592);
        this->currentCommandIndex++;
        NODELET_INFO("table_rotate_to_launcher");
        this->_has_table_rotated = true;
    }
    else if(currentCommand == ControllerCommands::table_rotate_to_base)
    {   
        this->base_rotate(0);
        this->currentCommandIndex++;
        NODELET_INFO("table_rotate_to_base");
        this->_has_table_rotated = false;
    }
    else if(currentCommand == ControllerCommands::launcher_move_top)
    {   
        if(!this->_has_table_rotated && ((this->lastSolenoidOrder & (uint8_t)SolenoidValveCommands::launch_cmd) == (uint8_t)SolenoidValveCommands::launch_cmd)){
        this->launch_movelength=-34.1;
        this->launcher_move(launch_movelength);
        this->currentCommandIndex++;
        NODELET_INFO("launcher_move_top");
        }
    }
    else if(currentCommand == ControllerCommands::launcher_move_bottom)
    {   
        if(!this->_has_table_rotated){
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
        LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
        LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        act_conf_cmd_msg.data = (uint8_t)MotorCommands::homing_cmd;
        LaunchForceConfCmd_pub.publish(act_conf_cmd_msg);
        this->currentCommandIndex++;
        NODELET_INFO("launcher_move_bottom");
        }
    }
    else if(currentCommand == ControllerCommands::grab_launcher)
    {  
        this->grab_launcher();
        this->currentCommandIndex++;
        NODELET_INFO("grab_launcher");
    }
    else if(currentCommand == ControllerCommands::release_launcher)
    {   
        if(!this->_has_table_rotated){
        this->release_launcher();
        this->currentCommandIndex++;
        NODELET_INFO("Launch!!");
        }
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
PLUGINLIB_EXPORT_CLASS(tr::tr_nodelet_main, nodelet::Nodelet);