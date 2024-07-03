#define HYUNDAI_UDP_SERVER
// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>
#include <cmath>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

// setup the initial name
using namespace ros;
using namespace std;

#define DF_UDP_BUFFER_SIZE  128
#define DF_UDP_PORTNUM      60001
#define DF_UDP_SERVER_ADDR  "127.0.0.1" // LOCAL
// #define DF_UDP_SERVER_ADDR  "192.168.0.143" //seung-il notebook ip
// #define DF_UDP_SERVER_ADDR  "192.168.0.130" //IPG PC


#pragma pack (push, 1)
typedef struct TX_message_data {
	struct {
		unsigned char MsgID;
	} Header;
	
    struct {
		double SteeringWheel;   // rad
		double Ax; 				// [m/s^2]
		int GearNo; 			// 1: Driving, 0: Stop, -1: Back, -9: Parking
		char Light; 			//
    } DriveCont;
};
#pragma pack(pop)

class SERVER_TO_SEND
{
    public:
        SERVER_TO_SEND(ros::NodeHandle& n);        
        ~SERVER_TO_SEND();
        void VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg);
        void ThrottleReverseCallback(const std_msgs::Bool& msg);
        void TurningSignalCallback(const std_msgs::Int32MultiArrayConstPtr& msg);
        void ControlTargetCallback(const visualization_msgs::MarkerConstPtr& msg);
        void RouteLaneChangeCallback(const std_msgs::StringConstPtr& msg);
        void MissionStateCallback(const std_msgs::StringConstPtr& msg);
        void LeftTurnCallback(const std_msgs::StringConstPtr& msg);
        

        float steering_angle;
        float acceleration;
        bool ThrotthleReverse;
        geometry_msgs::Pose ControlTarget;
        ros::NodeHandle nh;
        ros::Publisher pubSignalVisual;
        ros::Subscriber subVehCommand, subThrottleReverse;
        ros::Subscriber subTurningSignal;
        ros::Subscriber subControlTarget;
        ros::Subscriber subRouteLaneChange, subMissionState, subLeftTurn;

        char TurningLightMap; // 0: Off, 1: Left indicator, 2: Right indicator, 3: Hazard

        std::string RouteLaneChange, MissionState, LeftTurn;
        
};

SERVER_TO_SEND::SERVER_TO_SEND(ros::NodeHandle& n) : ThrotthleReverse(false), TurningLightMap(0)
{
    subVehCommand = nh.subscribe("/Ackermann/command/joy", 10, &SERVER_TO_SEND::VehCommandCallback, this);
    subThrottleReverse = nh.subscribe("/Bool/throtthle_reverse", 10, &SERVER_TO_SEND::ThrottleReverseCallback, this);
    subTurningSignal = nh.subscribe("/light_signal", 10, &SERVER_TO_SEND::TurningSignalCallback, this);
    subControlTarget = nh.subscribe("/Marker/PurePursuit/Target", 10, &SERVER_TO_SEND::ControlTargetCallback, this);
    subRouteLaneChange = nh.subscribe("/String/RouteLaneChange", 10, &SERVER_TO_SEND::RouteLaneChangeCallback, this);
    subMissionState = nh.subscribe("/decision/mission_state", 10, &SERVER_TO_SEND::MissionStateCallback, this);
    subLeftTurn = nh.subscribe("/String/LeftTurnOwnRisk", 10, &SERVER_TO_SEND::LeftTurnCallback, this);

    pubSignalVisual = nh.advertise<std_msgs::String>("/String/SignalVisual", 1);

    // example_sub = nh.subscribe("/exmaple_topic", 10, &SERVER_TO_SEND::example_callback, this);
    ROS_DEBUG("SERVER_TO_SEND is created");
};

SERVER_TO_SEND::~SERVER_TO_SEND() 
{    
    ROS_INFO("SERVER_TO_SEND destructor.");
}

void SERVER_TO_SEND::VehCommandCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    steering_angle = msg.drive.steering_angle * M_PI / 180 ; //unit: rad
    acceleration = msg.drive.acceleration;
}

void SERVER_TO_SEND::ThrottleReverseCallback(const std_msgs::Bool& msg)
{
    ThrotthleReverse = msg.data;
}

void SERVER_TO_SEND::ControlTargetCallback(const visualization_msgs::MarkerConstPtr& msg)
{
    ControlTarget = msg->pose;
}

void SERVER_TO_SEND::LeftTurnCallback(const std_msgs::StringConstPtr& msg)
{
    LeftTurn = msg->data;
}

void SERVER_TO_SEND::TurningSignalCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{   
    if(!msg->data.empty())
    {
        if(msg->data[0] == 2 &&  msg->data[1] == 0){
            TurningLightMap = 2;
        }else if (msg->data[1] == 2 &&  msg->data[0] == 0){
            TurningLightMap = 1;
        }else if (msg->data[0] == 0 &&  msg->data[1] == 0){
            TurningLightMap = 0;
        }
    }

}

void SERVER_TO_SEND::RouteLaneChangeCallback(const std_msgs::StringConstPtr& msg)
{
    RouteLaneChange = msg->data;
}

void SERVER_TO_SEND::MissionStateCallback(const std_msgs::StringConstPtr& msg)
{
    MissionState = msg->data;
}

// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    init(argc, argv, "UnmannedSol_UDP_TX");

    // assign node SERVER_TO_SEND
    ros::NodeHandle nh_;

    // for debugging
    printf("Initiate: Server_TX\n");
    ros::Rate loop_rate(100);

    int    Socket;
    struct sockaddr_in ServerAddr;
    //struct struct_t_UDP           StrUDP;
    //struct sockaddr_in            MyAddr;
    struct TX_message_data        TX_buff;


    // Socket Creation
    Socket = socket(PF_INET, SOCK_DGRAM, 0);
    int enable = 1;
    setsockopt(Socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    if(Socket == -1){
        printf("[ERROR] 'socket()'\n");
        return -1;
    }
    else{
        printf("[DONE] UDP socket is created\n");
    }

    // UDP-IP Setting
    memset(&ServerAddr, 0, sizeof(ServerAddr)); // Clear to 0
    ServerAddr.sin_family      = PF_INET;
    ServerAddr.sin_port        = htons(DF_UDP_PORTNUM); // PORT#
    ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_SERVER_ADDR); // IP for Server (Normally PC IP)
    

    SERVER_TO_SEND _server_to_send(nh_);
    double prevTime = 0;
    double count = 0;
    char prevSignal;
    while(ros::ok()){
        
        char light_signal = 1;
        TX_buff.Header.MsgID = 0xFF;
        TX_buff.DriveCont.Ax = _server_to_send.acceleration;
        TX_buff.DriveCont.SteeringWheel = _server_to_send.steering_angle; //unit: rad

        // TurningLightMap -->  0: Off, 1: Left indicator, 2: Right indicator, 3: Hazard
        double currentTime = ros::Time::now().toSec();
        double dt = currentTime - prevTime;
        count += dt;
        if(count > 5)
        {
            _server_to_send.TurningLightMap = 0;
            count = 0;
        }
        prevTime = currentTime;

        // if(_server_to_send.steering_angle * 180 / M_PI < -100)
        // {
        //     _server_to_send.TurningLightMap = 2;
        // }
        // if(_server_to_send.steering_angle * 180 / M_PI > 100)
        // {
        //     _server_to_send.TurningLightMap = 1;
        // }
        std_msgs::String LightVisualMsg;
        std::string LightVisual = "";
        double lightonThres = 0.4; //[m]
        if(_server_to_send.ControlTarget.position.y > lightonThres) 
        {
            _server_to_send.TurningLightMap = 1;
            LightVisual = "Left";
        }
        if(_server_to_send.ControlTarget.position.y < -lightonThres)         
        {
            _server_to_send.TurningLightMap = 2;
            LightVisual = "Right";
        }

        if(fabs(_server_to_send.ControlTarget.position.y) < lightonThres)
        {
            _server_to_send.TurningLightMap = 2;
            LightVisual = "Right";            
        }



        if(_server_to_send.RouteLaneChange == "left")
        {
            _server_to_send.TurningLightMap = 1;
            LightVisual = "Left";
        }
        if(_server_to_send.RouteLaneChange == "right")
        {
            _server_to_send.TurningLightMap = 2;
            LightVisual = "Right";
        }

        if(_server_to_send.LeftTurn == "LeftTurnOwnRisk")
        {
            _server_to_send.TurningLightMap = 1;
            LightVisual = "Left";
        }



        if(count < 5 && LightVisual=="")
        {
            _server_to_send.TurningLightMap = prevSignal;
        }

        if(_server_to_send.MissionState=="EmergencyTripod")
        {
            _server_to_send.TurningLightMap = 3;
            LightVisual = "Emergency";
        }

        prevSignal = _server_to_send.TurningLightMap;

        LightVisualMsg.data = LightVisual;
        _server_to_send.pubSignalVisual.publish(LightVisualMsg);
        // std::cout << "Light:" << (int)TX_buff.DriveCont.Light << std::endl;
        TX_buff.DriveCont.Light = _server_to_send.TurningLightMap; //unit: rad


        if(_server_to_send.ThrotthleReverse == true)
            TX_buff.DriveCont.GearNo = -1;
        else
        {
            TX_buff.DriveCont.GearNo = 1;
        }

        sendto(Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&ServerAddr), sizeof(ServerAddr));
        // std::cout << "--------------------------------------------------------" << std::endl;
        // ROS_INFO("Accel/Decel cmd : %f, Steering cmd[deg]:  %f)", 
        //          _server_to_send.acceleration, _server_to_send.steering_angle * 180 / M_PI);

        loop_rate.sleep();
        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: Server_TX\n");
    close(Socket);

    return 0;
}

