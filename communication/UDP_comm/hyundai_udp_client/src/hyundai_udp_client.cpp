#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

class CLIENT_TO_RECEIVE
{
    public:
        CLIENT_TO_RECEIVE(ros::NodeHandle& n);        
        ~CLIENT_TO_RECEIVE();
        void CarMakerVehStateCallback(const geometry_msgs::TwistStampedConstPtr& msg);
        void CarMakerImuCallback(const sensor_msgs::ImuConstPtr &msg);
        void run();
        // float steering_angle;
        // float speed;
        
        ros::NodeHandle nh;
        ros::Publisher pubAckermann, pubImu;
        ros::Subscriber subVehState, subImu;
        ackermann_msgs::AckermannDriveStamped ackermannMsg;
        sensor_msgs::Imu ImuMsg;

        int cnt1, cnt2;
        
};

CLIENT_TO_RECEIVE::CLIENT_TO_RECEIVE(ros::NodeHandle& n) : cnt1(0), cnt2(0)
{
    pubAckermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/Ackermann/veh_state", 10); 
    pubImu = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);

    subVehState = nh.subscribe("/Twist/veh_state",10,&CLIENT_TO_RECEIVE::CarMakerVehStateCallback, this);
    subImu = nh.subscribe("/carmaker/imu/data",10,&CLIENT_TO_RECEIVE::CarMakerImuCallback, this);
};

CLIENT_TO_RECEIVE::~CLIENT_TO_RECEIVE() 
{    
    ROS_INFO("CLIENT_TO_RECEIVE destructor.");
}

void CLIENT_TO_RECEIVE::CarMakerVehStateCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    cnt1 ++;
    ackermannMsg.header.stamp = ros::Time::now();
    ackermannMsg.header.frame_id = "base_link";
    ackermannMsg.drive.speed = msg->twist.linear.x;
    ackermannMsg.drive.acceleration = msg->twist.linear.y;
    ackermannMsg.drive.steering_angle = msg->twist.angular.z;
    if(cnt1%10 == 0)
        pubAckermann.publish(ackermannMsg);   
}

void CLIENT_TO_RECEIVE::CarMakerImuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    cnt2 ++;
    ImuMsg = *msg;
    ImuMsg.header.stamp = ros::Time::now();
    ImuMsg.header.frame_id = "base_link";
    if(cnt2%10 ==0)
        pubImu.publish(ImuMsg);

}

void CLIENT_TO_RECEIVE::run()
{
}

// node main loop, for ROS
int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "Twist_to_Ackermann_vehstate");

    ros::NodeHandle nh_;
    CLIENT_TO_RECEIVE converter(nh_);    
    // ros::Rate loop_rate(100);
    ros::spin();
    // while(ros::ok())
    // {
    //     converter.run();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}

