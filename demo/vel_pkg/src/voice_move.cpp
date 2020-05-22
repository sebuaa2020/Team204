#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define CMD_STOP        0
#define CMD_FORWARD     1
#define CMD_BACKWARD    2  
#define CMD_LEFT        3
#define CMD_RIGHT       4   

#define CMD_DURATION    30

bool swit;
static ros::Publisher move_pub;
static ros::Publisher spk_pub;
static int nCmd = CMD_STOP;
static int nCount = 0;

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[KeywordCB] - %s",msg->data.c_str());
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;

    bool bCmd = false;
    int nFindIndex = 0;
    nFindIndex = msg->data.find("forward");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move x");
        bCmd = true;
        nCmd = CMD_FORWARD;
    }
    nFindIndex = msg->data.find("backward");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move -x");
        bCmd = true;
        nCmd = CMD_BACKWARD;
    }
    nFindIndex = msg->data.find("left");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move y");
        bCmd = true;
        nCmd = CMD_LEFT;
    }

    nFindIndex = msg->data.find("right");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move -y");
        bCmd = true;
        nCmd = CMD_RIGHT;
    }

    nFindIndex = msg->data.find("stop");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - stop");
        bCmd = true;
        nCmd = CMD_STOP;
    }


    std_msgs::String strSpeak;
    if(bCmd == true)
    {
        nCount = CMD_DURATION;
        strSpeak.data = "好的";
        spk_pub.publish(strSpeak);
    }
    else
    {
        //strSpeak = *msg;
    }
}

void voice_ctrl(const std_msgs::Int32::ConstPtr & msg)
{
   if (msg -> data == 0)
   {
      swit = false;
   }
   else
   {
     swit = true;
   }
}

int main(int argc, char** argv)
{
    swit = false;
    ros::init(argc, argv, "voice_move");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    ros::Subscriber voice_switch = n.subscribe("/voice_ctrl", 10, voice_ctrl);
    spk_pub = n.advertise<std_msgs::String>("/xfyun/tts", 20);
    move_pub = n.advertise<geometry_msgs::Twist>("/move_base", 10);

    std_msgs::Int32  move_msg;

    ros::Rate r(10);
    while(ros::ok())
    {
        geometry_msgs::Twist vel_cmd;
        if(nCount > 0)
        {
            nCount --;
            move_msg.data = nCmd;
        }
        else
        {
            move_msg.data = CMD_STOP;
        }
       if (swit) move_pub.publish(move_msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}