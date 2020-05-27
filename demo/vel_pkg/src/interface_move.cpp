#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <string>
#include <map>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#define BUF_LEN 1

#define STOP 9
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define SP_UP 5
#define SP_DOWN 6
#define TU_UP 7
#define TU_DOWN 8


using namespace std;

double speed = 0.2;
double turn = 1;

bool swit;
int nState;

const std::string msg = "Moving around:\n\
   u    i    o\n\
   j    k    l\n\
   m    ,    .\n\
\n\
q/z : increase/decrease max speeds by 10%\n\
w/x : increase/decrease only linear speed by 10%\n\
e/c : increase/decrease only angular speed by 10%\n\
space key, k : force stop\n\
anything else : stop smoothly\n\
\n\
CTRL-C to quit\n\
";

const map<char,pair<int,int> > moveBindings = {
  {'i',make_pair(1,0)},
  {'o',make_pair(1,-1)},
  {'j',make_pair(0,1)},
  {'l',make_pair(0,-1)},
  {'u',make_pair(1,1)},
  {',',make_pair(-1,0)},
  {'.',make_pair(-1,1)},
  {'m',make_pair(-1,-1)},
};

const map<char,pair<double,double> > speedBindings = {
  {'q',make_pair(1.1,1.1)},
  {'z',make_pair(0.9,0.9)},
  {'w',make_pair(1.1,1.0)},
  {'x',make_pair(0.9,1.0)},
  {'e',make_pair(1.0,1.1)},
  {'c',make_pair(1.0,0.9)},
};


void action(const std_msgs::Int32::ConstPtr & msg)
{
    nState = msg->data;
}

int getKey()
{
    int res = 0;
    switch(nState)
{
    case STOP:
        res = 'k';
        nState = 0;
        break;
    case FORWARD:
        res = 'i';
        nState = 0;
        break;
    case BACKWARD:
        res = ',';
        nState = 0;
        break;
    case LEFT:
        res = 'j';
        nState = 0;
        break;
    case RIGHT:
        res = 'l';
        nState = 0;
        break;
    case SP_UP:
        res = 'w';
        nState = 0;
        break;
    case SP_DOWN:
        res = 'x';
        nState = 0;
        break;
    case TU_UP:
        res = 'e';
        nState = 0;
        break;
    case TU_DOWN:
        res = 'c';
        nState = 0;
        break;
    default:
        res = ' ';
        break;
}
return res;
}

std::string vels(double speed,double turn)
{
  std::stringstream ss;
  ss << "currently:\tspeed " << speed << "\tturn " << turn;
  return ss.str();
}

void inter_ctrl(const std_msgs::Int32::ConstPtr & msg)
{
   if (msg->data == 0)
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
  ROS_INFO("inter move start\n");
  swit = true;
  nState = 0;
  ros::init(argc, argv, "interface_move");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<std_msgs::Float32MultiArray>("/move_vel", 10);
  ros::Subscriber sub_sr = n.subscribe("/inter_switch", 10, inter_ctrl);
  ros::Subscriber sub = n.subscribe("/inter_move", 10, action);

  int x = 0;
  int th = 0;
  int status = 0;
  int count = 0;
  double target_speed = 0;
  double target_turn = 0;
  double control_speed = 0;
  double control_turn = 0;

  while(1)
  {
    int key = getKey();
    if (key != ' ') printf("enter %c\n", key);
    if(key < 0)
    {
      return -1;
    }

    auto it = moveBindings.find((char)key);
    if(it != moveBindings.end())
    {
      x = (it->second).first;
      th = (it->second).second;
      count = 0;
    }else 
    {
      auto it = speedBindings.find((char)key);
      if(it != speedBindings.end())
      {
        speed = speed * (it->second).first;
        turn = turn * (it->second).second;
        count = 0;
        cout << vels(speed,turn)<< endl;
        if(status == 14) cout << msg;
        status = (status + 1) % 15;
      }else if(key == ' ' || key == 'k')
      {
        x = 0;
        th = 0;
        control_speed = 0;
        control_turn = 0;
      }else
      {
        count = count + 1;
        if(count > 4)
        {
          x = 0;
          th = 0;
        }
      }
    }

    target_speed = speed * x;
    target_turn = turn * th;

    if(target_speed > control_speed)
      control_speed = min(target_speed, control_speed + 0.02);
    else if(target_speed < control_speed)
      control_speed = max(target_speed, control_speed - 0.02);
    else control_speed = target_speed;

    if(target_turn > control_turn)
      control_turn = min(target_turn, control_turn + 0.1);
    else if(target_turn < control_turn)
      control_turn = max(target_turn, control_turn - 0.1);
    else control_turn = target_turn;

  std_msgs::Float32MultiArray  vel_msg;
   vel_msg.data.push_back(control_speed);
    vel_msg.data.push_back(control_turn);
    if(swit) {vel_pub.publish(vel_msg);}
    ros::spinOnce();
    ros::Duration(0.8).sleep();
  }
}
