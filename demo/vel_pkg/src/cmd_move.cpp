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
#include <std_msgs/Float32MultiArray.h>
#define BUF_LEN 1
using namespace std;

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

struct termios orig_termios;
struct termios raw;

int setTermiosOrigAndRaw()
{
  if (!isatty(STDIN_FILENO)) /* 判断是否是终端 */
  		return -1;
  if (tcgetattr(STDIN_FILENO,&orig_termios) == -1)
		return -1;
  
  raw = orig_termios;
  raw.c_lflag &= ~(ECHO | ICANON);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;

  return 0;
}

int getKey()
{
  if (tcsetattr(STDIN_FILENO,TCSAFLUSH,&raw) < 0)
    return -1;

  int ret;
  struct timeval tv;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 单位微秒

  int ans = select(STDIN_FILENO+1,&readfds,NULL,NULL,&tv);
  if(ans == -1)
  {
    ret = -1;
  } else if(!ans) {
    ret = 0;
  } else if(FD_ISSET(STDIN_FILENO, &readfds))
  {
    // cout << "left time:" << tv.tv_usec << endl;
    char buf[BUF_LEN+1];
    int len;
    len = read(STDIN_FILENO,buf,BUF_LEN);
    if(len == -1)
    {
      ret = -1;
    } else if(!len)
    {
      ret = 0;
    } else 
    {
      ret = (int)(buf[0]);
    }
  }

  if (tcsetattr(STDIN_FILENO,TCSAFLUSH,&orig_termios) < 0)
    return -1;

  return ret;
}

std::string vels(double speed,double turn)
{
  std::stringstream ss;
  ss << "currently:\tspeed " << speed << "\tturn " << turn;
  return ss.str();
}

double speed = 0.2;
double turn = 1;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_move");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<std_msgs::Float32MultiArray>("/move_vel", 10);


  if(setTermiosOrigAndRaw() < 0) return -1;
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
    vel_pub.publish(vel_msg);
    ros::spinOnce();
  }
}
