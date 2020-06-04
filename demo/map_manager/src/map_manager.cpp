#include <cstdlib>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <wpr_msgs/instruction.h>

#define CMD_STOP 0
#define CMD_FORWARD 1
#define CMD_BACKWARD 2
#define CMD_LEFT 3
#define CMD_RIGHT 4

#define CMD_DURATION 30

using namespace std;

bool swit;
static ros::Publisher main_pub;
static int nCmd = CMD_STOP;
static int nCount = 0;

void save_map(char* name)
{
    char buffer[100];
    vector<string> files;
    ifstream in("../maps/maps.txt");
    if (!in.is_open()) {
        cout << "Error opening file";
        exit(1);
    }
    while (!in.eof()) {
        in.getline(buffer, 100);
        files.push_back(buffer);
    }
    files.erase(files.end() - 1);
    in.close();
    if (find(files.begin(), files.end(), name) != files.end()) {
        printf("unlegal name\n");
    } else {
        files.push_back(name);
    }
    char str[100];
    sprintf(str, "rosrun map_server map_saver -f ../maps/%s", name);
    int res = system(str);
    if (res != 0)
        return;
    ofstream out("../maps/maps.txt");
    for (int i = 0; i < files.size(); i++) {
        out << files[i] << endl;
    }
    return;
}

void list_map()
{
    char buffer[100];

    // char buf[1024] = { 0 };
    // getcwd(buf, sizeof(buf));
    // cout << string(buf, buf + strlen(buf)) << endl;

    ifstream in("../maps/maps.txt");
    if (!in.is_open()) {
        cout << "Error opening file";
        exit(1);
    }
    // cout << "going to read file" << endl;
    cout <<"***************" << endl;
    cout << "all map :" << endl;
    while (!in.eof()) {
        in.getline(buffer, 100);
        cout << buffer << endl;
    }
    in.close();
    cout <<"***************" << endl;
    
}

void load_map(char* name)
{
    char buffer[100];
    vector<string> files;
    ifstream in("../maps/maps.txt");
    if (!in.is_open()) {
        cout << "Error opening file";
        exit(1);
    }
    while (!in.eof()) {
        in.getline(buffer, 100);
        files.push_back(buffer);
    }
    files.erase(files.end() - 1);

    if (find(files.begin(), files.end(), name) == files.end()) {
        wpr_msgs::instruction msg;
        msg.type = wpr_msgs::instruction::MAP_LOAD_ERROR;
        main_pub.publish(msg);
        return;
    }
    char str[100];
    sprintf(str, "rosrun map_server map_server ../maps/%s.yaml &", name);
    int res = system(str);
    if (res != 0) {
        printf("wdnmd\n");
    }
    ROS_INFO("load map success\n");
}

void unload_map()
{
    system("rosnode list | grep map_server | xargs -I {} rosnode kill {}");
    ROS_INFO("unload map success\n");
}

void del_map(char* name)
{
    cout << "trying to delete map: "<< string(name) << endl;
    char buffer[100];
    vector<string> files;
    ifstream in("../maps/maps.txt");
    if (!in.is_open()) {
        cout << "Error opening file";
        exit(1);
    }
    while (!in.eof()) {
        in.getline(buffer, 100);
        files.push_back(buffer);
    }
    files.erase(files.end() - 1);
    vector<string>::iterator it = find(files.begin(), files.end(), name);
    if (it == files.end()) {
        wpr_msgs::instruction msg;
        msg.type = wpr_msgs::instruction::MAP_DEL_ERROR;
        main_pub.publish(msg);
        return;
    }
    files.erase(it);
    char str[100];
    sprintf(str, "rm ../maps/%s.yaml", name);
    int res = system(str);
    if (res != 0) {
        printf("wdnmd\n");
    }
    sprintf(str, "rm ../maps/%s.pgm", name);
    res = system(str);
    if (res != 0) {
        printf("wdnmd\n");
    }
    ofstream out("../maps/maps.txt");
    for (int i = 0; i < files.size(); i++) {
        out << files[i] << endl;
    }
    return;
}

void KeywordCB(const std_msgs::String::ConstPtr& msg)
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
    char para[100];
    nFindIndex = msg->data.find("load");
    if (nFindIndex >= 0) {
        string str = msg->data.substr(5, msg->data.size());
        strcpy(para, str.data());
        load_map(para);
    }
    nFindIndex = msg->data.find("save");
    if (nFindIndex >= 0) {
        string str = msg->data.substr(5, msg->data.size());
        strcpy(para, str.data());
        save_map(para);
    }
    nFindIndex = msg->data.find("delete");
    if (nFindIndex >= 0) {
        string str = msg->data.substr(7, msg->data.size());
        strcpy(para, str.data());
        del_map(para);
    }

    nFindIndex = msg->data.find("unload");
    if (nFindIndex >= 0) {
        // string str = msg->data.substr(5, msg->data.size());
        // strcpy(para, str.data());
        unload_map();
    }

    nFindIndex = msg->data.find("list");
    if (nFindIndex >= 0) {
        // string str = msg->data.substr(5, msg->data.size());
        // strcpy(para, str.data());
        list_map();
    }
}

void map_ctrl(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 0) {
        swit = false;
    } else {
        swit = true;
    }
}

int main(int argc, char** argv)
{
    swit = true;
    ros::init(argc, argv, "map_manager");
    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/map_switch", 10, map_ctrl);
    ros::Subscriber sub = n.subscribe("/map_manager", 10, KeywordCB);
    main_pub = n.advertise<wpr_msgs::instruction>("instruction", 1000);
    ros::spin();

    return 0;
}
