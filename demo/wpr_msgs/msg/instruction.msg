int8 type
string description
string argStr
int32 argInt
navGoal goalPose

###########################
# 运动控制相关指令
int8 FORWARD = 1
int8 BACKWARD = 2
int8 TURNLEFT = 3
int8 TURNRIGHT = 4
int8 LINEAR_SPEEDUP = 5
int8 LINEAR_SPEEDDOWN = 6
int8 ANGULAR_SPEEDUP = 7
int8 ANGULAR_SPEEDDOWN = 8

############################
# 建图相关指令
int8 MAPPING_START = 9
int8 MAPPING_END = 10

############################
# 地图管理相关指令
int8 LIST_MAP = 11
int8 DELETE_MAP = 12
int8 LOAD_MAP = 13
int8 UNLOAD_MAP = 14

############################
# 导航相关指令
int8 NAV_START = 15
int8 NAV_CANCEL = 16

############################
# 目标抓取相关指令
int8 GRAB_START = 17



################################################
# 异常处理
int8 NAV_UNREACHABLE = 101
int8 NAV_ARRIVED = 102
int8 NAV_REJECTED = 103
