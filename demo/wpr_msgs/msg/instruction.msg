int8 type
string description
string argStr
int32 argInt
navGoal goalPose
float32[] box_plane
float32[] box_obj

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
int8 STOP = 9

############################
# 建图相关指令
int8 MAPPING_START = 10
int8 MAPPING_END = 11

############################
# 地图管理相关指令
int8 LIST_MAP = 12
int8 DELETE_MAP = 13
int8 LOAD_MAP = 14
int8 UNLOAD_MAP = 15
int8 SAVE_MAP = 16

############################
# 导航相关指令
int8 NAV_START = 17
int8 NAV_CANCEL = 18


############################
# 目标抓取相关指令
int8 GRAB_START = 19
int8 RELEASE_START = 20

############################
# 障碍物检测相关指令
int8 BARRIER_START = 21
int8 BARRIER_END = 22


################################################
# 异常处理
int8 NAV_UNREACHABLE = 101
int8 NAV_ARRIVED = 102
int8 NAV_STUCK = 103


int8 MAP_DEL_ERROR = 104
int8 MAP_LOAD_ERROR = 105
