from tkinter import *
import tkinter.font as tkFont
import os

main_win = Tk()
main_win.title("简易机器人GUI界面")
main_win.geometry("1000x1000")

def create_map():
    os.system("gnome-terminal " + "-- bash -c \"source ~/Documents/demo/devel/setup.bash; roslaunch robot_sim_demo robot_spawn.launch\"")
    os.system("gnome-terminal " + "-- bash -c \"source ~/Documents/demo/devel/setup.bash; rosrun vel_pkg wpb_home_velocity_control\"")

def nav():
    os.system("gnome-terminal  -- " + "Instruction of navgatoin.")

def obj():
    os.system("gnome-terminal  -- " + "Instruction of recognizing the object.")
    
    
ft_function = tkFont.Font(size=15)
bt_map = Button(main_win, text="开始建图",font=ft_function, command=create_map)
bt_map.place(relx=0.3, rely=0.4, anchor='center', relwidth=0.15, relheight=0.08)

bt_nav = Button(main_win, text="顶点巡航",font=ft_function, command=None)
bt_nav.place(relx=0.5, rely=0.4, anchor='center', relwidth=0.15, relheight=0.08)

bt_obj = Button(main_win, text="目标抓取",font=ft_function, command=None)
bt_obj.place(relx=0.7, rely=0.4, anchor='center', relwidth=0.15, relheight=0.08)

ft_direction = tkFont.Font(size=15)
bt_left = Button(main_win, text="左转", font=ft_direction, command=None)
bt_left.place(relx=0.4, rely=0.7, anchor='center', relwidth=0.1, relheight=0.05)

bt_right = Button(main_win, text="右转", font=ft_direction, command=None)
bt_right.place(relx=0.6, rely=0.7, anchor='center', relwidth=0.1, relheight=0.05)

bt_forward = Button(main_win, text="前进", font=ft_direction, command=None)
bt_forward.place(relx=0.5, rely=0.6, anchor='center', relwidth=0.1, relheight=0.05)

bt_backward = Button(main_win, text="后退", font=ft_direction, command=None)
bt_backward.place(relx=0.5, rely=0.8, anchor='center', relwidth=0.1, relheight=0.05)

ft_Inf = tkFont.Font(size=15)
l_Inf = Label(main_win, font=ft_Inf, text="Team 204")
l_Inf.place(relx=0.5, rely=0.95, anchor='center', relwidth=0.1, relheight=0.05)
main_win.mainloop()
