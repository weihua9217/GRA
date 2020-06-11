from Class_Define import *
from Function_Define import *
from tkinter import *
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
# ===== global value======
Obstacle_list = list()
Robot_list = list() #[initial_robot,final_robot,initial_robot,final_robot......]
map_robot_list = list()
map_obstacle_list = list()
map_cp_list = list()
g_PtField1 = list()
g_PtField2 = list()
obstacleshowed,robotshowed=False,False
# ====== ARRANGE DATA =========================
with open("obstacle.txt","r") as f:
    data = f.readlines()
with open("robot.txt","r") as f2:
    data2 = f2.readlines()
W = list() #obstacle
for i in data: #讀檔
    if(i[0]!='#'):
        W.append(i)
W2 = list() #robot
for i in data2:
    if(i[0]!='#'):
        W2.append(i)

# ======== Obstacle list ======================
index,index2 = 0,0
for i in range(int(W[index])):
    index+=1
    obstacle = Obstacle("o")
    # number of polygons
    for j in range(int(W[index])):
        index+=1
        polygon = Polygon("p")
        # number of vertices
        for k in range(int(W[index])):
            index+=1
            vertice = Vertice("v")
            a = W[index].split()
            vertice.AddPosition(float(a[0]),float(a[1]))
            polygon.AddVertice(vertice)
        obstacle.AddPolygon(polygon)
    index += 1
    b = W[index].split()
    obstacle.Addcon(float(b[0]),float(b[1]),float(b[2]))
    Obstacle_list.append(obstacle)

# ========= Robot list ========================
for i in range(int(W2[index2])):
    index2 += 1
    init_robot = Robot("r1")
    final_robot = Robot("r2")
    for j in range(int(W2[index2])):
        index2 += 1
        polygon1 = Polygon("p1")
        polygon2 = Polygon("p2")
        for k in range(int(W2[index2])):
            index2 += 1
            vertice1 = Vertice("v1")
            vertice2 = Vertice("v2")
            a = W2[index2].split()
            vertice1.AddPosition(float(a[0]),float(a[1]))
            vertice2.AddPosition(float(a[0]),float(a[1]))
            polygon1.AddVertice(vertice1)
            polygon2.AddVertice(vertice2)
        init_robot.AddPolygon(polygon1)
        final_robot.AddPolygon(polygon2)
    index2 += 1
    b = W2[index2].split()
    init_robot.AddCon(float(b[0]),float(b[1]),float(b[2]))
    index2+=1
    b = W2[index2].split()
    final_robot.AddCon(float(b[0]),float(b[1]),float(b[2]))
    index2+=1
    for i in range(int(W2[index2])):
        index2+=1
        cp = ControlPoint("c")
        cp2 = ControlPoint("c")
        b = W2[index2].split()
        cp.AddPosition(float(b[0]),float(b[1]))
        cp2.AddPosition(float(b[0]), float(b[1]))
        init_robot.AddControl(cp)
        final_robot.AddControl(cp2)
    Robot_list.append(init_robot)
    Robot_list.append(final_robot)
obstacle_bt,robot_bt = False,False

# ==== CANVAS CONVERT and SHOW ================
cv_ObstacleList,cv_OLineList = list(),list()
def ShowObtacle(move=0,index=0,clockwise=False): #delete obstacle[index]
    global obstacle_bt,dx,dy
    obstacle_bt = True
    if move == 1: #move
        path(cv_ObstacleList,cv_OLineList,Obstacle_list,index,cv,dx,dy)
    elif move==2: #rotate
        rotation(cv_ObstacleList,cv_OLineList,Obstacle_list,index,cv,clockwise)
    else:
        for obstacle in Obstacle_list:
            bdbox = [sys.float_info.max, sys.float_info.max, sys.float_info.min, sys.float_info.min]
            # check all polygon
            cv_PolygonList = list()
            for polygon in obstacle.poly:
                all_vertice = list()
                for vertice in polygon.ver:
                    ConvertedVertice = list()
                    x = vertice.pos[0]
                    y = vertice.pos[1]
                    c_x = cconvert_x(obstacle.con,x,y)
                    c_y = cconvert_y(obstacle.con,x,y)
                    cc_x = canvas_x(c_x)
                    cc_y = canvas_y(c_y)
                    if c_x < bdbox[0]:
                        bdbox[0] = c_x
                    if c_x > bdbox[2]:
                        bdbox[2] = c_x
                    if c_y < bdbox[1]:
                        bdbox[1] = c_y
                    if c_y > bdbox[3]:
                        bdbox[3] = c_y
                    ConvertedVertice.append(cc_x) #畫圖
                    ConvertedVertice.append(cc_y)
                    all_vertice.append(ConvertedVertice)
                c_poly = cv.create_polygon(all_vertice, fill=obstacle.color)
                cv_PolygonList.append(c_poly)
            cv_ObstacleList.append(cv_PolygonList)
            # bdbox convert and append to obstacle
            for k in range(len(bdbox)):
                obstacle.bdbox.append(bdbox[k])
                if k%2==0:
                    bdbox[k]=canvas_x(bdbox[k])
                else:
                    bdbox[k]=canvas_y(bdbox[k])
            cv_line = cv.create_line(bdbox,fill='red')
            cv_OLineList.append(cv_line)
    cv.pack()
# show robot and robot's target in canvas
# notice: just consider one robot with its initial position and final position
cv_RobotList,cv_RLineList = list(),list()
def ShowRobot(move=0,index=0,clockwise=False):
    global robot_bt,dx,dy
    robot_bt = True
    if move==1:
        path(cv_RobotList,cv_RLineList,Robot_list,index,cv,dx,dy)
    elif move==2:
        rotation(cv_RobotList, cv_RLineList, Robot_list, index, cv, clockwise)
    else:
        for i in range(2):
            bdbox = [sys.float_info.max, sys.float_info.max, sys.float_info.min, sys.float_info.min]
            cv_PolygonList = list()
            if i%2==1:
                Robot_list[i].color='grey'
            for polygon in Robot_list[i].poly:
                all_vertice = list()
                for vertice in polygon.ver:
                    ConvertVertice = list()
                    x = vertice.pos[0]
                    y = vertice.pos[1]
                    c_x = cconvert_x(Robot_list[i].con,x,y)
                    c_y = cconvert_y(Robot_list[i].con,x,y)
                    cc_x = canvas_x(c_x)
                    cc_y = canvas_y(c_y)
                    if c_x<bdbox[0]:
                        bdbox[0] = c_x
                    if c_x>bdbox[2]:
                        bdbox[2] = c_x
                    if c_y<bdbox[1]:
                        bdbox[1] = c_y
                    if c_y>bdbox[3]:
                        bdbox[3] = c_y
                    ConvertVertice.append(cc_x)
                    ConvertVertice.append(cc_y)
                    all_vertice.append(ConvertVertice)
                c_poly = cv.create_polygon(all_vertice,fill = Robot_list[i].color)
                cv_PolygonList.append(c_poly)
            cv_RobotList.append(cv_PolygonList)
            for k in range(len(bdbox)):
                Robot_list[i].bdbox.append(bdbox[k])
                if k%2==0:
                    bdbox[k] = canvas_x(bdbox[k])
                else:
                    bdbox[k] = canvas_y(bdbox[k])
            cv_line = cv.create_line(bdbox,fill='red')
            cv_RLineList.append(cv_line)
    cv.pack()
# ========== MOUSE EVENT =============================
p_x,p_y,first,dx,dy,delete_index,type=0,0,0,0,0,0,0
# to check which element is selected and move motion
def motion(event):
    global p_x,p_y,first,dx,dy,obstacle_bt,robot_bt
    global delete_index,type
    if first==0: #check
        x = event.x
        y = event.y
        p_x = x
        p_y = y
        if obstacle_bt == True:
            ob_index=0
            for obstacle in Obstacle_list:
                x0 = canvas_x(obstacle.bdbox[0])
                x1 = canvas_x(obstacle.bdbox[2])
                y0 = canvas_y(obstacle.bdbox[1])
                y1 = canvas_y(obstacle.bdbox[3])
                if x> x0 and x<x1 and y<y0 and y>y1:#差一個負號!
                    obstacle.selected = True
                    first+=1
                    type=1
                    delete_index=ob_index
                ob_index+=1
        if robot_bt == True:
            rb_index = 0
            for i in range(2):
                x0 = canvas_x(Robot_list[i].bdbox[0])
                x1 = canvas_x(Robot_list[i].bdbox[2])
                y0 = canvas_y(Robot_list[i].bdbox[1])
                y1 = canvas_y(Robot_list[i].bdbox[3])
                if x>x0 and x<x1 and y<y0 and y>y1:
                    Robot_list[i].selected = True
                    first+=1
                    type=2
                    delete_index=rb_index
                rb_index+=1
    else: # move
        x = event.x
        y = event.y
        dx = x-p_x
        dy = y-p_y
        if type==1 and first!=0:
            ShowObtacle(1,delete_index)
        elif type==2 and first!=0:
            ShowRobot(1,delete_index)
        p_x = x
        p_y = y
def release(event):
    global first
    first=0
    for obstacle in Obstacle_list:
        obstacle.selected = False
    for robot in Robot_list:
        robot.selected = False
rtype,rdelete_index=0,-1
rselected = False

# ========= rotate motion ===============
def rmotion(event):
    global rtype,rdelete_index,rselected,obstacle_bt,robot_bt
    x = event.x
    y = event.y
    rselected = False
    if obstacle_bt == True:
        ob_index = 0
        for obstacle in Obstacle_list:
            x0 = canvas_x(obstacle.bdbox[0])
            x1 = canvas_x(obstacle.bdbox[2])
            y0 = canvas_y(obstacle.bdbox[1])
            y1 = canvas_y(obstacle.bdbox[3])
            if x > x0 and x < x1 and y < y0 and y > y1:  # 差一個負號!
                obstacle.rselected = True
                rtype = 1
                rselected=True
                rdelete_index = ob_index
            ob_index += 1
    if robot_bt == True:
        rb_index = 0
        for i in range(2):
            x0 = canvas_x(Robot_list[i].bdbox[0])
            x1 = canvas_x(Robot_list[i].bdbox[2])
            y0 = canvas_y(Robot_list[i].bdbox[1])
            y1 = canvas_y(Robot_list[i].bdbox[3])
            if x > x0 and x < x1 and y < y0 and y > y1:
                Robot_list[i].rselected = True
                rtype = 2
                rselected = True
                rdelete_index = rb_index
            rb_index += 1
def rotate(event):
    global rtype
    if event.delta<0:
        if rselected==True:
            if rtype==1:
                ShowObtacle(2,rdelete_index,True)
            if rtype==2:
                ShowRobot(2,rdelete_index,True)
    else:
        if rselected==True:
            if rtype==1:
                ShowObtacle(2,rdelete_index,False)
            if rtype==2:
                ShowRobot(2,rdelete_index,False)

# ==========button func==================
def bt_start():
    showPtfield()
    CanvasRemove(cv_RLineList,cv)
    all_node = list()
    if(BFS(Obstacle_list, Robot_list,g_PtField1,g_PtField2,all_node)):
        print("success")
        for node in all_node:
            time.sleep(0.035)
            robot_animate(cv_RobotList,Robot_list,cv,node,cv_RLineList)
    else:
        print("fail")
# ======Show obstacles on canvas
def bt_showObstacle():
    global obstacleshowed
    if obstacleshowed==False:
        ShowObtacle(0,0)
        obstacleshowed=True

# ======Show robot on canvas =========
def bt_showRobot():
    global robotshowed
    if robotshowed==False:
        ShowRobot(0,0)
        robotshowed=True

# ======Canvas to planner convert ======
def convert_planner():
    map_robot_list.clear()
    map_cp_list.clear()
    for i in range(2):
        map_robot = Robot("r")
        for polygon in Robot_list[i].poly:
            map_polygon = Polygon("p")
            for vertice in polygon.ver:
                map_vertice = Vertice("v")
                x=vertice.pos[0]
                y=vertice.pos[1]
                c_x = cconvert_x(Robot_list[i].con,x,y)
                c_y = cconvert_y(Robot_list[i].con,x,y)
                map_vertice.pos.append(c_x)
                map_vertice.pos.append(c_y)
                map_polygon.AddVertice(map_vertice)
            map_robot.AddPolygon(map_polygon)

        map_robot.bdbox.append(cconvert_x(Robot_list[i].con,Robot_list[i].bdbox[0],Robot_list[i].bdbox[1]))
        map_robot.bdbox.append(cconvert_y(Robot_list[i].con,Robot_list[i].bdbox[0],Robot_list[i].bdbox[1]))
        map_robot.bdbox.append(cconvert_x(Robot_list[i].con,Robot_list[i].bdbox[2],Robot_list[i].bdbox[3]))
        map_robot.bdbox.append(cconvert_y(Robot_list[i].con,Robot_list[i].bdbox[2],Robot_list[i].bdbox[3]))

        for j in range(2):
            map_cp = ControlPoint("c")
            x = Robot_list[i].control[j].pos[0]
            y = Robot_list[i].control[j].pos[1]
            c_x = cconvert_x(Robot_list[i].con,x,y)
            c_y = cconvert_y(Robot_list[i].con,x,y)
            map_cp.AddPosition(c_x,c_y)
            map_cp_list.append(map_cp)

        map_robot_list.append(map_robot)
    map_obstacle_list.clear()
    for obstacle in Obstacle_list:
        map_obstacle = Obstacle("o")
        for polygon in obstacle.poly:
            map_polygon = Polygon("p")
            for vertice in polygon.ver:
                map_vertice = Vertice("v")
                map_vertice.pos.append(cconvert_x(obstacle.con,vertice.pos[0],vertice.pos[1]))
                map_vertice.pos.append(cconvert_y(obstacle.con,vertice.pos[0],vertice.pos[1]))
                map_polygon.AddVertice(map_vertice)
            map_obstacle.AddPolygon(map_polygon)

        map_obstacle.bdbox.append(cconvert_x(obstacle.con, obstacle.bdbox[0], obstacle.bdbox[1]))
        map_obstacle.bdbox.append(cconvert_y(obstacle.con, obstacle.bdbox[0], obstacle.bdbox[1]))
        map_obstacle.bdbox.append(cconvert_x(obstacle.con, obstacle.bdbox[2], obstacle.bdbox[3]))
        map_obstacle.bdbox.append(cconvert_y(obstacle.con, obstacle.bdbox[2], obstacle.bdbox[3]))

        map_obstacle_list.append(map_obstacle)

# ==========Potiential Field ========
def showPtfield():
    if(len(g_PtField1)==0):
        for i in range(128):
            field = list()
            field2 = list()
            for j in range(128):
                field.append(0)
                field2.append(0)
            g_PtField1.append(field)
            g_PtField2.append(field2)

    PtField = list()
    for i in range(128):
        Field = list()
        for j in range(128):
            Field.append(254)
        PtField.append(Field)
    convert_planner()
    #obstacle
    for obstacle in map_obstacle_list:
        for polygon in obstacle.poly:
            Max = np.array([0]*128)
            Min = np.array([1000]*128)
            Min_x =1000
            Max_x =0
            for index in range(len(polygon.ver)):
                if(index==0):
                    pre_index = len(polygon.ver)-1
                else:
                    pre_index = index-1
                x1 = polygon.ver[pre_index].pos[0]
                y1 = polygon.ver[pre_index].pos[1]
                x2 = polygon.ver[index].pos[0]
                y2 = polygon.ver[index].pos[1]
                v_x = x2 - x1
                v_y = y2 - y1
                for i in range(1,1000,1):
                    t_x = int((i/1000)*v_x+x1)
                    t_y = int((i/1000)*v_y+y1)
                    PtField[t_x][t_y] = 255
                    if(t_y>Max[t_x]):Max[t_x] = t_y
                    if(t_x>Max_x):Max_x=t_x
                    if (t_y<Min[t_x]): Min[t_x]=t_y
                    if(t_x<Min_x):Min_x=t_x
            for i in range(Min_x,Max_x,1):
                x1 =i
                y1 =Min[i]
                x2 =i
                y2 =Max[i]
                v_x = x2-x1
                v_y = y2-y1
                for i in range(1,1000,1):
                    t_x = int((i/1000)*v_x+x1)
                    t_y = int((i/1000)*v_y+y1)
                    PtField[t_x][t_y] = 255
    #robot
    # for i in range(2):
    #     fill = 254
    #     if i==1:
    #         fill = 254
    #     for polygon in map_robot_list[i].poly:
    #         Max = np.array([0] * 128)
    #         Min = np.array([1000] * 128)
    #         Min_x = 1000
    #         Max_x = 0
    #         for index in range(len(polygon.ver)):
    #             if (index == 0):
    #                 pre_index = len(polygon.ver) - 1
    #             else:
    #                 pre_index = index - 1
    #             x1 = polygon.ver[pre_index].pos[0]
    #             y1 = polygon.ver[pre_index].pos[1]
    #             x2 = polygon.ver[index].pos[0]
    #             y2 = polygon.ver[index].pos[1]
    #             v_x = x2 - x1
    #             v_y = y2 - y1
    #             for i in range(1, 1000, 1):
    #                 t_x = int((i / 1000) * v_x + x1)
    #                 t_y = int((i / 1000) * v_y + y1)
    #                 PtField[t_x][t_y] = fill
    #                 if (t_y > Max[t_x]): Max[t_x] = t_y
    #                 if (t_x > Max_x): Max_x = t_x
    #                 if (t_y < Min[t_x]): Min[t_x] = t_y
    #                 if (t_x < Min_x): Min_x = t_x
    #         for i in range(Min_x,Max_x,1):
    #             x1 =i
    #             y1 =Min[i]
    #             x2 =i
    #             y2 =Max[i]
    #             v_x = x2-x1
    #             v_y = y2-y1
    #             for i in range(1,1000,1):
    #                 t_x = int((i/1000)*v_x+x1)
    #                 t_y = int((i/1000)*v_y+y1)
    #                 PtField[t_x][t_y] = fill
    PtField2 = list()
    for i in range(128):
        PtField2_row = list()
        for j in range(128):
            PtField2_row.append(PtField[i][j])
        PtField2.append(PtField2_row)

    for i in range(2,4):
        x = int(map_cp_list[i].pos[0])
        y = int(map_cp_list[i].pos[1])
        if i==2:
            PtField[x][y] = 0
            PFrun(PtField,x,y)
        if i==3:
            PtField2[x][y] = 0
            PFrun(PtField2,x,y)
    cv.pack()

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # for i in range(1,128,1):
    #     for j in range(1,128,1):
    #         if PtField[i][j] ==255:
    #             ax.scatter(i, j, PtField[i][j], color='red', s=0.1)
    #         else:
    #             ax.scatter(i,j,PtField[i][j],color = 'black',s=0.1)
    # plt.show()
    # print(PtField[64][76])
    # print(PtField2[64][62])

    for i in range(128):
        for j in range(128):
            g_PtField1[i][j] = PtField[i][j]
            g_PtField2[i][j] = PtField2[i][j]

    pd.DataFrame(g_PtField1).to_csv("output.csv")
    pd.DataFrame(g_PtField2).to_csv("output2.csv")
    # print("done")

if __name__ == '__main__':
    window = Tk() #mp
    window.geometry("400x440")
    window.title("Motion Planning")
    canvas_frame = Frame(window)
    canvas_frame.pack()
    cv = Canvas(canvas_frame,bg='#FFFAFA',width=400, height=400)
    cv.pack()
    # button
    start_bt = Button(window,text='Start',font=('Arial',8),bg="#FFA07A", width=15,height=1,command=bt_start)
    Obstacle_bt = Button(window,text='Obstacle',font=('Arial',8),bg="#87CEFA", width=15,height=1,command=bt_showObstacle)
    Robot_bt = Button(window,text='Robot',font=('Arial',8),bg="#87CEFA", width=15,height=1,command=bt_showRobot)
    Ptfield_bt = Button(window,text='PTField',font=('Arial',8),bg="#87CEFA",width=15,height=1,command=showPtfield)
    Ptfield_bt.pack(side=RIGHT,padx=1,pady=3)
    Robot_bt.pack(side=RIGHT,padx=1,pady=3)
    Obstacle_bt.pack(side=RIGHT,padx=1,pady=3)
    start_bt.pack(side=RIGHT,padx=1,pady=3)
    # motion
    window.bind('<B1-Motion>',motion)
    window.bind('<ButtonRelease-1>',release)
    window.bind('<Motion>',rmotion)
    window.bind('<MouseWheel>',rotate)
    window.mainloop()