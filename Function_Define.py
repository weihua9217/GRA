import math
import sys
from Class_Define import *
import copy
from tkinter import *

def CanvasRemove(list,cv):
    for index in range(len(list)):
        cv.delete(list[index])
# ============= action ===================================
def path(cv_targetlist, cv_linelist, targetlist,index,canvas,dx,dy):
    CanvasRemove(cv_targetlist[index],canvas)
    canvas.delete(cv_linelist[index])
    targetlist[index].con[0] = targetlist[index].con[0] + map_x(dx)
    targetlist[index].con[1] = targetlist[index].con[1] - dy * 128 / 400
    cv_polygon = list()
    for polygon in targetlist[index].poly:
        all_vertice = list()
        for vertice in polygon.ver:
            c_x = cconvert_x(targetlist[index].con,vertice.pos[0],vertice.pos[1])
            c_y = cconvert_y(targetlist[index].con,vertice.pos[0],vertice.pos[1])
            all_vertice.append(c_x*400/128)
            all_vertice.append(400-c_y*(400/128))
        c_poly = canvas.create_polygon(all_vertice,fill=targetlist[index].color)
        cv_polygon.append(c_poly)
    n_bdbox = list()
    for i in range(len(targetlist[index].bdbox)):
        if i%2 ==0:
            targetlist[index].bdbox[i] = targetlist[index].bdbox[i] + map_x(dx)
            n_bdbox.append(targetlist[index].bdbox[i]*400/128)
        else:
            targetlist[index].bdbox[i] = targetlist[index].bdbox[i] - dy*128/400
            n_bdbox.append(400-targetlist[index].bdbox[i]*(400/128))
    c_line = canvas.create_line(n_bdbox,fill='red')
    cv_linelist[index] = c_line
    cv_targetlist[index] = cv_polygon
def rotation(cv_targetlist, cv_linelist, targetlist,index,canvas,clockwise):
    CanvasRemove(cv_targetlist[index],canvas)
    canvas.delete(cv_linelist[index])
    centerX = (targetlist[index].bdbox[0] + targetlist[index].bdbox[2]) / 2
    centerY = (targetlist[index].bdbox[1] + targetlist[index].bdbox[3]) / 2
    d = 10  # degree
    if clockwise == False:
        d = -10
    targetlist[index].con[2] = targetlist[index].con[2]+d
    if targetlist[index].con[2]>360:
        targetlist[index].con[2] -= 360
    if targetlist[index].con[2]<0:
        targetlist[index].con[2] +=360

    cv_polygon = list()
    for polygon in targetlist[index].poly:
        all_vertice = list()
        for vertice in polygon.ver:
            c_x = cconvert_x(targetlist[index].con, vertice.pos[0], vertice.pos[1])
            c_y = cconvert_y(targetlist[index].con, vertice.pos[0], vertice.pos[1])
            all_vertice.append(c_x*400/128)
            all_vertice.append(400-(c_y)*400/128)
        c_poly = canvas.create_polygon(all_vertice,fill=targetlist[index].color)
        cv_polygon.append(c_poly)

    n_bdbox = [sys.float_info.max, sys.float_info.max, sys.float_info.min, sys.float_info.min]
    for polygon in targetlist[index].poly:
        for vertice in polygon.ver:
            c_x = cconvert_x(targetlist[index].con, vertice.pos[0], vertice.pos[1])
            c_y = cconvert_y(targetlist[index].con, vertice.pos[0], vertice.pos[1])
            if c_x < n_bdbox[0]:
                n_bdbox[0] = c_x
            if c_x > n_bdbox[2]:
                n_bdbox[2] = c_x
            if c_y < n_bdbox[1]:
                n_bdbox[1] = c_y
            if c_y > n_bdbox[3]:
                n_bdbox[3] = c_y
    for i in range(4):
        targetlist[index].bdbox[i] = n_bdbox[i]
    n_bdbox[0] = n_bdbox[0]*400/128
    n_bdbox[2] = n_bdbox[2]*400/128
    n_bdbox[1] = 400 - n_bdbox[1]*400/128
    n_bdbox[3] = 400 - n_bdbox[3]*400/128
    c_line = canvas.create_line(n_bdbox,fill='red')
    cv_linelist[index] = c_line
    cv_targetlist[index] = cv_polygon

# ============= PtField ====================================
def direction(c_x,c_y,dir,field,current):
    n_x,n_y=0,0
    if(dir==0):
        n_x = c_x
        n_y = c_y + 1
    elif(dir==1):
        n_x = c_x
        n_y = c_y - 1
    elif(dir==2):
        n_x = c_x - 1
        n_y = c_y
    elif(dir==3):
        n_x = c_x + 1
        n_y = c_y
    if(-1<n_x<128 and -1<n_y<128):
        if (field[n_x][n_y]==254):
            field[n_x][n_y] = field[c_x][c_y]+1
            nn = list()
            nn.append(n_x)
            nn.append(n_y)
            current.append(nn)
def PFrun(field,x,y):
    current = list()
    cn = list()
    cn.append(x)
    cn.append(y)
    current.append(cn)
    while(len(current)!=0):
        c_x = current[0][0]
        c_y = current[0][1]
        #up
        direction(c_x,c_y,0,field,current)
        #down
        direction(c_x, c_y, 1,field,current)
        #left
        direction(c_x, c_y, 2,field,current)
        #right
        direction(c_x, c_y, 3,field,current)
        del current[0]

# ========== convert function ==============================
def map_x(x):
    return x*128/400
def map_y(y):
    return (400-y)*128/400
def canvas_x(x):
    return x*400/128
def canvas_y(y):
    return 400-y*400/128
def cconvert_x(con,x,y):
    return  con[0]+math.cos(math.radians(con[2]))*x - math.sin(math.radians(con[2]))*y
def cconvert_y(con,x,y):
    return  con[1]+math.sin(math.radians(con[2]))*x + math.cos(math.radians(con[2]))*y

# ================== collision ============================
def bdbox_collision(x0,y0,x1,y1,a0,b0,a1,b1):
    if (x1>=a0 and x0<=a1 and y1>=b0 and y0<=b1):
        #print("bdbox_collision")
        return 1
    if(x0>127 or x0<0 or x1>127 or x1<0 or y0>127 or y0<0 or y1>127 or y1<0):
        return 2
    if(a0>127 or a0<0 or a1>127 or a1<0 or b0>127 or b0<0 or b1>127 or b1<0):
        return 2
    else:
        return 0
#p0,p1,p2,p3 are vertice objects
def vertice_collision(p0,p1,p2,p3):
    # print("====")
    v1 = p1 -p0
    v2 = p2 -p0
    v3 = p3 -p0
    vv1 = p3 -p2
    vv2 = p0 -p2
    vv3 = p1 -p2
    v1.ConvertVertical()
    vv1.ConvertVertical()
    if(v1.InnerProduct(v2)*v1.InnerProduct(v3)<0 and vv1.InnerProduct(vv2)*vv1.InnerProduct(vv3)<0):
        #print("line_collision")
        return 1
    else:
        return 0
def collision(obstaclelist,robotlist):
    #bdbox check first
    # print("check")
    # print(robotlist[0].con)
    # update bounded box !===
    new_bdbox = [sys.float_info.max, sys.float_info.max, sys.float_info.min, sys.float_info.min]
    for polygon in robotlist[0].poly:
        for vertice in polygon.ver:
            x = vertice.pos[0]
            y = vertice.pos[1]
            c_x = cconvert_x(robotlist[0].con, x, y)
            c_y = cconvert_y(robotlist[0].con, x, y)
            if c_x < new_bdbox[0]:
                new_bdbox[0] = c_x
            if c_x > new_bdbox[2]:
                new_bdbox[2] = c_x
            if c_y < new_bdbox[1]:
                new_bdbox[1] = c_y
            if c_y > new_bdbox[3]:
                new_bdbox[3] = c_y

    for i in range(4):
        # print(new_bdbox[i])
        robotlist[0].bdbox[i] = new_bdbox[i]
    # =======================
    collide = False
    all_bdbox = list()
    for obstacle in obstaclelist:
        all_bdbox.append(obstacle.bdbox)
    for i in range(2):
        all_bdbox.append(robotlist[i].bdbox)
    for bd_i in range(len(all_bdbox)):
        for bd_j in range(bd_i+1,len(all_bdbox)):
            o1 = 0  # if is robot = 1 ,obstacle = -1
            a = all_bdbox[bd_i]
            b = all_bdbox[bd_j]
            if(bdbox_collision(a[0],a[1],a[2],a[3],b[0],b[1],b[2],b[3])==2):
                collide= True
                # print("boundary collection")
            elif(bdbox_collision(a[0],a[1],a[2],a[3],b[0],b[1],b[2],b[3])==1):
                if(bd_i>=len(obstaclelist)):
                    obj1 = robotlist[bd_i-len(obstaclelist)]
                    o1=1
                else:
                    obj1 = obstaclelist[bd_i]
                    o1=-1
                if(bd_j>=len(obstaclelist)):
                    obj2 = robotlist[bd_j-len(obstaclelist)]
                    o2 =1
                else:
                    obj2 = obstaclelist[bd_j]
                    o2=-1
                if o1*o2<0:
                    objlist = list()
                    objlist.append(obj1)
                    objlist.append(obj2)
                    Obj1_AllLine = list()
                    Obj2_AllLine = list()
                    for obj_num in range(len(objlist)):
                        for polygon in objlist[obj_num].poly:
                            for i in range(len(polygon.ver)):
                                line = list()
                                if(i == len(polygon.ver)-1):
                                    j=0
                                else:
                                    j=i+1
                                vertice1 = Vertice("v1")
                                vertice2 = Vertice("v2")
                                ov1 = polygon.ver[i]
                                ov2 = polygon.ver[j]
                                con = objlist[obj_num].con
                                vertice1.AddPosition(cconvert_x(con,ov1.pos[0],ov1.pos[1]),cconvert_y(con,ov1.pos[0],ov1.pos[1]))
                                vertice2.AddPosition(cconvert_x(con,ov2.pos[0],ov2.pos[1]),cconvert_y(con,ov2.pos[0],ov2.pos[1]))
                                line.append(vertice1)
                                line.append(vertice2)
                                if(obj_num==0):
                                    Obj1_AllLine.append(line)
                                else:
                                    Obj2_AllLine.append(line)
                    # print(len(Obj1_AllLine),len(Obj2_AllLine))
                    for obj1_index in range(len(Obj1_AllLine)):
                        for obj2_index in range(len(Obj2_AllLine)):
                            # print(obj1_index, obj2_index)
                            obj1_v = Obj1_AllLine[obj1_index]
                            obj2_v = Obj2_AllLine[obj2_index]
                            if(vertice_collision(obj1_v[0],obj1_v[1],obj2_v[0],obj2_v[1])):
                                collide = True
    return collide
#input configuration, control point, PTField // output U(q)

# =================== BFS ==============================
def Arbitration(PtField,PtField2,x0,y0,x1,y1,con):
    c_x0 = int(cconvert_x(con,x0,y0))
    c_y0 = int(cconvert_y(con,x0,y0))
    c_x1 = int(cconvert_x(con,x1,y1))
    c_y1 = int(cconvert_y(con,x1,y1))
    # print("x,y is",c_x0,c_y0,c_x1,c_y1)
    # print("pt is",PtField[c_x0][c_y0],PtField2[c_x1][c_y1])
    return 0.9*PtField[c_x0][c_y0] + 0.1*PtField2[c_x1][c_y1]
def Empty(Tree):   #if empty return 1
    for i in range(256):
        if len(Tree[i])>0:
            return 0
    return 1
def First(Tree):
    min=-1
    for i in range(256):
        if len(Tree[i])!=0:
            min = i
            break
    # print(min)
    return Tree[min][0]
def First(Tree):
    min=-1
    for i in range(256):
        if len(Tree[i])!=0:
            min = i
            break
    # print(min)
    return Tree[min][0]
def delfirst(Tree):
    min = -1
    for i in range(256):
        if len(Tree[i])!=0:
            min = i
            break
    del Tree[min][0]
def BFS(obstaclelist,robotlist,PtField1,PtField2,all_node):
    Tree = list()
    for i in range(256):
        nodes = list()
        Tree.append(nodes)

    if (collision(obstaclelist, robotlist)):
        print("collision")
    s_robot_c0 = robotlist[0].control[0]
    s_robot_c1 = robotlist[0].control[1]

    mark =list()    # dx=1 dy=1 dd=3
    for i in range(128):
        y = list()
        for j in range(128):
            d = list()
            for k in range(360):
                d.append(0)
            y.append(d)
        mark.append(y)

    #initial position and U
    U = Arbitration(PtField1, PtField2, s_robot_c0.pos[0], s_robot_c0.pos[1], s_robot_c1.pos[0], s_robot_c1.pos[1],
                    robotlist[0].con)

    for i in range(3):
        robotlist[0].con[i] = int(robotlist[0].con[i])

    init_node = ListNode(robotlist[0].con,U)
    # print("u is ",U)

    Tree[int(U)].append(init_node)

    # print("array pos is",int(U))
    #mark it
    x = int(robotlist[0].con[0])
    y = int(robotlist[0].con[1])
    d = int(robotlist[0].con[2])
    mark[x][y][d] = 1
    success=0
    time=0
    #start loop
    while(Empty(Tree)==0 and success==0):
        time+=1
        c_node = First(Tree)
        delfirst(Tree)
        c_con = c_node.configuration
        dx = [+1, -1, 0, 0, 0, 0]
        dy = [0, 0, +1, -1, 0, 0]
        dd = [0, 0, 0, 0, +10, -10]
        #print("current is",c_con[0], c_con[1], c_con[2])
        for i in range(6):
            robotlist2 = copy.deepcopy(robotlist)
            n_x = int(c_con[0])+dx[i]
            n_y = int(c_con[1])+dy[i]
            n_d = int(c_con[2])+dd[i]
            if(n_d>=360):
                n_d-=360
            if(n_d<0):
                n_d+=360
            if(n_x>=128):
                n_x-=128
            if(n_x<0):
                n_x+=128
            if(n_y>=128):
                n_y-=128
            if(n_y<0):
                n_y+=128
            # print("next is", n_x, n_y, n_d)
            if(mark[n_x][n_y][n_d]==0):
                #print("next is unmark")
                n_con = [n_x,n_y,n_d]
                n_c0_x = cconvert_x(n_con,s_robot_c0.pos[0],s_robot_c0.pos[1])
                n_c0_y = cconvert_y(n_con,s_robot_c0.pos[0],s_robot_c0.pos[1])
                n_c1_x = cconvert_x(n_con,s_robot_c1.pos[0],s_robot_c1.pos[1])
                n_c1_y = cconvert_y(n_con,s_robot_c1.pos[0],s_robot_c1.pos[1])
                if (n_c0_x>=127 or n_c0_x<0 or n_c0_y>=127 or n_c0_y<0 or n_c1_x>=127 or n_c1_x<0 or n_c1_y>=127 or n_c1_y<0):
                    # print("here")
                    continue
                else:
                    for i in range(3):
                        robotlist2[0].con[i] = n_con[i]
                    if(collision(obstaclelist,robotlist2)):
                        # print("collision")
                        continue
                    else:
                        # print("valid next is",n_con[0],n_con[1],n_con[2])
                        U = Arbitration(PtField1, PtField2, s_robot_c0.pos[0], s_robot_c0.pos[1], s_robot_c1.pos[0],
                                        s_robot_c1.pos[1],n_con)
                        # print("U is",U)
                        if(U==0):
                            # print("find")
                            # print(n_con)
                            goal_node=ListNode(n_con,U)
                            goal_node.previous = c_node
                            all_node.append(goal_node)
                            success +=1
                        else:
                            next_node = ListNode(n_con,U)
                            next_node.previous = c_node
                            # print("u is",int(U))
                            Tree[int(U)].insert(0,next_node)
                    mark[n_x][n_y][n_d]=1
            else:
                pass
                # print("it has been marked",n_x,n_y,n_d)
    if success:
        while (1):
            # print(all_node[0].configuration)
            if (all_node[0].previous == None):
                break
            all_node.insert(0, all_node[0].previous)
        return 1
    else:
        return 0



# ================ animate ===========================
def robot_animate(cv_targetlist,targetlist,canvas,node,cv_linelist):
    CanvasRemove(cv_targetlist[0],canvas)
    cv_polygon = list()
    for polygon in targetlist[0].poly:
        all_vertice = list()
        for vertice in polygon.ver:
            c_x = cconvert_x(node.configuration,vertice.pos[0],vertice.pos[1])
            c_y = cconvert_y(node.configuration,vertice.pos[0],vertice.pos[1])
            all_vertice.append(c_x*400/128)
            all_vertice.append(400-c_y*(400/128))
        c_poly = canvas.create_polygon(all_vertice,fill=targetlist[0].color)
        cv_polygon.append(c_poly)
    cv_targetlist[0] = cv_polygon
    canvas.update()
