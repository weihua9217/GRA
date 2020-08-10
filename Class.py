class Robot():
    def __init__(self,name):
        self.name = name
        self.poly = list()
        self.con = list()
        self.control = list()
        self.bdbox = list()
        self.fbdbox = list()
        self.selected = False
        self.rselected = False
        self.color = '#444444'

    def AddPolygon(self,polygon):
        self.poly.append(polygon)

    def AddCon(self,x,y,z):
        self.con.append(x)
        self.con.append(y)
        self.con.append(z)

    def AddControl(self,x):
        self.control.append(x)

    def ShowPolyNum(self):
        print(len(self.poly))

    def ShowPolyPosition(self):
        for i in self.poly:
            i.ShowVerticePostion()
            print("============")


class Obstacle():
    def __init__(self, name):
        self.name = name
        self.poly = list()
        self.con = list()
        self.bdbox = list()
        self.selected = False
        self.rselected = False
        self.color = "#303F9F"
    def AddPolygon(self,polygon):
        self.poly.append(polygon)

    def Addcon(self,x,y,z):
        self.con.append(x)
        self.con.append(y)
        self.con.append(z)

    def ShowPolyNum(self):
        print(len(self.poly))

    def ShowPolyPosition(self):
        print(len(self.poly))
        for i in self.poly:
            i.ShowVerticePostion()
            print("============")

class Polygon():
    def __init__(self, name):
        self.name = name
        self.ver = list()

    def AddVertice(self,vertice):
        self.ver.append(vertice)

    def ShowVerticeNum(self):
        print(len(self.ver))

    def ShowVerticePostion(self):
        for i in self.ver:
            i.ShowPosition()


class Vertice():
    def __init__(self, name):
        self.name = name
        self.pos = list()

    def __add__(self,o):
        v = Vertice("temp")
        for i in range(len(self.pos)):
            v.pos.append(self.pos[i] + o.pos[i])
        return v

    def __sub__(self,o):
        v = Vertice("temp")
        for i in range(len(self.pos)):
            v.pos.append(self.pos[i] - o.pos[i])
        return v

    def AddPosition(self,x,y):
        self.pos.append(x)
        self.pos.append(y)

    def ShowPosition(self):
        print(self.pos[0],self.pos[1])

    def ConvertVertical(self):
        temp = self.pos[0]
        self.pos[0] = self.pos[1]
        self.pos[1] = -temp

    def InnerProduct(self,o):
        return (self.pos[0]*o.pos[0]+self.pos[1]*o.pos[1])


class ControlPoint():
    def __init__(self,name):
        self.name = name
        self.pos = list()

    def AddPosition(self,x,y):
        self.pos.append(x)
        self.pos.append(y)


class ListNode():
    def __init__(self,c,u):
        self.configuration = c
        self.U = u
        self.previous = None

class DoubleLinkedList():
    def __init__(self):
        self.head = None
        self.tail = None