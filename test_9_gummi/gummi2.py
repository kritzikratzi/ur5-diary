import urx
import time
#from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from robotiq_hande import Robotiq_HandE
import math
import urllib.request, json 
import subprocess

a = 0.5
v = 0.5


rob = urx.Robot("192.168.0.100"); 
robotiqgrip = Robotiq_HandE(rob)

#robotiqgrip.open_gripper()
#robotiqgrip.open_mm(12)
g_pts = []; 
def append_pt(pt): 
    global g_pts;
    g_pts.append(pt)
    
def run_movej(aa=a,vv=v,r=0.005): 
    global g_pts;
    rob.movexs("movej", g_pts, a, v, r)
    g_pts = []
    
def run_movel(aa=a,vv=v,r=0.005): 
    global g_pts;
    rob.movexs("movel", g_pts, a, v, r)
    g_pts = []
    


def place(dx,dy):
    x = 0.175 + dx
    y = 0.400 + dy
    append_pt((-0.250,0.400,0.1,0,0,0))
    append_pt((0.175,0.400,0.1,0,0,0))
    append_pt((x,y,0.1,0,0,0))
    append_pt((x,y,0.014,0,0,0))
    run_movel(a,v)
    time.sleep(1)
    robotiqgrip.open_mm(12)
    append_pt((x,y,0.1,0,0,0))
    append_pt((-0.250,0.400,0.1,0,0,0))
    append_pt((-0.250,-0.260,0.1,0,0,0))
    run_movej()
    robotiqgrip.open_mm(20)

def swap_many_places(src,dest):

    robotiqgrip.open_mm(14)
    append_pt((-0.250,0.400,0.1,0,0,0))
    append_pt((0.175,0.400,0.1,0,0,0))
    for pt_src,pt_dest in zip(src,dest):
        x1 = 0.175 + pt_src[0]
        y1 = 0.400 + pt_src[1]
        x2 = 0.175 + pt_dest[0]
        y2 = 0.400 + pt_dest[1]
        
        append_pt((x1,y1,0.1,0,0,0))
        append_pt((x1,y1,0.014,0,0,0))
        run_movel(a,v)
        robotiqgrip.open_mm(8)
        time.sleep(1)
        append_pt((x1,y1,0.1,0,0,0))
        append_pt((x2,y2,0.1,0,0,0))
        append_pt((x2,y2,0.014,0,0,0))
        run_movel(a,v)
        robotiqgrip.open_mm(20)
        time.sleep(1)
        append_pt((x2,y2,0.1,0,0,0))
        
    append_pt((-0.250,0.400,0.1,0,0,0))
    append_pt((-0.250,-0.260,0.1,0,0,0))
    run_movej()

def swap_places(dx1,dy1,dx2,dy2):
    swap_many_places([(dx1,dy1)],[(dx2,dy2)])
    


#zzz=0.022
#rob.movel((-0.092, -0.509, zzz,0,0,0), a,v);
#time.sleep(1);
#rob.movel((0.058,-0.514,zzz,0,0,0), a, v);
#time.sleep(1);
#rob.movel((0.0635,-0.364,zzz,0,0,0), a,v );
#time.sleep(1);
#rob.movel((-0.0865,-0.359,zzz,0,0,0), a, v);

num_placed = 0
max_num_placed = 0.5/0.025

def prog_pick(pts,cnt=0): 
    global num_placed
    # pick up
    print("---- pick up sequence ---")
    picked = 0
    for pt in pts: 
        poshi = (pt["x"],pt["y"],0.06,0,0,-pt["rz"]+3.1415/2-3.1415)
        posmi = (pt["x"],pt["y"],0.02,0,0,-pt["rz"]+3.1415/2-3.1415)
        poslo = (pt["x"],pt["y"],0.005,0,0,-pt["rz"]+3.1415/2-3.1415)
        append_pt(poshi)
        append_pt(posmi)
        run_movej(0.5,1.5)
        robotiqgrip.open_mm(20)
        rob.movel(poslo, 0.2,0.2)
        robotiqgrip.open_mm(8)
        append_pt(posmi)
        append_pt(poshi)
        #append_pt((-0.160,-0.380,0.04,0,0,0))
        append_pt((-0.250,-0.260,0.005,0,0,0))
        run_movel(0.15,1.5)
        #rob.movel((-0.160,-0.380,0.04,0,0,0), 0.5, 0.5)
        robotiqgrip.open_mm(20)
        time.sleep(0.5)
        rob.movel((-0.250,-0.260,0.02,0,0,0), a, v)
        rob.movel((-0.250,-0.260,0.02,0,3*3.1415/2,0), a, v)
        rob.movel((-0.250,-0.260,0.015,0,3*3.1415/2,0), a, v)
        time.sleep(0.5)
        robotiqgrip.open_mm(8)
        rob.movel((-0.250,-0.260,0.02,0,3*3.1415/2,0), a, v)
        rob.movel((-0.250,-0.260,0.02,0,0,0), a, v)
        
        num_placed = num_placed + 1
        picked = picked + 1
        if cnt != 0 and picked >= cnt:
            break
    
    return 0 if cnt == 0 else picked


def pick1(): 
    while True: 
        with urllib.request.urlopen("http://localhost:8081/bears") as url:
            data = json.loads(url.read().decode())
            cnt = prog_pick(data["points"],1)
            if cnt >= 1: 
                return
            else:
                time.sleep(1)

def make_offset(n, dx, dy): 
    return [(dx*i,dy*i) for i in range(n)]

def add_x(el, dx):
    return [(p[0]+dx,p[1]) for p in el]

def add_y(el, dy):
    return [(p[0],p[1]+dy) for p in el]

def pick_and_place_many(places):
    for p in places: 
        pick1()
        place(p[0],p[1])
        subprocess.run(["./gphoto2_capture.sh"])
        
n=5
places = make_offset(n, 0,0.03)
places = add_y(places, -0.05)
pick_and_place_many(places)

n=5
places2 = make_offset(n, 0,0.03)
places2 = add_x(places2, 0.03)
places2 = add_y(places2, 0.035)
pick_and_place_many(places2)

n=5
places3 = make_offset(n, 0,0.03)
places3 = add_x(places3, 0.06)
places3 = add_y(places3, 0.0)
pick_and_place_many(places3)

#swap_many_places(places, places2)


#swap_places(0.03,+0.03*3, -0.07,+0.03*3)

#pick()
#place(0)
#for i in range(0,15):
#    pick()
#    place(i/15.0)

rob.close()