import urx
import time
#from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from robotiq_hande import Robotiq_HandE
import math
import urllib.request, json 

a = 0.5
v = 0.5


rob = urx.Robot("192.168.0.100"); 
robotiqgrip = Robotiq_HandE(rob)

#robotiqgrip.open_gripper()
#robotiqgrip.open_mm(12)

z_high = -0.00
z_pick = -0.038
z_place = -0.052
z_placelo = -0.055

home = (-0.2042,0.5419,-0.017, 0.0129,3.1115,0.0568)

def pick(): 
    inter = (-0.1642,0.5347,-0.017, 0.0275,2.6592,-0.077)
    grab_hi = (-0.1642,0.5371,0.0341, 0.0518,1.5647,-0.0421)
    grab_lo = (-0.1642,0.5371,0.0166, 0.0518,1.5647,-0.0421)
    
    robotiqgrip.open_mm(30)
    #rob.movel(inter,0.1,0.1)
    rob.movexs("movej", [home,inter,grab_hi], 1.5, 2, 0.005)
    rob.movel(grab_lo, 0.1, 0.1)
    robotiqgrip.open_mm(9)
    time.sleep(1)
    rob.movexs("movej", [grab_hi,inter,home], 1, 2, 0.005)

def pick_xy_rz(x,y,rz): 
    grab_hi = (x,y,0.0341, 0,0,rz)
    grab_lo = (x,y,0.0066, 0,0,rz)
    
    robotiqgrip.open_mm(15)
    #rob.movel(inter,0.1,0.1)
    rob.movexs("movel", [grab_hi,grab_lo], 0.5, 1, 0.005)
    robotiqgrip.open_mm(6)
    time.sleep(1)
    rob.movexs("movel", [grab_hi], 0.5, 1, 0.005)


def place(xpct):
    a = 1
    v = 1.5
    x0=0.27
    x1=0.0
    x = x0*xpct + x1*(1-xpct)
    y0=0.40769145690429764
    #rob.movel((x0, y0, z_high, math.pi, 0, 0), a, v)
    #rob.movel((x1, y0, z_high, math.pi, 0, 0), a, v)
    hi = (x, y0, z_high, 0,math.pi, 0)
    mi = (x, y0, z_place, 0,math.pi, 0)
    lo = (x, y0, z_placelo, 0,math.pi, 0)
    robotiqgrip.open_mm(9)
    rob.movexs("movej", [hi,mi], 1.5, 2, 0.025)
    robotiqgrip.open_mm(11)
    rob.movel(lo,0.1,0.1)
    robotiqgrip.open_mm(20)
    time.sleep(1.0)
    rob.movexs("movej", [hi,home], 1.5, 2, 0.025)

#zzz=0.022
#rob.movel((-0.092, -0.509, zzz,0,0,0), a,v);
#time.sleep(1);
#rob.movel((0.058,-0.514,zzz,0,0,0), a, v);
#time.sleep(1);
#rob.movel((0.0635,-0.364,zzz,0,0,0), a,v );
#time.sleep(1);
#rob.movel((-0.0865,-0.359,zzz,0,0,0), a, v);

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
    
def prog_touch(pts): 
    print("---- touch sequence ---")
    for pt in pts: 
        pos = (pt["x"],pt["y"],0.04,0,0,-pt["rz"]+3.1415/2)
        posl = (pt["x"],pt["y"],0.01,0,0,-pt["rz"]+3.1415/2)
        append_pt(pos); 
        append_pt(posl); 
        append_pt(pos); 
    
    run_movel(0.5,0.5)
    rob.movel((-0.160,-0.380,0.04,0,0,0), 0.5, 0.5)

num_placed = 0
max_num_placed = 0.5/0.025

def prog_pick(pts,cnt=0): 
    global num_placed
    # pick up
    print("---- pick up sequence ---")
    picked = 0
    for pt in pts: 
        poshi = (pt["x"],pt["y"],0.06,0,0,-pt["rz"]+3.1415/2)
        posmi = (pt["x"],pt["y"],0.02,0,0,-pt["rz"]+3.1415/2)
        poslo = (pt["x"],pt["y"],0.005,0,0,-pt["rz"]+3.1415/2)
        append_pt(poshi)
        append_pt(posmi)
        run_movej(0.5,1.5)
        robotiqgrip.open_mm(20)
        rob.movel(poslo, 0.2,0.2)
        robotiqgrip.open_mm(8)
        append_pt(posmi)
        append_pt(poshi)
        #append_pt((-0.160,-0.380,0.04,0,0,0))
        place_a = (-0.250,-0.230,0.04,0,0,0)
        place_b = ( 0.250,-0.230,0.04,0,0,0)
        tgt_hi = (-0.250 + 0.5*(num_placed/max_num_placed), -0.260,0.04, 0,0,0)
        tgt_lo = list(tgt_hi)
        tgt_lo[2] = 0.005
        append_pt(tgt_hi)
        append_pt(tgt_lo)
        run_movel(0.15,1.5)
        #rob.movel((-0.160,-0.380,0.04,0,0,0), 0.5, 0.5)
        robotiqgrip.open_mm(15)
        time.sleep(0.5)
        append_pt(tgt_hi)
        
        num_placed = num_placed + 1
        picked = picked + 1
        if cnt != 0 and picked >= cnt:
            break
    
    return 0 if cnt == 0 else (len(pts)-picked)
    
loop = True
fails = 0
#robotiqgrip.open_mm(0)

#loop = False
#rob.movel((-0.250,-0.230,0.04,0,0,0), 0.1, 0.2)
#rob.movel((0.250,-0.230,0.04,0,0,0), 0.1, 0.2)

first = True
if True: 
    # touch all 
    for idx in range(0,int(max_num_placed)): 
        pos = (-0.250 + 0.5*(idx/max_num_placed), -0.260,0.04, 0,0,0)
        posl = list(pos)
        posl[2] = 0.01
        append_pt(pos); 
        append_pt(posl); 
        append_pt(pos); 

    run_movel(0.5,0.5)
    rob.movel((-0.160,-0.380,0.04,0,0,0), 0.5, 0.5)
    num_placed = 0

while loop: 
    with urllib.request.urlopen("http://localhost:8081/bears") as url:
        data = json.loads(url.read().decode())
            
        if "points" in data.keys() and len(data["points"])>0: 
            if first: 
                #prog_touch(data["points"])
                first = False
                
            fails = 0
            prog_pick(data["points"],1)
                
            #append_pt((-0.160,-0.380,0.04,0,0,0))
            #run_movel()
        else: 
            fails = fails + 1
            time.sleep(0.1)
            #if fails > 20:
            #    #loop = False
            #    break

#pick()
#place(0)
#for i in range(0,15):
#    pick()
#    place(i/15.0)

rob.close()