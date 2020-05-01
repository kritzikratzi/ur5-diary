import urx


a = 0.5
v = 1.5
nx = 10
ny = 10
cx = 0
cy = -0.45
z_up = 0.05
z_down = 0
size = 0.15

rob = urx.Robot("192.168.0.100"); 

def scale_linc(x, from_0, from_1, to_0, to_1): 
    return (x-from_0)/(from_1-from_0)*(to_1-to_0) + to_0

def scale_lin(x, from_0, from_1, to_0, to_1): 
    res = (x-from_0)/(from_1-from_0)*(to_1-to_0) + to_0
    if res<to_0: return to_0; 
    if res>to_1: return to_1; 
    return res


def grid_pos(ix,iy, z0): 
    return (
        scale_linc(ix, 0,nx-1, cx-size, cx+size), 
        scale_linc(iy, 0,ny-1, cy-size, cy+size), 
        z0, 
        0, 0, 0
    )

for ix in range(5,10): 
    rob.movel(grid_pos(ix,0,z_up), a, v)
    rob.movel(grid_pos(ix,0,z_down), a, v)
    rob.movel(grid_pos(ix,ny,z_down), a, v)
    rob.movel(grid_pos(ix,ny,z_up), a, v)
    
for iy in range(0,10): 
    rob.movel(grid_pos( 0, iy,z_up), a, v)
    rob.movel(grid_pos( 0, iy,z_down), a, v)
    rob.movel(grid_pos(nx, iy,z_down), a, v)
    rob.movel(grid_pos(nx, iy,z_up), a, v)
    
rob.close()