import urx


a = 0.9
v = 2.0


rob = urx.Robot("192.168.0.100"); 
rob.movel((+0.1,-0.5,0.05, 0, 0, 0), a, v)
rob.movel((+0.1,-0.5,0.05, 1, 0, 0), a, v)
rob.movel((+0.1,-0.5,0.05, 0, 1, 0), a, v)
rob.movel((+0.1,-0.5,0.05, 0, 0, 1), a, v)
rob.movel((+0.1,-0.5,0.05, 0, 0, 0), a, v)
rob.movel((+0.1,-0.5,0.00, 0, 0, 0), a, v)
rob.movel((-0.1,-0.5,0.00, 0, 0, 0), a, v)
rob.movel((-0.1,-0.5,0.05, 0, 0, 0), a, v)
rob.close()