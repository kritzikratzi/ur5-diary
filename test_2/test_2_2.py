import rtde_control
rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")
res = rtde_c.moveL([-0.143, -0.435, 0.10, 0, 0, 0], 0.5, 0.3)
res = rtde_c.moveL([-0.143, -0.435, 0.20, 0, 0, 0], 0.5, 0.3)
res = rtde_c.moveL([-0.143, -0.435, 0.30, 0, 0, 0], 0.5, 0.3)
print("result: " + str(res))