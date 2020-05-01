import urx
from scipy.io import wavfile

fs, data_full = wavfile.read('shuttle-table.wav')
data = data_full[0::10]


wav_amplitude = 0

for coord in data: 
    wav_amplitude = 1.0*max(abs(coord[0]), wav_amplitude )
    wav_amplitude = 1.0*max(abs(coord[1]), wav_amplitude )
    
print("max wav amplitude = ", wav_amplitude)

a = 1.0
v = 1.2
nx = 10
ny = 10
cx = 0
cy = -0.4
z_up = 0.05
z_down = 0
size = 0.15

rob = urx.Robot("192.168.0.100"); 

def scale_linc(x, from_0, from_1, to_0, to_1): 
    return (x*1.0-from_0)/(from_1-from_0)*(to_1-to_0) + to_0

def scale_lin(x, from_0, from_1, to_0, to_1): 
    res = (x*1.0-from_0)/(from_1-from_0)*(to_1-to_0) + to_0
    if res<to_0: return to_0; 
    if res>to_1: return to_1; 
    return res

def movel_wav(idx, z0): 
    pos = (
        scale_linc(data[idx][0], -wav_amplitude,wav_amplitude, cx-size, cx+size), 
        scale_linc(data[idx][1], -wav_amplitude,wav_amplitude, cy-size, cy+size), 
        z0, 
        0, 0, 0
    )
    print(idx, " => ", pos)
    rob.movel(pos,a,v)

movel_wav(0,0.05)
for idx in range(len(data)): 
    #print("pt ", idx+1, " / ", len(data))
    movel_wav(idx,0)

movel_wav(-1,0.05)