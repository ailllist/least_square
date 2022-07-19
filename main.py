import GetSetPos as gp
import read_obs_rinex as ror
import numpy as np
import math

STOP_ITER = 1e-5

def RotSatPos(sat: gp.GpsSat, obs):
    STT = obs/299_792_458
    ome_e = 7.2921151467e-5
    rota = ome_e * STT
    R_e = np.array([[math.cos(rota), math.sin(rota), 0], [-1 * math.sin(rota), math.cos(rota), 0], [0, 0, 1]])
    P_SatPos = np.array([sat.xk, sat.yk, sat.zk]).T
    SatPos = R_e @ P_SatPos
    sat.xk, sat.yk, sat.zk = SatPos[0], SatPos[1], SatPos[2]

def least_square_est(H: np.ndarray, y: np.ndarray, x: np.ndarray):
    # TODO x관련 식 추가 해줘야됨.
    # print(H.T@H)
    p_x_hat = np.linalg.inv(H.T@H)
    
    x_hat = p_x_hat@H.T@y
    x = x + x_hat
    norm = np.linalg.norm(x_hat)
    print("x_hat: ", x_hat)
    print("norm(x_hat[x, y, z, delta_tr]): ", norm)
    print("x: ", x)
    print("---------------------------------------------------")
    return x, norm

TRUE_POS = [-3062023.912, 4055448.883, 3841819.124]

APPROX_POSITION_XYZ = [-3062023.5630, 4055449.0330, 3841819.2130]
calc_time = [1, 45, 0] # hour, min, sec, GPS_Week_sec
tgpt = ror.tot_sat_pos # Total Gps Pos for time (dict)
keys = list(tgpt.keys())
# 15  2  1  0  0  0.0000000
time_dist = float("inf")
best_time = None
for i in keys:
    tmp_str = ""
    for num, j in enumerate(i):
        if j == " " and i[num+1] == " ":
            pass
        else:
            tmp_str += j

    time_list = tmp_str.split(" ")
    tmp_dist = (calc_time[0] - int(time_list[3]))*3600 + \
    (calc_time[1] - int(time_list[4]))*60 + 0 - float(time_list[5])

    if tmp_dist >= 0 and time_dist > tmp_dist:
        time_dist = tmp_dist
        best_time = i

print("best_time : ", best_time)
                                                                
_15osat = tgpt[best_time]
obs_sats = list(_15osat.keys())
obs_gps_sats = []
for i in obs_sats:
    if i[0] == "G":
        obs_gps_sats.append(i)
    else:
        pass

pre_time = ""
for num, i in enumerate(best_time):
    if i == " " and best_time[num+1] == " ":
        pass
    else:
        pre_time += i

obs_time = pre_time.split(" ")
obs_cal_time = [int(obs_time[3]), int(obs_time[4]), 0]

tot_data = {} # 관측된 모든 GPS 위성이 들었는 dict 객체

for i in obs_gps_sats:
    if i[1] == "0":
        GpsSat_list = gp.read_data(i[2])
    else:
        GpsSat_list = gp.read_data(i[1:])
    bd_sat = gp.find_best_time(GpsSat_list, calc_time)
    print(bd_sat.PRN, bd_sat.Epoch)
    tot_data[i] = bd_sat
gps_prn_list = list(tot_data.keys())

H = np.zeros((len(gps_prn_list), 4))
y = np.zeros(len(gps_prn_list))
x = np.array([-3062023.5630, 4055449.0330, 3841819.2130, 1])

for eph in range(10):
    print("-------------------------%d-------------------------" % eph)

    for num, i in enumerate(gps_prn_list):
    
        CA_code = float(_15osat[i]["C1"][0].strip())
    
        STT = CA_code / 299_792_458
        tot_data[i].calc_gps_pos(obs_cal_time, STT) # calc_time 은 obs_rinex기준으로 둘 것
        # tot_data[i].calc_gps_pos(calc_time)
        RotSatPos(tot_data[i], CA_code)
        tk = tot_data[i].calc_tk(calc_time, STT)
        delta_ts = tot_data[i].SV_clock_bias + tot_data[i].SV_clock_drift * tk
        
        cal_x = tot_data[i].xk - x[0]
        cal_y = tot_data[i].yk - x[1]
        cal_z = tot_data[i].zk - x[2]
        rho = math.sqrt(cal_x**2 + cal_y**2 + cal_z**2) + x[3] - 299_792_458*(-1*delta_ts) 
    
        y[num] = CA_code - rho
        H[num][0] = -1 * (cal_x)/rho
        H[num][1] = -1 * (cal_y)/rho
        H[num][2] = -1 * (cal_z)/rho
        H[num][3] = 1 # (m)
    
    print("H array : ")
    print(H)
    print("y array : ")
    print(y)
    x, norm = least_square_est(H, y, x)

    if norm < STOP_ITER:
        print("Iteration stop at %d" % eph)
        break

p_3D_Error = TRUE_POS[:3] - x[:3]
print(TRUE_POS, x)
print(p_3D_Error)