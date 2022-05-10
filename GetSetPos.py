import get_GPS_info
from math import sin, cos, atan2, sqrt


class GpsSat:

    def __init__(self, d_arr):
        self.raw_data = d_arr
        self.PRN = d_arr[0][0].split(" ")[0]
        self.Epoch = d_arr[0][0].split(" ")[1:]  # year, month, day, hour, min, sec
        self.SV_clock_bias = float(d_arr[0][1])  # s
        self.SV_clock_drift = float(d_arr[0][2])  # s/s
        self.SV_clock_drift_rate = float(d_arr[0][3])  # s/s^2

        self.IODE = float(d_arr[1][0])  # -
        self.Crs = float(d_arr[1][1])  # rad
        self.delta_n = float(d_arr[1][2])  # rad/s
        self.M_0 = float(d_arr[1][3])  # rad

        self.Cuc = float(d_arr[2][0])  # rad
        self.e = float(d_arr[2][1])  # -
        self.Cus = float(d_arr[2][2])  # rad
        self.sqrt_a = float(d_arr[2][3])  # Warning! (sqrt(m))

        self.toe = float(d_arr[3][0])  # GPS Week Sec
        self.Cic = float(d_arr[3][1])  # rad
        self.Omega_0 = float(d_arr[3][2])  # rad
        self.Cis = float(d_arr[3][3])  # rad

        self.i_0 = float(d_arr[4][0])  # rad
        self.Crc = float(d_arr[4][1])  # rad
        self.Omega_s = float(d_arr[4][2])  # rad
        self.Omega_dot = float(d_arr[4][3])  # rad/s

        self.i_dot = float(d_arr[5][0])  # rad/s
        self.Codes_on_L2_channel = float(d_arr[5][1])  # -
        self.GPS_Week_Number = float(d_arr[5][2])  # -
        self.L2_P_data_flag = float(d_arr[5][3])  # -

        self.SV_accuracy = float(d_arr[6][0])  # m
        self.SV_health = float(d_arr[6][1])  # -
        self.TGD = float(d_arr[6][2])  # s
        self.IODC = float(d_arr[6][3])  # -

        self.Tx_time_of_message = float(d_arr[7][0])  # GPS Week Sec
        self.Fit_interval = float(d_arr[7][1])  # h
        self.sqare1 = float(d_arr[7][2])  # -
        self.sqare2 = float(d_arr[7][3])  # -

        # step 1
        self.mu = 3.986005e+14
        self.Omega_e_dot = 7.2921151467e-5

        self.xk, self.yk, self.zk = 0, 0, 0

    def calc_mean_motion(self):  # step 2
        n0 = sqrt(self.mu / self.sqrt_a**6)
        return n0

    def calc_rmean_motion(self, n0):  # step 3
        n = n0 + self.delta_n
        return n

    def calc_tk(self, cal_time, STT=0):  # step 4 time 2015/2/1 (일요일) 17:45 -> GPS Week Second
        t = cal_time[0] * 3600 + cal_time[1] * 60 + cal_time[2] * 3600 * 24 - STT
        tk = t - self.toe
        return tk

    def calc_Mk(self, n, tk):  # step 5
        Mk = self.M_0 + n * tk
        return Mk

    def calc_Ek(self, Mk):  # step 6
        Ek = calc_E(Mk, self.e)
        return Ek

    def calc_TA(self, Ek):  # step 7
        fk = atan2(sqrt(1 - self.e ** 2) * sin(Ek), cos(Ek) - self.e)
        return fk

    def calc_aol(self, fk):  # step 8
        phik = fk + self.Omega_s
        return phik

    def calc_delta_uri(self, phik):  # step 9
        delta_uk = self.Cus * sin(2 * phik) + self.Cuc * cos(2 * phik)
        delta_rk = self.Crs * sin(2 * phik) + self.Crc * cos(2 * phik)
        delta_ik = self.Cis * sin(2 * phik) + self.Cic * cos(2 * phik)
        return delta_uk, delta_rk, delta_ik

    def calc_uri(self, delta_uk, delta_rk, delta_ik, phik, Ek, tk):  # step 10
        uk = phik + delta_uk
        rk = (self.sqrt_a ** 2) * (1 - self.e * cos(Ek)) + delta_rk
        ik = self.i_0 + delta_ik + (self.i_dot) * tk
        return uk, rk, ik

    def calc_xkd_ykd(self, uk, rk):  # step 11
        xkd = rk * cos(uk)
        ykd = rk * sin(uk)
        return xkd, ykd

    def calc_Omega_k(self, tk):  # step 12
        Omega_k = self.Omega_0 + (self.Omega_dot - self.Omega_e_dot) * tk - \
                  self.Omega_e_dot * self.toe
        return Omega_k

    def calc_xyz(self, Omega_k, xkd, ykd, ik):  # step 13
        xk = xkd * cos(Omega_k) - ykd * cos(ik) * sin(Omega_k)
        yk = xkd * sin(Omega_k) + ykd * cos(ik) * cos(Omega_k)
        zk = ykd * sin(ik)
        return xk, yk, zk

    def calc_gps_pos(self, cal_time, STT=0):
        n0 = self.calc_mean_motion()
        n = self.calc_rmean_motion(n0)
        tk = self.calc_tk(cal_time, STT)
        Mk = self.calc_Mk(n, tk)
        Ek = self.calc_Ek(Mk)
        fk = self.calc_TA(Ek)
        phik = self.calc_aol(fk)
        delta_uk, delta_rk, delta_ik = self.calc_delta_uri(phik)
        uk, rk, ik = self.calc_uri(delta_uk, delta_rk, delta_ik, phik, Ek, tk)
        xkd, ykd = self.calc_xkd_ykd(uk, rk)
        Omega_k = self.calc_Omega_k(tk)
        self.xk, self.yk, self.zk = self.calc_xyz(Omega_k, xkd, ykd, ik)

    def __call__(self):
        print(f"PRN : {self.PRN}\nepoch : {str(self.Epoch)}")
        try:
            print(f"xk, yk, zk : {self.xk}, {self.yk}, {self.zk}")
        except:
            print("Yet calc gps sat pos")

    def show_raw(self):
        for num, i in enumerate(self.raw_data):
            print(num, i)

def calc_E(M, e):
    Ek = M
    exc = 1e-10

    for i in range(5):
        fE = M - Ek + e * sin(Ek)
        fpE = -1 + e * cos(Ek)
        Ekp1 = Ek - (fE / fpE)
        if abs(Ek - Ekp1) < exc:
            return Ekp1
        Ek = Ekp1

def calc_gps_pos(best_data, cal_time):
    n0 = best_data.calc_mean_motion()
    n = best_data.calc_rmean_motion(n0)
    tk = best_data.calc_tk(cal_time)
    Mk = best_data.calc_Mk(n, tk)
    print("Mk : ", Mk)
    Ek = best_data.calc_Ek(Mk)
    print("Ek : ", Ek)
    fk = best_data.calc_TA(Ek)
    print("fk : ", fk)
    phik = best_data.calc_aol(fk)
    delta_uk, delta_rk, delta_ik = best_data.calc_delta_uri(phik)
    uk, rk, ik = best_data.calc_uri(delta_uk, delta_rk, delta_ik, phik, Ek, tk)
    xkd, ykd = best_data.calc_xkd_ykd(uk, rk)
    Omega_k = best_data.calc_Omega_k(tk)
    xk, yk, zk = best_data.calc_xyz(Omega_k, xkd, ykd, ik)
    return xk, yk, zk

def find_best_time(GpsSat_list, cal_time):
    best_data = None
    best_time_val = float("inf")

    for i in GpsSat_list:
        # print(i.Epoch)
        Sat_time = int(i.Epoch[3]) * 3600 + int(i.Epoch[4]) * 60
        Now_time = int(cal_time[0]) * 3600 + int(cal_time[1]) * 60
        time_val = Now_time - Sat_time
        if time_val > 0:
            if time_val < best_time_val:
                best_time_val = time_val
                best_data = i
        else:
            continue
    
    return best_data

def read_data(prn):
    PRN_list = get_GPS_info.tot_GPS

    target_sat = PRN_list[f"{prn}"]
    GpsSat_list = []
    for i in target_sat:
        GpsSat_list.append(GpsSat(i))

    return GpsSat_list

if __name__ == "__main__":
    GROUND_TRUE = [6731.601514, 15907.205394, -20553.641738]
    PRN_NUMBER = 2
    CAL_TIME = [17, 45, 0] # hour, minute, gps week day

    GpsSat_list = read_data(PRN_NUMBER)
    best_data = find_best_time(GpsSat_list, CAL_TIME)
    best_data.show_raw()
    # [print(i) for i in best_data.raw_data]
    xk, yk, zk = calc_gps_pos(best_data, CAL_TIME)
    print("using data PRN: %d, yy/mm/dd/hh/mm/ss: %02d/%02d/%02d/%02d/%02d/%02d"
          % (PRN_NUMBER, int(best_data.Epoch[0]), int(best_data.Epoch[1]), int(best_data.Epoch[2])
             , int(best_data.Epoch[3]), int(best_data.Epoch[4]), int(float(best_data.Epoch[5]))))
    print(f"Pred X: {xk/1000}, Pred Y: {yk/1000}, Pred Z: {zk/1000}")
    print(f"True X: {GROUND_TRUE[0]}, True Y: {GROUND_TRUE[1]}, True Z: {GROUND_TRUE[2]}")
    dist2 = (xk/1000-GROUND_TRUE[0]) ** 2 + (yk/1000-GROUND_TRUE[1]) ** 2 + (zk/1000-GROUND_TRUE[2]) ** 2
    Error_3D = sqrt(dist2)*1000
    print("x Error: %.9f, y Error: %.9f, z Error: %.9f" % (
    xk / 1000 - GROUND_TRUE[0], yk / 1000 - GROUND_TRUE[1], zk / 1000 - GROUND_TRUE[2]))
    print("3D Error: %.9f m" % Error_3D)