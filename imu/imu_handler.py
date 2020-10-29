#!/usr/bin/python

#import sys
import time
#import math
import IMU
import datetime
#import os
import numpy as np
import math
import signal
import pickle as pk
from scipy.linalg import sqrtm
"""

take measurements -- update state accordingly

measurements are from acc and mag. and world position from gps?

state is world attitude, world_attitude_dot and world velocity

from acc mag and world ..
get new state
    - get new world attitude first
    - calc ang velocities
    
    -aplly measuremnts to new 


"""
class IMU_Handler:
    def __init__(self):
        self.HIGHEST_INT = 32767
        self.mag_limits = None
        
        self.data = [] #store recent data aquires
        self.stop_aquire = False
        self.cal_filename = 'calibration.pik'
        self.mag_valid = False
        self.G_GAIN = 0.07
        self.last_read = None
        self.DOWN = np.array([0, 0, -1])
        self.GRAVITY = 0.001 # nominal acc magnitude equivalent to 1G
        self.dacc_dG = 0.001
        self.data_dt = None
        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            print(" No BerryIMU found")
        else:
            IMU.initIMU()
        self.cal_dict = {}
        
    def calibrated_mag(self, mag):
        hHat = np.matmul(self.cal_dict['Ainv'], mag.T - self.cal_dict['b'])
        hHat = np.dot(hHat.T, hHat)
#        err = (hHat[0][0] - 1)**2
        return hHat
    
    def close(self, h1, h2):
        print(type(h1), h1)
        print(type(h2), h2)
        self.stop_aquire = True
#        self.calculate_calibration(self.data)
#        self.save_calibration()
#        exit(0)
    
    def static_calibration_capture(self, N=20):
        accs = []
        gyrs = []
        for i in range(N):
            print(i)
            x=self.read_data()
#            print(x)
            accs.append(x[:3])
            gyrs.append(x[3:6])
#            self.data[i] = x
            time.sleep(0.05)
        
        gyrs = np.array(gyrs)
        accs = np.array(accs)
        
        av_gyr = np.average(gyrs, axis=0)
        std_gyr = np.std(gyrs, axis=0)
        
        av_g = np.average(accs, axis=0)
        std_g = np.std(accs, axis=0)
        
        
        
        self.cal_dict['av_g'] = av_g
        self.cal_dict['std_g'] = std_g
        self.cal_dict['av_gyr'] = av_gyr
        self.cal_dict['std_gyr'] = std_gyr
        return av_g, std_g, av_gyr, std_gyr
    
    def dynamic_calibration_capture(self):
        signal.signal(signal.SIGINT, self.close)
#        data = []
        self.data = []
        while not self.stop_aquire:
#        for i in range(N):
#            print(i)
            x=self.read_data()
            print(x)
            self.data.append(x)
#            self.data[i] = x
            time.sleep(0.05)
        self.stop_aquire = False
        self.data = np.array(self.data)
        
    
    def calculate_mag_calibration(self, data):
#        acc = data[:, 0]
#        gyr = data[:, 1]
        mag = data[:, 6:]
        
        Q, n, d = fitEllipsoid(mag[:, 0], mag[:, 1], mag[:, 2])
#        calibration = [Q, n, d]
        Qinv = np.linalg.inv(Q)
        b = -np.dot(Qinv, n)
        Ainv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(Qinv, n)) - d) * sqrtm(Q))
        
        self.cal_dict = {'Q': Q,
                    'Ainv': Ainv,
                    'b': b,
                    'n': n,
                    'd': d}
        
    def save_calibration(self):
#        np.savetxt('calibration.csv', self.mag_limits, delimiter=',')
        with open(self.cal_filename, '+wb') as f:
            pk.dump(self.cal_dict, f)
            print('calibration saved')
    def load_calibration(self):
#        np.savetxt('calibration.csv', self.mag_limits, delimiter=',')
        with open(self.cal_filename, 'rb') as f:
            self.cal_dict = pk.load(f)
            print('calibration loaded')
            

    def trim_heading(self, hdg):
        if hdg < 0:
            return hdg + 2 * np.pi
        else:
            return hdg


    
    def get_reading(self):
        """
        apply
            mag correction
            gyro gain 
            acc gain
            
        sets:
            mag heading
            
            gyr_w
            
            gyr_pos
            
            acc_w_dot
            
            acc_pos
            
#        """
#        now = datetime.datetime.now()
#        self.data_dt = (now - self.last_read).microseconds/(1000000*1.0)
        raw =  self.read_data()
        acc, gyr, mag = raw[:3], raw[3:6], raw[6:]
        
        mag = self.calibrated_mag(mag)
        #Convert Gyro raw to rad per second
        print(type(gyr), gyr)
        gyr *= self.G_GAIN


        
        acc_magnitude = np.linalg.norm(acc)
        acc_hat = acc / acc_magnitude
        
        mag_magnitude = np.linalg.norm(acc)
        mag_hat = mag / mag_magnitude
        
#        
#        
#        gravity_hat = np.linal.norm(np.cross(mag_hat, self.DOWN))
#        
#        gravity = self.GRAVITY * gravity_hat
#        
#        adj_acc = acc - gravity
#        
#        adj_acc_hat = adj_acc / np.linalg.norm(adj_acc)
#        
        
        #Calculate pitch and roll
        pitch = np.arcsin(acc_hat[0])
        roll = -np.arcsin(acc_hat[1] / np.cos(pitch))
        
                #magenometer
        #Calculate heading
#        heading = self.trim_heading(math.atan2(MAGy,MAGx))
        MAGx, MAGy, MAGz = mag
        magXcomp = MAGx*np.cos(pitch) - MAGz*np.sin(pitch)
        magYcomp = MAGx*np.sin(roll)*np.sin(pitch) + MAGy*np.cos(roll) + MAGz*np.sin(roll) * np.cos(pitch)

        yaw = self.trim_heading(np.arctan2(magYcomp,magXcomp))
    
#        self.last_read = now
        
        attitude = np.array([pitch, roll, yaw])
        ret = np.array([attitude, acc_hat, mag_hat])
        return ret
    
    def read_data(self):
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()
        return np.array([ACCx, ACCy, ACCz,
                         GYRx, GYRy, GYRz,
                         MAGx, MAGy, MAGz], dtype=float)

def fitEllipsoid(magX, magY, magZ):
    a1 = magX ** 2
    a2 = magY ** 2
    a3 = magZ ** 2
    a4 = 2 * np.multiply(magY, magZ)
    a5 = 2 * np.multiply(magX, magZ)
    a6 = 2 * np.multiply(magX, magY)
    a7 = 2 * magX
    a8 = 2 * magY
    a9 = 2 * magZ
    a10 = np.ones(len(magX)).T
    D = np.array([a1, a2, a3, a4, a5, a6, a7, a8, a9, a10])

    # Eqn 7, k = 4
    C1 = np.array([[-1, 1, 1, 0, 0, 0],
                   [1, -1, 1, 0, 0, 0],
                   [1, 1, -1, 0, 0, 0],
                   [0, 0, 0, -4, 0, 0],
                   [0, 0, 0, 0, -4, 0],
                   [0, 0, 0, 0, 0, -4]])

    # Eqn 11
    S = np.matmul(D, D.T)
    S11 = S[:6, :6]
    S12 = S[:6, 6:]
    S21 = S[6:, :6]
    S22 = S[6:, 6:]

    # Eqn 15, find eigenvalue and vector
    # Since S is symmetric, S12.T = S21
    tmp = np.matmul(np.linalg.inv(C1), S11 - np.matmul(S12, np.matmul(np.linalg.inv(S22), S21)))
    eigenValue, eigenVector = np.linalg.eig(tmp)
    u1 = eigenVector[:, np.argmax(eigenValue)]

    # Eqn 13 solution
    u2 = np.matmul(-np.matmul(np.linalg.inv(S22), S21), u1)

    # Total solution
    u = np.concatenate([u1, u2]).T

    Q = np.array([[u[0], u[5], u[4]],
                  [u[5], u[1], u[3]],
                  [u[4], u[3], u[2]]])

    n = np.array([[u[6]],
                  [u[7]],
                  [u[8]]])

    d = u[9]

    return Q, n, d


        #Calculate the angles from the gyro.
#        self.phi += w_x * dt # roll?
#        self.theta += w_y * dt # pitch
#        self.psi += w_z * dt # yaw
#        
#        
        
        #Accelerometer
# for filtering
#        AccXangle =  np.arctan2(ACCy,ACCz)
#        AccYangle =  np.arctan2(ACCz,ACCx) + np.pi
#        if AccYangle > 90:
#            AccYangle -= 270.0
#        else:
#            AccYangle += 90.0
#
#RAD_TO_DEG = 57.29578
#M_PI = 3.14159265358979323846
#G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
#AA =  0.40              # Complementary filter constant
#MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
#ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
#ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
#MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay
#
#
#
################## Compass Calibration values ############
## Use calibrateBerryIMU.py to get calibration values
## Calibrating the compass isnt mandatory, however a calibrated
## compass will result in a more accurate heading value.
#
#magXmin =  0
#magYmin =  0
#magZmin =  0
#magXmax =  0
#magYmax =  0
#magZmax =  0
#
#
#'''
#Here is an example:
#magXmin =  -1748
#magYmin =  -1025
#magZmin =  -1876
#magXmax =  959
#magYmax =  1651
#magZmax =  708
#Dont use the above values, these are just an example.
#'''
################ END Calibration offsets #################
#
#
##Kalman filter variables
#Q_angle = 0.02
#Q_gyro = 0.0015
#R_angle = 0.005
#y_bias = 0.0
#x_bias = 0.0
#XP_00 = 0.0
#XP_01 = 0.0
#XP_10 = 0.0
#XP_11 = 0.0
#YP_00 = 0.0
#YP_01 = 0.0
#YP_10 = 0.0
#YP_11 = 0.0
#KFangleX = 0.0
#KFangleY = 0.0
#
#
#
#def kalmanFilterY ( accAngle, gyroRate, DT):
#    y=0.0
#    S=0.0
#
#    global KFangleY
#    global Q_angle
#    global Q_gyro
#    global y_bias
#    global YP_00
#    global YP_01
#    global YP_10
#    global YP_11
#
#    KFangleY = KFangleY + DT * (gyroRate - y_bias)
#
#    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
#    YP_01 = YP_01 + ( - DT * YP_11 )
#    YP_10 = YP_10 + ( - DT * YP_11 )
#    YP_11 = YP_11 + ( + Q_gyro * DT )
#
#    y = accAngle - KFangleY
#    S = YP_00 + R_angle
#    K_0 = YP_00 / S
#    K_1 = YP_10 / S
#
#    KFangleY = KFangleY + ( K_0 * y )
#    y_bias = y_bias + ( K_1 * y )
#
#    YP_00 = YP_00 - ( K_0 * YP_00 )
#    YP_01 = YP_01 - ( K_0 * YP_01 )
#    YP_10 = YP_10 - ( K_1 * YP_00 )
#    YP_11 = YP_11 - ( K_1 * YP_01 )
#
#    return KFangleY
#
#def kalmanFilterX ( accAngle, gyroRate, DT):
#    x=0.0
#    S=0.0
#
#    global KFangleX
#    global Q_angle
#    global Q_gyro
#    global x_bias
#    global XP_00
#    global XP_01
#    global XP_10
#    global XP_11
#
#
#    KFangleX = KFangleX + DT * (gyroRate - x_bias)
#
#    XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
#    XP_01 = XP_01 + ( - DT * XP_11 )
#    XP_10 = XP_10 + ( - DT * XP_11 )
#    XP_11 = XP_11 + ( + Q_gyro * DT )
#
#    x = accAngle - KFangleX
#    S = XP_00 + R_angle
#    K_0 = XP_00 / S
#    K_1 = XP_10 / S
#
#    KFangleX = KFangleX + ( K_0 * x )
#    x_bias = x_bias + ( K_1 * x )
#
#    XP_00 = XP_00 - ( K_0 * XP_00 )
#    XP_01 = XP_01 - ( K_0 * XP_01 )
#    XP_10 = XP_10 - ( K_1 * XP_00 )
#    XP_11 = XP_11 - ( K_1 * XP_01 )
#
#    return KFangleX
#
#
#gyroXangle = 0.0
#gyroYangle = 0.0
#gyroZangle = 0.0
#CFangleX = 0.0
#CFangleY = 0.0
#CFangleXFiltered = 0.0
#CFangleYFiltered = 0.0
#kalmanX = 0.0
#kalmanY = 0.0
#oldXMagRawValue = 0
#oldYMagRawValue = 0
#oldZMagRawValue = 0
#oldXAccRawValue = 0
#oldYAccRawValue = 0
#oldZAccRawValue = 0
#
#a = datetime.datetime.now()
#
#
#
##Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
#acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
#acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
#acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
#acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
#acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
#acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
#mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
#mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
#mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
#mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
#mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
#mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE
#
#IMU.detectIMU()     #Detect if BerryIMU is connected.
#if(IMU.BerryIMUversion == 99):
#    print(" No BerryIMU found... exiting ")
#    sys.exit()
#IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
#
#
#while True:
#
#    #Read the accelerometer,gyroscope and magnetometer values
#    ACCx = IMU.readACCx()
#    ACCy = IMU.readACCy()
#    ACCz = IMU.readACCz()
#    GYRx = IMU.readGYRx()
#    GYRy = IMU.readGYRy()
#    GYRz = IMU.readGYRz()
#    MAGx = IMU.readMAGx()
#    MAGy = IMU.readMAGy()
#    MAGz = IMU.readMAGz()
#
#
#    #Apply compass calibration
#    MAGx -= (magXmin + magXmax) /2
#    MAGy -= (magYmin + magYmax) /2
#    MAGz -= (magZmin + magZmax) /2
#
#
#    ##Calculate loop Period(LP). How long between Gyro Reads
#    b = datetime.datetime.now() - a
#    a = datetime.datetime.now()
#    LP = b.microseconds/(1000000*1.0)
#    outputString = "Loop Time %5.2f " % ( LP )
#
#
#
#    ###############################################
#    #### Apply low pass filter ####
#    ###############################################
#    MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
#    MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
#    MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
#    ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
#    ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
#    ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);
#
#    oldXMagRawValue = MAGx
#    oldYMagRawValue = MAGy
#    oldZMagRawValue = MAGz
#    oldXAccRawValue = ACCx
#    oldYAccRawValue = ACCy
#    oldZAccRawValue = ACCz
#
#    #########################################
#    #### Median filter for accelerometer ####
#    #########################################
#    # cycle the table
#    for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
#        acc_medianTable1X[x] = acc_medianTable1X[x-1]
#        acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
#        acc_medianTable1Z[x] = acc_medianTable1Z[x-1]
#
#    # Insert the lates values
#    acc_medianTable1X[0] = ACCx
#    acc_medianTable1Y[0] = ACCy
#    acc_medianTable1Z[0] = ACCz
#
#    # Copy the tables
#    acc_medianTable2X = acc_medianTable1X[:]
#    acc_medianTable2Y = acc_medianTable1Y[:]
#    acc_medianTable2Z = acc_medianTable1Z[:]
#
#    # Sort table 2
#    acc_medianTable2X.sort()
#    acc_medianTable2Y.sort()
#    acc_medianTable2Z.sort()
#
#    # The middle value is the value we are interested in
#    ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
#    ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
#    ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];
#
#
#
#    #########################################
#    #### Median filter for magnetometer ####
#    #########################################
#    # cycle the table
#    for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
#        mag_medianTable1X[x] = mag_medianTable1X[x-1]
#        mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
#        mag_medianTable1Z[x] = mag_medianTable1Z[x-1]
#
#    # Insert the latest values
#    mag_medianTable1X[0] = MAGx
#    mag_medianTable1Y[0] = MAGy
#    mag_medianTable1Z[0] = MAGz
#
#    # Copy the tables
#    mag_medianTable2X = mag_medianTable1X[:]
#    mag_medianTable2Y = mag_medianTable1Y[:]
#    mag_medianTable2Z = mag_medianTable1Z[:]
#
#    # Sort table 2
#    mag_medianTable2X.sort()
#    mag_medianTable2Y.sort()
#    mag_medianTable2Z.sort()
#
#    # The middle value is the value we are interested in
#    MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
#    MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
#    MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];
#
#
#
#    #Convert Gyro raw to degrees per second
#    rate_gyr_x =  GYRx * G_GAIN
#    rate_gyr_y =  GYRy * G_GAIN
#    rate_gyr_z =  GYRz * G_GAIN
#
#
#    #Calculate the angles from the gyro.
#    gyroXangle+=rate_gyr_x*LP
#    gyroYangle+=rate_gyr_y*LP
#    gyroZangle+=rate_gyr_z*LP
#
#    #Convert Accelerometer values to degrees
#    AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
#    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
#
#
#    #Change the rotation value of the accelerometer to -/+ 180 and
#    #move the Y axis '0' point to up.  This makes it easier to read.
#    if AccYangle > 90:
#        AccYangle -= 270.0
#    else:
#        AccYangle += 90.0
#
#
#
#    #Complementary filter used to combine the accelerometer and gyro values.
#    CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
#    CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
#
#    #Kalman filter used to combine the accelerometer and gyro values.
#    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
#    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)
#
#    #Calculate heading
#    heading = 180 * math.atan2(MAGy,MAGx)/M_PI
#
#    #Only have our heading between 0 and 360
#    if heading < 0:
#        heading += 360
#
#    ####################################################################
#    ###################Tilt compensated heading#########################
#    ####################################################################
#    #Normalize accelerometer raw values.
#    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
#    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
#
#
#    #Calculate pitch and roll
#    pitch = math.asin(accXnorm)
#    roll = -math.asin(accYnorm/math.cos(pitch))
#
#
#    #Calculate the new tilt compensated values
#    #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
#    #This needs to be taken into consideration when performing the calculations
#
#    #X compensation
#    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
#        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
#    else:                                                                #LSM9DS1
#        magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)
#
#    #Y compensation
#    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
#        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
#    else:                                                                #LSM9DS1
#        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)
#
#
#
#
#
#    #Calculate tilt compensated heading
#    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
#
#    if tiltCompensatedHeading < 0:
#        tiltCompensatedHeading += 360
#
#
#    ##################### END Tilt Compensation ########################
#
#
#    if 1:                       #Change to '0' to stop showing the angles from the accelerometer
#        outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f  #  " % (AccXangle, AccYangle)
#
#    if 1:                       #Change to '0' to stop  showing the angles from the gyro
#        outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)
#
#    if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
#        outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)
#
#    if 1:                       #Change to '0' to stop  showing the heading
#        outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)
#
#    if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
#        outputString +="# kalmanX %5.2f   kalmanY %5.2f #" % (kalmanX,kalmanY)
#
#    print(outputString)
#
#    #slow program down a bit, makes the output more readable
#    time.sleep(0.03)

