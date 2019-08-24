import time #Library Waktu 
import os #Library OS 
import serial #Library Serial Communication 
import threading #Library Threading 
import queue #Library untuk membagi antar variabel atau task atau thread 
import pymodbus #Library untuk Modbus pada Python
import matplotlib.pyplot as pyplot
import numpy as np
#import pandas as pd
import scipy.linalg
import osqp
import scipy.sparse as spa
import smbus            #import SMBus module of I2C

#Untuk memperpendek commad/code nya, agar langsung spesifik ke folder nya 
from queue import Empty, Full, Queue
from pymodbus.client.sync import ModbusSerialClient as ModbusClient #initialize a serial RTU client instance
from numpy import zeros
from datetime import datetime

#komunikasi Modbus Motor 1
client= ModbusClient(method = "ascii", port="/dev/ttyUSB4",timeout=0.003, stopbits = 1, bytesize = 8, parity = 'N', baudrate = 115200)
servo = "ServoOn" #Perintah mengaktifkan Servo

#komunikasi Modbus Motor 2
client2= ModbusClient(method = "ascii", port="/dev/ttyUSB1",timeout=0.003, stopbits = 1, bytesize = 8, parity = 'N', baudrate = 115200)
servo2 = "ServoOn" #Perintah mengaktifkan Servo

#parameter
# Ts = 0.03; # sampling  sec
# T = 600 #lama simulasi
# filepath = "Look_Up_Table.csv"
# df = pd.read_csv(filepath)

datashare1 = Queue() #Untuk Menyimpan Sudut Perintah motor 1 (dalam PUU)
datashare2 = Queue() #Untuk Menyimpan Sudut Perintah motor 2 (dalam PUU)
datashare3 = Queue() #untuk menyimpan variabel yang akan dicetak

##Logging Data Implementasi
file1 = open("/home/pi/Desktop/indra/logdata_stepw.csv", "a+") #file mencetak variabel 
if os.stat("/home/pi//Desktop/indra/logdata_stepw.csv").st_size == 0:
    file1.write("Time; Sinyal Kontrol ax; Sinyal Kontrol ay; Sinyal Kontrol wx; Sinyal Kontrol wy;  Referensi w_x; Referensi w_y;Referensi a_x; Referensi a_y;  w_x; w_y;a_x; a_y; Sudut Roll; Sudut Pitch; Px; Vx; Py; Vy \n")
 
if (servo == "ServoOn"):
    path5value = (0x0011, 0x0000) #Perintah kontrol register aktif untuk kontrol Input 
    speedpath6 = client.write_registers(0x0216, path5value, unit = 0x007F)

    valueservo = (0x0001, 0x0000) #Perintah register servo On 
    setservo = client.write_registers(0x0214, valueservo, unit = 0x007F)
 
else :
   path5value = (0x0111, 0x0000) #perintah register non-aktif untuk kontrol input 
   speedpath6 = client.write_registers(0x0216, path5value, unit = 0x007F)

   valueservo = (0x0101, 0x0000) #perintah non-aktif servo (Servo off) 
   setservo = client.write_registers(0x0214, valueservo, unit = 0x007F)

if (servo2 == "ServoOn"):
    path5value = (0x0011, 0x0000) #Perintah kontrol register aktif untuk kontrol Input 
    speedpath6 = client2.write_registers(0x0216, path5value, unit = 0x007F)

    valueservo = (0x0001, 0x0000) #Perintah register servo On 
    setservo = client2.write_registers(0x0214, valueservo, unit = 0x007F)
 
else :
   path5value = (0x0111, 0x0000) #perintah register non-aktif untuk kontrol input 
   speedpath6 = client2.write_registers(0x0216, path5value, unit = 0x007F)

   valueservo = (0x0101, 0x0000) #perintah non-aktif servo (Servo off) 
   setservo = client2.write_registers(0x0214, valueservo, unit = 0x007F)

#some MPU6050 Registers and their Address 
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

########### Inisiasi Pembacaan accelerometer dan gyroscope##########
def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

######################################################

#Fungsi konversi
def split (num) :
    if (num < 0) :
            temp = abs(num)
            temp = temp^0xFFFFFFFF
            temp = temp+1
    else :
            temp = num
    return (temp & 0xFFFF, temp >> 16)
            
def hex2python(num0, num1) :
    temp = (num1<<16 & 0xFFFF0000) + num0
    
    if ((temp>>31) == 1) :
        temp = temp-1
        temp = temp^0xFFFFFFFF
        temp = -1*temp
    return temp

#Fungsi inverse kinematics
def inverse_kinematics(angle_x):
    angle_x = angle_x*0.01745333 #apa ini konversi?
    x_calc = np.sqrt(0.64 - np.power((np.sin(angle_x)*0.8),2))-0.6
    y_calc = 0.3 + (np.sin(angle_x)*0.8)
    angle_gear = np.arctan(y_calc/x_calc)- np.arccos((x_calc*x_calc + y_calc*y_calc + 0.04-0.09)/(2*0.2*np.sqrt(x_calc*x_calc + y_calc*y_calc)))
    angle_mot =  angle_gear*60.00*57.297469
    #Pembatasan#
    if angle_mot > 2000: #ini derajat
        angle_mot = 2000
    elif angle_mot < -2000:
        angle_mot = -2000
    
    angle_puu = int (angle_mot*100000.00 / 360.00)  #Convert derajat to PUU
    return(angle_puu)

def input1(): 
    time_now = time.time()
    #####################Inisialisasi Perhitungan Motion Cueing#####################
    #Model dari Plant (digital)
    Ad = np.array([-0.0115609575275586,0.0916994835880069,0.0384323981955624,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.183489157426914,0.923526186464424,0.483606177707615,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.00500679180390518,-0.00209840894147771,0.999549313335066,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.0136580743586876,0.0907862489978198,0.0382337758286765,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.204531409268370,0.914358962897026,0.481611905518551,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.00571045506196287,-0.00240490449962375,0.999482641401992,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.406374713625912,0.332685881974741,0,0,0,4.99973889715005,0,0,0,0,0,0,0,0,0,0,-0.0945825962454189,0.972872233452501,0,0,0,0.152726480191358,0,0,0,0,0,0,0,0,0,0,0,0,0.406374713625912,0.332685881974741,-4.99973889715005,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.0945825962454189,0.972872233452501,-0.152726480191358,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0.500000000000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0.500000000000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
    Ad = np.reshape(Ad,(16,16))
    Bd = np.array([0,0,0.916994835880069,0,0,0,-0.764738135355761,0,0,0,-0.0209840894147771,0,0,0,0,0.907862489978198,0,0,0,-0.856410371029737,0,0,0,-0.0240490449962375,0.510177438484699,0,0,1.42245346397693,0.0155843347134039,0,0,0.0674826556099679,0,0.510177438484699,-1.42245346397693,0,0,0.0155843347134039,-0.0674826556099679,0,0,0,0.500000000000000,0,0,0,0,0.500000000000000,0.125000000000000,0,0,0,0.500000000000000,0,0,0,0,0,0.125000000000000,0,0,0,0.500000000000000,0])
    Bd = np.reshape(Bd,(16,4))
    Cd = np.array([1.0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
    Cd = np.reshape(Cd,(10,16))
    Dd = 0  

    #Panjang horizon
    N = 10

    #Bobot q dan r
    q = 10*np.array([[100.0,0.0,0,0,0,0,0,0,0,0], [0.0,100.0,0,0,0,0,0,0,0,0], [0.0,0.0,10,0,0,0,0,0,0,0],[0.0,0.0,0,50,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0]])
    r = np.array([[2500.0,0,0,0],[0,2500.0,0,0],[0,0,2500,0],[0,0,0,2500.0]])

    #Constraints dari MPC
    ymin = np.array([[-1000],[-1000],[-1000],[-1000],[-0.174],[-0.174],[-0.1],[-1000],[-0.1],[-1000]])
    ymax = np.array([[1000],[1000],[1000],[1000],[0.174],[0.174],[0.1],[1000],[0.1],[1000]])

    ########Augmentasi Plant dengan Integrator (3.46)##########
    sizeAd = Ad.shape
    rowAd = sizeAd[0]
    colAd = sizeAd[1]
    sizeBd = Bd.shape
    rowBd = sizeBd[0]
    colBd = sizeBd[1]
    sizeCd = Cd.shape
    rowCd = sizeCd[0]


    #Augmented A
    Atemp = np.concatenate((Ad, Bd), axis=1)
    Atemp2 = np.concatenate((np.zeros((sizeBd[1],sizeAd[0])), np.identity(sizeBd[1])), axis = 1)
    A = np.concatenate((Atemp,Atemp2), axis=0)
    #Augmented B
    B = np.concatenate((Bd, np.identity(sizeBd[1])), axis=0)
    #Augmented C
    C = np.concatenate((Cd, np.zeros((sizeCd[0],sizeBd[1]))), axis=1)

    orde = A.shape[0] # orde augmented sistem
    m = B.shape[1] # Banyak Input mpc tracking
    n = C.shape[0] # Bantak Output mpc tracking



    ######Perumusan MPC########
    #Matriks bobot Q untuk augmented (sampai horizon terakhir)
    Q = q
    for ii in range(2, N+1):
        #blkdiag
        Q = scipy.linalg.block_diag(Q,q)

    #Matriks bobot R untuk augmented (sampai horizon terakhir)
    R = r
    for ii in range(2, N+1):
        #blkdiag
        R = scipy.linalg.block_diag(R,r)

        
    #Matriks PHI
    Phi = np.matmul(C,A)
    for iii in range(2, N+1):
        Phi = np.concatenate((Phi, np.matmul(C, np.linalg.matrix_power(A,iii))), axis=0)
        
    #Matriks GAMMA
    #Baris pertama matriks
    Gamma = np.matmul(C,B)
    for xx in range(2, N+1):
        Gamma = np.concatenate((Gamma, np.zeros((n,m))),axis=1)
    #matriks penuh
    for row in range(2,N+1):
        Gammatemp = (C @ np.linalg.matrix_power(A,row-1)) @ B
        for col in range(2,N+1):
            if ((row-col)<0) :
                Gammatemp = np.concatenate((Gammatemp, np.zeros((n,m))),axis=1)
            else :
                Gammatemp = np.concatenate((Gammatemp, (C @ np.linalg.matrix_power(A,row-col)) @ B),axis=1)
        Gamma = np.concatenate((Gamma, Gammatemp),axis=0)
            
    #Matrix M
    M = ((Gamma.T @ Q) @ Gamma) + R

    #Matrix H
    H = Gamma.T @ Q

    #Matrix E1
    E1 = np.concatenate((np.identity(m), np.zeros((m,m*(N-1)))), axis=1)

    #Matrix constraints
    Ymin = ymin
    for iii in range(2,N+1):
        Ymin = np.concatenate((Ymin,ymin),axis=0)
        
    Ymax = ymax
    for iii in range(2,N+1):
        Ymax = np.concatenate((Ymax,ymax),axis=0)

    ######Variabel-variabel dari Sistem########
    xkbar = np.zeros((orde,1)) #State augmentasi

    xk = np.zeros((orde-m,1)) #State plant saat ini (k)
    xkk = np.zeros((orde-m,1))
    xkk1 = np.zeros((orde-m,1))

    u_delta = np.zeros((m,1)) #Delta u kendali
    U_delta = np.zeros((N*m,1)) #Delta u untuk N horizon
    uk_1 = np.zeros((m,1)) #Sinyal kendali sbelumnya (k-1)
    uk = np.zeros((m,1)) #Sinyal kendali saat ini (k)
    
    #Variabel sistem untuk verifikasi
    xkk_ver = np.zeros((orde-m,1))
    xkk1_ver = np.zeros((orde-m,1))
    uk_ver = np.zeros((m,1))

    #######Proses Simulasi#######
    #Waktu simulasi dan time sampling
    T = 250
    Ts = 0.5

    #Inisiasi qp solver
    myqp = osqp.OSQP()
    
    #Variabel sampling ke-k_time
    k_time = 0
    #for kk in range (1,int(T/Ts)): #Setpoint 
    while True:    
        if ((((time.time() - time_now)>=Ts) or (k_time==0)) and (k_time<=int(T/Ts))):
            k_time = k_time+1
            try:
                time_now=time.time()
                ##Sinyal Referensi##
                if (k_time*Ts<T/7):
                    wref = np.array([[0.0],[0.0],[0],[0],[0],[0],[0],[0],[0],[0]])
                    Wref = wref
                    for oo in range(2,N+1):
                        Wref = np.concatenate((Wref,wref), axis=0)
                elif (k_time*Ts<T/2):
                    wref = 0.025*np.array([[1.00],[1.0],[0.0],[0.0],[0],[0],[0],[0],[0],[0]])
                    #np.sin(2*kk*np.pi*(Ts/5))*
                    Wref = wref
                    for oo in range(2,N+1):
                        Wref = np.concatenate((Wref,wref), axis=0)
                elif (k_time*Ts<T/1.5):
                    wref = -0.025*np.array([[1.0],[1.0],[0.0],[0.0],[0],[0],[0],[0],[0],[0]])
                    #np.sin(2*kk*np.pi*(Ts/5))*
                    Wref = wref
                    for oo in range(2,N+1):
                        Wref = np.concatenate((Wref,wref), axis=0)
                else:
                    wref = 0*np.array([[0],[0],[1.0],[1.0],[0],[0],[0],[0],[0],[0]])
                    #np.sin(2*kk*np.pi*(Ts/5))*
                    Wref = wref
                    for oo in range(2,N+1):
                        Wref = np.concatenate((Wref,wref), axis=0)

                ##Perhitungan QP (J=0.5*x'*MM*x + F'*x, cons: CCC<=cons*x<=CCC2)##
                xkbar = np.concatenate((xkk1, uk_1), axis=0)
                MM = spa.csc_matrix(M)
                #Matriks F, yg dimasukkan adalah F', bukan F
                F = H @ ((Phi @ xkbar) - Wref)
                #Matriks CC dan CCC(untuk constraints)
                cons = spa.csc_matrix(Gamma)
                CCC = Ymin - (Phi @ xkbar)
                CCC2 = Ymax -(Phi @ xkbar)
                
                #QP
                if k_time==1 :
                    myqp.setup(MM, F, cons, CCC,CCC2)
                    tempU = myqp.solve()
                    U_delta = np.reshape(tempU.x, (m*N,1))
                else :
                    myqp.update(q=F, l=CCC, u=CCC2)
                    tempU = myqp.solve()
                    U_delta = np.reshape(tempU.x, (m*N,1))
                
                # print(U_delta)
                # print(U_delta.shape)
                
                u_delta = E1 @ U_delta
                
                ##Masukkan hasil ke estimator##
                uk = u_delta + uk_1
                
                xkk = xkk1
                xkk1 = (Ad @ xkk) + (Bd @ uk)
                yk = Cd @ xkk
                    
                #Update variabel
                uk_1 = uk
                
                ##Perhitungan sudut untuk motor 1 dan 2##
                angle_motor1 = yk[5,0]+yk[4,0] #sudut motor 1 = pitch+roll
                angle_motor2 = yk[5,0]-yk[4,0] #sudut motor 2 = pitch-roll
                            
                ###inverse kinematics, ubah ke derajat dulu? ###
                puu_motor1 = inverse_kinematics(angle_motor1*360.0/(2*np.pi))
                puu_motor2 = inverse_kinematics(angle_motor2*360.0/(2*np.pi))
                
                print(angle_motor1)
                
                ##Pengaturan data yang akan dikirim ke kontrol motor##
                datashare1.put(-puu_motor1) #Menyimpan sudut input ke Datashare 1
                datashare2.put(-puu_motor2)
                
                ##Pembacaan data sensor##
                #Read Accelerometer raw value
                acc_x = read_raw_data(ACCEL_XOUT_H)
                acc_y = read_raw_data(ACCEL_YOUT_H)
                acc_z = read_raw_data(ACCEL_ZOUT_H)
                
                #Read Gyroscope raw value
                gyro_x = read_raw_data(GYRO_XOUT_H)
                gyro_y = read_raw_data(GYRO_YOUT_H)
                gyro_z = read_raw_data(GYRO_ZOUT_H)
                
                #Full scale range +/- 250 degree/C as per sensitivity scale factor
                Ax = acc_x/16384.0
                Ay = acc_y/16384.0
                Az = acc_z/16384.0
                
                Gx = (gyro_x/131.0)+0.45
                Gy = (gyro_y/131.0)-0.4
                Gz = (gyro_z/131.0)
                
                ##Mengubah data menjadi sensasi dengan memasukkan ke model vestibular##
                uk_ver = np.array([[uk[0,0]],[uk[1,0]],[uk[2,0]],[uk[3,0]]])
                #uk_ver = np.array([[uk[0,0]],[uk[1,0]],[0.0],[0.0]])
                
                xkk_ver = xkk1_ver
                xkk1_ver = (Ad @ xkk_ver) + (Bd @ uk_ver)
                yk_ver = Cd @ xkk_ver
                 
                ##Menulis data ke file eksternal##
                now = time.time()
                datalog1 =(str(now), ";", str(uk[0,0]), ";" ,str(uk[1,0]), ";" ,str(uk[2,0]), ";" ,str(uk[3,0]), ";" ,str(wref[0,0]), ";" ,str(wref[1,0]), ";" ,str(wref[2,0]), ";" ,str(wref[3,0]), ";" , str(yk[0,0]), ";", str(yk[1,0]), ";", str(yk_ver[2,0]), ";", str(yk_ver[3,0]), ";", str(yk[4,0]), ";", str(yk[5,0]), ";", str(yk[6,0]), ";", str(yk[7,0]), ";", str(yk[8,0]), ";", str(yk[9,0])) #Menyimpan ke file csv  
                file1.writelines(datalog1) #Ngeprint ke bawah 
                file1.write("\n") 
                file1.flush()
                
                
            except Exception as e:
                print("error t1", e) #jika gagal, Print tampilan Errornya 
                pass #Lanjut ke Loop Awal
            print(k_time)
   
def motor1():   
    while (True):
        try:
            data1 = datashare1.get() #Mendapatkan sudut input, dalam PUU
            # sudut_in = data[0]
                                    
            # posisi = client.read_holding_registers(0x0520, 2, unit = 0x007F) #Baca aktual posisi 
            # posisireg0 = posisi.getRegister(0)
            # posisireg1 = posisi.getRegister(1)
            # puu = hex2python(posisireg0, posisireg1) #Representasi nilai yang cocok 
            # sudut_out= (puu*360)/100000 #Convert PUU ke Sudut
            # print sudut_out

            # # speed = client.read_holding_registers(0x0012, 2, unit = 0x007F) #Baca aktual posisi 
            # # speedreg0 = speed.getRegister(0)
            # # speedreg1 = speed.getRegister(1)
            # # speed_out = hex2python(speedreg0, speedreg1) #Representasi nilai yang cocok
           
            # sudut_out_arr = df.t_m
            # t_cab_arr = df.t_cab
            
            # ##Interpolasi
            # lower = np.where(sudut_out_arr <= sudut_out)[-1][-1]
            # upper = np.where(sudut_out_arr >= sudut_out)[0][0]

            # if (lower == upper):
                # t_cab = t_cab_arr[lower]
            # else:
                # t_cab = t_cab_arr[lower] + ((t_cab_arr[upper] - t_cab_arr[lower])/(sudut_out_arr[upper] - sudut_out_arr[lower]))*(sudut_out - sudut_out_arr[lower])
            # print t_cab
            
            ##Pembatasan perintah sudut
            
            sig_c_hex = split(data1)             #Convert PUU dalam HEX

            value = (0x0012, 0x0001)
            anglepath1 = client.write_registers(0x0606, sig_c_hex, unit = 0x007f)
            anglepath2 = client.write_registers(0x0604, value, unit = 0x007f)
            startpath = client.write_registers(0x050e, split(1), unit = 0x007f)
            
            #datashare2.put([data[0].item(), sudut_out, sig_c, TL])
        except Exception as e:
            print("error t2", e) #Menampilkan error Thread ke 2 
            pass

def motor2():
    while (True):
        try:
            data2 = datashare2.get() #Mendapatkan sudut input, dalam PUU
            
            # sudut_in = data[0]
                                    
            # posisi = client2.read_holding_registers(0x0520, 2, unit = 0x007F) #Baca aktual posisi 
            # posisireg0 = posisi.getRegister(0)
            # posisireg1 = posisi.getRegister(1)
            # puu = hex2python(posisireg0, posisireg1) #Representasi nilai yang cocok 
            # sudut_out= (puu*360)/100000 #Convert PUU ke Sudut
            # print sudut_out
            
            # # speed = client2.read_holding_registers(0x0012, 2, unit = 0x007F) #Baca aktual posisi 
            # # speedreg0 = speed.getRegister(0)
            # # speedreg1 = speed.getRegister(1)
            # # speed_out = hex2python(speedreg0, speedreg1) #Representasi nilai yang cocok
            
            # sudut_out_arr = df.t_m
            # t_cab_arr = df.t_cab
    
            # lower = np.where(sudut_out_arr <= sudut_out)[-1][-1]
            # upper = np.where(sudut_out_arr >= sudut_out)[0][0]

            # if (lower == upper):
                # t_cab = t_cab_arr[lower]
            # else:
                # t_cab = t_cab_arr[lower] + ((t_cab_arr[upper] - t_cab_arr[lower])/(sudut_out_arr[upper] - sudut_out_arr[lower]))*(sudut_out - sudut_out_arr[lower])
            # print t_cab
           
            sig_c_hex = split(data2)             #Convert PUU dalam HEX

            value = (0x0012, 0x0001)
            anglepath1 = client2.write_registers(0x0606, sig_c_hex, unit = 0x007f)
            anglepath2 = client2.write_registers(0x0604, value, unit = 0x007f)
            startpath = client2.write_registers(0x050e, split(1), unit = 0x007f)         
           
            #datashare3.put([data[0].item(), sudut_out, sig_c, TL])
        except Exception as e:
            print("error t3", e) #Menampilkan error Thread ke 2 
            pass


if __name__ == "__main__": #Menjalankan semua Threading 
    # creating thread
    t1=threading.Thread(target=input1, args=())
    t2=threading.Thread(target=motor1, args=())
    
    t3=threading.Thread(target=motor2, args=())
    #t5=threading.Thread(target=data2, args=())

    #starting thread
    t1.start()
    t2.start()
    #t3.start()
    t3.start()
    #t5.start()
    
    #wait until thread completely executed
    t1.join()
    t2.join()
    #t3.join()
    t3.join()
    #t5.join()


