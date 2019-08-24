import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import osqp
import scipy.sparse as spa
import os

#######File eksternal#######
file1 = open("D:/Indra/ITB S2/Tesis/pythonCode/logdata_stepa.csv", "a+") #file mencetak variabel 
if os.stat("D:/Indra/ITB S2/Tesis/pythonCode/logdata_stepa.csv").st_size == 0:
    file1.write("Sinyal Kontrol ax; Sinyal Kontrol ay; Sinyal Kontrol wx; Sinyal Kontrol wy; Referensi w_x; Referensi w_y;Referensi a_x; Referensi a_y;  w_x; w_y;a_x; a_y; Sudut Roll; Sudut Pitch; Px; Vx; Py; Vy \n")


########Inisialisasi#########
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
q = 10*np.array([[100.0,0.0,0,0,0,0,0,0,0,0], [0.0,100.0,0,0,0,0,0,0,0,0], [0.0,0.0,10,0,0,0,0,0,0,0],[0.0,0.0,0,10,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0],[0.0,0.0,0,0,0,0,0,0,0,0]])
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



#######Proses Simulasi#######
#Waktu simulasi dan time sampling
T = 250
Ts = 0.5

#Inisiasi qp solver
myqp = osqp.OSQP()

#variabel untuk mengeplot
xplot = np.array([0])
yplot1 = np.array([0])
yplot2 = np.array([0])
yplot3 = np.array([0])
yplot4 = np.array([0])
yplot5 = np.array([0])
yplot6 = np.array([0])
yplot7 = np.array([0])
yplot8 = np.array([0])
yplot9 = np.array([0])
yplot10 = np.array([0])

print(M.shape)

for kk in range(1,int(T/Ts)):
	##Sinyal Referensi##
	##Sinyal Referensi##
	if (kk*Ts<T/8):
		wref = np.array([[0.0],[0.0],[0],[0],[0],[0],[0],[0],[0],[0]])
		Wref = wref
		for oo in range(2,N+1):
			Wref = np.concatenate((Wref,wref), axis=0)
	elif (kk*Ts<T/2):
		wref = 0.0025*np.sin(2*np.pi*Ts*kk/10)*np.array([[1],[1],[0.0],[0.0],[0],[0],[0],[0],[0],[0]])
		#np.sin(2*kk*np.pi*(Ts/5))*
		Wref = wref
		for oo in range(2,N+1):
			Wref = np.concatenate((Wref,wref), axis=0)
	elif (kk*Ts<T/1.3):
		wref = 0.0025*np.sin(2*np.pi*Ts*kk/10)*np.array([[1],[1],[0.0],[0.0],[0],[0],[0],[0],[0],[0]])
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
	if kk==1 :
		myqp.setup(MM, F, cons, CCC,CCC2)
		tempU = myqp.solve()
		U_delta = np.reshape(tempU.x, (m*N,1))
	else :
		myqp.update(q=F, l=CCC, u=CCC2)
		tempU = myqp.solve()
		U_delta = np.reshape(tempU.x, (m*N,1))
	
	print(U_delta)
	
	
	u_delta = E1 @ U_delta
	
	##Masukkan hasil ke plant##
	uk = u_delta + uk_1
	
	xkk = xkk1
	xkk1 = (Ad @ xkk) + (Bd @ uk)
	yk = Cd @ xkk
	print(yk[1])	
	#Update variabel
	uk_1 = uk #Apakah ini terbalik? 
	
	#Variabel log
	datalog1 =(str(uk[0,0]), ";" ,str(uk[1,0]), ";" ,str(uk[2,0]), ";" ,str(uk[3,0]), ";" ,str(wref[0,0]), ";" ,str(wref[1,0]), ";" ,str(wref[2,0]), ";" ,str(wref[3,0]), ";" , str(yk[0,0]), ";", str(yk[1,0]), ";", str(yk[2,0]), ";", str(yk[3,0]), ";", str(yk[4,0]), ";", str(yk[5,0]), ";", str(yk[6,0]), ";", str(yk[7,0]), ";", str(yk[8,0]), ";", str(yk[9,0])) #Menyimpan ke file csv  
	file1.writelines(datalog1) #Ngeprint ke bawah 
	file1.write("\n") 
	file1.flush()
	
	#Variabel plot
	xplot = np.concatenate((xplot,np.array([kk*Ts])))
	yplot1 = np.concatenate((yplot1,yk[0]))
	yplot2 = np.concatenate((yplot2,yk[1]))
	yplot3 = np.concatenate((yplot3,yk[2]))
	yplot4 = np.concatenate((yplot4,yk[3]))
	yplot5 = np.concatenate((yplot5,yk[4]))
	yplot6 = np.concatenate((yplot6,yk[5]))
	# yplot7 = np.concatenate((yplot7,wref[0]))
	# yplot8 = np.concatenate((yplot8,wref[1]))
	# yplot9 = np.concatenate((yplot9,wref[2]))
	# yplot10 = np.concatenate((yplot10,wref[3]))
	yplot7 = np.concatenate((yplot7,yk[6]))
	yplot8 = np.concatenate((yplot8,yk[7]))
	yplot9 = np.concatenate((yplot9,yk[8]))
	yplot10 = np.concatenate((yplot10,yk[9]))
	# yplot7 = np.concatenate((yplot7,uk[0]))
	# yplot8 = np.concatenate((yplot8,uk[1]))
	# yplot9 = np.concatenate((yplot9,uk[2]))
	# yplot10 = np.concatenate((yplot10,uk[3]))

print(uk)
#Pengeplotan
plt.figure(1)
plt.subplot(10,1,1)             
plt.plot(xplot,yplot1)
plt.subplot(10,1,2)             
plt.plot(xplot,yplot2)
plt.subplot(10,1,3)             
plt.plot(xplot,yplot3)
plt.subplot(10,1,4)             
plt.plot(xplot,yplot4)
plt.subplot(10,1,5)             
plt.plot(xplot,yplot5)
plt.subplot(10,1,6)             
plt.plot(xplot,yplot6)
plt.subplot(10,1,7)             
plt.plot(xplot,yplot7)
plt.subplot(10,1,8)             
plt.plot(xplot,yplot8)
plt.subplot(10,1,9)             
plt.plot(xplot,yplot9)
plt.subplot(10,1,10)             
plt.plot(xplot,yplot10)

plt.show()	