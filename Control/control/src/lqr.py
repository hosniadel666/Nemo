#!/usr/bin/env python

import math
import rospy
import controlpy
import numpy as np
from std_msgs.msg import String
from control.msg import ControlMsg
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray

class control ():
	def __init__ (self):
		rospy.init_node('control')

		self.setpoint = {'zo' : 2, 'phio' : 0, 'thto' : 0, 'epso' : math.pi/4, 'uo' : 0.7, 'vo' : 0, 'wo' : 0, 'po' : 0, 'qo' : 0, 'ro' : 0} 
		self.actual = {'zo' : 2, 'phio' : 0, 'thto' : 0, 'epso' : 0, 'uo' : 0.7, 'vo' : 0, 'wo' : 0, 'po' : 0, 'qo' : 0, 'ro' : 0} 
		self.typeofmotion = "forward" 
		self.output = np.zeros((8,1), dtype=np.float32)
		self.pub = rospy.Publisher('/control/output', ControlMsg)
		self.a_pub = rospy.Publisher('/motion_model/A', numpy_msg(Floats))
		self.b_pub = rospy.Publisher('/motion_model/B', numpy_msg(Floats))
		self.thrust_pub = rospy.Publisher('motion_model/thrusts', numpy_msg(Floats))

		rospy.Subscriber('/control/setpoint', ControlMsg, self.setpoint_callback)
		rospy.Subscriber('/control/actual', ControlMsg, self.actual_callback)
		rospy.Subscriber('/control/typeofmotion', String, self.typeofmotion_callback)

		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			self.controlFunction()
			self.pub.publish(self.output)
			rate.sleep()

	def setpoint_callback(self, msg):
		self.setpoint['zo']		= msg.data[0]
		self.setpoint['phio']	= msg.data[1]
		self.setpoint['thto']	= msg.data[2]
		self.setpoint['epso']	= msg.data[3]
		self.setpoint['uo']		= msg.data[4]
		self.setpoint['vo']		= msg.data[5]
		self.setpoint['wo']		= msg.data[6]
		self.setpoint['po']		= msg.data[7]
		self.setpoint['qo']		= msg.data[8]
		self.setpoint['ro']		= msg.data[9]
		print(self.setpoint)

	def actual_callback(self, msg):
		self.actual['zo']	= msg.data[0]
		self.actual['phio']	= msg.data[1]
		self.actual['thto']	= msg.data[2]
		self.actual['epso']	= msg.data[3]
		self.actual['uo']	= msg.data[4]
		self.actual['vo']	= msg.data[5]
		self.actual['wo']	= msg.data[6]
		self.actual['po']	= msg.data[7]
		self.actual['qo']	= msg.data[8]
		self.actual['ro']	= msg.data[9]
		print(self.actual)

	def typeofmotion_callback(self, msg):
		self.typeofmotion = msg
	
	#get A and B "Dynamic Model"
	def getAandB (self):
	    A=np.zeros((10,10))
	    B=np.zeros((10,10))
	    
		#dynamic model
	    
	    m=51 #AUV mass in KG
	    V=0.05 #AUV Volume in m3
	    Ixx=1.83  #AUV moment of inertia about X in KG.m2
	    Iyy=2.42  #AUV moment of inertia about Y in KG.m2
	    Izz=2.9   #AUV moment of inertia about Z in KG.m2
	    Ixy=-0.03
	    Ixz = -0.07
	    Iyx = -0.03
	    Iyz = -0.01
	    Izx = -0.07
	    Izy = -0.01
	    g = 9.81 #gravitational constant
	    W = m * g  #AUV weight in Newtons
	    raw = 1000  #Water density in kg/m3
	    B = raw * V * g  #buoyancy force
	    COB =np.array([[-0.00963],[  -0.001725],[ -0.3622]])    # Center of buoyancy vector (vector from COM to COB)
	    
	    #Coefficients of Added Mass

	    Xu_dot = 0
	    Xv_dot = 0
	    Xw_dot = 0
	    Xp_dot = 0
	    Xq_dot = 0
	    Xr_dot = 0
	    Yu_dot = 0
	    Yv_dot = 0
	    Yw_dot = 0
	    Yp_dot = 0
	    Yq_dot = 0
	    Yr_dot = 0
	    Zu_dot = 0
	    Zv_dot = 0
	    Zw_dot = 0
	    Zp_dot = 0
	    Zq_dot = 0
	    Zr_dot = 0
	    Ku_dot = 0
	    Kv_dot = 0
	    Kw_dot = 0
	    Kp_dot = 0
	    Kq_dot = 0
	    Kr_dot = 0
	    Mu_dot = 0
	    Mv_dot = 0
	    Mw_dot = 0
	    Mp_dot = 0
	    Mq_dot = 0
	    Mr_dot = 0
	    Nu_dot = 0
	    Nv_dot = 0
	    Nw_dot = 0
	    Np_dot = 0
	    Nq_dot = 0
	    Nr_dot = 0

	    #Coefficients of Drag

	    Xuu = 156.7
	    Xvv = 0
	    Xww = 0
	    Xpp = 0
	    Xqq = 0
	    Xrr = 0
	    Yuu = 0
	    Yvv = 149.26
	    Yww = 0
	    Ypp = 0
	    Yqq = 0
	    Yrr = 0
	    Zuu = 0
	    Zvv = 0
	    Zww = 389.34
	    Zpp = 0
	    Zqq = -5.514
	    Zrr = -52.5
	    Kuu = 0
	    Kvv = -38.36
	    Kww = 0.6716
	    Kpp = 9
	    Kqq = 0
	    Krr = 0
	    Muu = 9.543
	    Mvv = 0
	    Mww = -21.608
	    Mpp = 0
	    Mqq = 9.363
	    Mrr = 0
	    Nuu = 0.27
	    Nvv = 13.76
	    Nww = 0
	    Npp = 0
	    Nqq = 0
	    Nrr = 1

	    #Coefficients of Lift

	    Xlu = 0
	    Xlv = 4.71
	    Xlw = 3.1
	    Xlp = 0
	    Xlq = 5.38
	    Xlr = 0
	    Ylu = 6.792
	    Ylv = 0
	    Ylw = -3.7
	    Ylp = 0
	    Ylq = 0
	    Ylr = 0
	    Zlu = 10.588
	    Zlv = 33.64
	    Zlw = 0
	    Zlp = 0
	    Zlq = 0
	    Zlr = 0
	    Klu = -0.2899
	    Klv = -0.058
	    Klw = 0.225
	    Klp = 0
	    Klq = 0
	    Klr = 0
	    Mlu = -1
	    Mlv = -2.814
	    Mlw = 0.18879
	    Mlp = 0
	    Mlq = 0
	    Mlr = 0
	    Nlu = 0.63
	    Nlv = 0
	    Nlw = 0.346
	    Nlp = 0
	    Nlq = 0
	    Nlr = 0

	    #Thruster Coefficients 

	    d1x = 0
	    d2x = 0
	    d3x = 0
	    d4x = 0
	    d5x = 0.26
	    d6x = 0.26 
	    d7x = 0.26 
	    d8x = 0.26 
	    d1y = 0 
	    d2y = 0
	    d3y = 0
	    d4y = 0
	    d5y = 0.2571
	    d6y = 0.2571
	    d7y = 0.2571
	    d8y = 0.2571
	    d1xx = 0.3
	    d2xx = 0.3
	    d3xx = 0.3
	    d4xx = 0.3
	    d1yy = 0.3
	    d2yy = 0.3
	    d3yy = 0.3
	    d4yy = 0.3

	    #key matrices


	    Mrb = np.array([ [m,0,0,0,0,0],
		[0,m,0,0,0,0],
		[0,0,m,0,0,0],
		[0,0,0,Ixx,Ixy,Ixz],
		[0,0,0,Iyx,Iyy,Iyz],
		[0,0,0,Izx,Izy,Izz]])#Inertia Tensor


	    M_a = np.array([[Xu_dot ,Xv_dot, Xw_dot, Xp_dot, Xq_dot, Xr_dot],
		[Yu_dot, Yv_dot, Yw_dot, Yp_dot, Yq_dot, Yr_dot],
		[Zu_dot, Zv_dot, Zw_dot, Zp_dot, Zq_dot, Zr_dot],
		[Ku_dot, Kv_dot, Kw_dot, Kp_dot, Kq_dot, Kr_dot],
		[Mu_dot, Mv_dot, Mw_dot, Mp_dot, Mq_dot, Mr_dot],
		[Nu_dot, Nv_dot, Nw_dot, Np_dot, Nq_dot, Nr_dot]]) #Added Mass inertia tensor

	    M_t = Mrb - M_a #Total inertia tensor

	    M_t_inv = np.linalg.inv(M_t) #Inverse of total inertia tensor
	   
	    ################################################################
	    #This Partition is used to compute the K controller gain using LQR

	    #Here, we can control Surge speed, Depth, Steering and Style mission angles
	    #First we need to compute partial derivatives to construct the A matrix
	    df_du = np.zeros((6,1))
	    df_dv =np.zeros((6,1))
	    df_dw =np.zeros((6,1))
	    df_dp =np.zeros((6,1))
	    df_dq =np.zeros((6,1))
	    df_dr =np.zeros((6,1))
	    df_dphi =np.zeros((6,1))
	    df_dtht =np.zeros((6,1))
	    ## df_deps
	    ## df_dz
	    #Also we need to compute the partial derivatives for the B matrix
	    df_dt1 =np.zeros((6,1))
	    df_dt2 =np.zeros((6,1))
	    df_dt3 =np.zeros((6,1))
	    df_dt4 =np.zeros((6,1))
	    df_dt5 =np.zeros((6,1))
	    df_dt6 =np.zeros((6,1))
	    df_dt7 =np.zeros((6,1))
	    df_dt8 =np.zeros((6,1))

	    #Compute partial derivatives for first part of the matrix
	    for i in range(0,6):
	       
		df_du[i][0] = M_t_inv[i][0]*self.setpoint['qo']*Zu_dot - M_t_inv[i][0]*self.setpoint['ro']*Yu_dot\
		-M_t_inv[i][1]*self.setpoint['po']*Zu_dot - M_t_inv[i][1]*self.setpoint['ro']*m + M_t_inv[i][1]*self.setpoint['ro']*Xu_dot\
		+M_t_inv[i][2]*self.setpoint['po']*Yu_dot + M_t_inv[i][2]*self.setpoint['qo']*m - M_t_inv[i][2]*self.setpoint['qo']*Xu_dot\
		+ M_t_inv[i][3]*self.setpoint['vo']*Zu_dot - M_t_inv[i][3]*self.setpoint['wo']*Yu_dot + M_t_inv[i][3]*self.setpoint['qo']*Nu_dot\
		- M_t_inv[i][3]*self.setpoint['ro']*Mu_dot + 2*M_t_inv[i][4]*m*self.setpoint['uo'] - 2*M_t_inv[i][4]*Zu_dot*self.setpoint['uo']\
		- M_t_inv[i][4]*self.setpoint['vo']*Zv_dot - M_t_inv[i][4]*self.setpoint['wo']*Zw_dot - M_t_inv[i][4]*self.setpoint['po']*Zp_dot\
		- M_t_inv[i][4]*self.setpoint['qo']*Zq_dot - M_t_inv[i][4]*self.setpoint['ro']*Zr_dot - M_t_inv[i][4]*self.setpoint['wo']*m\
		+ M_t_inv[i][4]*self.setpoint['wo']*Xu_dot - M_t_inv[i][4]*self.setpoint['po']*Nu_dot + M_t_inv[i][4]*self.setpoint['ro']*Ku_dot\
		- M_t_inv[i][5]*self.setpoint['vo']*m + 2*M_t_inv[i][5]*self.setpoint['uo']*Yu_dot + M_t_inv[i][5]*self.setpoint['vo']*Yv_dot\
		+ M_t_inv[i][5]*self.setpoint['wo']*Yw_dot + M_t_inv[i][5]*self.setpoint['po']*Yp_dot + M_t_inv[i][5]*self.setpoint['qo']*Yq_dot\
		+ M_t_inv[i][5]*self.setpoint['ro']*Yr_dot + M_t_inv[i][5]*self.setpoint['vo']*m - M_t_inv[i][5]*self.setpoint['vo']*Xu_dot\
		+ M_t_inv[i][5]*self.setpoint['po']*Mu_dot - M_t_inv[i][5]*self.setpoint['qo']*Ku_dot - 2*M_t_inv[i][0]*Xuu*abs(self.setpoint['uo'])\
		+ 2*M_t_inv[i][0]*Xlu*self.setpoint['uo'] - 2*M_t_inv[i][1]*Yuu*abs(self.setpoint['uo'])\
		+ 2*M_t_inv[i][1]*Ylu*self.setpoint['uo'] - 2*M_t_inv[i][2]*Zuu*abs(self.setpoint['uo'])\
		+ 2*M_t_inv[i][2]*Zlu*self.setpoint['uo'] - 2*M_t_inv[i][3]*Kuu*abs(self.setpoint['uo'])\
		+ 2*M_t_inv[i][3]*Klu*self.setpoint['uo'] - 2*M_t_inv[i][4]*Muu*abs(self.setpoint['uo'])\
		+ 2*M_t_inv[i][4]*Mlu*self.setpoint['uo'] - 2*M_t_inv[i][5]*Nuu*abs(self.setpoint['uo'])\
		+ 2*M_t_inv[i][5]*Nlu*self.setpoint['uo']
		
		df_dv[i][0] = M_t_inv[i][0]*self.setpoint['qo']*Zv_dot + M_t_inv[i][0]*self.setpoint['ro']*m\
		    - M_t_inv[i][0]*self.setpoint['ro']*Yv_dot - M_t_inv[i][0]*self.setpoint['po']*Zv_dot + M_t_inv[i][0]*self.setpoint['ro']*Xv_dot\
		    - M_t_inv[i][1]*self.setpoint['po']*m + M_t_inv[i][1]*self.setpoint['po']*Yv_dot - M_t_inv[i][1]*self.setpoint['qo']*Xv_dot\
		    - M_t_inv[i][2]*self.setpoint['wo']*m + M_t_inv[i][2]*self.setpoint['uo']*Zu_dot + 2*M_t_inv[i][2]*self.setpoint['vo']*Zv_dot\
		    + M_t_inv[i][2]*self.setpoint['wo']*Zw_dot + M_t_inv[i][2]*Zp_dot*self.setpoint['po'] + M_t_inv[i][2]*Zq_dot*self.setpoint['qo']\
		    - M_t_inv[i][2]*self.setpoint['ro']*Zr_dot + M_t_inv[i][2]*self.setpoint['wo']*m - M_t_inv[i][2]*self.setpoint['wo']*Yv_dot\
		    + M_t_inv[i][2]*self.setpoint['qo']*Nv_dot - M_t_inv[i][2]*self.setpoint['ro']*Mv_dot - M_t_inv[i][3]*self.setpoint['uo']*Zv_dot\
		    + M_t_inv[i][3]*self.setpoint['wo']*Xv_dot - M_t_inv[i][3]*self.setpoint['po']*Nv_dot + M_t_inv[i][3]*self.setpoint['ro']*Kv_dot\
		    - M_t_inv[i][4]*self.setpoint['uo']*m + M_t_inv[i][4]*self.setpoint['uo']*Yv_dot + M_t_inv[i][4]*self.setpoint['uo']*m\
		    - M_t_inv[i][4]*self.setpoint['uo']*Xu_dot - 2*M_t_inv[i][4]*self.setpoint['vo']*Xv_dot - M_t_inv[i][4]*self.setpoint['wo']*Xw_dot\
		    - M_t_inv[i][4]*self.setpoint['po']*Xp_dot - M_t_inv[i][4]*self.setpoint['qo']*Xq_dot - M_t_inv[i][4]*self.setpoint['ro']*Xr_dot- M_t_inv[i][4]*self.setpoint['po']*Mv_dot\
		    - M_t_inv[i][4]*self.setpoint['qo']*Kv_dot - 2*M_t_inv[i][0]*abs(self.setpoint['vo'])*Xvv + 2*M_t_inv[i][0]*Xlv*self.setpoint['vo']\
		    - 2*M_t_inv[i][0]*Yvv*abs(self.setpoint['vo']) + 2*M_t_inv[i][0]*Ylv*self.setpoint['vo'] - 2*M_t_inv[i][1]*Zvv*abs(self.setpoint['vo']) + 2*M_t_inv[i][1]*Zlv*self.setpoint['vo']- 2*M_t_inv[i][2]*Kvv*abs(self.setpoint['vo']) + 2*M_t_inv[i][2]*Klv*self.setpoint['vo'] - 2*M_t_inv[i][3]*Mvv*abs(self.setpoint['vo']) + 2*M_t_inv[i][3]*Mlv*self.setpoint['vo'] - 2*M_t_inv[i][4]*Nvv*abs(self.setpoint['vo']) + 2*M_t_inv[i][4]*Nlv*self.setpoint['vo'] 

		df_dw[i][0] = -M_t_inv[i][0]*self.setpoint['qo']*m + M_t_inv[i][0]*self.setpoint['qo']*Zw_dot\
		- M_t_inv[i][0]*self.setpoint['ro']*Yw_dot + M_t_inv[i][1]*self.setpoint['po']*m - M_t_inv[i][1]*self.setpoint['po']*Zw_dot\
		+ M_t_inv[i][1]*self.setpoint['ro']*Xw_dot + M_t_inv[i][2]*self.setpoint['po']*Yw_dot - M_t_inv[i][2]*self.setpoint['qo']*Xw_dot\
		- M_t_inv[i][3]*self.setpoint['vo']*m + M_t_inv[i][3]*self.setpoint['vo']*Zw_dot + M_t_inv[i][3]*self.setpoint['vo']*m\
		- M_t_inv[i][3]*self.setpoint['uo']*Yu_dot - M_t_inv[i][3]*Yv_dot*self.setpoint['vo'] - 2*M_t_inv[i][3]*Yw_dot*self.setpoint['wo']\
		- M_t_inv[i][3]*self.setpoint['po']*Yp_dot - M_t_inv[i][3]*self.setpoint['qo']*Yq_dot - M_t_inv[i][3]*self.setpoint['ro']*Yr_dot\
		+ M_t_inv[i][3]*self.setpoint['qo']*Nw_dot - M_t_inv[i][3]*self.setpoint['ro']*Mw_dot - M_t_inv[i][4]*self.setpoint['uo']*Zw_dot\
		- M_t_inv[i][4]*self.setpoint['uo']*m + M_t_inv[i][4]*self.setpoint['uo']*Xu_dot + M_t_inv[i][4]*self.setpoint['vo']*Xv_dot\
		+ 2*M_t_inv[i][4]*self.setpoint['wo']*Xw_dot + M_t_inv[i][4]*self.setpoint['po']*Xp_dot + M_t_inv[i][4]*self.setpoint['qo']*Xq_dot\
		+ M_t_inv[i][4]*self.setpoint['ro']*Xr_dot - M_t_inv[i][4]*self.setpoint['po']*Nw_dot + M_t_inv[i][4]*self.setpoint['ro']*Kw_dot\
		+ M_t_inv[i][5]*self.setpoint['uo']*Yw_dot - M_t_inv[i][5]*self.setpoint['vo']*Xw_dot + M_t_inv[i][5]*self.setpoint['po']*Mw_dot\
		- M_t_inv[i][5]*self.setpoint['qo']*Kw_dot - 2*M_t_inv[i][0]*abs(self.setpoint['wo'])*Xww + 2*M_t_inv[i][0]*Xlw*self.setpoint['wo']\
		- 2*M_t_inv[i][1]*Yww*abs(self.setpoint['wo']) + 2*M_t_inv[i][1]*Ylw*self.setpoint['wo']\
		- 2*M_t_inv[i][2]*Zww*abs(self.setpoint['wo']) + 2*M_t_inv[i][2]*Zlw*self.setpoint['wo']\
		- 2*M_t_inv[i][3]*Kww*abs(self.setpoint['wo']) + 2*M_t_inv[i][3]*Klw*self.setpoint['wo']\
		- 2*M_t_inv[i][4]*Mww*abs(self.setpoint['wo']) + 2*M_t_inv[i][4]*Mlw*self.setpoint['wo']\
		- 2*M_t_inv[i][5]*Nww*abs(self.setpoint['wo']) + 2*M_t_inv[i][5]*Nlw*self.setpoint['wo']

		df_dp[i][0] = M_t_inv[i][0]*self.setpoint['qo']*Zp_dot - M_t_inv[i][0]*self.setpoint['ro']*Yp_dot\
		    +M_t_inv[i][1]*self.setpoint['wo']*m - M_t_inv[i][1]*self.setpoint['uo']*Zu_dot - M_t_inv[i][1]*self.setpoint['vo']*Zv_dot\
		    -M_t_inv[i][1]*self.setpoint['wo']*Zw_dot - 2*M_t_inv[i][1]*self.setpoint['po']*Zp_dot - M_t_inv[i][1]*self.setpoint['qo']*Zq_dot\
		    - M_t_inv[i][1]*self.setpoint['ro']*Zr_dot - M_t_inv[i][1]*self.setpoint['ro']*Xp_dot - M_t_inv[i][2]*self.setpoint['vo']*m\
		    + M_t_inv[i][2]*self.setpoint['uo']*Yu_dot + M_t_inv[i][2]*Yv_dot*self.setpoint['vo'] - M_t_inv[i][2]*Yw_dot*self.setpoint['wo']\
		    + 2*M_t_inv[i][2]*self.setpoint['po']*Yp_dot + M_t_inv[i][2]*self.setpoint['qo']*Yq_dot + M_t_inv[i][2]*self.setpoint['ro']*Yr_dot\
		    - M_t_inv[i][2]*self.setpoint['qo']*Xp_dot + M_t_inv[i][3]*self.setpoint['vo']*Zp_dot - M_t_inv[i][3]*self.setpoint['wo']*Yp_dot\
		    - M_t_inv[i][3]*self.setpoint['qo']*Izx + M_t_inv[i][3]*self.setpoint['qo']*Np_dot + M_t_inv[i][3]*self.setpoint['ro']*Iyx\
		    - M_t_inv[i][3]*self.setpoint['ro']*Mp_dot - M_t_inv[i][4]*self.setpoint['uo']*Zp_dot + M_t_inv[i][4]*self.setpoint['wo']*Xp_dot\
		    + 2*M_t_inv[i][4]*self.setpoint['po']*Izx + M_t_inv[i][4]*self.setpoint['qo']*Izy + M_t_inv[i][4]*self.setpoint['ro']*Izz\
		    - M_t_inv[i][4]*self.setpoint['uo']*Nu_dot - M_t_inv[i][4]*self.setpoint['vo']*Nv_dot - M_t_inv[i][4]*self.setpoint['wo']*Nw_dot\
		    - 2*M_t_inv[i][4]*self.setpoint['po']*Np_dot - M_t_inv[i][4]*Nq_dot*self.setpoint['qo'] - M_t_inv[i][4]*Nr_dot*self.setpoint['ro']\
		    - M_t_inv[i][4]*self.setpoint['ro']*Ixx + M_t_inv[i][4]*self.setpoint['ro']*Kp_dot + M_t_inv[i][5]*self.setpoint['uo']*Yp_dot\
		    - M_t_inv[i][5]*self.setpoint['vo']*Xp_dot - 2*M_t_inv[i][5]*self.setpoint['po']*Iyx - M_t_inv[i][5]*self.setpoint['qo']*Iyy\
		    - M_t_inv[i][5]*self.setpoint['ro']*Iyz + M_t_inv[i][5]*Mu_dot*self.setpoint['uo'] + M_t_inv[i][5]*Mv_dot*self.setpoint['vo']\
		    + M_t_inv[i][5]*Mw_dot*self.setpoint['wo'] + 2*M_t_inv[i][5]*Mp_dot*self.setpoint['po'] + M_t_inv[i][5]*Mq_dot*self.setpoint['qo']\
		    - M_t_inv[i][5]*Mr_dot*self.setpoint['ro'] + M_t_inv[i][5]*self.setpoint['qo']*Ixx - M_t_inv[i][5]*self.setpoint['qo']*Kp_dot\
		     - 2*M_t_inv[i][0]*abs(self.setpoint['po'])*Xpp + 2*M_t_inv[i][0]*Xlp*self.setpoint['po']\
		    - 2*M_t_inv[i][1]*Ypp*abs(self.setpoint['po']) + 2*M_t_inv[i][1]*Ylp*self.setpoint['po']\
		    - 2*M_t_inv[i][2]*Zpp*abs(self.setpoint['po']) + 2*M_t_inv[i][2]*Zlp*self.setpoint['po']- 2*M_t_inv[i][3]*Kpp*abs(self.setpoint['po']) + 2*M_t_inv[i][3]*Klp*self.setpoint['po']- 2*M_t_inv[i][4]*Mpp*abs(self.setpoint['po']) + 2*M_t_inv[i][4]*Mlp*self.setpoint['po']- 2*M_t_inv[i][5]*Npp*abs(self.setpoint['po']) + 2*M_t_inv[i][5]*Nlp*self.setpoint['po'] 

		df_dq[i][0] = -M_t_inv[i][0]*self.setpoint['wo']*m + M_t_inv[i][0]*self.setpoint['uo']*Zu_dot\
		    +M_t_inv[i][0]*self.setpoint['vo']*Zv_dot + M_t_inv[i][0]*self.setpoint['wo']*Zw_dot + M_t_inv[i][0]*self.setpoint['po']*Zp_dot\
		    +2*M_t_inv[i][0]*self.setpoint['qo']*Zq_dot + M_t_inv[i][0]*self.setpoint['ro']*Zr_dot - M_t_inv[i][0]*self.setpoint['ro']*Yq_dot\
		    - M_t_inv[i][1]*self.setpoint['po']*Zq_dot + M_t_inv[i][1]*self.setpoint['ro']*Xq_dot + M_t_inv[i][2]*self.setpoint['po']*Yq_dot\
		    + M_t_inv[i][2]*self.setpoint['uo']*m - M_t_inv[i][2]*Xu_dot*self.setpoint['uo'] - M_t_inv[i][2]*Xv_dot*self.setpoint['vo']\
		    - M_t_inv[i][2]*self.setpoint['wo']*Xw_dot - M_t_inv[i][2]*self.setpoint['po']*Xp_dot + 2*M_t_inv[i][2]*self.setpoint['qo']*Xq_dot\
		    - M_t_inv[i][2]*self.setpoint['ro']*Xr_dot + M_t_inv[i][3]*self.setpoint['vo']*Zq_dot - M_t_inv[i][3]*self.setpoint['wo']*Yq_dot\
		    - M_t_inv[i][3]*self.setpoint['po']*Izx - 2*M_t_inv[i][3]*self.setpoint['qo']*Izy - M_t_inv[i][3]*self.setpoint['ro']*Izz\
		    + M_t_inv[i][3]*self.setpoint['uo']*Nu_dot + M_t_inv[i][3]*self.setpoint['vo']*Nv_dot + M_t_inv[i][3]*self.setpoint['wo']*Nw_dot\
		    + M_t_inv[i][3]*self.setpoint['po']*Np_dot + M_t_inv[i][3]*self.setpoint['qo']*Nq_dot + M_t_inv[i][3]*self.setpoint['ro']*Nr_dot\
		    + M_t_inv[i][3]*self.setpoint['ro']*Iyy - M_t_inv[i][3]*self.setpoint['ro']*Mq_dot - M_t_inv[i][4]*self.setpoint['uo']*Zq_dot\
		    + M_t_inv[i][4]*self.setpoint['wo']*Xq_dot + M_t_inv[i][4]*Izy*self.setpoint['po'] - M_t_inv[i][4]*Nq_dot*self.setpoint['po']\
		    - M_t_inv[i][4]*self.setpoint['ro']*Ixy + M_t_inv[i][4]*self.setpoint['ro']*Kq_dot + M_t_inv[i][5]*self.setpoint['uo']*Yq_dot\
		    - M_t_inv[i][5]*self.setpoint['vo']*Xq_dot - M_t_inv[i][5]*self.setpoint['po']*Iyy + M_t_inv[i][5]*self.setpoint['po']*Mq_dot\
		    + M_t_inv[i][5]*self.setpoint['po']*Ixx + 2*M_t_inv[i][5]*Ixy*self.setpoint['qo'] + M_t_inv[i][5]*Ixz*self.setpoint['ro']\
		    - M_t_inv[i][5]*Ku_dot*self.setpoint['uo'] - M_t_inv[i][5]*Kv_dot*self.setpoint['vo'] - M_t_inv[i][5]*Kw_dot*self.setpoint['wo']\
		    - M_t_inv[i][5]*Kp_dot*self.setpoint['po'] - 2*M_t_inv[i][5]*self.setpoint['qo']*Kq_dot - M_t_inv[i][5]*self.setpoint['ro']*Kr_dot\
		    - 2*M_t_inv[i][0]*abs(self.setpoint['qo'])*Xqq + 2*M_t_inv[i][0]*Xlq*self.setpoint['qo']\
		    - 2*M_t_inv[i][1]*Yqq*abs(self.setpoint['qo']) + 2*M_t_inv[i][1]*Ylq*self.setpoint['qo']\
		    - 2*M_t_inv[i][2]*Zqq*abs(self.setpoint['qo']) + 2*M_t_inv[i][2]*Zlq*self.setpoint['qo']- 2*M_t_inv[i][3]*Kqq*abs(self.setpoint['qo']) + 2*M_t_inv[i][3]*Klq*self.setpoint['qo']- 2*M_t_inv[i][4]*Mqq*abs(self.setpoint['qo']) + 2*M_t_inv[i][4]*Mlq*self.setpoint['qo']- 2*M_t_inv[i][5]*Nqq*abs(self.setpoint['qo']) + 2*M_t_inv[i][5]*Nlq*self.setpoint['qo'] 

		df_dr[i][0] = M_t_inv[i][0]*self.setpoint['qo']*Zr_dot + M_t_inv[i][0]*self.setpoint['vo']*m - M_t_inv[i][0]*self.setpoint['qo']*Yq_dot\
		    - 2*M_t_inv[i][0]*self.setpoint['ro']*Yr_dot - M_t_inv[i][1]*self.setpoint['po']*Zr_dot- M_t_inv[i][1]*self.setpoint['uo']*m + M_t_inv[i][1]*self.setpoint['uo']*Xu_dot\
		    + M_t_inv[i][1]*self.setpoint['vo']*Xu_dot + M_t_inv[i][1]*self.setpoint['wo']*Xw_dot + M_t_inv[i][1]*self.setpoint['po']*Xp_dot + M_t_inv[i][1]*self.setpoint['qo']*Xq_dot\
		    + 2*M_t_inv[i][1]*self.setpoint['ro']*Xr_dot + M_t_inv[i][2]*Yr_dot*self.setpoint['po'] - M_t_inv[i][2]*Xr_dot*self.setpoint['qo']+ M_t_inv[i][3]*self.setpoint['vo']*Zr_dot\
		    - M_t_inv[i][3]*self.setpoint['wo']*Yr_dot - M_t_inv[i][3]*self.setpoint['qo']*Izz + M_t_inv[i][3]*self.setpoint['qo']*Nr_dot + M_t_inv[i][3]*self.setpoint['po']*Iyx\
		    + M_t_inv[i][3]*self.setpoint['qo']*Iyy + 2*M_t_inv[i][3]*self.setpoint['ro']*Iyz - M_t_inv[i][3]*self.setpoint['uo']*Mu_dot - M_t_inv[i][3]*self.setpoint['vo']*Mv_dot\
		    - M_t_inv[i][3]*self.setpoint['wo']*Mw_dot - M_t_inv[i][3]*self.setpoint['po']*Mp_dot - M_t_inv[i][3]*self.setpoint['qo']*Mq_dot - 2* M_t_inv[i][3]*self.setpoint['ro']*Mr_dot\
		    - M_t_inv[i][4]*self.setpoint['uo']*Zr_dot + M_t_inv[i][4]*self.setpoint['wo']*Xr_dot + M_t_inv[i][4]*self.setpoint['po']*Izz - M_t_inv[i][4]*self.setpoint['po']*Nr_dot\
		    - M_t_inv[i][4]*self.setpoint['po']*Ixx- M_t_inv[i][4]*self.setpoint['qo']*Ixy - 2*M_t_inv[i][4]*Ixz*self.setpoint['ro'] + M_t_inv[i][4]*Ku_dot*self.setpoint['uo']\
		    - M_t_inv[i][4]*self.setpoint['vo']*Kv_dot + M_t_inv[i][4]*self.setpoint['wo']*Kw_dot + M_t_inv[i][4]*self.setpoint['po']*Kp_dot + M_t_inv[i][4]*self.setpoint['qo']*Kq_dot\
		    + 2*M_t_inv[i][4]*self.setpoint['ro']*Kr_dot + M_t_inv[i][5]*self.setpoint['uo']*Yr_dot - M_t_inv[i][5]*self.setpoint['vo']*Xr_dot -M_t_inv[i][5]*Iyz*self.setpoint['po']\
		    + M_t_inv[i][5]*Mr_dot*self.setpoint['po'] + M_t_inv[i][5]*Ixz*self.setpoint['qo'] - M_t_inv[i][5]*Kr_dot*self.setpoint['qo'] - 2*M_t_inv[i][0]*abs(self.setpoint['ro'])*Xrr\
		    + 2*M_t_inv[i][0]*Xlr*self.setpoint['ro'] - 2*M_t_inv[i][1]*Yrr*abs(self.setpoint['ro']) + 2*M_t_inv[i][1]*Ylr*self.setpoint['ro']- 2*M_t_inv[i][2]*Zrr*abs(self.setpoint['ro'])\
		    + 2*M_t_inv[i][2]*Zlr*self.setpoint['ro']- 2*M_t_inv[i][3]*Krr*abs(self.setpoint['ro']) + 2*M_t_inv[i][3]*Klr*self.setpoint['ro']- 2*M_t_inv[i][4]*Mrr*abs(self.setpoint['ro'])\
		    + 2*M_t_inv[i][4]*Mlr*self.setpoint['ro']- 2*M_t_inv[i][5]*Nrr*abs(self.setpoint['ro']) + 2*M_t_inv[i][5]*Nlr*self.setpoint['ro'] 
		
		df_dphi[i][0] = M_t_inv[i][1]*math.cos(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*W - M_t_inv[i][1]*math.cos(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*B\
		    - M_t_inv[i][2]*math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*W + M_t_inv[i][2]*math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B\
		    - M_t_inv[i][3]*COB[1][0]*math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B\
		    - M_t_inv[i][3]*COB[2][0]*math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B\
		    + M_t_inv[i][4]*COB[0][0]*math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B\
		    + M_t_inv[i][5]*COB[0][0]*math.cos(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*B 

		df_dtht[i][0] = -M_t_inv[i][0]*math.cos(self.setpoint['thto'])*W + M_t_inv[i][0]*math.cos(self.setpoint['thto'])*B\
		    - M_t_inv[i][1]*math.sin(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*W + M_t_inv[i][1]*math.sin(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B\
		    - M_t_inv[i][2]*math.sin(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*W + M_t_inv[i][2]*math.sin(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*B\
		    - M_t_inv[i][3]*COB[1][0]*math.sin(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*B\
		    + M_t_inv[i][3]*COB[2][0]*math.sin(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B\
		    - M_t_inv[i][4]*COB[2][0]*math.cos(self.setpoint['thto'])*B + M_t_inv[i][4]*COB[0][0]*math.sin(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*B\
		    - M_t_inv[i][5]*COB[0][0]*math.sin(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*B + M_t_inv[i][5]*COB[1][0]*math.cos(self.setpoint['thto'])*B 

		df_dt1[i][0] = M_t_inv[i][0] * math.sin(math.pi/4) + M_t_inv[i][1]*-math.cos(math.pi/4)\
		    + M_t_inv[i][3]*d1x*math.cos(math.pi/4)+ M_t_inv[i][4]*d1y*math.sin(math.pi/4)\
		    + M_t_inv[i][5]*(-d1xx*math.sin(math.pi/4)- d1yy*math.cos(math.pi/4)) 

		df_dt2[i][0] = M_t_inv[i][0] * math.sin(math.pi/4) + M_t_inv[i][1] * math.cos(math.pi/4)\
		    + M_t_inv[i][3]*-d2x*math.cos(math.pi/4)+ M_t_inv[i][4]*d2y*math.sin(math.pi/4)\
		    + M_t_inv[i][5]*(d2xx*math.sin(math.pi/4)+d2yy*math.cos(math.pi/4)) 

		df_dt3[i][0] = M_t_inv[i][0] * math.sin(math.pi/4) + M_t_inv[i][1] * math.cos(math.pi/4)\
		    + M_t_inv[i][3]*-d3x*math.cos(math.pi/4)+ M_t_inv[i][4]*d3y*math.sin(math.pi/4)\
		    + M_t_inv[i][5]*(-d3xx*math.sin(math.pi/4)-d3yy*math.cos(math.pi/4)) 

		df_dt4[i][0] = M_t_inv[i][0] * math.sin(math.pi/4) + M_t_inv[i][1] * -math.cos(math.pi/4)\
		    + M_t_inv[i][3]*d4x*math.cos(math.pi/4)+ M_t_inv[i][4]*d4y*math.sin(math.pi/4)\
		    + M_t_inv[i][5]*(d4xx*math.sin(math.pi/4)+d4yy*math.cos(math.pi/4)) 

		df_dt5[i][0] = M_t_inv[i][2] + M_t_inv[i][3]*d5x + M_t_inv[i][4]*-d5y 

		df_dt6[i][0] = M_t_inv[i][2] + M_t_inv[i][3]*-d6x + M_t_inv[i][4]*-d6y 

		df_dt7[i][0] = M_t_inv[i][2] + M_t_inv[i][3]*d7x + M_t_inv[i][4]*d7y 

		df_dt8[i][0] = M_t_inv[i][2] + M_t_inv[i][3]*-d8x + M_t_inv[i][4]*d8y 
	    
	    
	    #Now, we record the full list of partial derivatives 
	    df1_dz = 0 
	    df2_dz = 0 
	    df3_dz = 0 
	    df4_dz = 0 
	    df5_dz = 0 
	    df6_dz = 0 
	    df7_dz = 0 
	    df8_dz = 0 
	    df9_dz = 0 
	    df10_dz = 0 

	    df1_dphi = math.cos(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*self.setpoint['vo'] - math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*self.setpoint['wo'] 
	    df2_dphi = math.cos(self.setpoint['phio'])*math.tan(self.setpoint['thto'])*self.setpoint['qo'] - math.sin(self.setpoint['phio'])*math.tan(self.setpoint['thto'])*self.setpoint['ro'] 
	    df3_dphi = -self.setpoint['qo']*math.sin(self.setpoint['phio']) - self.setpoint['ro']*math.cos(self.setpoint['phio']) 
	    df4_dphi = self.setpoint['qo']*(math.cos(self.setpoint['phio'])/math.cos(self.setpoint['thto'])) - self.setpoint['ro']*(math.sin(self.setpoint['phio'])/math.cos(self.setpoint['thto'])) 
	    df5_dphi = df_dphi[0][0]
	    df6_dphi = df_dphi[1][0] 
	    df7_dphi = df_dphi[2][0]
	    df8_dphi = df_dphi[3][0] 
	    df9_dphi = df_dphi[4][0]
	    df10_dphi = df_dphi[5][0] 

	    df1_dtht = -math.cos(self.setpoint['thto'])*self.setpoint['uo'] - math.sin(self.setpoint['thto'])*math.cos(self.setpoint['phio'])*self.setpoint['wo'] - math.sin(self.setpoint['thto'])*math.sin(self.setpoint['phio'])*self.setpoint['vo'] 
	    df2_dtht = math.sin(self.setpoint['phio'])*(1/(math.cos(self.setpoint['thto'])**2))*self.setpoint['qo'] + math.cos(self.setpoint['phio'])*(1/(math.cos(self.setpoint['thto'])**2))*self.setpoint['ro'] 
	    df3_dtht = 0 
	    df4_dtht = self.setpoint['qo']*math.sin(self.setpoint['phio'])*math.tan(self.setpoint['thto'])*(1/math.cos(self.setpoint['thto'])) + self.setpoint['ro']*math.cos(self.setpoint['phio'])*math.tan(self.setpoint['thto'])*(1/math.cos(self.setpoint['thto'])) 
	    df5_dtht = df_dtht[0][0] 
	    df6_dtht = df_dtht[1][0] 
	    df7_dtht = df_dtht[2][0] 
	    df8_dtht = df_dtht[3][0]  
	    df9_dtht = df_dtht[4][0]
	    df10_dtht = df_dtht[5][0] 

	    df1_deps = 0 
	    df2_deps = 0 
	    df3_deps = 0 
	    df4_deps = 0 
	    df5_deps = 0 
	    df6_deps = 0 
	    df7_deps = 0 
	    df8_deps = 0 
	    df9_deps = 0 
	    df10_deps = 0 

	    df1_du = -math.sin(self.setpoint['thto']) 
	    df2_du = 0 
	    df3_du = 0 
	    df4_du = 0 
	    df5_du = df_du[0][0] 
	    df6_du = df_du[1][0]
	    df7_du = df_du[2][0] 
	    df8_du = df_du[3][0]
	    df9_du = df_du[4][0]
	    df10_du = df_du[5][0]

	    df1_dv = math.cos(self.setpoint['thto'])*math.sin(self.setpoint['phio']) 
	    df2_dv = 0 
	    df3_dv = 0 
	    df4_dv = 0 
	    df5_dv = df_dv[0][0]
	    df6_dv = df_dv[1][0] 
	    df7_dv = df_dv[2][0] 
	    df8_dv = df_dv[3][0] 
	    df9_dv = df_dv[4][0] 
	    df10_dv = df_dv[5][0] 

	    df1_dw = math.cos(self.setpoint['thto'])*math.cos(self.setpoint['phio']) 
	    df2_dw = 0 
	    df3_dw = 0 
	    df4_dw = 0 
	    df5_dw = df_dw[0][0] 
	    df6_dw = df_dw[1][0]
	    df7_dw = df_dw[2][0] 
	    df8_dw = df_dw[3][0]
	    df9_dw = df_dw[4][0] 
	    df10_dw = df_dw[5][0]

	    df1_dp = 0 
	    df2_dp = 1 
	    df3_dp = 0 
	    df4_dp = 0 
	    df5_dp = df_dp[0][0] 
	    df6_dp = df_dp[1][0] 
	    df7_dp = df_dp[2][0] 
	    df8_dp = df_dp[3][0]
	    df9_dp = df_dp[4][0]
	    df10_dp = df_dp[5][0]

	    df1_dq = 0 
	    df2_dq = math.sin(self.setpoint['phio'])*math.tan(self.setpoint['thto']) 
	    df3_dq = math.cos(self.setpoint['phio']) 
	    df4_dq = math.sin(self.setpoint['phio'])/math.cos(self.setpoint['thto']) 
	    df5_dq = df_dq[0][0] 
	    df6_dq = df_dq[1][0] 
	    df7_dq = df_dq[2][0] 
	    df8_dq = df_dq[3][0] 
	    df9_dq = df_dq[4][0] 
	    df10_dq = df_dq[5][0] 

	    df1_dr = 0 
	    df2_dr = math.cos(self.setpoint['phio'])*math.tan(self.setpoint['thto']) 
	    df3_dr = -math.sin(self.setpoint['phio']) 
	    df4_dr = math.cos(self.setpoint['phio'])/math.cos(self.setpoint['thto']) 
	    df5_dr = df_dr[0][0] 
	    df6_dr = df_dr[1][0] 
	    df7_dr = df_dr[2][0] 
	    df8_dr = df_dr[3][0] 
	    df9_dr = df_dr[4][0] 
	    df10_dr = df_dr[5][0] 

	    df1_dt1 = 0 
	    df2_dt1 = 0 
	    df3_dt1 = 0 
	    df4_dt1 = 0 
	    df5_dt1 = df_dt1[0][0] 
	    df6_dt1 = df_dt1[1][0] 
	    df7_dt1 = df_dt1[2][0] 
	    df8_dt1 = df_dt1[3][0] 
	    df9_dt1 = df_dt1[4][0] 
	    df10_dt1 = df_dt1[5][0] 

	    df1_dt2 = 0 
	    df2_dt2 = 0 
	    df3_dt2 = 0 
	    df4_dt2 = 0 
	    df5_dt2 = df_dt2[0][0] 
	    df6_dt2 = df_dt2[1][0] 
	    df7_dt2 = df_dt2[2][0] 
	    df8_dt2 = df_dt2[3][0] 
	    df9_dt2 = df_dt2[4][0] 
	    df10_dt2 = df_dt2[5][0] 

	    df1_dt3 = 0 
	    df2_dt3 = 0 
	    df3_dt3 = 0 
	    df4_dt3 = 0 
	    df5_dt3 = df_dt3[0][0] 
	    df6_dt3 = df_dt3[1][0] 
	    df7_dt3 = df_dt3[2][0] 
	    df8_dt3 = df_dt3[3][0] 
	    df9_dt3 = df_dt3[4][0] 
	    df10_dt3 = df_dt3[5][0] 

	    df1_dt4 = 0 
	    df2_dt4 = 0 
	    df3_dt4 = 0 
	    df4_dt4 = 0 
	    df5_dt4 = df_dt4[0][0] 
	    df6_dt4 = df_dt4[1][0] 
	    df7_dt4 = df_dt4[2][0] 
	    df8_dt4 = df_dt4[3][0] 
	    df9_dt4 = df_dt4[4][0] 
	    df10_dt4 = df_dt4[5][0] 

	    df1_dt5 = 0 
	    df2_dt5 = 0 
	    df3_dt5 = 0 
	    df4_dt5 = 0 
	    df5_dt5 = df_dt5[0][0] 
	    df6_dt5 = df_dt5[1][0] 
	    df7_dt5 = df_dt5[2][0] 
	    df8_dt5 = df_dt5[3][0] 
	    df9_dt5 = df_dt5[4][0] 
	    df10_dt5 = df_dt5[5][0] 

	    df1_dt6 = 0 
	    df2_dt6 = 0 
	    df3_dt6 = 0 
	    df4_dt6 = 0 
	    df5_dt6 = df_dt6[0][0] 
	    df6_dt6 = df_dt6[1][0] 
	    df7_dt6 = df_dt6[2][0] 
	    df8_dt6 = df_dt6[3][0] 
	    df9_dt6 = df_dt6[4][0] 
	    df10_dt6 = df_dt6[5][0] 

	    df1_dt7 = 0 
	    df2_dt7 = 0 
	    df3_dt7 = 0 
	    df4_dt7 = 0 
	    df5_dt7 = df_dt7[0][0] 
	    df6_dt7 = df_dt7[1][0] 
	    df7_dt7 = df_dt7[2][0] 
	    df8_dt7 = df_dt7[3][0] 
	    df9_dt7 = df_dt7[4][0] 
	    df10_dt7 = df_dt7[5][0] 

	    df1_dt8 = 0 
	    df2_dt8 = 0 
	    df3_dt8 = 0 
	    df4_dt8 = 0 
	    df5_dt8 = df_dt8[0][0] 
	    df6_dt8 = df_dt8[1][0] 
	    df7_dt8 = df_dt8[2][0] 
	    df8_dt8 = df_dt8[3][0] 
	    df9_dt8 = df_dt8[4][0] 
	    df10_dt8 = df_dt8[5][0] 


	    ##################################################################################


	    A_lqr=np.array([[ df1_dz, df1_dphi, df1_dtht, df1_deps, df1_du, df1_dv, df1_dw, df1_dp, df1_dq, df1_dr],
		        [df2_dz, df2_dphi, df2_dtht, df2_deps, df2_du, df2_dv, df2_dw, df2_dp, df2_dq, df2_dr],
		        [df3_dz, df3_dphi, df3_dtht, df3_deps, df3_du, df3_dv, df3_dw, df3_dp, df3_dq, df3_dr],
		        [df4_dz, df4_dphi, df4_dtht, df4_deps, df4_du, df4_dv, df4_dw, df4_dp, df4_dq, df4_dr],
		        [df5_dz, df5_dphi, df5_dtht, df5_deps, df5_du, df5_dv, df5_dw, df5_dp, df5_dq, df5_dr],
		        [df6_dz, df6_dphi, df6_dtht, df6_deps, df6_du, df6_dv, df6_dw, df6_dp, df6_dq, df6_dr],
		        [df7_dz, df7_dphi, df7_dtht, df7_deps, df7_du, df7_dv, df7_dw, df7_dp, df7_dq, df7_dr],
		        [df8_dz, df8_dphi, df8_dtht, df8_deps, df8_du, df8_dv, df8_dw, df8_dp, df8_dq, df8_dr],
		        [df9_dz, df9_dphi, df9_dtht, df9_deps, df9_du, df9_dv, df9_dw, df9_dp, df9_dq, df9_dr],
		        [df10_dz, df10_dphi, df10_dtht, df10_deps, df10_du, df10_dv, df10_dw,df10_dp, df10_dq, df10_dr]])



	    B_lqr = np.array([[ df1_dt1, df1_dt2, df1_dt3, df1_dt4, df1_dt5, df1_dt6, df1_dt7, df1_dt8],
		          [df2_dt1 ,df2_dt2, df2_dt3, df2_dt4 ,df2_dt5, df2_dt6 ,df2_dt7, df2_dt8],
		          [df3_dt1, df3_dt2, df3_dt3, df3_dt4, df3_dt5, df3_dt6, df3_dt7, df3_dt8],
		          [df4_dt1, df4_dt2, df4_dt3, df4_dt4, df4_dt5, df4_dt6, df4_dt7, df4_dt8],
		          [df5_dt1, df5_dt2, df5_dt3, df5_dt4, df5_dt5, df5_dt6, df5_dt7, df5_dt8],
		          [df6_dt1, df6_dt2, df6_dt3, df6_dt4, df6_dt5, df6_dt6,df6_dt7, df6_dt8],
		          [df7_dt1, df7_dt2, df7_dt3, df7_dt4, df7_dt5, df7_dt6, df7_dt7, df7_dt8],
		          [df8_dt1, df8_dt2, df8_dt3, df8_dt4, df8_dt5, df8_dt6, df8_dt7, df8_dt8],
		          [df9_dt1 ,df9_dt2 ,df9_dt3 ,df9_dt4, df9_dt5 ,df9_dt6 ,df9_dt7, df9_dt8],
		          [df10_dt1, df10_dt2, df10_dt3, df10_dt4, df10_dt5, df10_dt6, df10_dt7, df10_dt8]])
	    
	    if(self.typeofmotion=="forward"):
			x=1
	    elif(self.typeofmotion=="depth"):  #up and down
			x=1
	    elif(self.typeofmotion=="yaw"):   #right and left
			x=1
	    elif(self.typeofmotion=="style"):
			x=1
	    return A_lqr,B_lqr

	#get Q and R (needs to tune and update types of motion)
	def getQandR(self):
	   
		if(self.typeofmotion=="depthOnly"):	  #depth

			Q = np.array([[500000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
				  [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
				  [0, 0, 500000, 0, 0, 0, 0, 0, 0, 0],
				  [0, 0, 0, 500000, 0, 0, 0, 0, 0, 0],
				  [0, 0, 0, 0, 100000, 0, 0, 0, 0, 0],
				  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
				  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
				  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
				  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
				  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
		    
			R = np.array([[100, 0, 0, 0, 0, 0, 0, 0],
		          [0, 100, 0, 0, 0, 0, 0, 0],
		          [0, 0, 100, 0, 0, 0, 0, 0],
		          [0, 0, 0, 100, 0, 0, 0, 0],
		          [0, 0, 0, 0, 100, 0, 0, 0],
		          [0, 0, 0, 0, 0, 100, 0, 0],
		          [0, 0, 0, 0, 0, 0, 100, 0],
		          [0, 0, 0, 0, 0, 0, 0, 100]]) #penalize effort

		elif(self.typeofmotion=="forward"):	  #surge+maintaining depth  

			Q = np.array([[500000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 500000, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 500000, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 100000, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
		    
			R = np.array([[100, 0, 0, 0, 0, 0, 0, 0],
		          [0, 100, 0, 0, 0, 0, 0, 0],
		          [0, 0, 100, 0, 0, 0, 0, 0],
		          [0, 0, 0, 100, 0, 0, 0, 0],
		          [0, 0, 0, 0, 100, 0, 0, 0],
		          [0, 0, 0, 0, 0, 100, 0, 0],
		          [0, 0, 0, 0, 0, 0, 100, 0],
		          [0, 0, 0, 0, 0, 0, 0, 100]]) #penalize effort

		elif(self.typeofmotion=="backward"):	  #surge+maintaining depth

			Q = np.array([[500000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
		          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
		          [0, 0, 500000, 0, 0, 0, 0, 0, 0, 0],
		          [0, 0, 0, 500000, 0, 0, 0, 0, 0, 0],
		          [0, 0, 0, 0, 100000, 0, 0, 0, 0, 0],
		          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
		          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
		          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
		          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
		          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
	    
			R = np.array([[10, 0, 0, 0, 0, 0, 0, 0],
		          [0, 10, 0, 0, 0, 0, 0, 0],
		          [0, 0, 10, 0, 0, 0, 0, 0],
		          [0, 0, 0, 10, 0, 0, 0, 0],
		          [0, 0, 0, 0, 1000, 0, 0, 0],
		          [0, 0, 0, 0, 0, 1000, 0, 0],
		          [0, 0, 0, 0, 0, 0, 1000, 0],
		          [0, 0, 0, 0, 0, 0, 0, 1000]]) #penalize effort

		elif(self.typeofmotion=="right"):	  #heave+maintaining depth

			Q = np.array([[500000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 500000, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 500000, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 100000, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 10000000, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
	    
	   		R = np.array([[10, 0, 0, 0, 0, 0, 0, 0],
		          [0, 10, 0, 0, 0, 0, 0, 0],
		          [0, 0, 10, 0, 0, 0, 0, 0],
		          [0, 0, 0, 10, 0, 0, 0, 0],
		          [0, 0, 0, 0, 10, 0, 0, 0],
		          [0, 0, 0, 0, 0, 10, 0, 0],
		          [0, 0, 0, 0, 0, 0, 10, 0],
		          [0, 0, 0, 0, 0, 0, 0, 10]]) #penalize effort

	   	elif(self.typeofmotion=="left"):		  #heave+maintaining depth 
		 
			Q = np.array([[500000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 500000, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 500000, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 100000, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 100000, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
			
			R = np.array([[10, 0, 0, 0, 0, 0, 0, 0],
		          [0, 10, 0, 0, 0, 0, 0, 0],
		          [0, 0, 10, 0, 0, 0, 0, 0],
		          [0, 0, 0, 10, 0, 0, 0, 0],
		          [0, 0, 0, 0, 10, 0, 0, 0],
		          [0, 0, 0, 0, 0, 10, 0, 0],
		          [0, 0, 0, 0, 0, 0, 10, 0],
		          [0, 0, 0, 0, 0, 0, 0, 10]]) #penalize effort

		elif(self.typeofmotion=="style"):	  #rotation+maintaining depth  

			Q = np.array([[10000000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 5000, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 5000, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 100000, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 100000, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
		    
			R = np.array([[1000, 0, 0, 0, 0, 0, 0, 0],
		          [0, 1000, 0, 0, 0, 0, 0, 0],
		          [0, 0, 1000, 0, 0, 0, 0, 0],
		          [0, 0, 0, 1000, 0, 0, 0, 0],
		          [0, 0, 0, 0, 10, 0, 0, 0],
		          [0, 0, 0, 0, 0, 10, 0, 0],
		          [0, 0, 0, 0, 0, 0, 10, 0],
		          [0, 0, 0, 0, 0, 0, 0, 10]]) #penalize effort

		elif(self.typeofmotion=="forwardSteering"):#velocity+rotation+maintaining depth  

			Q = np.array([[10000000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 5000, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 5000, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 50000, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 1000, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
	    
			R = np.array([[10, 0, 0, 0, 0, 0, 0, 0],
		          [0, 10, 0, 0, 0, 0, 0, 0],
		          [0, 0, 10, 0, 0, 0, 0, 0],
		          [0, 0, 0, 10, 0, 0, 0, 0],
		          [0, 0, 0, 0, 10, 0, 0, 0],
		          [0, 0, 0, 0, 0, 10, 0, 0],
		          [0, 0, 0, 0, 0, 0, 10, 0],
		          [0, 0, 0, 0, 0, 0, 0, 10]]) #penalize effort

		elif(self.typeofmotion=="backwardSteering"):#velocity+rotation+maintaining depth

			Q = np.array([[10000000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 500000, 0, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 5000, 0, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 5000, 0, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 500000000, 0, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 5000000, 0, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) #penalize performance
		    
			R = np.array([[10, 0, 0, 0, 0, 0, 0, 0],
		          [0, 10, 0, 0, 0, 0, 0, 0],
		          [0, 0, 10, 0, 0, 0, 0, 0],
		          [0, 0, 0, 10, 0, 0, 0, 0],
		          [0, 0, 0, 0, 10, 0, 0, 0],
		          [0, 0, 0, 0, 0, 10, 0, 0],
		          [0, 0, 0, 0, 0, 0, 10, 0],
		          [0, 0, 0, 0, 0, 0, 0, 10]]) #penalize effort

		return Q,R  

	#generate thrust forces and limiting them
	def generateThrust(self, K):
	    Error=np.zeros((10,1))
	    Error[0]=self.setpoint['zo']-self.actual['zo']
	    Error[1]=self.setpoint['phio']-self.actual['phio']
	    Error[2]=self.setpoint['thto']-self.actual['thto']
	    Error[3]=self.setpoint['epso']-self.actual['epso']
	    Error[4]=self.setpoint['uo']-self.actual['uo']
	    Error[5]=self.setpoint['vo']-self.actual['vo']
	    Error[6]=self.setpoint['wo']-self.actual['wo']
	    Error[7]=self.setpoint['po']-self.actual['po']
	    Error[8]=self.setpoint['qo']-self.actual['qo']
	    Error[9]=self.setpoint['ro']-self.actual['ro']
	    T=np.zeros((8,1))
	    T=K.dot(Error)
	    for i in range (0,8):
			if T[i]>30:
			    T[i] = 30
			elif T[i] <-30:
			    T[i] = -30
	    return T 

	#LQR control function 
	def controlFunction(self):

	    A,B = self.getAandB()
	    self.a_pub.publish(A)
	    self.b_pub.publish(B)

	    Q, R = self.getQandR()
	    
	    gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(A,B,Q,R)
	    
	    TForces = self.generateThrust(gain)
	    
	    T_pounds = TForces / 4.448
	    print(T_pounds)
	    
	    self.output = T_pounds
	    self.thrust_pub.publish(self.output)

	    print(self.output)

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start mission_planner node.')