#!/usr/bin/env python

import numpy as np
from numpy import pi
from math import sin, cos
import rospy 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from image_proc.msg import FourierCoefsMsg
import matplotlib.pyplot as plt
import pandas as pd

# ~~ Setting and Constants ~~
numReadings   = 580
threshold     =   9.0
preturn_thresh = 5.0
plotCarts     =   1

# ~~ Variables ~~
global lastDepthScan
global lastNearnessScan
global a_0, a_1, a_2, b_1, b_2
global forward_speed, yaw_rate
lastDepthScan = [ 0 for i in range( numReadings ) ]
lastNearnessScan = [ 0 for i in range( numReadings ) ]
car_state = ''
len_hist = 1000
a_0_history = [0]*len_hist
a_1_history = [0]*len_hist
a_2_history = [0]*len_hist
b_1_history = [0]*len_hist
b_2_history = [0]*len_hist

forward_speed_history = [0]*len_hist
yaw_rate_history = [0]*len_hist

def scan_cb( msg ):
    global lastDepthScan
    """ Process the scan that comes back from the scanner """
    lastDepthScan = msg.data  #lastDepthScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def nearness_cb( msg ):
    global lastNearnessScan
    """ Process the scan that comes back from the scanner """
    lastNearnessScan = msg.data  #lastDepthScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]
    
def wfi_fourier_coeffs_cb(msg):
    global a_0, a_1, a_2, b_1, b_2
    """ Process the scan that comes back from the scanner """
    a_0 = msg.a_0
    a_1 = msg.a[0]
    a_2 = msg.a[1]
    b_1 = msg.b[0]
    b_2 = msg.b[1]
    
    a_0_history.pop(0)
    a_1_history.pop(0)
    a_2_history.pop(0)
    b_1_history.pop(0)
    b_2_history.pop(0)
    
    a_0_history.append(a_0)
    a_1_history.append(a_1)
    a_2_history.append(a_2)
    b_1_history.append(b_1)
    b_2_history.append(b_2)
	
def wfi_control_command_cb(msg):
    global forward_speed, yaw_rate
    """ Process the scan that comes back from the scanner """
    forward_speed = msg.linear.x
    yaw_rate = msg.angular.z

    forward_speed_history.pop(0)
    yaw_rate_history.pop(0)

    forward_speed_history.append(forward_speed)
    yaw_rate_history.append(yaw_rate)

def polr_2_cart_0Y( polarCoords ): # 0 angle is +Y North 
    """ Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ """
    return [ polarCoords[0] * sin( polarCoords[1] ) , polarCoords[0] * cos(polarCoords[1])  ]

minAng = -pi/4.0
maxAng = minAng + pi/2.0

def cart_scan( arr ):
    """ Represent the points in cartesian coordinates """
    arrLen = len( arr )
    angles = np.linspace( minAng , maxAng , arrLen )
    return [ polr_2_cart_0Y(  [ arr[i] , angles[i] ] ) for i in xrange( arrLen ) ]

rospy.init_node( 'scan_plot' , anonymous = True )
	
rospy.Subscriber( "camera/wfi/horiz/scan" , Float32MultiArray , scan_cb )
rospy.Subscriber( "camera/wfi/horiz/nearness" , Float32MultiArray , nearness_cb )
rospy.Subscriber( "camera/wfi/horiz/fourier_coefficients" , FourierCoefsMsg , wfi_fourier_coeffs_cb )
rospy.Subscriber( "camera/wfi/control_commands" , Twist , wfi_control_command_cb )

try:
	while ( not rospy.is_shutdown() ):
		lastDepthScanNP   = np.asarray( lastDepthScan )
		lastNearnessScanNP   = np.asarray( lastNearnessScan )

		plt.clf() # Clear all figures

		plt.figure(1)
		plt.subplot(2,3,1)
		# Figure 1, Horizontal Depth Scan: Array Index vs. Depth [m]
		# plt.figure(num=1, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')	
		plt.plot( lastDepthScanNP , 'b.' )
		plt.hold( False )
		plt.xlim( [ 0 , 580 ] )
		plt.ylim( [ 0 , 10 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Depth [m]")
		plt.title("Horiz. Depth (Array Values)")

		plt.subplot(2,3,2)
		# Figure 2, Horizontal Depth Scan: X [m] vs Y [m]
		# plt.figure(num=2, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')	
		points = cart_scan( lastDepthScanNP )
		X = [ elem[0] for elem in points ]
		Y = [ elem[1] for elem in points ]		    
		plt.scatter( X , Y ,c="k",marker=".")
		plt.axis( 'equal' )
		plt.grid( True )
		plt.xlim( [ -8 , 8 ] )
		plt.ylim( [   0 , 10 ] )
		plt.xlabel("X [m]")
		plt.ylabel("Y [m]")
		plt.title("Horiz. Depth (X-Y Coordinates)")
		
		plt.subplot(2,3,4)
		# Figure 3, Horizontal Nearness: Array Index vs. Nearness [1/m]
		# plt.figure(num=3, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')	
		plt.plot( lastNearnessScanNP , 'b.' )
		plt.hold( False )
		plt.xlim( [ 0 , 580 ] )
		plt.ylim( [ 0 , 2 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Nearness [1/m]")
		plt.title("Horiz. Nearness (Array Values)")

		plt.subplot(2,3,5)
		# Figure 2, Horizontal Nearness Scan: X [1/m] vs Y [1/m]
		# plt.figure(num=4, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')	
		points = cart_scan( lastNearnessScanNP )
		X = [ elem[0] for elem in points ]
		Y = [ elem[1] for elem in points ]		    
		plt.scatter( X , Y ,c="k",marker=".")
		plt.axis( 'equal' )
		plt.grid( True )
		plt.xlim( [ -8 , 8 ] )
		plt.ylim( [   0 , 2 ] )
		plt.xlabel("X [1/m]")
		plt.ylabel("Y [1/m]")
		plt.title("Horizontal Nearness Scan: X [m] vs Y [m]")

		# plt.figure(3)
		# points = cart_scan( lastNearnessScanNP )
		# X = [ elem[0] for elem in points ]
		# Y = [ elem[1] for elem in points ]		    
		# plt.scatter( X , Y )
		# plt.axis( 'equal' )
		# plt.title( "Occlusion Cart:\n" + str( car_state ) )
		    
		plt.figure(2)
		plt.subplot(7,1,1)
                plt.plot(a_0_history,'k')
		plt.hold(True)
		# plt.ylim([-5, 5])
		plt.xlim([0, len_hist])
		plt.ylabel('a_0')
		plt.text(len_hist, 0, '%.2f' % (a_0),bbox=dict(facecolor='red', alpha=0.5))

		plt.subplot(7,1,2)
		plt.plot(a_1_history,'k')
		plt.hold(True)
		# plt.ylim([-5, 5])
		plt.xlim([0, len_hist])
		plt.ylabel('a_1')
                plt.text(len_hist, 0, '%.2f' % (a_1),bbox=dict(facecolor='red', alpha=0.5))

                plt.subplot(7,1,3)
		plt.plot(a_2_history,'k')
		plt.hold(True)
		# plt.ylim([-5, 5])
		plt.xlim([0, len_hist])
		plt.ylabel('a_2')
                plt.text(len_hist, 0, '%.2f' % (a_2),bbox=dict(facecolor='red', alpha=0.5))

                plt.subplot(7,1,4)
		plt.plot(b_1_history,'k')
		plt.hold(True)
		# plt.ylim([-5, 5])
		plt.xlim([0, len_hist])
		plt.ylabel('b_1')
                plt.text(len_hist, 0, '%.2f' % (b_1),bbox=dict(facecolor='red', alpha=0.5))

		plt.subplot(7,1,5)
		plt.plot(b_2_history,'k')
		plt.hold(True)
		# plt.ylim([-5, 5])
		plt.xlim([0, len_hist])
		plt.ylabel('b_2')
                plt.text(len_hist, 0, '%.2f' % (b_2),bbox=dict(facecolor='red', alpha=0.5))

		plt.subplot(7,1,6)
		plt.plot(forward_speed_history,'k')
		plt.hold(True)
		# plt.ylim([-5, 5])
		plt.xlim([0, len_hist])
		plt.ylabel('forward_speed')
                plt.text(len_hist, 0, '%.2f' % (forward_speed),bbox=dict(facecolor='red', alpha=0.5))

                plt.subplot(7,1,7)
                plt.plot(yaw_rate_history,'k')
                plt.hold(True)
                #plt.ylim([-5, 5])
                #plt.xlim([0, len_hist])
                plt.ylabel('yaw_rate')
                plt.text(len_hist, 0, '%.2f' % (yaw_rate),bbox=dict(facecolor='red', alpha=0.5))

                dc = pd.DataFrame({'A' : [1, 2, 3, 4],'B' : [4, 3, 2, 1],'C' : [3, 4, 2, 2]})

                # plt.plot(dc)
                # plt.legend(dc.columns)
                # dcsummary = pd.DataFrame([dc.mean(), dc.sum()],index=['Mean','Total'])

                # plt.table(cellText=dcsummary.values,colWidths = [0.25]*len(dc.columns),
                # rowLabels=dcsummary.index,
                # colLabels=dcsummary.columns,
                # cellLoc = 'center', rowLoc = 'center',
                # loc='bottom', bbox=[0.25, -0.5, 0.5, 0.3])

		plt.pause( 0.001 )
	plt.show()
except KeyboardInterrupt:
	pass

