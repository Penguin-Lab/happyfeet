#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import numpy as np
import time
import math

# Classes
class Robot:
    def __init__(self, name, NAV_A = 95.0, FPC_CONVERGENCERADIUS = 50.0, CFG_MAXSPEED = 200.0, CFG_MINBATERRY = 5.5, CFG_FORCEFIELD_RADIUS = 300, FPC_Kw = 0.3):
        # Parâmetros do robô
        self.NAV_A = NAV_A # offset [mm]
        self.FPC_CONVERGENCERADIUS = FPC_CONVERGENCERADIUS # Outside circle radius in mm for FPController
        self.CFG_MINBATERRY = CFG_MINBATERRY # Minimum Battery Voltage
        self.CFG_MAXSPEED = CFG_MAXSPEED # [mm/s]
        self.CFG_FORCEFIELD_RADIUS = CFG_FORCEFIELD_RADIUS # Minimum US distance [mm]
        self.FPC_Kw = FPC_Kw

        # não lembro para quê serve !! (tlz desvio de obstáculo)
        self.flag_obstacle = False

        self.name = name
        self.odometry = "/" + self.name + "/odometry"
        rospy.init_node(self.name, anonymous=True)
        rospy.Subscriber(self.odometry, String, self.get_odometry)
        self.velocities = "/" + self.name + "/velocities"
        self.publisher = rospy.Publisher(self.velocities, String, queue_size=1)
        self.id = -1
        self.x = 0.0
        self.y = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.heading = 0.0
        self.battery = 0.1
        self.gamma = 0.0
        self.betha = 0.0
        self.alfha = 0.0
        self.theta = 0.0
        self.kx = 1.0
        self.ky = 1.0
        self.kw = 1.0
        self.us = [0.0,0.0,0.0]     # LEFT, FRONT, RIGHT
        self.pose = [self.x, self.y, self.heading]
   
    def get_odometry(self, message):
        m_decode = message.data
        m_decode = m_decode.split(">")
        if len(m_decode)>1:
            m_decode = m_decode[0].split("<")
            if len(m_decode)>1 and m_decode[1].count(";")==7:
                self.id, self.x, self.y, self.heading, self.battery, self.us[2], self.us[1], self.us[0] = [float(k) for k in m_decode[1].split(";")]
                self.pose = [self.x, self.y, self.heading]
                self.us[0] = self.us[0] * 10.0
                self.us[1] = self.us[1] * 10.0
                self.us[2] = self.us[2] * 10.0
    
    def send_velocities(self, id_msg, v, w):
        if v > 999.99: 
            v = 999.99
        elif v < -999.99:
            v = -999.99
        if w > 999.99:
            w = 999.99
        elif w < -999.99:
            w = -999.99
        v_string = "+"+"{:.{}f}".format(v, 2).zfill(6) if v >= 0 else "-"+"{:.{}f}".format(abs(v), 2).zfill(6)
        w_string = "+"+"{:.{}f}".format(w, 2).zfill(6) if w >= 0 else "-"+"{:.{}f}".format(abs(w), 2).zfill(6)
        data_out = "<"+";".join([str(id_msg).zfill(5),v_string,w_string])+">"
        self.publisher.publish(data_out)

    def final_pos_off_controller(self, desired_point):###########################################################################################################
        a_pos_x = self.x + self.NAV_A*np.cos(self.heading)
        a_pos_y = self.y + self.NAV_A*np.sin(self.heading)
        delta_y = desired_point[1] - a_pos_y
        delta_x = desired_point[0] - a_pos_x
        self.theta = np.arctan2(delta_y, delta_x)
        Rho = np.sqrt(delta_y*delta_y + delta_x*delta_x)
        self.alpha = (self.theta - self.heading)    # Error angle between correct angle and atual angle
    
        # Update outputs:
        v = self.CFG_MAXSPEED*np.tanh((self.kx/self.CFG_MAXSPEED)*delta_x)*np.cos(self.heading) + self.CFG_MAXSPEED*np.tanh((self.ky/self.CFG_MAXSPEED)*delta_y)*np.sin(self.heading)
        w = -self.CFG_MAXSPEED*np.tanh((self.kw/self.CFG_MAXSPEED)*delta_x)*np.sin(self.heading)/self.NAV_A + self.CFG_MAXSPEED*np.tanh((self.kw/self.CFG_MAXSPEED)*delta_y)*np.cos(self.heading)/self.NAV_A
    
        # Check the convergence radius:
        if (Rho < self.FPC_CONVERGENCERADIUS):
            return 0.0,0.0,Rho
        else :
            return v, w, Rho
    
    def Calculate_virtual_point(self, x, y):###################################################################################################################
        a_pos_x = self.x + self.NAV_A*np.cos(self.heading)
        a_pos_y = self.y + self.NAV_A*np.sin(self.heading)
        delta_y = y - a_pos_y
        delta_x = x - a_pos_x
        self.theta = np.arctan2(delta_y, delta_x)
        self.alpha = (self.theta - self.heading)
    
        dist_US_Esquerdo = self.us[0]
        dist_US_Frente = self.us[1]
        dist_US_Direito = self.us[2]
    
        # Como o numero de fuga do ultrassom e negativo e necessário "truncar" o valor que deu problema
        if ( dist_US_Frente < 0.0 ):
            dist_US_Frente = self.CFG_FORCEFIELD_RADIUS
        if (dist_US_Direito < 0.0):
            dist_US_Direito = self.CFG_FORCEFIELD_RADIUS
        if (dist_US_Esquerdo < 0.0):
            dist_US_Esquerdo = self.CFG_FORCEFIELD_RADIUS
    
        if ( dist_US_Frente <= dist_US_Direito ) and ( dist_US_Frente <= dist_US_Esquerdo ) :
            self.betha = 0
            self.kx = 1.0
            self.ky = 0.8
            self.kw = 0.6
    
            self.flag_obstacle = True
    
            sign = 1
          
        elif ( dist_US_Direito <= dist_US_Frente ) and ( dist_US_Direito <= dist_US_Esquerdo ) :
            if ( self.flag_obstacle == True ) :
                self.kx = 1.0
                self.ky = 2.0
                self.kw = 0.6
            else :
                self.kx = 0.8
                self.ky = 0.8
                self.kw = 0.6
            self.betha = -0.6806784
            sign = -1  
          
        else : #if ( dist_US_Esquerdo <= dist_US_Frente ) and ( dist_US_Esquerdo <= dist_US_Direito ) :
            if ( self.flag_obstacle == True ) :
                self.kx = 1.0
                self.ky = 2.0
                self.kw = 0.6
            else :
                self.kx = 0.8
                self.ky = 0.8
                self.kw = 0.6
            self.betha = 0.6806784
            sign = 1

    
        print( "\nObstruction detected\nID = " + str(self.id) + "   LEFT = " + str(dist_US_Esquerdo) + "    FRONT = " + str(dist_US_Frente) + "  RIGHT = "+ str(dist_US_Direito) )
        # print("Betha = " + str(self.betha) )
        # print("Alpha = " + str(self.alpha) )
    
        # Calc rotation angle 
        self.gamma = (sign*(math.pi/2)) - (self.betha - self.alpha) # FABRICIO
    
        # print( "Gamma = " + str(self.gamma) )
    
        # Coordinates transformation
        #  ->  Robot referential
    
        #print( "Theta = " + str(self.theta) )
        #print("X = " + str(x) + "     Y = "+ str(y) + "\n" )
    
    
        robot_Xd = (np.cos(self.theta)*x) + (np.sin(self.theta)*y) - (np.cos(self.theta)*self.x)-(np.sin(self.theta)*self.y)
        robot_Yd = -(np.sin(self.theta)*x) + (np.cos(self.theta)*y) + (np.sin(self.theta)*self.x)-(np.cos(self.theta)*self.y)
    
        #  Calc Robot Virtual Destination
        #  Rotate by the angle 'gamma' calculated:
        robot_VirX = (np.cos(self.gamma)*robot_Xd) + (np.sin(self.gamma)*robot_Yd)
        robot_VirY = -(np.sin(self.gamma)*robot_Xd) + (np.cos(self.gamma)*robot_Yd)
    
        # Calc Virtual Target:
        #  Robot Reference -> World Reference
        virtual_x = np.cos(self.theta)*robot_VirX - np.sin(self.theta)*robot_VirY + self.x
        virtual_y = np.sin(self.theta)*robot_VirX + np.cos(self.theta)*robot_VirY + self.y
        return virtual_x, virtual_y

    def go_to(self, x, y):###############################################################################################################################
        id_msg = 0
        
    
        self.flag_obstacle = False
        last_v = 0.0
        last_w = 0.0
        Rho = self.FPC_CONVERGENCERADIUS + 1
        
        time.sleep(2)
        
        while ((Rho >= self.FPC_CONVERGENCERADIUS) or (last_v != 0.0) or (last_w != 0.0) or (self.id != id_msg)):
            if self.battery < self.CFG_MINBATERRY:
                v = 0.0
                w = 0.0
            else :
                # calculos
                if (( 0 < self.us[0] < self.CFG_FORCEFIELD_RADIUS ) or ( 0 < self.us[1] < self.CFG_FORCEFIELD_RADIUS ) or ( 0 < self.us[2] < self.CFG_FORCEFIELD_RADIUS )) :
                    virtual_x ,virtual_y = self.Calculate_virtual_point(x, y)
                    #print ("KX = " + str(self.kx) + "  KY = " + str(self.ky) + "  KW = " + str(self.kw))
                    v, w, Rho = self.final_pos_off_controller([virtual_x, virtual_y])
                else :
                    self.kx = 1.0
                    self.ky = 1.0
                    self.kw = 1.0
                    v, w, Rho = self.final_pos_off_controller([x,y])
            if (self.id == id_msg):
                id_msg = id_msg + 1
                if id_msg > 99999 or id_msg < 0:
                    id_msg = 0
                last_v = v
                last_w = w
            self.send_velocities(id_msg, v, w)
            time.sleep(0.05)
        return False
    
    def go(self):########################################################################################################################################
        print("\nYou typed go_to.\n")
        xd = input("Xd: ")
        if (xd==""):
            xd = 1000.0
            print(xd)
        else:
            xd = float(xd)
        yd = input("Yd: ")
        if (yd ==""):
            yd = 0.0
            print(yd)
        else:
            yd = float(yd)
        self.go_to(xd, yd)
        print("------------------------------------------------------")
        return False
     
    def save_home(self):#################################################################################################################################
        self.home_x = self.x
        self.home_y = self.y
        print("Save " + str(self.home_x) + " as X and " + str(self.home_y) + " as Y")
        print("------------------------------------------------------")
        return False
    
    def go_home(self):###################################################################################################################################
        self.go_to(self.home_x,self.home_y)    
        print("------------------------------------------------------")
        return False
    
    def sair(self):######################################################################################################################################
        return True