#http://www.steves-internet-guide.com/into-mqtt-python-client/
import paho.mqtt.client as mqtt #import the client1
import paho.mqtt.subscribe as subscribe #import the client1
import numpy as np
import time

# Mqtt:
#12344321
last_id_msg = -1
def on_connect(client, userdata, flags, rc):
    client.subscribe("happy/odometry")

def on_message(client, userdata, message):
    #print(message.topic + " - " + str(message.payload))
    if (message.topic == "happy/odometry"):
        get_odometry(message)
    
broker_address = "localhost"#"test.mosquitto.org"#"localhost"#"192.168.1.184" 
broker_port = 1883
client = mqtt.Client("controller") #create new instance
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker_address,broker_port) #connect to broker

# Robot:
nav_pos_x = 0.0
nav_pos_y = 0.0
nav_heading = 0.0
battery = 0.1

def get_us(broker_address):
    msg = subscribe.simple("happy/us",hostname=broker_address)
    m_decode = str(msg.payload.decode("utf-8","ignore"))
    print(m_decode)#<<id;s1;s2;s3;battery>>
    if (len(m_decode.split("<<")) > 1):
        m_decode = m_decode.split("<<")[1]
        if (len(m_decode.split(">>")) > 1):
            m_decode = m_decode.split(">>")[0]
            if (len(m_decode.split(";")) > 1):
                m_decode = m_decode.split(";")
                return int(m_decode[0]), float(m_decode[1]), float(m_decode[2]), float(m_decode[3])
    else:
        return 0, -1.0, -1.0, -1.0
    
def get_odometry(message):
    global last_id_msg
    global nav_pos_x
    global nav_pos_y
    global nav_heading
    global battery
    m_decode = str(message.payload.decode("utf-8","ignore"))
    print(m_decode)#<<id;x;y;heading;battery>>
    m_decode = m_decode.split(">")
    if len(m_decode)>1:
        m_decode = m_decode[0].split("<")
        if len(m_decode)>1 and m_decode[1].count(";")==4:
            new_id_msg,x,y,heading,bat = [float(k) for k in m_decode[1].split(";")]
            if int(new_id_msg) > last_id_msg:
                last_id_msg = int(new_id_msg)
                nav_pos_x = x
                nav_pos_y = y
                nav_heading = heading
                battery = bat

id_msg = 0
def send_velocities(id_msg,v,w,broker_address):
    data_out = "<"+";".join([str(id_msg),"{:.{}f}".format(v, 2),"{:.{}f}".format(w, 2)])+">"
    id_msg = id_msg + 1
    client.publish("happy/velocities",data_out,retain=True)#publish
    return id_msg

# Controller:
NAV_A = 67.0/2.0 # mm
FPC_CONVERGENCERADIUS = 50.0 # Outside circle radius in mm for FPController
CFG_MINBATERRY = 5.5 # Minimum Battery Voltage
CFG_MAXSPEED = 250.0 # [mm/s]
FPC_Kw = 0.3

# Final Position Controller
def FPController(fpos_x,fpos_y,nav_pos_x,nav_pos_y,nav_heading):
    delta_y = fpos_y - nav_pos_y
    delta_x = fpos_x - nav_pos_x
    Rho = np.sqrt(delta_y*delta_y + delta_x*delta_x)
    theta = np.arctan2(delta_y, delta_x)
    alpha = (theta - nav_heading)    # Error angle between correct angle and atual angle

    # Update outputs:
    global CFG_MAXSPEED
    global FPC_Kw
    global FPC_CONVERGENCERADIUS
    new_v = CFG_MAXSPEED*np.tanh(Rho)*np.cos(alpha)
    new_w = FPC_Kw*alpha + new_v*np.sin(alpha)/Rho
  
    # Check the convergence radius:
    if (Rho < FPC_CONVERGENCERADIUS):
        return 0.0,0.0,Rho
    else:
        return new_v,new_w,Rho

# Final Position Controller with Offset
def FPControllerOff(fpos_x,fpos_y,nav_pos_x,nav_pos_y,nav_heading):
    global NAV_A
    global CFG_MAXSPEED
    global FPC_CONVERGENCERADIUS
    a_pos_x = nav_pos_x + NAV_A*np.cos(nav_heading)
    a_pos_y = nav_pos_y + NAV_A*np.sin(nav_heading)
    delta_y = fpos_y - a_pos_y
    delta_x = fpos_x - a_pos_x
    Rho = np.sqrt(delta_y*delta_y + delta_x*delta_x)
    theta = np.arctan2(delta_y, delta_x)
    alpha = (theta - nav_heading)    # Error angle between correct angle and atual angle

    # Update outputs:
    kx = 1.0
    ky = kx
    new_v = CFG_MAXSPEED*np.tanh((kx/CFG_MAXSPEED)*delta_x)*np.cos(nav_heading) + CFG_MAXSPEED*np.tanh((ky/CFG_MAXSPEED)*delta_y)*np.sin(nav_heading)
    new_w = -CFG_MAXSPEED*np.tanh((kx/CFG_MAXSPEED)*delta_x)*np.sin(nav_heading)/NAV_A + CFG_MAXSPEED*np.tanh((ky/CFG_MAXSPEED)*delta_y)*np.cos(nav_heading)/NAV_A
  
    # Check the convergence radius:
    if (Rho < FPC_CONVERGENCERADIUS):
        return 0.0,0.0,Rho
    else:
        return new_v,new_w,Rho

# Main:
print("--------- Starting final position controller ---------")
print("Xd: ")
xd = float(input())
#xd = 1000.0
print("Yd: ")
yd = float(input())
#yd = 1000.0
print("Desired point: (",xd,",",yd,")")
print("------------------------------------------------------")
#client.publish("happy/odometry","bazinga",retain=True)#--------------------
client.loop_start()
time.sleep(2)


Rho = FPC_CONVERGENCERADIUS + 1
while (Rho >= FPC_CONVERGENCERADIUS):#Rho >= FPC_CONVERGENCERADIUS)
    #time.sleep(0.05)
    print("Odometry: " + "x: " + str(nav_pos_x)+" y: "+str(nav_pos_y)+" angle: "+str(nav_heading)+" bat: "+str(battery))
    if battery == 0.0:
        v = 0.0
        w = 0.0
    else:
        v,w,Rho = FPControllerOff(xd,yd,nav_pos_x,nav_pos_y,nav_heading)
        # v,w,Rho = FPControllerOff(xd,yd,-91.6721,-98.22169,2.062927)
        id_msg = send_velocities(id_msg,v,w,broker_address)
        #print("Velocities: " + "v: " + str(v) + " w: " + str(w))
        time.sleep(0.05)

id_msg = send_velocities(id_msg,0.0,0.0,broker_address)
client.loop_stop()
