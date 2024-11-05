from pymavlink import mavutil
import time
import threading

from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.exceptions import PubNubException


COORDINATE_CHANGE=1000

SEARCHED=[]  #Store co-ordinates of targets found



#Delay is a decorator function that will not allow the function to be called for a specific amount of time(s)
def delay(seconds):
    def decorator(func):
        last_execution_time = 0

        def wrapper(*args, **kwargs):
            nonlocal last_execution_time
            current_time = time.time()

            if current_time - last_execution_time >= seconds:
                result = func(*args, **kwargs)
                last_execution_time = current_time
                return result
            else:
                print(f"Function '{func.__name__}' is on cooldown. Try again later.")

        return wrapper

    return decorator

#Check if the vehicle is ARMED     
def isArmed():
    msg=the_connection.recv_match(type="HEARTBEAT",blocking=True)
    if msg.base_mode!=None:
        if the_connection.motors_armed()==128:
            return 1
        else:
            return 0
        
#Returns the current latitude, longitude and the altitude of the vehicle
def Get_Attitude():   
	msg=the_connection.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
	return [msg.lat,msg.lon,msg.relative_alt//1000]

#Stall the program till the vehicle reaches the required altitude
def check_Altitude(alt):
     while True:
        print("Checking")        
        altitude=Get_Attitude()[2]
        if altitude>alt-1 and altitude<alt+1:
            print("In range")
            break    

#Move the vehicle in any direction
@delay(1)
def send_movement_command_XYZ(dist_x, dist_y, altitude):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system,the_connection.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
        1479, #ignore positon z and other pos arguments
        0, 0, 0, 
        dist_x, dist_y, altitude,        
        0, 0, 0, 
        0, 0))
def go_back(dist_x,dist_y,altitude):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system,the_connection.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
        1528, #ignore velocity z and other pos arguments
        dist_x, dist_y, altitude,
        0, 0, 0, 
        0, 0, 0, 
        0, 0))


#Function to receive Battery and global messages
def messages():
    the_connection.mav.command_long_send(
        the_connection.target_system,  # Target system ID
        the_connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # param1: Message ID to be streamed
        1000000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )
    response = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")
        
    the_connection.mav.command_long_send(
        the_connection.target_system,  # Target system ID
        the_connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,  # param1: Message ID to be streamed
        1000000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )
    
    response = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")
        
 
 #Change the mode to guided and got to 30m and takeoff to 30m   

def start():
    mode_id = the_connection.mode_mapping()['GUIDED']

    the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,1,0,0,0,0,0,0)

    

    while True:
        if isArmed()==1:
            break
        else:
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,1,0,0,0,0,0,0)
                              
    # the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF	, 0,0,0,0,0,0,0,30)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0,0,0,0,0,0,0,0)

#Change flight mode
def change_mode(mode):
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

#Change the altitude of the vehicle
def altitude(alt):
    attitude=Get_Attitude()
    lat,lon=attitude[0],attitude[1]
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10,the_connection.target_system,the_connection.target_component,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        3576,lat,lon,alt,0,0,0,0,0,0,0,0))
    
    check_Altitude(alt)

#Check if we have already visited the target
def if_searched():
    attitude=Get_Attitude()
    lat,long=attitude[0],attitude[1]
    searched=False
    # print(SEARCHED)
    for i in SEARCHED:
        if lat<=i[0]+COORDINATE_CHANGE and lat>=i[0]-COORDINATE_CHANGE and long<=i[1]+COORDINATE_CHANGE and long>=i[1]-COORDINATE_CHANGE:
            searched=True            
            print("Yes , searched")
            return searched

    return searched

def searched_coordinates(coords):
     SEARCHED.append(coords)


def Telemetry():
    messages()
    pnChannel = "raspi-tracker"
    pnconfig = PNConfiguration()
    pnconfig.subscribe_key = "sub-c-f3cca264-1519-4b08-b2e8-532ac5c43adc"
    pnconfig.publish_key = "pub-c-f27f9b1b-188a-4dfa-a409-1b151deba4f6"
    pnconfig.uuid="ac95452a-63ff-11ee-8c99-0242ac120002"
    pnconfig.ssl = False
    pubnub = PubNub(pnconfig)
    pubnub.subscribe().channels(pnChannel).execute()
    
    while True:
        battery= the_connection.recv_match(type='BATTERY_STATUS', blocking=True) 
        position=the_connection.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
        msg=the_connection.recv_match(type="HEARTBEAT",blocking=True)
        if msg.base_mode!=None:
            if the_connection.motors_armed()==128:
                arm="ARMED"
            else:
                arm="DISARMED"       
        
                
        lat=position.lat/10000000
        long=position.lon/10000000
        alt=position.relative_alt//1000
        volt=battery.voltages[0]/1000
        amp=battery.current_battery/100
        # print(lat,long,arm)
        try:
            envelope = pubnub.publish().channel(pnChannel).message({
            'lat':lat,
            'long':long,
            'alt':alt,
            'volt':volt,
            'amp':amp,
            'arm':arm            
            }).sync()
            print("publish timetoken: %d" % envelope.result.timetoken)
        except PubNubException as e:
            print(e)

def servo(value):
    the_connection.mav.command_long_send(
        the_connection.target_system,  # Target system ID
        the_connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # ID of command to send
        0,  # Confirmation
        8,  # param1: Servo instance number
        value, # param2: PWM
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )
    
        
def establish_connection():
    try:
        the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
        # the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
        return the_connection
    except Exception as e:
        print(f"Failed to connect: {str(e)}")
        return None


# Create and start a thread to maintain the connection
the_connection = establish_connection()
# Function to continuously maintain the connection and handle reconnection
def maintain_connection():
    while True:
        if the_connection is None or not the_connection.master.fd:
            print("Disconnected. Attempting to reconnect...")
            the_connection = establish_connection()
            if the_connection:
                print("Reconnected successfully")
                change_mode('AUTO')
                # You may want to reinitialize any state or threads that depend on the connection here
            else:
                time.sleep(5)  # Wait for a few seconds before attempting to reconnect again

if the_connection:
    messages()
    servo(1000)
    while True:
        ready=input("Is package secured?")
        if ready.lower()=="yes":
            servo(2000)
            break
    while True:
        ready=input("Ready to fly?")
        if ready.lower()=="yes":
            break
    # start()    

else:
    print("Failed to establish the initial connection.")
    

    
    




    

    


    

