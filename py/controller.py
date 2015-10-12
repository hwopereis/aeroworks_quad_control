#!/usr/bin/env python

"""
This Python executable is part of the ram ROS package. Its function is to handle the position control of a single quadcopter. The application is designed to work under Ubuntu Linux (and just that).
"""

# System imports
import sys
import time
from gi.repository import Gtk, cairo, Pango, PangoCairo, GObject, Gdk   # Interface design using Glade
import threading
import os
import xmlrpclib
import socket
import subprocess
import math

# ROS related imports
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf.transformations
import rospkg
from aeroquad.msg import nonlinearity #TJARK
from aeroquad.msg import gains #TJARK

# The RosConnector class is responsible for all communication with the ROS infrastructure. All nodes and subscribers, including callback, can be found in this class.
class RosConnector:
    def __init__(self, prefix):
        print "Prefix: " + prefix

        # These subscribers are self-explanatory: they send basic commands to the drone or controller
        self.pubSetpoint = rospy.Publisher("/"+prefix+"/setpoint", geometry_msgs.msg.Pose,queue_size=1)
        self.pubMocap = rospy.Publisher("/mavros/mocap/pose", geometry_msgs.msg.PoseStamped,queue_size=1)
        self.pubNonLinear = rospy.Publisher("/"+prefix+"/nonlinearity", nonlinearity,queue_size=1)
        self.pubGains = rospy.Publisher("/"+prefix+"/gains", gains,queue_size=1)
                         
        # Since we are able to change the source of the commands (controller or joystick), the connector listens for these messages and republishes the one that is currently configured to be the output.
        self.subJoystick = rospy.Subscriber("/"+prefix+"/cmd_vel_joy", geometry_msgs.msg.TwistStamped, self.joyCB)     
        self.subController = rospy.Subscriber("/"+prefix+"/cmd_vel_controller", geometry_msgs.msg.TwistStamped, self.controllerCB)
        self.subMocap = rospy.Subscriber("/"+prefix+"/unfiltered_pose",geometry_msgs.msg.PoseStamped,self.mocapCB)
        
        # Aeroquad publishers go to different locations
        self.pubCmd = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", geometry_msgs.msg.TwistStamped, queue_size=1)
        
        self.pubTakeoff = rospy.Publisher("/"+prefix+"/ardrone/takeoff", std_msgs.msg.Empty, queue_size=1)
        self.pubLand = rospy.Publisher("/"+prefix+"/ardrone/land", std_msgs.msg.Empty, queue_size=1)
        self.pubReset = rospy.Publisher("/"+prefix+"/ardrone/reset", std_msgs.msg.Empty, queue_size=1)            

        # For live setpoint updates in the user interface, listen for setpoints as well.
        self.subSetpoint = rospy.Subscriber("/"+prefix+"/setpoint", geometry_msgs.msg.Pose, self.setpointCB)
        self.setpoint = geometry_msgs.msg.Pose()
        self.pose = geometry_msgs.msg.PoseStamped()
        self.yaw = 0
        # Run the actual controller (C++ executable)
        self.pr = subprocess.Popen(["roslaunch","aeroquad", "controller_module.launch", "prefix:="+prefix])
        
    def mocapCB(self,msg):
        self.pose = msg
        self.pubMocap.publish(msg)

    def joyCB(self, msg):
        # If the joystick checkbox is ticked, publish the cmds from the cmd_vel_joy to cmd_vel
        global publishJoystick
        global publishSetpoint
        if publishJoystick and publishSetpoint:
            msg.twist.linear.x = msg.twist.linear.x
            msg.twist.linear.y = msg.twist.linear.y
            msg.twist.linear.z = msg.twist.linear.z            
            self.pubCmd.publish(msg)
    
    def algorithmCB(self,msg):
        # If the algorithm checkbox is ticked, receive the position input and send it to the position controller.
        global publishAlgorithm
        global publishSetpoint
        if publishAlgorithm and publishSetpoint:
            self.setSetpoint(msg.position.x,msg.position.y,msg.position.z,0)
            

    def setpointCB(self, msg):
        # As a setpoint is received, change the member of this class.
        # Since this has to do with the UI, enter threads first.
        Gdk.threads_enter()
        self.setpoint = msg
        Gdk.threads_leave()

            
    def clean(self):
        # On closing the application, terminate the running C++ controller.
        print "Cleaning!"
        try:
            self.pr.terminate()
        except (OSError, AttributeError):
            # can't kill a dead/non existent proc
            pass

    def takeOff(self):
        # Send take-off message
        msg = std_msgs.msg.Empty()
        print "Takeoff"
        self.pubTakeoff.publish(msg)
        
        
    def controllerCB(self, msg):
        # If a command from the controller is received, republish to cmd_vel, unless publish zero, publish joystick or publish nothing are ticked.
        global publishInterface
        global publishSetpoint
        global publishAlgorithm
        if (publishInterface or publishAlgorithm) and not publishSetpoint:
            # Publish an empty message (referring to "enable hovermode")
            self.pubCmd.publish(geometry_msgs.msg.TwistStamped())        
        if (publishInterface or publishAlgorithm) and publishSetpoint:
            # Publish controller output
            ##msg.twist.angular.z = self.yaw/360*2*math.pi

            self.pubCmd.publish(msg)

    def land(self):
        # Send land message
        msg = std_msgs.msg.Empty()
        self.pubLand.publish(msg)

    def reset(self):
        # Send reset message
        msg = std_msgs.msg.Empty()
        self.pubReset.publish(msg)

    def setSetpoint(self, x, y, z, yaw):
        # Function to set the setpoint of the drone. 
        msg = geometry_msgs.msg.Pose()
        msg.position.x = x
        msg.position.y = y;
        msg.position.z = z;

        self.yaw = yaw

        # Transformation from yaw to quaternion. All other attitudes are not important.
        quaternion = tf.transformations.quaternion_from_euler(0, 0, (yaw*2*math.pi)/(360))
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.pubSetpoint.publish(msg)


    def setNonLinear(self, x, y, z, xoff, yoff):
        # Set the nonlinearity of the controller, by sending a nonlinearity() message.
        msg = nonlinearity()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.xoff = xoff
        msg.yoff = yoff
        self.pubNonLinear.publish(msg)

    def setGains(self, p_z, d_z, p_rot, d_rot, p_trans, d_trans, i_action, v_damping, v_limit, i_enabled, v_enabled):
        # Set the gains, by sending a gains() message.
        msg = gains()
        msg.p_z = p_z
        msg.d_z = d_z
        msg.p_rot = p_rot
        msg.d_rot = d_rot
        msg.p_trans = p_trans
        msg.d_trans = d_trans
        msg.i_action = i_action
        msg.v_damping = v_damping
        msg.v_limit = v_limit
        msg.i_enabled = i_enabled
        msg.v_enabled = v_enabled
        self.pubGains.publish(msg)
# End RosConnector

def btnClose(widget, event):
    # This function is executed when the close button of the window is pressed.

    # Close the joy node.
    try:
        joyThread.poll()
        if(joyThread.returncode == None):
            joyThread.terminate()
    except (NameError, AttributeError):
        pass

    # Quit and return to terminal.
    Gtk.main_quit(widget, event)

def btnTakeOff(b):
    # Take-off button listener
    pass
        
def btnLand(b):
    # Land button listener
    pass
    
def btnReset(b):
    # Reset button listener
    pass


def radioInput(b):
    global publishJoystick
    global publishAlgorithm
    global publishInterface
    global publishSetpoint
    
    buttonLabel = b.get_label()
    
    if buttonLabel == "Joystick":
        publishJoystick = b.get_active()     
    elif buttonLabel == "A-star":
        publishAlgorithm = b.get_active()
    elif buttonLabel == "Interface":
        publishInterface = b.get_active()
        if publishSetpoint:
            setSetpoint()
    else:
        print "Radio input label does not match the options."
    


def boxSetpoint(box):
    # Called if the box for the setpoint publishing is ticked or unticked.
    global publishSetpoint
    global publishJoystick
    global publishAlgorithm
    publishSetpoint = box.get_active()
    if publishSetpoint and not publishJoystick and not publishAlgorithm:
        setSetpoint()

#def boxZero(box):
#    # Called if the box for the hover publishing is ticked or unticked.
#    global publishZero
#    publishZero = box.get_active()

#def boxNothing(box):
#    # Called if the box for the "nothing" publishing is ticked or unticked.
#    global publishNothing
#    publishNothing = box.get_active()

# Update setpoint whenever the scaleStores change value.
def scaleX(scale):
    setSetpoint()

def scaleY(scale):
    setSetpoint()

def scaleZ(scale):
    setSetpoint()

    
def scaleYaw(scale):
    setSetpoint()
    
def setSetpoint():
    # Get setpoint from sliders and send to controller IF we want to do that directly
    global publishInterface 
    global publishSetpoint

    if publishInterface and publishSetpoint:  
        setx = builder.get_object("scalestoreX").get_value()
        sety = builder.get_object("scalestoreY").get_value()
        setz = builder.get_object("scalestoreZ").get_value()
        setyaw = builder.get_object("scalestoreYaw").get_value()
        ros.setSetpoint(setx, sety, setz, setyaw)

def btnSetNonLinear(btn):
    # Read nonlinearity from the sliders, send to ROS connector
    x = builder.get_object("scalestoreNonLinX").get_value()
    y = builder.get_object("scalestoreNonLinY").get_value()
    z = builder.get_object("scalestoreNonLinZ").get_value()
    offx = builder.get_object("scalestoreOffsetX").get_value()
    offy = builder.get_object("scalestoreOffsetY").get_value()
    ros.setNonLinear(x,y,z,offx,offy)

def btnGains(btn):
    # Read gains from sliders, send to ROS connector
    p_z = builder.get_object("scalestoreGainPZ").get_value()
    d_z = builder.get_object("scalestoreGainDZ").get_value()
    p_rot = builder.get_object("scalestoreGainPRot").get_value()
    d_rot = builder.get_object("scalestoreGainDRot").get_value()
    p_trans = builder.get_object("scalestoreGainPTrans").get_value()
    d_trans = builder.get_object("scalestoreGainDTrans").get_value()
    i_action = builder.get_object("scalestoreGainI").get_value()
    v_damping = builder.get_object("scalestoreGainVel").get_value()
    i_enabled = builder.get_object("boxIAction").get_active()
    v_enabled = builder.get_object("boxVelDamping").get_active()
    v_limit = builder.get_object("scalestoreLimitVel").get_value()
    ros.setGains(p_z, d_z, p_rot, d_rot, p_trans, d_trans, i_action, v_damping, v_limit, i_enabled, v_enabled)

#def toggleJoystickCheckbox():
#    # Toggle if joystick control should be enabled or disabled.
#    currentstate = builder.get_object("boxJoystick").get_active()
#    if currentstate:
#        builder.get_object("boxJoystick").set_active(False)
#    else:
#        builder.get_object("boxJoystick").set_active(True)

def draw(w, d):
    pass



    
    
def readSetpointFromConnector():
    # If the live update box is ticked, update the sliders from the current ros setpoint in the Connector. This function is executed every 250ms.
    if builder.get_object("boxLive").get_active():
        
        if ros.setpoint.position.x > builder.get_object("scalestoreX").get_upper():
            builder.get_object("scalestoreX").set_upper(ros.setpoint.position.x)
        if ros.setpoint.position.x < builder.get_object("scalestoreX").get_lower():
            builder.get_object("scalestoreX").set_lower(ros.setpoint.position.x)            

        builder.get_object("scaleX").set_value(ros.setpoint.position.x)

        if ros.setpoint.position.y > builder.get_object("scalestoreY").get_upper():
            builder.get_object("scalestoreY").set_upper(ros.setpoint.position.y)
        if ros.setpoint.position.y < builder.get_object("scalestoreY").get_lower():
            builder.get_object("scalestoreY").set_lower(ros.setpoint.position.y)           

        builder.get_object("scaleY").set_value(ros.setpoint.position.y)
        
        if ros.setpoint.position.z > builder.get_object("scalestoreZ").get_upper():
            builder.get_object("scalestoreZ").set_upper(ros.setpoint.position.z)
        if ros.setpoint.position.z < builder.get_object("scalestoreZ").get_lower():
            builder.get_object("scalestoreZ").set_lower(ros.setpoint.position.z)      
        
        builder.get_object("scaleZ").set_value(ros.setpoint.position.z)
        
        euler = tf.transformations.euler_from_quaternion([ros.setpoint.orientation.x, ros.setpoint.orientation.y, ros.setpoint.orientation.z, ros.setpoint.orientation.w])
        builder.get_object("scaleYaw").set_value(euler[2])
    
    elif builder.get_object("boxGetCurrent").get_active():
        
        
        if ros.pose.pose.position.x > builder.get_object("scalestoreX").get_upper():
            builder.get_object("scalestoreX").set_upper(ros.pose.pose.position.x)
        if ros.pose.pose.position.x < builder.get_object("scalestoreX").get_lower():
            builder.get_object("scalestoreX").set_lower(ros.pose.pose.position.x)            

        builder.get_object("scaleX").set_value(ros.pose.pose.position.x)

        if ros.pose.pose.position.y > builder.get_object("scalestoreY").get_upper():
            builder.get_object("scalestoreY").set_upper(ros.pose.pose.position.y)
        if ros.pose.pose.position.y < builder.get_object("scalestoreY").get_lower():
            builder.get_object("scalestoreY").set_lower(ros.pose.pose.position.y)           

        builder.get_object("scaleY").set_value(ros.pose.pose.position.y)
        
        if ros.pose.pose.position.z > builder.get_object("scalestoreZ").get_upper():
            builder.get_object("scalestoreZ").set_upper(ros.pose.pose.position.z)
        if ros.pose.pose.position.z < builder.get_object("scalestoreZ").get_lower():
            builder.get_object("scalestoreZ").set_lower(ros.pose.pose.position.z)      
        
        builder.get_object("scaleZ").set_value(ros.pose.pose.position.z)
        
        euler = tf.transformations.euler_from_quaternion([ros.pose.pose.orientation.x, ros.pose.pose.orientation.y, ros.pose.pose.orientation.z, ros.pose.pose.orientation.w])
        builder.get_object("scaleYaw").set_value(euler[2]/2/math.pi*360)
        
    return True

# Main function
if __name__ == "__main__":

    # Check for the presence of a roscore by checking the ROS_MASTER_URI 
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('controller_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()



    # Initial values
    publishSetpoint = False
    publishJoystick = True
    publishAlgorithm = False
    publishInterface = False
    
    previoustime = 0

    # Initialize node
    rospy.init_node('controller_aeroquad')

    # Execute slider update from setpoint every 250 ms
    GObject.timeout_add(250, readSetpointFromConnector)

    # Threaded application, we want to be able to update the interface from this application directly
    GObject.threads_init()
    Gdk.threads_init()

    # Get glade file for the interface
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("aeroquad")
    gladefile = package_location+"/py/controller.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)


    # Init RosConnector for communication to ROS
    ros = RosConnector("aeroquad")

    # Connect all buttons from the Glade interface
    handlers = {
        "quit": btnClose,
        "btnTakeOff": btnTakeOff,
        "btnLand": btnLand,
        "btnReset": btnReset,
        "radioInput": radioInput,
        "boxSetpoint": boxSetpoint,
        "scaleX": scaleX,
        "scaleY": scaleY,
        "scaleZ": scaleZ,
        "scaleYaw": scaleYaw,
        "draw": draw,
        "btnSetNonLinear": btnSetNonLinear,
        "btnGains": btnGains
    }
    builder.connect_signals(handlers)
    
    
    # Get window and set title, show window
    window = builder.get_object("window1")
    window.set_title("Aeroquad controller")
    window.show_all()

    # Run main loop
    Gdk.threads_enter()
    Gtk.main()
    Gdk.threads_leave()
