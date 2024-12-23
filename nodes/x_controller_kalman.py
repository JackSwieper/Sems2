#!/usr/bin/env python3
"""
This node is a one dimensional controller for one transposing component of the robot.
It takes as input a current position and a given position setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""

import  rclpy                                                                           # python client for ROS2
import  rclpy.time                                                                      # for coding convinience sake
from    rclpy.node          import Node                                                 # relevant for ROS2 again for making this code part of an assembled one
from    rclpy.qos           import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy   # quality of service relevant stuff

from hippo_control_msgs.msg import ActuatorSetpoint                                     # given class for thruster output
from hippo_msgs.msg         import Float64Stamped                                       # given class for publishing our own data
from geometry_msgs.msg      import PoseStamped, PoseWithCovarianceStamped                            # given class for receiving position



class x_controller_node(Node):
    def __init__(self):                             # by initializing the node we are also initializing all relevant variables, publishers and subscriptions
        super().__init__(node_name ='x_controller')
        
        # setting the quality of service. straightup stole this from yaw_controller
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # initializing all variables for the PID controller
        # setpoint for where we wanna be at and postion for where we are
        self.setpoint = 1.0
        self.position = 1.0
        # variables for time relevant data to calculate derivative and integral parts
        self.present = None
        self.past = None
        self.dt = None
        # variables for the PID controller calculation
        self.error = None
        self.prior_error = 0.0
        self.integral = 0.0
        self.diff = None
        # parameters for the PID controller output
        self.k_p = 0.5
        self.k_i = 0.15
        self.k_d = 0.15
        self.windup_limit = 0.1

        # Motor-related publishers
        self.thrust_pub         = self.create_publisher (msg_type=ActuatorSetpoint, topic='thrust_setpoint', qos_profile=1)     # this topic name is not allowed to be changed!
        # PID-related publishers
        self.setpoint_error_pub = self.create_publisher (msg_type=Float64Stamped, topic='~/setpoint_error_x', qos_profile=1)
        self.p_gain_pub         = self.create_publisher (msg_type=Float64Stamped, topic='~/p_gain_x',         qos_profile=1)
        self.i_gain_pub         = self.create_publisher (msg_type=Float64Stamped, topic='~/i_gain_x',         qos_profile=1)
        self.d_gain_pub         = self.create_publisher (msg_type=Float64Stamped, topic='~/d_gain_x',         qos_profile=1)
                

        # subcription to get the setpoint from setpoint.py publisher
        self.setpoint_sub = self.create_subscription(
            msg_type=Float64Stamped,    
            topic='setpoint_publisher/setpoint/x',                                   
            callback=self.on_setpoint,
            qos_profile=qos)
        # subscription to get the current position
        self.position_sub = self.create_subscription(
            msg_type=PoseStamped,                                   
            topic='position_estimate',
            callback=self.on_position,
            qos_profile=qos)

    """
    #
    #       SUBSCRIPTIONS FUNCTIONS
    #
    """
    
    # callback function when receiving new setpoint data
    def on_setpoint(self, setpoint_msg: Float64Stamped) -> None:
        # updating the setpoint
        self.setpoint = setpoint_msg.data
        # safety - keeping the robot in the safe range (0.5m away from the  border of the pool)
        if  (self.setpoint < 0.5):
           self.setpoint = 0.5
        elif(self.setpoint > 1.5):
            self.setpoint = 1.5            
    
    # callback function when receiving new position from kalman filter
    def on_position(self, msg: PoseStamped) -> None:
        # updating the position
        self.position   = msg.pose.position.x   
        # calculating output, calling PID-controller function for this
        thrust = self.compute_control_output()   
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        # publishing the calculated thrust
        self.publish_thrust_msg(thrust=thrust, timestamp=timestamp)
        
    """
    #
    #       PUBLISHING FUNCTIONS
    #
    """
      
    # publishing calculated motor output
    def publish_thrust_msg(self, thrust: float, timestamp: rclpy.time.Time) -> None:
        # creating the message we want to publish later
        msg = ActuatorSetpoint()
        # we want to set the horizontal thrust exlusively. In the bluerovs defined coordinatino system thats y-axis while in the pool coord. thats x. Thats fucked up.
        # mask out xz-components.
        msg.ignore_x = True
        msg.ignore_y = False
        msg.ignore_z = True
        # setting the relevant component
        msg.y = thrust
        # adding a time stamp
        msg.header.stamp = timestamp.to_msg()
        # finally publishing the message
        self.thrust_pub.publish(msg)
    # publishing PID-related data
    def publish_setpoint_error_msg(self, setpoint_error: float, timestamp: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint_error
        msg.header.stamp = timestamp.to_msg()
        self.setpoint_error_pub.publish(msg)
    def publish_pgain_msg(self, p_gain: float, timestamp: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = p_gain
        msg.header.stamp = timestamp.to_msg()
        self.p_gain_pub.publish(msg)
    def publish_igain_msg(self, i_gain: float, timestamp: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = i_gain
        msg.header.stamp = timestamp.to_msg()
        self.i_gain_pub.publish(msg)
    def publish_dgain_msg(self, d_gain: float, timestamp: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = d_gain
        msg.header.stamp = timestamp.to_msg()
        self.d_gain_pub.publish(msg)

    """
    #
    #       Heart of this code - Regelungstechnik  
    #
    """

    def compute_control_output(self) -> float:     # Code of the PID controller
        # getting time
        current_time = self.get_clock().now()
        # for the first loop when variables are "None"
        if self.past is None:                  
            self.past = current_time
            self.dt = 0.0
        else:
            self.dt = (current_time - self.past).nanoseconds * 1e-09
            self.past = current_time
        
        # Implementing the PID controller form here on out by each component
        # P - Controller - calculating the error between setpoint and real position
        # the mathematical operation for this error can be illustrated by drawing the coordinatesystems 
        self.error = self.position - self.setpoint
        # I - Controller - calculating the steady error by adding it over time, limited by windup filter
        if(      self.k_i*(self.integral + (self.error * self.dt)) < self.windup_limit
            and  self.k_i*(self.integral + (self.error * self.dt)) >- self.windup_limit):
            self.integral += (self.error * self.dt)
        else:
            if(self.integral != 0):
                self.integral = self.integral/abs(self.integral) * self.windup_limit/self.k_i
        # D - Controller - calculating the changing rate of the error by deriving
        if self.dt > 0.0:
            self.diff = (self.error - self.prior_error) / self.dt   
        else:
            self.diff = 0.0

        # PID - Controller Output
        thrust = (self.k_p * self.error) + (self.k_d * self.diff) + (self.k_i * self.integral)

        # safety - dont activate the thrusters in the wrong direction when outside of the safe zone
        if(     (self.position < 0.5 and thrust > 0.0) 
           or   (self.position > 1.5 and thrust < 0.0)):
            return 0.0
        
        # messaging the PID relevant publishers for plotjuggler
        self.publish_setpoint_error_msg(setpoint_error = self.error,        timestamp = current_time) 
        self.publish_pgain_msg(         p_gain = self.k_p * self.error,     timestamp = current_time)
        self.publish_dgain_msg(         d_gain = self.k_d * self.diff,      timestamp = current_time)
        self.publish_igain_msg(         i_gain = self.k_i * self.integral,  timestamp = current_time)      
        # After going through all components update the error to be the last known value        
        self.prior_error = self.error
        
        # returning thrust but making sure its not too large else it will break the simulation
        if(thrust<1.0 and thrust>-1.0):
            return thrust
        else:
            return thrust/abs(thrust)

"""
#
#       SPINNING THE CODE
#
"""

def main():                     # defining the main function as the above coded node in a loop
    rclpy.init()
    node = x_controller_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':      # running the main function
    main()
