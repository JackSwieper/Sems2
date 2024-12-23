#!/usr/bin/env python3
import math

import rclpy                                                                    # ros relevant stuff
import rclpy.time                                                               # for time relevant stuff, whyever the fuck it isnt included in the one above...
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped            # current position. FIXME might not even need PoseWithCovarianceStamped
from hippo_control_msgs.msg import ActuatorSetpoint                             # motor relevant stuff
from hippo_msgs.msg import Float64Stamped                                       # so we can publish our own data
from rclpy.node import Node                                                     # so this code might run as part of a launch file
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy        # quality of service stuff, for subscription
from tf_transformations import euler_from_quaternion                            # relevant for getting the robots orientation


class yaw_controller_node(Node):
    def __init__(self):
        super().__init__(node_name='yaw_controller')

        # setting the quality of service. straightup stole this from yaw_controller
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # constant value for the yaw setpoint so the robot will keep facing the qr codes
        self.setpoint = math.pi / 2.0                                                           

        # variables for time relevant data to calculate derivative and integral parts
        self.present = None
        self.past = None
        self.dt = None
        # variables for the PID controller calculation
        self.error = None
        self.prior_error = 0.0
        self.integral = 0.0
        self.diff = None
        # PID tuning 
        self.k_p = 0.1                                                                          
        self.k_i = 0.05
        self.k_d = 0.02
        self.windup_limit = 0.1

        # subcription to get the current position
        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,            
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos,
        )
        #self.setpoint_sub = self.create_subscription(      #isnt really needed since the yaw is constant for this task
        #    Float64Stamped,
        #    topic='~/setpoint',
        #    callback=self.on_setpoint,
        #    qos_profile=qos,
        #)

        # Motor-related publisher
        self.torque_pub = self.create_publisher(msg_type=ActuatorSetpoint, topic='torque_setpoint', qos_profile=1)
        # PID-related publishers
        self.setpoint_error_pub = self.create_publisher (msg_type=Float64Stamped, topic='yaw_setpoint_error', qos_profile=1)
        self.p_gain_pub         = self.create_publisher (msg_type=Float64Stamped, topic='yaw_p_gain',         qos_profile=1)
        self.i_gain_pub         = self.create_publisher (msg_type=Float64Stamped, topic='yaw_i_gain',         qos_profile=1)
        self.d_gain_pub         = self.create_publisher (msg_type=Float64Stamped, topic='yaw_d_gain',         qos_profile=1)

    # PID-related functions to message the publishers
    def publish_setpointerror_msg(self, setpoint_error: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint_error
        msg.header.stamp = now.to_msg()
        self.setpoint_error_pub.publish(msg)
    def publish_pgain_msg(self, p_gain: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = p_gain
        msg.header.stamp = now.to_msg()
        self.p_gain_pub.publish(msg)
    def publish_igain_msg(self, i_gain: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = i_gain
        msg.header.stamp = now.to_msg()
        self.i_gain_pub.publish(msg)
    def publish_dgain_msg(self, d_gain: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = d_gain
        msg.header.stamp = now.to_msg()
        self.d_gain_pub.publish(msg)

    # function for normalizing a given angle (value) to be in range of [-pi; pi] 
    def wrap_pi(self, value: float):                                                            
        if (-math.pi < value) and (value < math.pi):
            return value
        range = 2 * math.pi
        num_wraps = math.floor((value + math.pi) / range)   
        # dont fully understand the maths on this but it works ey, dont touch :)
        return value - range * num_wraps

    #def on_setpoint(self, msg: Float64Stamped):        # callback function of the not needed setpoint subscription
    #    self.setpoint = self.wrap_pi(msg.data)

    # callback function for the position subscription
    def on_vision_pose(self, msg: PoseStamped):                                                 
        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert the quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # normalize the determined rotation angle
        yaw = self.wrap_pi(yaw)
        # calculating the needed motor output for correction
        control_output = self.compute_control_output(yaw)
        # getting the time to add to the published message
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.publish_control_output(control_output, timestamp)

    # function for calculating the motor output based on current position
    def compute_control_output(self, angle: float) -> float:                                                      # here is the controller for the yaw angle                                     
        # getting time
        current_time = self.get_clock().now()                                                   
        # for the launch while some variables are still none
        if self.past is None:                                                                   
            self.past = current_time
            self.dt = 0.0
        else:
            self.dt = (current_time - self.past).nanoseconds * 1e-09
            self.past = current_time
        
        # Implementing the PID controller form here on out by each component
        # P - Controller - calculating the error between setpoint and real position and normalizing it
        self.error = self.wrap_pi(self.setpoint - angle)  
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
   
        # After going through all components update the error to be the last known value        
        self.prior_error = self.error
        # PID - Controller Output
        thrust = (self.k_p * self.error) + (self.k_d * self.diff) + (self.k_i * self.integral)
        # messaging the PID relevant publishers for plotjuggler
        self.publish_setpointerror_msg( setpoint_error = self.error,        now = current_time) 
        self.publish_pgain_msg(         p_gain = self.k_p * self.error,     now = current_time)
        self.publish_dgain_msg(         d_gain = self.k_d * self.diff,      now = current_time)
        self.publish_igain_msg(         i_gain = self.k_i * self.integral,  now = current_time)                                                          

        # returning thrust but making sure its not too large else it will break the simulation
        if(thrust<1.0 and thrust>-1.0):
            return thrust
        else:
            return thrust/abs(thrust)
                                                               
    # function to message the publisher for motor output
    def publish_control_output(self, control_output: float, timestamp: rclpy.time.Time):
        # setting the message as an object of the motor class
        msg = ActuatorSetpoint()
        # adding time to the message, which was given as a parameter to this funciton
        msg.header.stamp = timestamp.to_msg()
        # yaw is the rotation around the vehicle's z axis, so we mask out the xy components
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False    
        # writing the motor output in the message
        msg.z = control_output
        # finally publishing the message
        self.torque_pub.publish(msg)


def main():
    rclpy.init()
    node = yaw_controller_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
