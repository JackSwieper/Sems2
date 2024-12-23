#!/usr/bin/env python3

"""
#
#   WARNING: This code sucks. Do not try to understand it. Created on too much coffeein and too little sleep.
#
"""
import numpy as np                                                                  # library with math functions
import rclpy                                                                        # python client for ROS2
from rclpy.qos          import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy   # for quality of messages sent
from rclpy.node         import Node                                                 # relevant for ROS2 again for making this code part of an assembled one
from rcl_interfaces.msg import SetParametersResult                                  # for handling dynamic parameter changes
from geometry_msgs.msg  import PoseStamped, PoseWithCovarianceStamped               # importing modules for the publishing structure of a message
from hippo_msgs.msg     import RangeMeasurement, RangeMeasurementArray              # importing modules that detect and measure the anchors
from tf_transformations import euler_from_quaternion                                # relevant for getting the orientation of the robot

class PositionKalmanFilter(Node):
    def __init__(self):                                                             # initializing all variables, publishers and subscriptions
        # idk self initializing?
        super().__init__(node_name='position_kalman_filter')
        
        # setting the quality of service.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # initializing dynamic parameters
        self.init_params()

        # getting the current timestamp to calculate time difference inbetween steps later
        self.time_last_prediction = self.get_clock().now()

        # number of states
        # im using a constant velocity model, so its 6: x, y, z, vx, vy, vz
        self.num_states = 6

        # initial guess for our states
        # dimension 6x1 (rows x colums)
        self.x0_est = np.zeros((self.num_states, 1)) #+ np.array([0.0, -4.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1,1) 
        # it seems like starting the initial state at 0 is way more stable

        # current state
        # this will be updated in Kalman filter algorithm
        # using np.copy so it is an indepentent array rather then a reference to the state x0 array
        # dimension 6x1 (rows x colums)
        self.state = np.copy(self.x0_est)

        # initial state covariance
        # expresses how sure we are about our starting state
        # matrix needs to be positive definite and symmetric so im seeting it as identity matrix
        # dimension 6x6 (rows x colums) 

        self.position_intial_stddev = 1.0          # in m, default: 0.5                                     
        self.velocity_initial_stddev = 0.4   # in m/s, default: 0.5                                                                                                    
        self.P0 = np.diag([self.position_intial_stddev**2, self.position_intial_stddev**2, self.position_intial_stddev**2,
                           self.velocity_initial_stddev**2, self.velocity_initial_stddev**2, self.velocity_initial_stddev**2])
                                                          
        # state covariance
        # expresses how sure we are about our current state
        # this will be updated in Kalman filter algorithm 
        # dimension 6x6 (rows x colums)
        self.P = np.copy(self.P0)

        # process noise covariance 
        # expresses how much noise is added when calculating the next predicition step
        # matrix needs to be positive definite and symmetric so im seeting it as identity matrix
        # standard deviation is noted as square hence the **2 operation
        # dimension 6x6 (rows x colums)
        self.process_noise_position_stddev = 0.2   # in m                            
        self.process_noise_velocity_stddev = 1.0   # in m/s                                                     
        self.Q = np.diag(np.array([self.process_noise_position_stddev **2, self.process_noise_position_stddev **2, self.process_noise_position_stddev **2,
                          self.process_noise_velocity_stddev**2, self.process_noise_velocity_stddev**2, self.process_noise_velocity_stddev**2]))

        # measurement noise covariance
        # expresses how much noise the measurement has
        # matrix needs to be positive definite and symmetric so im seeting it as identity matrix
        # R is a scalar since only the range is measured
        # dimension 1x1
        # as numpy array for flexibility, reshape makes it "vertical"
        self.range_noise_stddev = 0.1                   # in m
        self.R = np.array([self.range_noise_stddev**2]).reshape((-1, 1))

        # absolute position of the tags in the tank     
        # simulation position [0.7, 3.8, -0.5]                   
        self.tag0 = np.array([0.6, 3.7, -0.3])                                                              # FIXME this has to be adjusted in the laboratory, can cause offset 
        # i must only measure one since the distance to each other remains the same
        # hence why its tag0 + relative positon                                                             
        self.tag1 = np.copy(self.tag0) + np.array([0.6, 0.0, 0.0])
        self.tag2 = np.copy(self.tag0) + np.array([0.0, 0.0, -0.4])
        self.tag3 = np.copy(self.tag0) + np.array([0.6, 0.0, -0.4])

        # making all tag positions into one array
        self.tag_poses = np.array([self.tag0, self.tag1, self.tag2, self.tag3])                            

        # offset of camera to bluerov center
        self.offset = np.array([0.17, 0.0, 0.1])                                                             # FIXME this has to be adjusted in the laboratory

        # orientation, expecting to start facing the tags
        self.yaw = np.pi/2

        # publisher for the estimated position
        self.position_pub = self.create_publisher(msg_type=PoseStamped, topic='position_estimate', qos_profile=1)
        
        # subscription to the ranges, relevant for measuring the distance to the anchor
        self.ranges_sub = self.create_subscription(
            msg_type=RangeMeasurementArray,
            topic='ranges',
            callback=self.on_ranges,
            qos_profile=qos)
        # subscription to the vision pose topic, relevant for the yaw angle
        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)
        
        # do prediction step with 50 Hz
        self.process_update_timer = self.create_timer(timer_period_sec= 1.0/50.0, callback=self.on_prediction_step_timer)
    
    """
    #
    #       DYNAMIC PARAMETER STUFF
    #
    """

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('range_noise_stddev', rclpy.Parameter.Type.DOUBLE),
                ('process_noise_position_stddev', rclpy.Parameter.Type.DOUBLE),
                ('process_noise_velocity_stddev', rclpy.Parameter.Type.DOUBLE)
            ],
        )
        param = self.get_parameter('range_noise_stddev')
        self.get_logger().info(f'{param.name}={param.value}')
        self.range_noise_stddev = param.value

        param = self.get_parameter('process_noise_position_stddev')
        self.get_logger().info(f'{param.name}={param.value}')
        self.process_noise_position_stddev = param.value

        param = self.get_parameter('process_noise_velocity_stddev')
        self.get_logger().info(f'{param.name}={param.value}')
        self.process_noise_velocity_stddev = param.value

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'range_noise_stddev':
                self.range_noise_stddev = param.value
            elif param.name == 'process_noise_position_stddev':
                self.process_noise_position_stddev = param.value
                self.Q = np.diag(np.array([self.process_noise_position_stddev **2, self.process_noise_position_stddev **2, self.process_noise_position_stddev **2,
                          self.process_noise_velocity_stddev**2, self.process_noise_velocity_stddev**2, self.process_noise_velocity_stddev**2]))

            elif param.name == 'process_noise_velocity_stddev':
                self.process_noise_velocity_stddev = param.value
                self.Q = np.diag(np.array([self.process_noise_position_stddev **2, self.process_noise_position_stddev **2, self.process_noise_position_stddev **2,
                          self.process_noise_velocity_stddev**2, self.process_noise_velocity_stddev**2, self.process_noise_velocity_stddev**2]))

            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    """
    #
    #       SUBSCRIPTION FUNCTIONS
    #
    """

    # callback function of the range subsciption
    def on_ranges(self, ranges_msg: RangeMeasurementArray) -> None:
        # number of detected tags
        num_measurements = len(ranges_msg._measurements)

        # if no tags are detected, stop here
        #else proceed by going through them in order
        if not num_measurements:
            return

        # before the measurement update doing the prediction step
        # getting current time
        #now = self.get_clock().now()
        ## calculating time difference since last prediciton in seconds
        #dt = (now - self.time_last_prediction).nanoseconds * 1e-9
        ## calling the prediction function to calculate the new state
        #(self.state, self.P) = self.prediction(dt=dt, x_est=self.state, P=self.P)
        ## resetting the time since the last prediction
        #self.time_last_prediction = now

        # type hint that variable measurement is of the class RangeMeasurement
        measurement: RangeMeasurement
        # noqa: B007 is i guess some kind of hint for the RangeMEasurement class
        # but it does not have it though...noqa: A003 is probably what this should reference
        # going through the detected tags and their measurements in order
        for i, measurement in enumerate(ranges_msg.measurements):        
            # getting the detected tags id and measured value
            tag_id = measurement.id                           
            distance_measured = measurement.range
            # calculating the robots position based on the latest tag
            self.state, self.P = self.measurement_update(tag_id=tag_id, distance_measured=distance_measured, P=self.P, x_est=self.state)
            # after getting new measurements we can publish our position
            now = self.get_clock().now()
        self.publish_position_msg(timestamp=now)       
        
    # callback function to the vision pose subscription
    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # saving the orientation globally, normalizing it    
        self.yaw = self.wrap_pi(yaw)                                                       

    """
    #
    #       PUBLISHING FUNCTIONS
    #
    """

    # messaging the position to the publisher
    def publish_position_msg(self, timestamp: rclpy.time.Time) -> None:
        # giving the message a structure and timestamp
        msg = PoseStamped()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = 'map'
        # post processing step, offset off camera to main body
        state1 = self.trigonometry()
        # giving the relevant data to the msg
        msg.pose.position.x  = state1[0, 0]
        msg.pose.position.y  = state1[1, 0]
        msg.pose.position.z  = state1[2, 0]
        self.position_pub.publish(msg)
    
    """
    #
    #       Heart of this code - Applying the Kalman Filter  
    #
    """
    
    # callback of the timer 
    def on_prediction_step_timer(self):
        # getting current time
        now = self.get_clock().now()
        # calculating time difference since last prediciton in seconds
        dt = (now - self.time_last_prediction).nanoseconds * 1e-9
        # calling the prediction function to calculate the new state
        (self.state, self.P) = self.prediction(dt=dt, x_est=self.state, P=self.P)
        # resetting the time since the last prediction
        self.time_last_prediction = now
        self.publish_position_msg(timestamp=now)       
        # publish the estimated pose with constant rate
   
    # without any measurement update we model the state evolution with the prediction step
    # returns the new estimated state and covariance
    def prediction(self, dt, x_est, P):
        # predicting the new state by using function f
        x0_est_new = self.f_fun(dt=dt, x_est=x_est)
        # getting the jacobian matrix of f to calculate the new covariance of the predicted state
        f_jacobian = self.get_f_jacobian(dt=dt)
        P_new = f_jacobian @ P @ f_jacobian.transpose() + self.Q
        # function output
        return x0_est_new, P_new
    
    # by receiving a new measurement we can do the update step
    # returns the new state and covariance
    def measurement_update(self, tag_id, distance_measured, P, x_est): 
        # noqa: F841, referenced in the RangeMeasurement class
        # using known state to calculate estimated  distance with the measurement function
        # dimension 1x1
        distance_estimated = self.h_fun(tag_id=tag_id, x_est=x_est)
        # measurement residual (difference between real and estimated measurement)
        # dimension 1x1
        y = np.asarray(distance_measured - distance_estimated).reshape((-1,1))
        # calculating the jacobian of the measurement function
        # dimension 1x6
        h_jacobian = self.get_h_jacobian(tag_id=tag_id, x_est=x_est)
        # calculating covariance of residual
        # dimension 1x1
        S = h_jacobian @ P @ h_jacobian.transpose() + self.R
        # calculating near optimal Kalman gain
        # dimension 6x1
        K = P @ h_jacobian.transpose() @ np.linalg.inv(S)       
        # updating state
        # dimension 6x1
        x_est_new = x_est + K @ y
        # updating covariance
        P_new = (np.eye(self.num_states) - (K @ h_jacobian)) @ P
        return x_est_new, P_new

    """
    #
    #       Useful functions, making the code simpler to read  
    #
    """

    # function for normalizing a given angle (value) to be in range of [-pi; pi] 
    def wrap_pi(self, value: float):                                                            
        if (-np.pi < value) and (value < np.pi):
            return value
        range = 2 * np.pi
        num_wraps = np.floor((value + np.pi) / range)   
        # dont fully understand the maths on this but it works ey, dont touch :)
        return value - range * num_wraps
    
    # function that adds the small distances from camera to robot center with trigonometry
    # returns state vector
    def trigonometry(self):
        x_est = np.copy(self.state)
        x_est[0] = np.asarray(x_est[0] - self.offset[0]* np.cos(self.yaw*180/np.pi)).reshape(-1, 1)
        x_est[1] = np.asarray(x_est[1] + self.offset[0]* np.sin(self.yaw*180/np.pi)).reshape(-1, 1)
        x_est[2] = np.asarray(x_est[2] - self.offset[2]).reshape(-1, 1)
        return x_est

    """
    #
    #       Mathematic functions for the EKF
    #
    """   

    # function used for predicting the state
    # returns a 6x1 array
    def f_fun(self, dt, x_est):
        return np.array([
            [x_est[0, 0] + dt * x_est[3, 0]],
            [x_est[1, 0] + dt * x_est[4, 0]],
            [x_est[2, 0] + dt * x_est[5, 0]],
            [x_est[3, 0]],
            [x_est[4, 0]],
            [x_est[5, 0]]
        ]).reshape((-1,1))
    
    # this gives the jacobi matrix of f_fun used for state transition
    # returns a 6x6 matrix
    def get_f_jacobian(self, dt):
        F = np.eye(self.num_states)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F
    
    # function for calculating distance based on estimated state
    # returns a scalar 1x1 array
    def h_fun(self, tag_id, x_est):
        tag_x   = self.tag_poses[tag_id][0]
        tag_y   = self.tag_poses[tag_id][1]
        tag_z   = self.tag_poses[tag_id][2]
        robot_x = x_est[0, 0]
        robot_y = x_est[1, 0]
        robot_z = x_est[2, 0]
        return np.array([
            np.sqrt((tag_x - robot_x)**2 + (tag_y - robot_y)**2 + (tag_z - robot_z)**2)
        ]).reshape((-1,1))
    
    # this gives the jacobi matrix of h_fun
    # returns a 1x6 array, since h is only one function but must be derived for the 6 different states
    def get_h_jacobian(self,tag_id, x_est):
        tag_x   = self.tag_poses[tag_id][0]
        tag_y   = self.tag_poses[tag_id][1]
        tag_z   = self.tag_poses[tag_id][2]
        robot_x = x_est[0, 0]
        robot_y = x_est[1, 0]
        robot_z = x_est[2, 0]
        return np.array([
            (robot_x - tag_x)/np.sqrt((tag_x - robot_x)**2 + (tag_y - robot_y)**2 + (tag_z - robot_z)**2),
            (robot_y - tag_y)/np.sqrt((tag_x - robot_x)**2 + (tag_y - robot_y)**2 + (tag_z - robot_z)**2),
            (robot_z - tag_z)/np.sqrt((tag_x - robot_x)**2 + (tag_y - robot_y)**2 + (tag_z - robot_z)**2),
            0,
            0,
            0,
        ]).reshape(1, 6)  # Ensure that it has the shape (1, 6)

        
    
"""
#
#       SPINNING THE CODE
#
"""

def main():
    rclpy.init()
    node = PositionKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
