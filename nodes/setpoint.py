#!/usr/bin/env python3
"""
This node computes different setpoint functions for the controller.
The setpoint for each dimension jumps between two different depth values with a set duration.
The different dimension can be disabled for test purposes.
The setpoint functions can be changed by an integer described by a comment
"""

import  math                            # in this node of special importance to create dynamic setpoints
import  numpy as np                     # Three dimensions make arrays kinda useful
import  rclpy                           # python client for ROS2
from    rclpy.node      import Node     # relevant for ROS2 again for making this code part of an assembled one


from hippo_msgs.msg     import Float64Stamped       # given class for publishing our own data
from rcl_interfaces.msg import SetParametersResult  # sth sth ros2 (might not even do anything here)

class setpoint_node(Node):
    def __init__(self):                             # by initializing the node we are also initializing all relevant variables, publishers and subscriptions
        super().__init__(node_name='setpoint_publisher')

        # saving when the programm has been started
        self.start_time = self.get_clock().now()

        # change these parameters to adjust the setpoint
        self.setpoint_x = np.array([ 0.75,  1.25])    # min and max of x-dimension setpoint, safe: 0.5, 1.5
        self.setpoint_y = np.array([ 2.3,  2.8])    # min and max of y-dimension setpoint, safe: 0.5, 3.3
        self.setpoint_z = np.array([-0.2, -0.6])    # min and max of z-dimension setpoint, safe: -0.1, -0.8
        self.duration   = 80.0                      # duration of one period / geometic shape runthrough in seconds, mathematicly T
        self.function   = 420                       # choosing which function-setpoint the robot should follow                      
        """
        one dimensional stuff for testing the dimensions
        # 100 for sinx, y and z static
        # 200 for siny, x and z static
        # 300 for sinz, y and z static

        Complex 
        # 420 for a circle thats angled to the xy-plane
        """

        # creating a timer to depend the setpoint functions on, mathematicly t in range of [0, T]
        # timer period sets the callback to be called every 50ms
        self.timer = self.create_timer(timer_period_sec= 1/50, callback=self.on_timer)

        # publishing the setpoints
        self.setpoint_x_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/setpoint/x', qos_profile=1)
        self.setpoint_y_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/setpoint/y', qos_profile=1)
        self.setpoint_z_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/setpoint/z', qos_profile=1)

    # functions to message setpoints to publisher      
    def publish_setpoint_x(self, setpoint_x: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint_x
        msg.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_x_pub.publish(msg)
    def publish_setpoint_y(self, setpoint_y: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint_y
        msg.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_y_pub.publish(msg)
    def publish_setpoint_z(self, setpoint_z: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint_z
        msg.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_z_pub.publish(msg)

    # callback function of the prior created timer
    def on_timer(self) -> None:
        # getting the current time 
        now = self.get_clock().now()
        # calculating the time since the programm has been running (in nanoseconds)
        time = self.start_time - now
        # running variable i from the rest of the devision (hence the use of modulo operator %)
        # once i reaches the self.duration it will be devisible setting the modulo back to 0
        i = time.nanoseconds * 1e-9 % self.duration                   
        # calculating all setpoints in dependency of the running variable i
        set_x = self.calculate_x(i)
        set_y = self.calculate_y(i)
        set_z = self.calculate_z(i)
        # now push all those values to the publisher together with the current time
        self.publish_setpoint_x(setpoint_x=set_x, now=now)
        self.publish_setpoint_y(setpoint_y=set_y, now=now)
        self.publish_setpoint_z(setpoint_z=set_z, now=now)            

    # Heart of this code where the setpoints are calculated in dependency of the running variable i
    def calculate_x(self, i: float) -> float:                                       #TODO implement the functions here
        if(self.function == 100):   # sin
            offset = (self.setpoint_x[0] + self.setpoint_x[1]) / 2                       
            amplitude = self.setpoint_x[0] - offset
            return amplitude * math.sin(2 * math.pi/self.duration * i) + offset
        if(self.function == 420):   # circle thats angled to the xy-plane
            offset = (self.setpoint_x[0] + self.setpoint_x[1]) / 2                       
            amplitude = self.setpoint_x[0] - offset
            return amplitude * math.sin(2 * math.pi/self.duration * i) + offset    
        # if the function is not defined return static setpoint, middle of the input values
        return (self.setpoint_x[0]+self.setpoint_x[1])/2

    def calculate_y(self, i: float) -> float:
        if(self.function == 200):   # sin
            offset = (self.setpoint_y[0] + self.setpoint_y[1]) / 2                       
            amplitude = self.setpoint_y[0] - offset
            return amplitude * math.sin(2 * math.pi/self.duration * i) + offset
        if(self.function == 420):   # circle thats angled to the xy-plane
            offset = (self.setpoint_y[0] + self.setpoint_y[1]) / 2                       
            amplitude = self.setpoint_y[0] - offset
            return amplitude * math.cos(2 * math.pi/self.duration * i) + offset
        # if the function is not defined return static setpoint, middle of the input values
        return (self.setpoint_y[0]+self.setpoint_y[1])/2

    def calculate_z(self, i) -> float:
        if(self.function == 300):   # sin
            offset = (self.setpoint_z[0] + self.setpoint_z[1]) / 2                       
            amplitude = self.setpoint_z[0] - offset
            return amplitude * math.sin(2 * math.pi/self.duration * i) + offset
        if(self.function == 420):   # circle thats angled to the xy-plane
            offset = (self.setpoint_z[0] + self.setpoint_z[1]) / 2                       
            amplitude = self.setpoint_z[0] - offset
            return amplitude * math.cos(2 * math.pi/self.duration * i) + offset
        # if the function is not defined return static setpoint, middle of the input values
        return (self.setpoint_z[0]+self.setpoint_z[1])/2

def main():                         # defining the main function as the above coded node in a loop
    rclpy.init()
    node = setpoint_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':          # running the main function
    main()
