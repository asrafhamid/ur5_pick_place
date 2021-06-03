# /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.executors import MultiThreadedExecutor

from rochu_gripper_msgs.msg import GripperState, GripperMode, GripperRequest
from .rochu_gripper_fma5_class import RochuGripper

timer_period = 1
class RochuGripperNode(Node):
    def __init__(self):
        super().__init__('rochu_gripper_node')
        #read parameters from param server
        self.set_parameters()
        #publish and subscribe to different topics in parallel
        self.pub_group = MutuallyExclusiveCallbackGroup() 
        self.sub_group = MutuallyExclusiveCallbackGroup()

        self.last_value = 0

        self.rochu = RochuGripper(self.ip ,self.port)
        #check for modbus tcp connectibity
        if not self.rochu.c.is_open():
            if not self.rochu.c.open():
                self.connected_ = False
                self.get_logger().error("There was an issue connecting to Rochu gripper: " + str(self.name_)+ " through IP: " + str(self.ip) + " PORT: " + str(self.port))
                rclpy.shutdown()
            
            else:
                self.connected_ = True
                self.get_logger().info("Rochu gripper" + str(self.name_) +  " connected with IP: " + str(self.ip) + " PORT: " + str(self.port))
                num = 10
                
        else:
            self.connected_ = True
            self.get_logger().info("Rochu gripper" + str(self.name_) +  " connected with IP: " + str(self.ip) + " PORT: " + str(self.port))
            
        #publisher
        self.publisher_ = self.create_publisher(GripperState,'rochu/state',10)

        #create state heartbeat of 1 second interval
        self.timer = self.create_timer(timer_period,self.rochu_state_callback,callback_group=self.pub_group)
        #subscriber
        self.rochu_request_sub = self.create_subscription(GripperRequest,'rochu/request',self.rochu_request_callback,10,callback_group=self.sub_group)
        
    def set_parameters(self):
        #declare parameters
        self.declare_parameter("rochu.name","1")
        self.declare_parameter("rochu.ip","192.168.1.200")
        self.declare_parameter("rochu.port",502)
        self.declare_parameter("rochu.max_effort",100)
        self.declare_parameter("rochu.min_effort",0)

        #get parameters from yaml file
        self.name_ = self.get_parameter('rochu.name')._value
        self.get_logger().info('[PARAM] ROCHU_NAME: "%s"' % self.name_)
        self.ip = self.get_parameter('rochu.ip')._value
        self.get_logger().info('[PARAM] ROCHU_IP: "%s"' % self.ip)
        self.port = self.get_parameter('rochu.port')._value
        self.get_logger().info('[PARAM] ROCHU_PORT: "%s"' % self.port)
        self.max_effort = self.get_parameter('rochu.max_effort')._value
        if self.max_effort > 100 :
            self.max_effort = 100
            self.get_logger().warn("Max effort exceeds working pressure range of the gripper, set to 100kPa instead.")
        self.get_logger().info('[PARAM] ROCHU_MAX_EFFORT: "%s"' % self.max_effort)
        self.min_effort = self.get_parameter('rochu.min_effort')._value
        if self.min_effort < 0 :
            self.min_effort = 0
            self.get_logger().warn("Min effort falls below the working pressure range of the gripper, set to 0kPa instead.")
        self.get_logger().info('[PARAM] ROCHU_MIN_EFFORT: "%s"' % self.min_effort)

    def rochu_state_callback(self):

        state_msg = GripperState()
        if not self.rochu.c.is_open():
            self.get_logger().error("Unexpected disconnection from Rochu gripper: " + str(self.name_)+ " through IP: " + str(self.ip) + " PORT: " + str(self.port)+ ", attempting to reconnect")
            if not self.rochu.c.open():
                self.get_logger().error("Reconnection failed on Rochu gripper: " + str(self.name_)+ " through IP: " + str(self.ip) + " PORT: " + str(self.port))
                self.connected_ = False
            else:
                self.get_logger().info("Rochu gripper" + str(self.name_) +  " reconnected with IP: " + str(self.ip) + " PORT: " + str(self.port))
                self.connected_ = True
                
        if not self.connected_:
            current_state = 3
        else :
            current_state = self.rochu.get_gripper_state()

        #create timestamp from system time
        state_msg.stamp = self.get_clock().now().to_msg()
        state_msg.name = self.name_
        state_msg.current_mode.value = current_state
        state_msg.last_requested_effort = self.last_value
        state_msg.connected = self.connected_
        
        self.publisher_.publish(state_msg)
    
    def rochu_request_callback(self,request_msg):
        
        if request_msg.name == self.name_ and self.connected_ == True:
            self.get_logger().info('Processing Request for Rochu Gripper name: "%s"' % self.name_)
            #MODE_GRAB
            if request_msg.request_mode.value == 0 :
                percentage = request_msg.effort
                if percentage > 100 :
                    percentage = 100
                    self.get_logger().warn("Effort value exceeds 100 %, set to 100 % instead") 

                elif percentage < 0 :
                    percentage = 0
                    self.get_logger().warn("Effort value below 0 %, set to 0 % instead")
                if percentage < self.last_value:
                    percentage = self.last_value
                    self.get_logger().warn("Effort value is lower then previous value, the gripper cannot decrease its pressure, please set to IDLE state and retrigger")
                    
                try:
                    _, last_pressure = self.rochu.set_pressure_value(percentage,self.max_effort,self.min_effort)

                    self.get_logger().info("Setting pressure to %d %% in range [%d:%d] kPa."%(percentage,self.min_effort,self.max_effort))
                    
                    self.rochu.trigger_pressure()
                    self.last_value = percentage

                    self.get_logger().info("Setting mode to GRAB state")
                    
                except:
                    self.get_logger().error("Could not trigger GRAB mode")
            #MODE_IDLE
            elif request_msg.request_mode.value == 1 :
                try:
                    self.rochu.cancel_pressure()
                    self.rochu.cancel_vacuum()
                    _, last_pressure = self.rochu.set_pressure_value(0,self.max_effort,0)
                    self.get_logger().info("Setting mode to IDLE state")
                    self.last_value = 0
                
                except:
                    self.get_logger().error("Could not trigger IDLE mode")
            #MODE_RELEASE
            elif request_msg.request_mode.value == 2 :
                try:
                    self.rochu.trigger_vacuum() 
                    self.get_logger().info("Setting mode to RELEASE state")
                    self.last_value = -70
                except:
                    self.get_logger().error("Could not trigger RELEASE mode")
            else :
                self.get_logger().warn("INVALID mode set")
        elif self.connected_ == False:
            self.get_logger().error("Gripper not connected")

def main(args=None):
    rclpy.init(args=args)
    rochu_gripper_node = RochuGripperNode()
    # To create threads for parallel processing of heartbeat publisher and request callbacks
    executor = MultiThreadedExecutor(num_threads=1)

    executor.add_node(rochu_gripper_node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        rochu_gripper_node.destroy_node()

    
    rclpy.shutdown()    
    
if __name__ == '__main__':
    main()