#!usr/bin/env python3
import rclpy
import time
import threading
#from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.executors import MultiThreadedExecutor
from my_moving_robot_interfaces.action import LocationSpeed
from rclpy.action.server import ServerGoalHandle
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

class MyMovingRobotServer(LifecycleNode): 
    def __init__(self):
        super().__init__("moving_robot_server")
        self.get_logger().info("IN constructor")
        self.robot_position_=50
        self.goal_handle_ :ServerGoalHandle=None
        self.goal_lock_=threading.Lock()
        self.server_activated_ =False
        self.get_logger().info("Robot position: "+ str(self.robot_position_))


    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.declare_parameter("robot_name",rclpy.Parameter.Type.STRING)
        self.robot_name_=self.get_parameter("robot_name").value
        self.moving_robot_server_=ActionServer(
            self,
            LocationSpeed,
            "moving_robot"+self.robot_name_,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) :
        self.get_logger().info("IN on_activate")
        self.server_activated_ = True
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()
        return super().on_activate(previous_state)
    
    def on_deactivate(self, previous_state: LifecycleState) :
        self.get_logger().info("IN on_deactivate")
        self.server_activated_ = False
        return super().on_deactivate(previous_state)

    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.undeclare_parameter("robot_name")
        self.robot_name_ = ""
        self.moving_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state: LifecycleState) :
        self.get_logger().info("IN on_shutdown")
        self.undeclare_parameter("robot_name")
        self.robot_name_ = ""
        self.moving_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS

    def goal_callback(self,goal_request:LocationSpeed.Goal ):
        self.get_logger().info("Received a goal")

        if not self.server_activated_:
            self.get_logger().error("Node is not activated yet")

            return GoalResponse.REJECT

        #Validation of goal request
        if goal_request.position not in range (0, 100) or goal_request.velocity <= 0:
            self.get_logger().info("Rejecting the goal invalid position and/or velocity")
            return GoalResponse.REJECT
        
        #Policy: preempt existing goal when receiving new goal

        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self,goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_=goal_handle


        goal_position =goal_handle.request.position
        velocity =goal_handle.request.velocity

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback=LocationSpeed.Feedback()
        result=LocationSpeed.Result()

        while rclpy.ok():
            if not goal_handle.is_active:
                result.position=self.robot_position_
                result.message= "Preempted by new goal, or node deactivated"
                return result
            
            if goal_handle.is_cancel_requested:
                result.position =self.robot_position_
                if goal_position== self.robot_position_:
                    result.message="Success after cancel request"
                    goal_handle.succeed()
                else:
                    result.message="Canceled"
                    goal_handle.canceled()
                return result

            diff =goal_position - self.robot_position_

            if diff == 0:
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result
            elif diff >0:
                if diff>= velocity:
                    self.robot_position_+=velocity
                else:
                    self.robot_position_+=diff
            else:
                if abs(diff) >= velocity:
                    self.robot_position_-= velocity
                else:
                    self.robot_position_-=abs(diff)

            self.get_logger().info("Robot posiiton: "+ str(self.robot_position_))
            feedback.current_position =self.robot_position_
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)
    

def main(args=None):
    rclpy.init(args=args)
    node =MyMovingRobotServer() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__=="__main__":
    main()