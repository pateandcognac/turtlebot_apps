#!/usr/bin/env python

import rospy
import actionlib
from turtlebot_actions.msg import TurtlebotMoveAction, TurtlebotMoveGoal, TurtlebotMoveFeedback, TurtlebotMoveResult

def feedback_cb(feedback):
    rospy.loginfo("Feedback - Forward: {:.2f} m, Turn: {:.2f} rad".format(feedback.forward_distance, feedback.turn_distance))

def main():
    rospy.init_node("turtlebot_move_client")

    # Create action client
    client = actionlib.SimpleActionClient("turtlebot_move", TurtlebotMoveAction)
    
    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()
    rospy.loginfo("Action server available!")

    # Create goal
    goal = TurtlebotMoveGoal()
    goal.turn_distance = 3.14159 / 4  # 45 degrees in radians
    goal.forward_distance = 0.5  # Move forward 0.5 meters

    rospy.loginfo("Sending goal: Turn {:.2f} rad, Move {:.2f} m".format(goal.turn_distance, goal.forward_distance))
    
    # Send goal with feedback callback
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Wait for result
    client.wait_for_result()
    
    # Get and print final result
    result = client.get_result()
    if result:
        rospy.loginfo("Action completed! Final Forward: {:.2f} m, Final Turn: {:.2f} rad".format(result.forward_distance, result.turn_distance))
    else:
        rospy.logwarn("Action failed or was preempted.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
