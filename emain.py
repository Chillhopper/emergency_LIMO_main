
#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8
from enum import Enum

# Define an enum class for goal status
class GoalStatus(Enum):
    SUCCEEDED = 3
    CANCELLED = 2
    PREEMPTED = 8
    ABORTED = 4

# Coordinates for respective landmarks
coords = [[-0.386695382833, 0.0424879298046, 0.0, 0.0, 0.0, -0.00250490153905, 0.999996862729], #index 0 ORGIN 
          [-1.69126015951, -0.0463478109254, 0.0, 0.0, 0.0, -0.997993223511, 0.0633208166852], #index 1 SENTOSA
          [-1.62103342139, -1.397156136, 0.0, 0.0, 0.0, 0.964335910138, 0.264681416837], #index 2 WOT
          [-0.583018437589, -1.43758706949, 0.0, 0.0, 0.0, 0.0277631894284, 0.999614528362], #Index 3 USS
          [-1.98383854798, 1.55219513185, 0.0, 0.0, 0.0, -0.957368842284, 0.288868308792],  #Index 4 SEAAQ
          [0.892950614959, -1.42552667051, 0.0, 0.0, 0.0, -0.994537768641, 0.104377328704],  #Index 5 Fort Siloso
          [0.897141718353, -0.31868321714, 0.0, 0.0, 0.0, -0.246880601993, 0.969045906219],  #Index 6 Merlion
          [-0.186661168817, 1.53146227893, 0.0, 0.0, 0.0, -0.998496600755, 0.0548136687429],  #Index 7 Rainbow Road
          [0.425864884563, 1.56001601173, 0.0, 0.0, 0.0, 0.95305514272, 0.302796788186]] #Index 8 iFly/Luge

# Callback function definitions
def active_cb(extra=0):
    rospy.loginfo("Goal pose being processed")

# the callback for monitoring the progress of the navigation goal. It checks for the presence of a message on the "/stopButton" topic, 
# which presumably signals the user wants to stop the current navigation goal. 
# If the message with data 10 is received, it calls the stop() function.
def feedback_cb(feedback):
    print("Press 's' to stop")
    myval = rospy.wait_for_message('/stopButton', Int8)
    if myval.data == 10:
        print(myval)
        stop()

# the callback called when the navigation goal reaches a final state (either succeeded, aborted, or canceled). 
# It logs information based on the status received.
def done_cb(status, result):
    status_enum = GoalStatus(status)
    if status_enum == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached")
    elif status_enum in [GoalStatus.CANCELLED, GoalStatus.PREEMPTED]:
        rospy.loginfo("Goal cancelled")
    elif status_enum == GoalStatus.ABORTED:
        rospy.loginfo("Goal aborted")

# Function definitions

# Function to send a stop request to cancel all active goals
def stop():
    print("Stop goal sent")
    navclient.cancel_all_goals()

# Function to send a navigation goal to the action server
def send_goal(navclient, goal):
    navclient.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

# Function to wait for the result of a navigation goal
def wait_for_result(navclient):
    finished = navclient.wait_for_result()
    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo(navclient.get_result())

# Function to navigate to a specific landmark based on its coordinates
def sui(coordList):
    rospy.loginfo("Waiting for server")
    navclient.wait_for_server()
    rospy.loginfo("Server found")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coordList[0]
    goal.target_pose.pose.position.y = coordList[1]
    goal.target_pose.pose.position.z = coordList[2]
    goal.target_pose.pose.orientation.x = coordList[3]
    goal.target_pose.pose.orientation.y = coordList[4]
    goal.target_pose.pose.orientation.z = coordList[5]
    goal.target_pose.pose.orientation.w = coordList[6]

    send_goal(navclient, goal)
    wait_for_result(navclient)

# waits for a message on the topic "/button" (a topic that presumably receives button presses or commands). 
# Once a message is received, it returns the data from the message (the index of the area to navigate to) and stores it in the command variable.
def gui_listener():
    message = rospy.wait_for_message('/button', Int8)
    return_val = message.data
    return return_val

if __name__ == "__main__":
    rospy.init_node('goal_pose') #initializes the ROS node with the name "goal_pose"
    navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not rospy.is_shutdown():
        print("Select zone.....")
        command = gui_listener()
        if command is not None:
            sui(coords[command])

