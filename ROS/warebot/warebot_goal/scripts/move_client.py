#!/usr/bin/env python

import rospy
import actionlib #Import SimpleActionClient (move_base implements a SimpleActionServer)
# Brings in the .action file and messages used by the move_base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from Tkinter import *
from PIL import Image, ImageTk
import os.path

def movebase_client(x_nav, y_nav):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    wait = client.wait_for_server(rospy.Duration(5.0))

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = (x_nav-416.0)/100.0
    print("X Sent!")
    goal.target_pose.pose.position.y = (y_nav-1135.0)/100.0
    print("Y Sent!")
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)

   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("ERROR: Action Server uavailable!")
        rospy.signal_shutdown("ERROR: Action Server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')

        root = Tk()
        root.resizable(False, False)
        #Set up a tkinter canvas
        frame = Frame(root, width=1356, height=843, bd=2, relief=SUNKEN)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        canvas = Canvas(frame, width=1356, height=843, bd=0)
        canvas.grid(row=0, column=0, sticky=N+S+E+W)

        frame.pack(fill=BOTH,expand=True)

        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "images/map.png")

        #Add image
        img = ImageTk.PhotoImage(Image.open(path))
        canvas.create_image(0,0,image=img,anchor="nw")
        

        #function to be called when mouse is clicked
        def printcoords(event):
            #outputting x and y coords to console
            print (event.x, event.y)
            
            result = movebase_client(event.y, event.x)
            #if result:
                #rospy.loginfo("Goal execution done!")

        #mouseclick event

        canvas.bind("<Button 1>", printcoords)

        root.mainloop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")