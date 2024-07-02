# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import yaml
from pathlib import Path

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




class Movement(object):
    """ Movement class enables the user to save points, plan paths
and execute them. """

    def __init__(self):
        super(Movement, self).__init__()


        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("movement", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)


        #display_trajectory_publisher = rospy.Publisher(
        #    "/move_group/display_planned_path",
        #    moveit_msgs.msg.DisplayTrajectory,
        #    queue_size=20,
        #)


        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        #self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
       
       
       
       
        
    
######################################    
# The saving the points part of code #
######################################      
    
    def create_directory(self, path_name):
    
    
        directory = Path('~/Plans/{}/'.format(str(path_name)))
        directory = directory.expanduser()   
        directory.mkdir(parents = True)                             # creates directories, raises error if a dir with the same name already exists 
        return directory                                            # so it's not possible to create two paths with the same name
    
    
    def save_point(self, path_name, point_name, directory):
    
    
        self.move_group.remember_joint_values(point_name)           # method for saving current joint values of a point to a dictionary
        
        dicti = self.move_group.get_remembered_joint_values()       # returns the dictionary of saved joint values
        
        state = self.move_group.get_current_state()                 # returns the current robot state
        

        state_file = str(point_name) + '.yaml'
        state_filepath = directory / state_file                     # returns complete state file file path 
       
        with state_filepath.open('w') as u:                         # writes the state to a yaml file
            yaml.dump(state, u)                                         
             
                     
        print("\nSaved joint values:\n")                            # outputs the dictionary to the terminal
        for x, y in dicti.items():                                  # to make life easier
            print(str(x) + ":  ", str(y) + "\n")


        
    def remove_point(self, path_name, point_name, directory):                                   ######### add editing of previously saved points???  ##########
    
    
        if point_name not in self.move_group.get_remembered_joint_values():     # checks if the point exists in the dictionary
            
            dicti = self.move_group.get_remembered_joint_values()               # returns the dictionary of saved joint values
            
            print("\nNo such point in:\n")                                      # outputs the dictionary to the terminal
            for x, y in dicti.items():                                          # to make life easier
                print(str(x) + ":  ", str(y) + "\n")
        
            return print('No point removed!\n')

        
        else:                                                                   # if the point exists in the dictionary, it will soon cease to exist:
        
            self.move_group.forget_joint_values(point_name)                     # method for removing a saved point from the dictionary
        
            dicti = self.move_group.get_remembered_joint_values()               # returns the dictionary of saved joint values
        
            state = self.move_group.get_current_state()                         # returns the current robot state
        

            state_file = str(point_name) + '.yaml'
            state_filepath = directory / state_file                             # returns complete state file file path 
       
            state_filepath.unlink()                                             # removes the state yaml file from the path directory
                                       
             
            print("\nSaved joint values:\n")                                    # outputs the dictionary to the terminal
            for x, y in dicti.items():                                          # to make life easier
                print(str(x) + ":  ", str(y) + "\n")
    
    
  
  
  
  
#######################################    
# The executing the path part of code #
#######################################    
    
    def go_to_start_state(self):                                    # Method for moving from Panda's zero configuration (a singularity) to a better one 
        
        joint_goal = self.move_group.get_current_joint_values()     # Get the current configuration of the group as a list (these are values published on /joint_states)
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        self.move_group.go(joint_goal, wait=True)                   # go() is a method using the move_group_interface.cpp method move() which plans 
                                                                    # and executes a trajectory.
        self.move_group.stop()                                      # Calling stop() ensures that there is no residual movement.





    #def plan_cartesian_path(self, scale=1): build capability later
    #def display_trajectory(self, plan): myb add later for repeated plan visualization
 
 
    
    def load_files(self, path_name):
        
        path_points = []
        file_paths = []
        
        directory = Path('~/Plans/{}/'.format(str(path_name)))     
        directory = directory.expanduser()                          # Sets up the Path object for the directory in which the individual path point files are saved
  
        if directory.exists():
  
            for file_path in directory.iterdir():                       # Code section for reading and sorting file paths for every point file in the path directory
                file_paths.append(file_path)                            # The sorting is done to get the proper sequence of points which *should* have been saved
            file_paths.sort()                                           # in numerical or alphabetical order.
        
            for file_path in file_paths:                                # Code section for reading the path point files and saving the content to the path_points list
                with file_path.open('r') as f:
                    robot_state = yaml.load(f, Loader=yaml.Loader)
                path_points.append(robot_state)

            return path_points
        
        else:
            return False


    
    def plan_traj(self, path_points):                              # A method that plans and visualizes the trajectory between given points/states 
        
        trajectories = []   

        for robot_state in path_points:                            # The robot_state variable represents an individual saved path point from the load_files method 
        
            (success_flag, trajectory_message, planning_time, error_code)=\
            self.move_group.plan(robot_state.joint_state.position[0:7])     # Uses the saved position values of the 7 arm joints to plan a trajectory
                                                                            
                                                                            
            trajectories.append(trajectory_message)                # Saves the trajectory for later execution. A trajectory_message is a RobotTrajectory object


            rospy.sleep(2)                                         # Pauses code execution to allow the path visualization to finish. Visualization speed    
                                                                   # adjustable in RViz.
            self.move_group.set_start_state(robot_state)           # msg=RobotState; move_group.py line 159, m_g_i.cpp lines 451 and 1624
            
        self.move_group.set_start_state_to_current_state()         # Sets the start state to the state of the physical robot to prepare for execution
        
        return trajectories

        # reference for the plan() method: move_group.py line 619, move_group_interface.cpp line 814, wrap_python_move_group_cpp line 477
        # plan() -> set joint/pose target (dict, JointState) -> c++ method *_g.plan() -> msg.RobotTrajectory() etc.


    def execute_path(self, trajectories):                          # Method for executing the plan from the plan_traj method

        for trajectory in trajectories:
        
            self.move_group.execute(trajectory, wait=True)         # Execute a trajectory that has already been planned
            
            self.move_group.stop()                                 # Calling stop() ensures that there is no residual movement
            







def main():

    try:

        m = Movement()                                                      # Creates a class instance which allows for either a "Save path" or a "Execute path" loop.

        while 1:        
        
            print("\n\n\n***********************************************")
            action = input("Save a new path (s) or execute a path (e)? ")

                                                                            #-------------------------------------#
            if action == "s":                                               #           SAVE PATH LOOP            #
                                                                            #-------------------------------------#
                print("""


=================================================================
Save path loop

Saved points are always sorted regardless of 
order in which they are saved.

Give names to the points in the numerical or alphabetical order.

s - save point
r - remove point
skip - cancel "s" or "r" action if choosen by mistake
d - exit the "Save path" loop when finished

Press Ctrl-D to exit the program at any time.
=================================================================""")
        
        
                path_name=input("\nName of a path: ")                       # Name of a path we want to create by saving individual points
                directory = m.create_directory(path_name)                   # Creates the directory object and the actual folder in the file system
        
        
                while 1:
        
                    print("\n\n--------------------------")
                    action2 = input("Save/Remove/Done? ")
                    
                    if action2 == "s":
                        point_name = input("Name of the point: ")
                        if point_name == "skip":                            
                            continue                                           
                        m.save_point(path_name, point_name, directory)
            
                    elif action2 == "r":
                        point_name = input("Name of the point: ")              
                        if point_name == "skip":                                  
                            continue
                        m.remove_point(path_name, point_name, directory)

                    elif action2 == "d":
                        break

                    else: continue                                          # if invalid input is given, the loop restarts from the print() function


                                                                            #------------------------------------#
            elif action == "e":                                             #         EXECUTE PATH LOOP          #
                                                                            #------------------------------------#
                print("""


=================================================================
Execute path loop

A path must be planned first to be executable.

p - plan and visualize a saved path
e - execute a planned path
ss - go to start state of the robot
d - exit the "Execute path" loop when finished

Press Ctrl-D to exit the program at any time.
=================================================================

Moving to start state...""")
                 
                m.go_to_start_state()                                       # moves the robot to the start state

                path_name=input("\nName of the path: ")                     # name of the saved path we want to execute
        
                path_points = m.load_files(path_name)                       # loads the saved points of the path we want to execute
        
                if not path_points == False:
                                                                            
                    while 1:
        
                        print("\n\n--------------------------")
                        action3 = input("Plan/Execute/StartState/Done? ")
            
                        if action3 == "p":
                            trajectories = m.plan_traj(path_points)         # plans the trajectory using the points from the loaded files
            
                        elif action3 == "e":
                            if "trajectories" not in locals():
                                print("\nA path must be planned first.")
                                continue
                            else:    
                                m.execute_path(trajectories)                # executes the planned trajectories
                                                         
                        elif action3 == "ss":
                            m.go_to_start_state()                           # moves the robot to the start state
                    
                        elif action3 == "d":                            
                            break                                           # exits the "Execute path" loop
                                                           
                        else: continue                                      # if invalid input is given, the loop restarts from the print() function
                        
                else:
                    print("\nInput a valid path name.")
                    continue  

            else: 
                print("\nProgram will restart.")                            # If there is a mistake in the input for the "action" variable, the program restarts
                continue

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

