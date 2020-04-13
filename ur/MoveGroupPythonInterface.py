import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from utils import angle_between

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    """   
  move_group_interface::MoveGroup group("manipulator");
  // See ompl_planning.yaml for a complete list
  group.setPlannerId("SBLkConfigDefault");
    """
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    #move_group.set_pose_reference_frame("/shoulder_link")
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    pose_reference_frame = move_group.get_pose_reference_frame()
    print "============ Pose reference frame: %s" % pose_reference_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.theta = 0
    self.initpose = 0

  def go_to_init_pos(self, joint_goal):

    self.move_group.go(joint_goal, wait=True)		
    self.move_group.stop()

    self.initpose = self.move_group.get_current_pose().pose
    self.theta = angle_between(np.array([self.initpose.position.x, self.initpose.position.y]), np.array([1,0]))


  def go_to_joint_state(self, joint_goal):

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)		

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, pose_goal):

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    """
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1
    pose_goal.position.x = 0.154251060219
    pose_goal.position.y = -0.118073619747
    pose_goal.position.z = 0.694148264405
    """
    self.move_group.set_pose_target(pose_goal)
    
    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, xy_goal, scale=1):

    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = self.move_group.get_current_pose().pose	# apparently I shouldn't add this first point
    #waypoints.append(copy.deepcopy(wpose))
    
    x2 = (self.initpose.position.x**2 + self.initpose.position.y**2)**0.5
    for i in range(len(xy_goal)):
	    wpose.orientation.w = pi
	    wpose.orientation.x = 1
	    wpose.orientation.y = 0
	    wpose.orientation.z = 0



	    y2 = xy_goal[i][0]

    	    wpose.position.y = np.cos(self.theta)*y2 - np.sin(self.theta)*x2
    	    wpose.position.x = np.cos(self.theta)*x2 + np.sin(self.theta)*y2
	    wpose.position.z = xy_goal[i][1]

	    #wpose.position.x = xy_goal[i][0]
	    waypoints.append(copy.deepcopy(wpose))
    """
    oc = OrientationConstraint()
    print(oc)
    oc.header.frame_id = 'base_link'
    oc.link_name = 'wrist_3_link'
    oc.orientation.w = 1
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14
    oc.weight = 1.0
    print(oc)
    self.move_group.set_path_constraints(oc)
    """  
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,  	# waypoints to follow
                                       0.01,        	# eef_step
                                       0)		# jump_threshold

    return plan, fraction


  def display_trajectory(self, plan):

    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):

    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


 
