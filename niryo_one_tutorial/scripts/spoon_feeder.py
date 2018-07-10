#!/usr/bin/env python
import rospy

import tracker_interface as tracker
from feeding_state_transition_logic import transitionLogicDictionary, State 
from niryo_one_tutorial.srv import PlayTrajectory
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion


class SpoonFeeder:
  def __init__(self):
    self.defaultQuat = Quaternion(0.5, 0.5, 0.5, 0.5)
    self.tracker = tracker.TrackerInterface(self.defaultQuat)
    self.play_trajectory_topic = "/Tapo/example_poses"
    self._play_trajectory = rospy.ServiceProxy("play_trajectory", PlayTrajectory)
    rospy.logwarn("TrackerInterface successfully initialized")
    self._set_state(State.MOVE_TO_PLATE)

    while not rospy.is_shutdown():
      with transitionLogicDictionary[self.state]() as transitionLogic:
        rospy.logwarn("About to wait and return")
        nextState = transitionLogic.wait_and_return_next_state() 
      rospy.logwarn("returned")
      self._set_state(nextState)

  def _set_state(self, state):
    rospy.logwarn("State is now %s" % state)
    self.state = state 
    self._update_tracker_based_on_state()

  def _update_tracker_based_on_state(self):
    if self.state == State.MOVE_TO_PLATE: 
      self.tracker.start_tracking_fixed_target([0.2,-0.2,0.1])
      self.is_first_move_to_plate = False
    elif self.state == State.PICK_UP_FOOD:
      self.xoffset = 0.03-0.1
      self.yoffset = 0.015+0.1
      self.zoffset = -0.04+0.05
      self.tracker.start_updating_target_to_pose(self.play_trajectory_topic,[self.xoffset, self.yoffset, self.zoffset])
      self._play_trajectory(String(self.play_trajectory_topic))
    elif self.state == State.MOVE_TO_SCALE: 
      self.tracker.start_tracking_fixed_target([0.05,0.3,0.3])
    else:
      rospy.logerr("The state %s is not known"%self.state)


if __name__=="__main__":
  rospy.init_node('spoon_feeder', anonymous=True)
  s = SpoonFeeder()
