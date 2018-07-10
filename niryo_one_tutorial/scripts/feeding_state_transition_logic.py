import rospy
from enum import Enum

from std_msgs.msg import Bool, Float64

class State(Enum):
  MOVE_TO_PLATE = 1
  PICK_UP_FOOD = 2
  MOVE_TO_MOUTH = 4
  MOVE_TO_SCALE = 7 
  DUMP_ON_SCALE = 8
  WAIT_FOR_WEIGHT_INPUT = 9
  MOVE_BACK_TO_MOUTH = 10 

distance_to_goal_topic = "/distance_to_target" # std_msgs/Float64
food_acquired_topic = "/food_acquired" # std_msgs/Bool from PlayTapoTrajectory

class TransitionLogic:
  def __enter__(self):
    return self
  def __exit__(self, exc_type, exc_value, traceback):
    # if your extension creates ros subscribers, be sure to overwrite this 
    pass

  # this method subscribes to any relevant ros topics
  # blocks, and returns a new state when the robot should transition
  # from the current state to a new state.
  # This class should be extended by any logic implementation for each of the different states
  def wait_and_return_next_state(self):
    rospy.logerr("This method shouldn't be called from this base class, but should be implemented in the child class")

class PickUpStateTransitionLogic(TransitionLogic):
  def __enter__(self):
    self.food_acquired = False
    self.listenForFoodAcquired = rospy.Subscriber(food_acquired_topic, Bool, self.update_food_acquired)
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForFoodAcquired.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Picking up food")
    r = rospy.Rate(10) # 10Hz
    while not self.food_acquired:
      r.sleep()
    return State.MOVE_TO_SCALE
  
  def update_food_acquired(self, message):
    self.food_acquired = message.data

class MoveToPlateStateTransitionLogic(TransitionLogic):
  def __enter__(self):
    # this might seem like a weird place to put this, but it avoids problems where we have food picked up
    self.distance_to_goal = None
    self.listenForSuccess = rospy.Subscriber(distance_to_goal_topic, Float64, self.update_distance_to_goal)
    return self
    
  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForSuccess.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Moving back to plate")
    r = rospy.Rate(10)
    epsilon_to_plate = 0.01
    while (self.distance_to_goal is None or self.distance_to_goal > epsilon_to_plate):
      r.sleep()
    return State.PICK_UP_FOOD 
  
  def update_distance_to_goal(self, message):
    self.distance_to_goal = message.data

class MoveToScaleStateTransitionLogic(TransitionLogic):
  def __enter__(self):
    self.distance_to_goal = None
    self.listenForSuccess = rospy.Subscriber(distance_to_goal_topic, Float64, self.update_distance_to_goal)
    return self
    
  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForSuccess.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Moving back to plate")
    r = rospy.Rate(10)
    epsilon_to_plate = 0.05
    while (self.distance_to_goal is None or self.distance_to_goal > epsilon_to_plate):
      r.sleep()
    rospy.sleep(4)
    return State.MOVE_TO_PLATE
  
  def update_distance_to_goal(self, message):
    self.distance_to_goal = message.data

# a dictionary that gives you the logic class constructor associated with each state
transitionLogicDictionary = { 
                     State.MOVE_TO_PLATE   : MoveToPlateStateTransitionLogic,
                     State.PICK_UP_FOOD    : PickUpStateTransitionLogic,
                     State.MOVE_TO_SCALE   : MoveToScaleStateTransitionLogic,
                   }
