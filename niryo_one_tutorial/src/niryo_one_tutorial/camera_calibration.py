"""Calibration from camera to robot"""
import rospkg
import yaml
import tf
import numpy as np
import rospy

class CameraCalibration:
  def __init__(self):
    """
    cameraCalibration.yml is something like:
    Rotation:
    - - 0.9989181262157819
      - 0.01931543906095123
      - -0.04230237500938314
    - - 0.04514525926005835
      - -0.18453994506190463
      - 0.981787611575381
    - - 0.011157180821961618
      - -0.9826351929838623
      - -0.18521229663733005
    Translation:
    - - 0.5391208676662875
    - - -0.6604205518057711
    - - 0.4549267234161455
    """
    rospack = rospkg.RosPack()
    ada_tut_path = rospack.get_path("niryo_one_tutorial")
    with open(ada_tut_path + '/config/calibrationParameters.yml','r') as f:
      calibParams = yaml.load(f)
      rospy.logwarn("calibParams %s"% calibParams)
      rotation = np.array(calibParams["Rotation"])
      translation = np.array(calibParams["Translation"])
     
      self.camera_to_robot = np.concatenate((np.concatenate((rotation,translation), axis=1),
                                     [[ 0, 0, 0, 1]]))
    
    self.br = tf.TransformBroadcaster()
    rospy.logwarn("camera calibration initialized")
    
    rospy.Timer(rospy.Duration(0.01), self.broadcastTransform)
    rospy.logwarn("sent first message")

  def broadcastTransform(self, event):
    #rospy.logwarn("broadcasting tf")
    self.br.sendTransform(self.camera_to_robot[0:3,3],
                          # note we pass in 4x4 to this method... https://github.com/ros/geometry/issues/64
                          tf.transformations.quaternion_from_matrix(self.camera_to_robot),
                          rospy.Time.now(),
                          "camera_rgb_optical_frame",
                          "base_link")

  # point should be a length 4 np.array giving the location of the target point in the camera frame
  def convert_to_robot_frame(self, point):
    return self.camera_to_robot.dot(point.transpose())
