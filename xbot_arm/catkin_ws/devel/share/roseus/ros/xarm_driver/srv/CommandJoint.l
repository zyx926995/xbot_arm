;; Auto-generated. Do not edit!


(when (boundp 'xarm_driver::CommandJoint)
  (if (not (find-package "XARM_DRIVER"))
    (make-package "XARM_DRIVER"))
  (shadow 'CommandJoint (find-package "XARM_DRIVER")))
(unless (find-package "XARM_DRIVER::COMMANDJOINT")
  (make-package "XARM_DRIVER::COMMANDJOINT"))
(unless (find-package "XARM_DRIVER::COMMANDJOINTREQUEST")
  (make-package "XARM_DRIVER::COMMANDJOINTREQUEST"))
(unless (find-package "XARM_DRIVER::COMMANDJOINTRESPONSE")
  (make-package "XARM_DRIVER::COMMANDJOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass xarm_driver::CommandJointRequest
  :super ros::object
  :slots (_JointState ))

(defmethod xarm_driver::CommandJointRequest
  (:init
   (&key
    ((:JointState __JointState) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _JointState __JointState)
   self)
  (:JointState
   (&rest __JointState)
   (if (keywordp (car __JointState))
       (send* _JointState __JointState)
     (progn
       (if __JointState (setq _JointState (car __JointState)))
       _JointState)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _JointState
    (send _JointState :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _JointState
       (send _JointState :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _JointState
     (send _JointState :deserialize buf ptr-) (incf ptr- (send _JointState :serialization-length))
   ;;
   self)
  )

(defclass xarm_driver::CommandJointResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_driver::CommandJointResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_driver::CommandJoint
  :super ros::object
  :slots ())

(setf (get xarm_driver::CommandJoint :md5sum-) "4db41b5b4927bdd7563885ffb68a3317")
(setf (get xarm_driver::CommandJoint :datatype-) "xarm_driver/CommandJoint")
(setf (get xarm_driver::CommandJoint :request) xarm_driver::CommandJointRequest)
(setf (get xarm_driver::CommandJoint :response) xarm_driver::CommandJointResponse)

(defmethod xarm_driver::CommandJointRequest
  (:response () (instance xarm_driver::CommandJointResponse :init)))

(setf (get xarm_driver::CommandJointRequest :md5sum-) "4db41b5b4927bdd7563885ffb68a3317")
(setf (get xarm_driver::CommandJointRequest :datatype-) "xarm_driver/CommandJointRequest")
(setf (get xarm_driver::CommandJointRequest :definition-)
      "sensor_msgs/JointState JointState

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
---
bool success


")

(setf (get xarm_driver::CommandJointResponse :md5sum-) "4db41b5b4927bdd7563885ffb68a3317")
(setf (get xarm_driver::CommandJointResponse :datatype-) "xarm_driver/CommandJointResponse")
(setf (get xarm_driver::CommandJointResponse :definition-)
      "sensor_msgs/JointState JointState

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
---
bool success


")



(provide :xarm_driver/CommandJoint "4db41b5b4927bdd7563885ffb68a3317")


