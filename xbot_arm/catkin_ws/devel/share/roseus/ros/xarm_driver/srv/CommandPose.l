;; Auto-generated. Do not edit!


(when (boundp 'xarm_driver::CommandPose)
  (if (not (find-package "XARM_DRIVER"))
    (make-package "XARM_DRIVER"))
  (shadow 'CommandPose (find-package "XARM_DRIVER")))
(unless (find-package "XARM_DRIVER::COMMANDPOSE")
  (make-package "XARM_DRIVER::COMMANDPOSE"))
(unless (find-package "XARM_DRIVER::COMMANDPOSEREQUEST")
  (make-package "XARM_DRIVER::COMMANDPOSEREQUEST"))
(unless (find-package "XARM_DRIVER::COMMANDPOSERESPONSE")
  (make-package "XARM_DRIVER::COMMANDPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass xarm_driver::CommandPoseRequest
  :super ros::object
  :slots (_PoseStamped ))

(defmethod xarm_driver::CommandPoseRequest
  (:init
   (&key
    ((:PoseStamped __PoseStamped) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _PoseStamped __PoseStamped)
   self)
  (:PoseStamped
   (&rest __PoseStamped)
   (if (keywordp (car __PoseStamped))
       (send* _PoseStamped __PoseStamped)
     (progn
       (if __PoseStamped (setq _PoseStamped (car __PoseStamped)))
       _PoseStamped)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _PoseStamped
    (send _PoseStamped :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _PoseStamped
       (send _PoseStamped :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _PoseStamped
     (send _PoseStamped :deserialize buf ptr-) (incf ptr- (send _PoseStamped :serialization-length))
   ;;
   self)
  )

(defclass xarm_driver::CommandPoseResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_driver::CommandPoseResponse
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

(defclass xarm_driver::CommandPose
  :super ros::object
  :slots ())

(setf (get xarm_driver::CommandPose :md5sum-) "0ad5c450393fff0205f2e85ec899fdbf")
(setf (get xarm_driver::CommandPose :datatype-) "xarm_driver/CommandPose")
(setf (get xarm_driver::CommandPose :request) xarm_driver::CommandPoseRequest)
(setf (get xarm_driver::CommandPose :response) xarm_driver::CommandPoseResponse)

(defmethod xarm_driver::CommandPoseRequest
  (:response () (instance xarm_driver::CommandPoseResponse :init)))

(setf (get xarm_driver::CommandPoseRequest :md5sum-) "0ad5c450393fff0205f2e85ec899fdbf")
(setf (get xarm_driver::CommandPoseRequest :datatype-) "xarm_driver/CommandPoseRequest")
(setf (get xarm_driver::CommandPoseRequest :definition-)
      "geometry_msgs/PoseStamped PoseStamped

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success


")

(setf (get xarm_driver::CommandPoseResponse :md5sum-) "0ad5c450393fff0205f2e85ec899fdbf")
(setf (get xarm_driver::CommandPoseResponse :datatype-) "xarm_driver/CommandPoseResponse")
(setf (get xarm_driver::CommandPoseResponse :definition-)
      "geometry_msgs/PoseStamped PoseStamped

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success


")



(provide :xarm_driver/CommandPose "0ad5c450393fff0205f2e85ec899fdbf")


