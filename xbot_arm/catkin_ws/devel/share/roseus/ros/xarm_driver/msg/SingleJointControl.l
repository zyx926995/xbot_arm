;; Auto-generated. Do not edit!


(when (boundp 'xarm_driver::SingleJointControl)
  (if (not (find-package "XARM_DRIVER"))
    (make-package "XARM_DRIVER"))
  (shadow 'SingleJointControl (find-package "XARM_DRIVER")))
(unless (find-package "XARM_DRIVER::SINGLEJOINTCONTROL")
  (make-package "XARM_DRIVER::SINGLEJOINTCONTROL"))

(in-package "ROS")
;;//! \htmlinclude SingleJointControl.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass xarm_driver::SingleJointControl
  :super ros::object
  :slots (_header _id _rad ))

(defmethod xarm_driver::SingleJointControl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:rad __rad) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _rad (float __rad))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:rad
   (&optional __rad)
   (if __rad (setq _rad __rad)) _rad)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _id
    1
    ;; float32 _rad
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _id
       (write-byte _id s)
     ;; float32 _rad
       (sys::poke _rad (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _rad
     (setq _rad (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xarm_driver::SingleJointControl :md5sum-) "81f0959ebd00b7475651c3cbb5d24d85")
(setf (get xarm_driver::SingleJointControl :datatype-) "xarm_driver/SingleJointControl")
(setf (get xarm_driver::SingleJointControl :definition-)
      "Header header
uint8   id
float32   rad

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

")



(provide :xarm_driver/SingleJointControl "81f0959ebd00b7475651c3cbb5d24d85")


