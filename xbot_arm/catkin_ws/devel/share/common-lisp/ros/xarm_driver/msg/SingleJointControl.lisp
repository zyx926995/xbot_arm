; Auto-generated. Do not edit!


(cl:in-package xarm_driver-msg)


;//! \htmlinclude SingleJointControl.msg.html

(cl:defclass <SingleJointControl> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (rad
    :reader rad
    :initarg :rad
    :type cl:float
    :initform 0.0))
)

(cl:defclass SingleJointControl (<SingleJointControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SingleJointControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SingleJointControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-msg:<SingleJointControl> is deprecated: use xarm_driver-msg:SingleJointControl instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SingleJointControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:header-val is deprecated.  Use xarm_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SingleJointControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:id-val is deprecated.  Use xarm_driver-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'rad-val :lambda-list '(m))
(cl:defmethod rad-val ((m <SingleJointControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:rad-val is deprecated.  Use xarm_driver-msg:rad instead.")
  (rad m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SingleJointControl>) ostream)
  "Serializes a message object of type '<SingleJointControl>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rad))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SingleJointControl>) istream)
  "Deserializes a message object of type '<SingleJointControl>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rad) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SingleJointControl>)))
  "Returns string type for a message object of type '<SingleJointControl>"
  "xarm_driver/SingleJointControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SingleJointControl)))
  "Returns string type for a message object of type 'SingleJointControl"
  "xarm_driver/SingleJointControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SingleJointControl>)))
  "Returns md5sum for a message object of type '<SingleJointControl>"
  "81f0959ebd00b7475651c3cbb5d24d85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SingleJointControl)))
  "Returns md5sum for a message object of type 'SingleJointControl"
  "81f0959ebd00b7475651c3cbb5d24d85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SingleJointControl>)))
  "Returns full string definition for message of type '<SingleJointControl>"
  (cl:format cl:nil "Header header~%uint8   id~%float32   rad~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SingleJointControl)))
  "Returns full string definition for message of type 'SingleJointControl"
  (cl:format cl:nil "Header header~%uint8   id~%float32   rad~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SingleJointControl>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SingleJointControl>))
  "Converts a ROS message object to a list"
  (cl:list 'SingleJointControl
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':rad (rad msg))
))
