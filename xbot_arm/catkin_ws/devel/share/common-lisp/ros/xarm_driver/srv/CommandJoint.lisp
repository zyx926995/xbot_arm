; Auto-generated. Do not edit!


(cl:in-package xarm_driver-srv)


;//! \htmlinclude CommandJoint-request.msg.html

(cl:defclass <CommandJoint-request> (roslisp-msg-protocol:ros-message)
  ((JointState
    :reader JointState
    :initarg :JointState
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass CommandJoint-request (<CommandJoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandJoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandJoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-srv:<CommandJoint-request> is deprecated: use xarm_driver-srv:CommandJoint-request instead.")))

(cl:ensure-generic-function 'JointState-val :lambda-list '(m))
(cl:defmethod JointState-val ((m <CommandJoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-srv:JointState-val is deprecated.  Use xarm_driver-srv:JointState instead.")
  (JointState m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandJoint-request>) ostream)
  "Serializes a message object of type '<CommandJoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'JointState) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandJoint-request>) istream)
  "Deserializes a message object of type '<CommandJoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'JointState) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandJoint-request>)))
  "Returns string type for a service object of type '<CommandJoint-request>"
  "xarm_driver/CommandJointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandJoint-request)))
  "Returns string type for a service object of type 'CommandJoint-request"
  "xarm_driver/CommandJointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandJoint-request>)))
  "Returns md5sum for a message object of type '<CommandJoint-request>"
  "4db41b5b4927bdd7563885ffb68a3317")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandJoint-request)))
  "Returns md5sum for a message object of type 'CommandJoint-request"
  "4db41b5b4927bdd7563885ffb68a3317")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandJoint-request>)))
  "Returns full string definition for message of type '<CommandJoint-request>"
  (cl:format cl:nil "sensor_msgs/JointState JointState~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandJoint-request)))
  "Returns full string definition for message of type 'CommandJoint-request"
  (cl:format cl:nil "sensor_msgs/JointState JointState~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandJoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'JointState))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandJoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandJoint-request
    (cl:cons ':JointState (JointState msg))
))
;//! \htmlinclude CommandJoint-response.msg.html

(cl:defclass <CommandJoint-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CommandJoint-response (<CommandJoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandJoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandJoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-srv:<CommandJoint-response> is deprecated: use xarm_driver-srv:CommandJoint-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CommandJoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-srv:success-val is deprecated.  Use xarm_driver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandJoint-response>) ostream)
  "Serializes a message object of type '<CommandJoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandJoint-response>) istream)
  "Deserializes a message object of type '<CommandJoint-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandJoint-response>)))
  "Returns string type for a service object of type '<CommandJoint-response>"
  "xarm_driver/CommandJointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandJoint-response)))
  "Returns string type for a service object of type 'CommandJoint-response"
  "xarm_driver/CommandJointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandJoint-response>)))
  "Returns md5sum for a message object of type '<CommandJoint-response>"
  "4db41b5b4927bdd7563885ffb68a3317")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandJoint-response)))
  "Returns md5sum for a message object of type 'CommandJoint-response"
  "4db41b5b4927bdd7563885ffb68a3317")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandJoint-response>)))
  "Returns full string definition for message of type '<CommandJoint-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandJoint-response)))
  "Returns full string definition for message of type 'CommandJoint-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandJoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandJoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandJoint-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CommandJoint)))
  'CommandJoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CommandJoint)))
  'CommandJoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandJoint)))
  "Returns string type for a service object of type '<CommandJoint>"
  "xarm_driver/CommandJoint")