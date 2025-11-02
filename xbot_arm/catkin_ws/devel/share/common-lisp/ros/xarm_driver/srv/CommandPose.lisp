; Auto-generated. Do not edit!


(cl:in-package xarm_driver-srv)


;//! \htmlinclude CommandPose-request.msg.html

(cl:defclass <CommandPose-request> (roslisp-msg-protocol:ros-message)
  ((PoseStamped
    :reader PoseStamped
    :initarg :PoseStamped
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass CommandPose-request (<CommandPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-srv:<CommandPose-request> is deprecated: use xarm_driver-srv:CommandPose-request instead.")))

(cl:ensure-generic-function 'PoseStamped-val :lambda-list '(m))
(cl:defmethod PoseStamped-val ((m <CommandPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-srv:PoseStamped-val is deprecated.  Use xarm_driver-srv:PoseStamped instead.")
  (PoseStamped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandPose-request>) ostream)
  "Serializes a message object of type '<CommandPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'PoseStamped) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandPose-request>) istream)
  "Deserializes a message object of type '<CommandPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'PoseStamped) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandPose-request>)))
  "Returns string type for a service object of type '<CommandPose-request>"
  "xarm_driver/CommandPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandPose-request)))
  "Returns string type for a service object of type 'CommandPose-request"
  "xarm_driver/CommandPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandPose-request>)))
  "Returns md5sum for a message object of type '<CommandPose-request>"
  "0ad5c450393fff0205f2e85ec899fdbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandPose-request)))
  "Returns md5sum for a message object of type 'CommandPose-request"
  "0ad5c450393fff0205f2e85ec899fdbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandPose-request>)))
  "Returns full string definition for message of type '<CommandPose-request>"
  (cl:format cl:nil "geometry_msgs/PoseStamped PoseStamped~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandPose-request)))
  "Returns full string definition for message of type 'CommandPose-request"
  (cl:format cl:nil "geometry_msgs/PoseStamped PoseStamped~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'PoseStamped))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandPose-request
    (cl:cons ':PoseStamped (PoseStamped msg))
))
;//! \htmlinclude CommandPose-response.msg.html

(cl:defclass <CommandPose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CommandPose-response (<CommandPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-srv:<CommandPose-response> is deprecated: use xarm_driver-srv:CommandPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CommandPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-srv:success-val is deprecated.  Use xarm_driver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandPose-response>) ostream)
  "Serializes a message object of type '<CommandPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandPose-response>) istream)
  "Deserializes a message object of type '<CommandPose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandPose-response>)))
  "Returns string type for a service object of type '<CommandPose-response>"
  "xarm_driver/CommandPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandPose-response)))
  "Returns string type for a service object of type 'CommandPose-response"
  "xarm_driver/CommandPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandPose-response>)))
  "Returns md5sum for a message object of type '<CommandPose-response>"
  "0ad5c450393fff0205f2e85ec899fdbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandPose-response)))
  "Returns md5sum for a message object of type 'CommandPose-response"
  "0ad5c450393fff0205f2e85ec899fdbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandPose-response>)))
  "Returns full string definition for message of type '<CommandPose-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandPose-response)))
  "Returns full string definition for message of type 'CommandPose-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandPose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CommandPose)))
  'CommandPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CommandPose)))
  'CommandPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandPose)))
  "Returns string type for a service object of type '<CommandPose>"
  "xarm_driver/CommandPose")