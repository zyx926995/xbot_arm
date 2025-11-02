; Auto-generated. Do not edit!


(cl:in-package xarm_driver-srv)


;//! \htmlinclude CallVersion-request.msg.html

(cl:defclass <CallVersion-request> (roslisp-msg-protocol:ros-message)
  ((empty
    :reader empty
    :initarg :empty
    :type std_msgs-msg:Empty
    :initform (cl:make-instance 'std_msgs-msg:Empty)))
)

(cl:defclass CallVersion-request (<CallVersion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallVersion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallVersion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-srv:<CallVersion-request> is deprecated: use xarm_driver-srv:CallVersion-request instead.")))

(cl:ensure-generic-function 'empty-val :lambda-list '(m))
(cl:defmethod empty-val ((m <CallVersion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-srv:empty-val is deprecated.  Use xarm_driver-srv:empty instead.")
  (empty m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallVersion-request>) ostream)
  "Serializes a message object of type '<CallVersion-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'empty) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallVersion-request>) istream)
  "Deserializes a message object of type '<CallVersion-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'empty) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallVersion-request>)))
  "Returns string type for a service object of type '<CallVersion-request>"
  "xarm_driver/CallVersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallVersion-request)))
  "Returns string type for a service object of type 'CallVersion-request"
  "xarm_driver/CallVersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallVersion-request>)))
  "Returns md5sum for a message object of type '<CallVersion-request>"
  "bdd34b709b1964ee2118cbfc3e1ee913")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallVersion-request)))
  "Returns md5sum for a message object of type 'CallVersion-request"
  "bdd34b709b1964ee2118cbfc3e1ee913")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallVersion-request>)))
  "Returns full string definition for message of type '<CallVersion-request>"
  (cl:format cl:nil "std_msgs/Empty empty~%~%================================================================================~%MSG: std_msgs/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallVersion-request)))
  "Returns full string definition for message of type 'CallVersion-request"
  (cl:format cl:nil "std_msgs/Empty empty~%~%================================================================================~%MSG: std_msgs/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallVersion-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'empty))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallVersion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CallVersion-request
    (cl:cons ':empty (empty msg))
))
;//! \htmlinclude CallVersion-response.msg.html

(cl:defclass <CallVersion-response> (roslisp-msg-protocol:ros-message)
  ((version
    :reader version
    :initarg :version
    :type cl:string
    :initform ""))
)

(cl:defclass CallVersion-response (<CallVersion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallVersion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallVersion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-srv:<CallVersion-response> is deprecated: use xarm_driver-srv:CallVersion-response instead.")))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <CallVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-srv:version-val is deprecated.  Use xarm_driver-srv:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallVersion-response>) ostream)
  "Serializes a message object of type '<CallVersion-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallVersion-response>) istream)
  "Deserializes a message object of type '<CallVersion-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallVersion-response>)))
  "Returns string type for a service object of type '<CallVersion-response>"
  "xarm_driver/CallVersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallVersion-response)))
  "Returns string type for a service object of type 'CallVersion-response"
  "xarm_driver/CallVersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallVersion-response>)))
  "Returns md5sum for a message object of type '<CallVersion-response>"
  "bdd34b709b1964ee2118cbfc3e1ee913")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallVersion-response)))
  "Returns md5sum for a message object of type 'CallVersion-response"
  "bdd34b709b1964ee2118cbfc3e1ee913")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallVersion-response>)))
  "Returns full string definition for message of type '<CallVersion-response>"
  (cl:format cl:nil "string version~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallVersion-response)))
  "Returns full string definition for message of type 'CallVersion-response"
  (cl:format cl:nil "string version~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallVersion-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallVersion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CallVersion-response
    (cl:cons ':version (version msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CallVersion)))
  'CallVersion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CallVersion)))
  'CallVersion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallVersion)))
  "Returns string type for a service object of type '<CallVersion>"
  "xarm_driver/CallVersion")