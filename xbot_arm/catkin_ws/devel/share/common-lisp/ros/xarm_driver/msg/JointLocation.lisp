; Auto-generated. Do not edit!


(cl:in-package xarm_driver-msg)


;//! \htmlinclude JointLocation.msg.html

(cl:defclass <JointLocation> (roslisp-msg-protocol:ros-message)
  ((gripper
    :reader gripper
    :initarg :gripper
    :type cl:float
    :initform 0.0)
   (arm_1
    :reader arm_1
    :initarg :arm_1
    :type cl:float
    :initform 0.0)
   (arm_2
    :reader arm_2
    :initarg :arm_2
    :type cl:float
    :initform 0.0)
   (arm_3
    :reader arm_3
    :initarg :arm_3
    :type cl:float
    :initform 0.0)
   (arm_4
    :reader arm_4
    :initarg :arm_4
    :type cl:float
    :initform 0.0)
   (arm_5
    :reader arm_5
    :initarg :arm_5
    :type cl:float
    :initform 0.0)
   (arm_6
    :reader arm_6
    :initarg :arm_6
    :type cl:float
    :initform 0.0))
)

(cl:defclass JointLocation (<JointLocation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointLocation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointLocation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_driver-msg:<JointLocation> is deprecated: use xarm_driver-msg:JointLocation instead.")))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:gripper-val is deprecated.  Use xarm_driver-msg:gripper instead.")
  (gripper m))

(cl:ensure-generic-function 'arm_1-val :lambda-list '(m))
(cl:defmethod arm_1-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:arm_1-val is deprecated.  Use xarm_driver-msg:arm_1 instead.")
  (arm_1 m))

(cl:ensure-generic-function 'arm_2-val :lambda-list '(m))
(cl:defmethod arm_2-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:arm_2-val is deprecated.  Use xarm_driver-msg:arm_2 instead.")
  (arm_2 m))

(cl:ensure-generic-function 'arm_3-val :lambda-list '(m))
(cl:defmethod arm_3-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:arm_3-val is deprecated.  Use xarm_driver-msg:arm_3 instead.")
  (arm_3 m))

(cl:ensure-generic-function 'arm_4-val :lambda-list '(m))
(cl:defmethod arm_4-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:arm_4-val is deprecated.  Use xarm_driver-msg:arm_4 instead.")
  (arm_4 m))

(cl:ensure-generic-function 'arm_5-val :lambda-list '(m))
(cl:defmethod arm_5-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:arm_5-val is deprecated.  Use xarm_driver-msg:arm_5 instead.")
  (arm_5 m))

(cl:ensure-generic-function 'arm_6-val :lambda-list '(m))
(cl:defmethod arm_6-val ((m <JointLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_driver-msg:arm_6-val is deprecated.  Use xarm_driver-msg:arm_6 instead.")
  (arm_6 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointLocation>) ostream)
  "Serializes a message object of type '<JointLocation>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gripper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'arm_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'arm_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'arm_3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'arm_4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'arm_5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'arm_6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointLocation>) istream)
  "Deserializes a message object of type '<JointLocation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_5) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm_6) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointLocation>)))
  "Returns string type for a message object of type '<JointLocation>"
  "xarm_driver/JointLocation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointLocation)))
  "Returns string type for a message object of type 'JointLocation"
  "xarm_driver/JointLocation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointLocation>)))
  "Returns md5sum for a message object of type '<JointLocation>"
  "bfa33ef15f3f0cc9e792db497bca66aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointLocation)))
  "Returns md5sum for a message object of type 'JointLocation"
  "bfa33ef15f3f0cc9e792db497bca66aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointLocation>)))
  "Returns full string definition for message of type '<JointLocation>"
  (cl:format cl:nil "float64 gripper~%float64 arm_1~%float64 arm_2~%float64 arm_3~%float64 arm_4~%float64 arm_5~%float64 arm_6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointLocation)))
  "Returns full string definition for message of type 'JointLocation"
  (cl:format cl:nil "float64 gripper~%float64 arm_1~%float64 arm_2~%float64 arm_3~%float64 arm_4~%float64 arm_5~%float64 arm_6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointLocation>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointLocation>))
  "Converts a ROS message object to a list"
  (cl:list 'JointLocation
    (cl:cons ':gripper (gripper msg))
    (cl:cons ':arm_1 (arm_1 msg))
    (cl:cons ':arm_2 (arm_2 msg))
    (cl:cons ':arm_3 (arm_3 msg))
    (cl:cons ':arm_4 (arm_4 msg))
    (cl:cons ':arm_5 (arm_5 msg))
    (cl:cons ':arm_6 (arm_6 msg))
))
