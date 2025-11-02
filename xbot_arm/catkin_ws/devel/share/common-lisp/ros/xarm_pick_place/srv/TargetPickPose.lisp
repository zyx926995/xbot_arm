; Auto-generated. Do not edit!


(cl:in-package xarm_pick_place-srv)


;//! \htmlinclude TargetPickPose-request.msg.html

(cl:defclass <TargetPickPose-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass TargetPickPose-request (<TargetPickPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetPickPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetPickPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<TargetPickPose-request> is deprecated: use xarm_pick_place-srv:TargetPickPose-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <TargetPickPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:x-val is deprecated.  Use xarm_pick_place-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <TargetPickPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:y-val is deprecated.  Use xarm_pick_place-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <TargetPickPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:z-val is deprecated.  Use xarm_pick_place-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetPickPose-request>) ostream)
  "Serializes a message object of type '<TargetPickPose-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetPickPose-request>) istream)
  "Deserializes a message object of type '<TargetPickPose-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetPickPose-request>)))
  "Returns string type for a service object of type '<TargetPickPose-request>"
  "xarm_pick_place/TargetPickPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetPickPose-request)))
  "Returns string type for a service object of type 'TargetPickPose-request"
  "xarm_pick_place/TargetPickPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetPickPose-request>)))
  "Returns md5sum for a message object of type '<TargetPickPose-request>"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetPickPose-request)))
  "Returns md5sum for a message object of type 'TargetPickPose-request"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetPickPose-request>)))
  "Returns full string definition for message of type '<TargetPickPose-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetPickPose-request)))
  "Returns full string definition for message of type 'TargetPickPose-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetPickPose-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetPickPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetPickPose-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude TargetPickPose-response.msg.html

(cl:defclass <TargetPickPose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TargetPickPose-response (<TargetPickPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetPickPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetPickPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<TargetPickPose-response> is deprecated: use xarm_pick_place-srv:TargetPickPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TargetPickPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:success-val is deprecated.  Use xarm_pick_place-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetPickPose-response>) ostream)
  "Serializes a message object of type '<TargetPickPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetPickPose-response>) istream)
  "Deserializes a message object of type '<TargetPickPose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetPickPose-response>)))
  "Returns string type for a service object of type '<TargetPickPose-response>"
  "xarm_pick_place/TargetPickPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetPickPose-response)))
  "Returns string type for a service object of type 'TargetPickPose-response"
  "xarm_pick_place/TargetPickPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetPickPose-response>)))
  "Returns md5sum for a message object of type '<TargetPickPose-response>"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetPickPose-response)))
  "Returns md5sum for a message object of type 'TargetPickPose-response"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetPickPose-response>)))
  "Returns full string definition for message of type '<TargetPickPose-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetPickPose-response)))
  "Returns full string definition for message of type 'TargetPickPose-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetPickPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetPickPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetPickPose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TargetPickPose)))
  'TargetPickPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TargetPickPose)))
  'TargetPickPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetPickPose)))
  "Returns string type for a service object of type '<TargetPickPose>"
  "xarm_pick_place/TargetPickPose")