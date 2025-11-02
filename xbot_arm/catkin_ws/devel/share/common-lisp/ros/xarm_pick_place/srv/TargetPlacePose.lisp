; Auto-generated. Do not edit!


(cl:in-package xarm_pick_place-srv)


;//! \htmlinclude TargetPlacePose-request.msg.html

(cl:defclass <TargetPlacePose-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TargetPlacePose-request (<TargetPlacePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetPlacePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetPlacePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<TargetPlacePose-request> is deprecated: use xarm_pick_place-srv:TargetPlacePose-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <TargetPlacePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:x-val is deprecated.  Use xarm_pick_place-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <TargetPlacePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:y-val is deprecated.  Use xarm_pick_place-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <TargetPlacePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:z-val is deprecated.  Use xarm_pick_place-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetPlacePose-request>) ostream)
  "Serializes a message object of type '<TargetPlacePose-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetPlacePose-request>) istream)
  "Deserializes a message object of type '<TargetPlacePose-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetPlacePose-request>)))
  "Returns string type for a service object of type '<TargetPlacePose-request>"
  "xarm_pick_place/TargetPlacePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetPlacePose-request)))
  "Returns string type for a service object of type 'TargetPlacePose-request"
  "xarm_pick_place/TargetPlacePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetPlacePose-request>)))
  "Returns md5sum for a message object of type '<TargetPlacePose-request>"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetPlacePose-request)))
  "Returns md5sum for a message object of type 'TargetPlacePose-request"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetPlacePose-request>)))
  "Returns full string definition for message of type '<TargetPlacePose-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetPlacePose-request)))
  "Returns full string definition for message of type 'TargetPlacePose-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetPlacePose-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetPlacePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetPlacePose-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude TargetPlacePose-response.msg.html

(cl:defclass <TargetPlacePose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TargetPlacePose-response (<TargetPlacePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetPlacePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetPlacePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<TargetPlacePose-response> is deprecated: use xarm_pick_place-srv:TargetPlacePose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TargetPlacePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:success-val is deprecated.  Use xarm_pick_place-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetPlacePose-response>) ostream)
  "Serializes a message object of type '<TargetPlacePose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetPlacePose-response>) istream)
  "Deserializes a message object of type '<TargetPlacePose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetPlacePose-response>)))
  "Returns string type for a service object of type '<TargetPlacePose-response>"
  "xarm_pick_place/TargetPlacePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetPlacePose-response)))
  "Returns string type for a service object of type 'TargetPlacePose-response"
  "xarm_pick_place/TargetPlacePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetPlacePose-response>)))
  "Returns md5sum for a message object of type '<TargetPlacePose-response>"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetPlacePose-response)))
  "Returns md5sum for a message object of type 'TargetPlacePose-response"
  "58d59f258ca9f2d2ba375d9428a7f1de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetPlacePose-response>)))
  "Returns full string definition for message of type '<TargetPlacePose-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetPlacePose-response)))
  "Returns full string definition for message of type 'TargetPlacePose-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetPlacePose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetPlacePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetPlacePose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TargetPlacePose)))
  'TargetPlacePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TargetPlacePose)))
  'TargetPlacePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetPlacePose)))
  "Returns string type for a service object of type '<TargetPlacePose>"
  "xarm_pick_place/TargetPlacePose")