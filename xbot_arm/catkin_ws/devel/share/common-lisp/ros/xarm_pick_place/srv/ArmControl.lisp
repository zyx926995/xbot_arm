; Auto-generated. Do not edit!


(cl:in-package xarm_pick_place-srv)


;//! \htmlinclude ArmControl-request.msg.html

(cl:defclass <ArmControl-request> (roslisp-msg-protocol:ros-message)
  ((joints_values
    :reader joints_values
    :initarg :joints_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ArmControl-request (<ArmControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<ArmControl-request> is deprecated: use xarm_pick_place-srv:ArmControl-request instead.")))

(cl:ensure-generic-function 'joints_values-val :lambda-list '(m))
(cl:defmethod joints_values-val ((m <ArmControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:joints_values-val is deprecated.  Use xarm_pick_place-srv:joints_values instead.")
  (joints_values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmControl-request>) ostream)
  "Serializes a message object of type '<ArmControl-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joints_values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmControl-request>) istream)
  "Deserializes a message object of type '<ArmControl-request>"
  (cl:setf (cl:slot-value msg 'joints_values) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'joints_values)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmControl-request>)))
  "Returns string type for a service object of type '<ArmControl-request>"
  "xarm_pick_place/ArmControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmControl-request)))
  "Returns string type for a service object of type 'ArmControl-request"
  "xarm_pick_place/ArmControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmControl-request>)))
  "Returns md5sum for a message object of type '<ArmControl-request>"
  "bdae41a092f6ee8c19a55dc64ce0bff1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmControl-request)))
  "Returns md5sum for a message object of type 'ArmControl-request"
  "bdae41a092f6ee8c19a55dc64ce0bff1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmControl-request>)))
  "Returns full string definition for message of type '<ArmControl-request>"
  (cl:format cl:nil "float32[6] joints_values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmControl-request)))
  "Returns full string definition for message of type 'ArmControl-request"
  (cl:format cl:nil "float32[6] joints_values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmControl-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joints_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmControl-request
    (cl:cons ':joints_values (joints_values msg))
))
;//! \htmlinclude ArmControl-response.msg.html

(cl:defclass <ArmControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArmControl-response (<ArmControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<ArmControl-response> is deprecated: use xarm_pick_place-srv:ArmControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ArmControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:success-val is deprecated.  Use xarm_pick_place-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmControl-response>) ostream)
  "Serializes a message object of type '<ArmControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmControl-response>) istream)
  "Deserializes a message object of type '<ArmControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmControl-response>)))
  "Returns string type for a service object of type '<ArmControl-response>"
  "xarm_pick_place/ArmControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmControl-response)))
  "Returns string type for a service object of type 'ArmControl-response"
  "xarm_pick_place/ArmControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmControl-response>)))
  "Returns md5sum for a message object of type '<ArmControl-response>"
  "bdae41a092f6ee8c19a55dc64ce0bff1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmControl-response)))
  "Returns md5sum for a message object of type 'ArmControl-response"
  "bdae41a092f6ee8c19a55dc64ce0bff1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmControl-response>)))
  "Returns full string definition for message of type '<ArmControl-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmControl-response)))
  "Returns full string definition for message of type 'ArmControl-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ArmControl)))
  'ArmControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ArmControl)))
  'ArmControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmControl)))
  "Returns string type for a service object of type '<ArmControl>"
  "xarm_pick_place/ArmControl")