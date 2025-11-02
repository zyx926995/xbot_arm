; Auto-generated. Do not edit!


(cl:in-package xarm_pick_place-srv)


;//! \htmlinclude RemoveObjects-request.msg.html

(cl:defclass <RemoveObjects-request> (roslisp-msg-protocol:ros-message)
  ((objects_id
    :reader objects_id
    :initarg :objects_id
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass RemoveObjects-request (<RemoveObjects-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveObjects-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveObjects-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<RemoveObjects-request> is deprecated: use xarm_pick_place-srv:RemoveObjects-request instead.")))

(cl:ensure-generic-function 'objects_id-val :lambda-list '(m))
(cl:defmethod objects_id-val ((m <RemoveObjects-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:objects_id-val is deprecated.  Use xarm_pick_place-srv:objects_id instead.")
  (objects_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveObjects-request>) ostream)
  "Serializes a message object of type '<RemoveObjects-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'objects_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveObjects-request>) istream)
  "Deserializes a message object of type '<RemoveObjects-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveObjects-request>)))
  "Returns string type for a service object of type '<RemoveObjects-request>"
  "xarm_pick_place/RemoveObjectsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveObjects-request)))
  "Returns string type for a service object of type 'RemoveObjects-request"
  "xarm_pick_place/RemoveObjectsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveObjects-request>)))
  "Returns md5sum for a message object of type '<RemoveObjects-request>"
  "1c917203483ea92cdcc10db8ea0c542a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveObjects-request)))
  "Returns md5sum for a message object of type 'RemoveObjects-request"
  "1c917203483ea92cdcc10db8ea0c542a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveObjects-request>)))
  "Returns full string definition for message of type '<RemoveObjects-request>"
  (cl:format cl:nil "string[] objects_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveObjects-request)))
  "Returns full string definition for message of type 'RemoveObjects-request"
  (cl:format cl:nil "string[] objects_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveObjects-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveObjects-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveObjects-request
    (cl:cons ':objects_id (objects_id msg))
))
;//! \htmlinclude RemoveObjects-response.msg.html

(cl:defclass <RemoveObjects-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RemoveObjects-response (<RemoveObjects-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveObjects-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveObjects-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<RemoveObjects-response> is deprecated: use xarm_pick_place-srv:RemoveObjects-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RemoveObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:success-val is deprecated.  Use xarm_pick_place-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveObjects-response>) ostream)
  "Serializes a message object of type '<RemoveObjects-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveObjects-response>) istream)
  "Deserializes a message object of type '<RemoveObjects-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveObjects-response>)))
  "Returns string type for a service object of type '<RemoveObjects-response>"
  "xarm_pick_place/RemoveObjectsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveObjects-response)))
  "Returns string type for a service object of type 'RemoveObjects-response"
  "xarm_pick_place/RemoveObjectsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveObjects-response>)))
  "Returns md5sum for a message object of type '<RemoveObjects-response>"
  "1c917203483ea92cdcc10db8ea0c542a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveObjects-response)))
  "Returns md5sum for a message object of type 'RemoveObjects-response"
  "1c917203483ea92cdcc10db8ea0c542a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveObjects-response>)))
  "Returns full string definition for message of type '<RemoveObjects-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveObjects-response)))
  "Returns full string definition for message of type 'RemoveObjects-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveObjects-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveObjects-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveObjects-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RemoveObjects)))
  'RemoveObjects-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RemoveObjects)))
  'RemoveObjects-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveObjects)))
  "Returns string type for a service object of type '<RemoveObjects>"
  "xarm_pick_place/RemoveObjects")