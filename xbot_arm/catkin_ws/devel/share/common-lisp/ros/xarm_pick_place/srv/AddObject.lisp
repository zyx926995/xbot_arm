; Auto-generated. Do not edit!


(cl:in-package xarm_pick_place-srv)


;//! \htmlinclude AddObject-request.msg.html

(cl:defclass <AddObject-request> (roslisp-msg-protocol:ros-message)
  ((collision_object
    :reader collision_object
    :initarg :collision_object
    :type moveit_msgs-msg:CollisionObject
    :initform (cl:make-instance 'moveit_msgs-msg:CollisionObject)))
)

(cl:defclass AddObject-request (<AddObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<AddObject-request> is deprecated: use xarm_pick_place-srv:AddObject-request instead.")))

(cl:ensure-generic-function 'collision_object-val :lambda-list '(m))
(cl:defmethod collision_object-val ((m <AddObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:collision_object-val is deprecated.  Use xarm_pick_place-srv:collision_object instead.")
  (collision_object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddObject-request>) ostream)
  "Serializes a message object of type '<AddObject-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'collision_object) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddObject-request>) istream)
  "Deserializes a message object of type '<AddObject-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'collision_object) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddObject-request>)))
  "Returns string type for a service object of type '<AddObject-request>"
  "xarm_pick_place/AddObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddObject-request)))
  "Returns string type for a service object of type 'AddObject-request"
  "xarm_pick_place/AddObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddObject-request>)))
  "Returns md5sum for a message object of type '<AddObject-request>"
  "4688b67f368aa70bf4c9081a2a2d142a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddObject-request)))
  "Returns md5sum for a message object of type 'AddObject-request"
  "4688b67f368aa70bf4c9081a2a2d142a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddObject-request>)))
  "Returns full string definition for message of type '<AddObject-request>"
  (cl:format cl:nil "moveit_msgs/CollisionObject collision_object~%~%================================================================================~%MSG: moveit_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object (name used in MoveIt)~%string id~%~%# The object type in a database of known objects~%object_recognition_msgs/ObjectType type~%~%# the the collision geometries associated with the object;~%# their poses are with respect to the specified header~%~%# solid geometric primitives~%shape_msgs/SolidPrimitive[] primitives~%geometry_msgs/Pose[] primitive_poses~%~%# meshes~%shape_msgs/Mesh[] meshes~%geometry_msgs/Pose[] mesh_poses~%~%# bounding planes (equation is specified, but the plane can be oriented using an additional pose)~%shape_msgs/Plane[] planes~%geometry_msgs/Pose[] plane_poses~%~%# Adds the object to the planning scene. If the object previously existed, it is replaced.~%byte ADD=0~%~%# Removes the object from the environment entirely (everything that matches the specified id)~%byte REMOVE=1~%~%# Append to an object that already exists in the planning scene. If the does not exist, it is added.~%byte APPEND=2~%~%# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)~%# if solely moving the object is desired~%byte MOVE=3~%~%# Operation to be performed~%byte operation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: object_recognition_msgs/ObjectType~%################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%================================================================================~%MSG: shape_msgs/SolidPrimitive~%# Define box, sphere, cylinder, cone ~%# All shapes are defined to have their bounding boxes centered around 0,0,0.~%~%uint8 BOX=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 CONE=4~%~%# The type of the shape~%uint8 type~%~%~%# The dimensions of the shape~%float64[] dimensions~%~%# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array~%~%# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding~%# sides of the box.~%uint8 BOX_X=0~%uint8 BOX_Y=1~%uint8 BOX_Z=2~%~%~%# For the SPHERE type, only one component is used, and it gives the radius of~%# the sphere.~%uint8 SPHERE_RADIUS=0~%~%~%# For the CYLINDER and CONE types, the center line is oriented along~%# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component~%# of dimensions gives the height of the cylinder (cone).  The~%# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the~%# radius of the base of the cylinder (cone).  Cone and cylinder~%# primitives are defined to be circular. The tip of the cone is~%# pointing up, along +Z axis.~%~%uint8 CYLINDER_HEIGHT=0~%uint8 CYLINDER_RADIUS=1~%~%uint8 CONE_HEIGHT=0~%uint8 CONE_RADIUS=1~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: shape_msgs/Plane~%# Representation of a plane, using the plane equation ax + by + cz + d = 0~%~%# a := coef[0]~%# b := coef[1]~%# c := coef[2]~%# d := coef[3]~%~%float64[4] coef~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddObject-request)))
  "Returns full string definition for message of type 'AddObject-request"
  (cl:format cl:nil "moveit_msgs/CollisionObject collision_object~%~%================================================================================~%MSG: moveit_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object (name used in MoveIt)~%string id~%~%# The object type in a database of known objects~%object_recognition_msgs/ObjectType type~%~%# the the collision geometries associated with the object;~%# their poses are with respect to the specified header~%~%# solid geometric primitives~%shape_msgs/SolidPrimitive[] primitives~%geometry_msgs/Pose[] primitive_poses~%~%# meshes~%shape_msgs/Mesh[] meshes~%geometry_msgs/Pose[] mesh_poses~%~%# bounding planes (equation is specified, but the plane can be oriented using an additional pose)~%shape_msgs/Plane[] planes~%geometry_msgs/Pose[] plane_poses~%~%# Adds the object to the planning scene. If the object previously existed, it is replaced.~%byte ADD=0~%~%# Removes the object from the environment entirely (everything that matches the specified id)~%byte REMOVE=1~%~%# Append to an object that already exists in the planning scene. If the does not exist, it is added.~%byte APPEND=2~%~%# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)~%# if solely moving the object is desired~%byte MOVE=3~%~%# Operation to be performed~%byte operation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: object_recognition_msgs/ObjectType~%################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%================================================================================~%MSG: shape_msgs/SolidPrimitive~%# Define box, sphere, cylinder, cone ~%# All shapes are defined to have their bounding boxes centered around 0,0,0.~%~%uint8 BOX=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 CONE=4~%~%# The type of the shape~%uint8 type~%~%~%# The dimensions of the shape~%float64[] dimensions~%~%# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array~%~%# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding~%# sides of the box.~%uint8 BOX_X=0~%uint8 BOX_Y=1~%uint8 BOX_Z=2~%~%~%# For the SPHERE type, only one component is used, and it gives the radius of~%# the sphere.~%uint8 SPHERE_RADIUS=0~%~%~%# For the CYLINDER and CONE types, the center line is oriented along~%# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component~%# of dimensions gives the height of the cylinder (cone).  The~%# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the~%# radius of the base of the cylinder (cone).  Cone and cylinder~%# primitives are defined to be circular. The tip of the cone is~%# pointing up, along +Z axis.~%~%uint8 CYLINDER_HEIGHT=0~%uint8 CYLINDER_RADIUS=1~%~%uint8 CONE_HEIGHT=0~%uint8 CONE_RADIUS=1~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: shape_msgs/Plane~%# Representation of a plane, using the plane equation ax + by + cz + d = 0~%~%# a := coef[0]~%# b := coef[1]~%# c := coef[2]~%# d := coef[3]~%~%float64[4] coef~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddObject-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'collision_object))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddObject-request
    (cl:cons ':collision_object (collision_object msg))
))
;//! \htmlinclude AddObject-response.msg.html

(cl:defclass <AddObject-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddObject-response (<AddObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_pick_place-srv:<AddObject-response> is deprecated: use xarm_pick_place-srv:AddObject-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_pick_place-srv:success-val is deprecated.  Use xarm_pick_place-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddObject-response>) ostream)
  "Serializes a message object of type '<AddObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddObject-response>) istream)
  "Deserializes a message object of type '<AddObject-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddObject-response>)))
  "Returns string type for a service object of type '<AddObject-response>"
  "xarm_pick_place/AddObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddObject-response)))
  "Returns string type for a service object of type 'AddObject-response"
  "xarm_pick_place/AddObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddObject-response>)))
  "Returns md5sum for a message object of type '<AddObject-response>"
  "4688b67f368aa70bf4c9081a2a2d142a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddObject-response)))
  "Returns md5sum for a message object of type 'AddObject-response"
  "4688b67f368aa70bf4c9081a2a2d142a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddObject-response>)))
  "Returns full string definition for message of type '<AddObject-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddObject-response)))
  "Returns full string definition for message of type 'AddObject-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddObject-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddObject-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddObject)))
  'AddObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddObject)))
  'AddObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddObject)))
  "Returns string type for a service object of type '<AddObject>"
  "xarm_pick_place/AddObject")