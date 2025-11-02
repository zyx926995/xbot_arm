;; Auto-generated. Do not edit!


(when (boundp 'xarm_pick_place::AddObject)
  (if (not (find-package "XARM_PICK_PLACE"))
    (make-package "XARM_PICK_PLACE"))
  (shadow 'AddObject (find-package "XARM_PICK_PLACE")))
(unless (find-package "XARM_PICK_PLACE::ADDOBJECT")
  (make-package "XARM_PICK_PLACE::ADDOBJECT"))
(unless (find-package "XARM_PICK_PLACE::ADDOBJECTREQUEST")
  (make-package "XARM_PICK_PLACE::ADDOBJECTREQUEST"))
(unless (find-package "XARM_PICK_PLACE::ADDOBJECTRESPONSE")
  (make-package "XARM_PICK_PLACE::ADDOBJECTRESPONSE"))

(in-package "ROS")

(if (not (find-package "MOVEIT_MSGS"))
  (ros::roseus-add-msgs "moveit_msgs"))




(defclass xarm_pick_place::AddObjectRequest
  :super ros::object
  :slots (_collision_object ))

(defmethod xarm_pick_place::AddObjectRequest
  (:init
   (&key
    ((:collision_object __collision_object) (instance moveit_msgs::CollisionObject :init))
    )
   (send-super :init)
   (setq _collision_object __collision_object)
   self)
  (:collision_object
   (&rest __collision_object)
   (if (keywordp (car __collision_object))
       (send* _collision_object __collision_object)
     (progn
       (if __collision_object (setq _collision_object (car __collision_object)))
       _collision_object)))
  (:serialization-length
   ()
   (+
    ;; moveit_msgs/CollisionObject _collision_object
    (send _collision_object :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; moveit_msgs/CollisionObject _collision_object
       (send _collision_object :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; moveit_msgs/CollisionObject _collision_object
     (send _collision_object :deserialize buf ptr-) (incf ptr- (send _collision_object :serialization-length))
   ;;
   self)
  )

(defclass xarm_pick_place::AddObjectResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_pick_place::AddObjectResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_pick_place::AddObject
  :super ros::object
  :slots ())

(setf (get xarm_pick_place::AddObject :md5sum-) "4688b67f368aa70bf4c9081a2a2d142a")
(setf (get xarm_pick_place::AddObject :datatype-) "xarm_pick_place/AddObject")
(setf (get xarm_pick_place::AddObject :request) xarm_pick_place::AddObjectRequest)
(setf (get xarm_pick_place::AddObject :response) xarm_pick_place::AddObjectResponse)

(defmethod xarm_pick_place::AddObjectRequest
  (:response () (instance xarm_pick_place::AddObjectResponse :init)))

(setf (get xarm_pick_place::AddObjectRequest :md5sum-) "4688b67f368aa70bf4c9081a2a2d142a")
(setf (get xarm_pick_place::AddObjectRequest :datatype-) "xarm_pick_place/AddObjectRequest")
(setf (get xarm_pick_place::AddObjectRequest :definition-)
      "moveit_msgs/CollisionObject collision_object

================================================================================
MSG: moveit_msgs/CollisionObject
# a header, used for interpreting the poses
Header header

# the id of the object (name used in MoveIt)
string id

# The object type in a database of known objects
object_recognition_msgs/ObjectType type

# the the collision geometries associated with the object;
# their poses are with respect to the specified header

# solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses

# Adds the object to the planning scene. If the object previously existed, it is replaced.
byte ADD=0

# Removes the object from the environment entirely (everything that matches the specified id)
byte REMOVE=1

# Append to an object that already exists in the planning scene. If the does not exist, it is added.
byte APPEND=2

# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
# if solely moving the object is desired
byte MOVE=3

# Operation to be performed
byte operation

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: object_recognition_msgs/ObjectType
################################################## OBJECT ID #########################################################

# Contains information about the type of a found object. Those two sets of parameters together uniquely define an
# object

# The key of the found object: the unique identifier in the given db
string key

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
# The object_recognition_core as a generic DB type that can read those fields
# Current examples:
# For CouchDB:
#   type: 'CouchDB'
#   root: 'http://localhost:5984'
#   collection: 'object_recognition'
# For SQL household database:
#   type: 'SqlHousehold'
#   host: 'wgs36'
#   port: 5432
#   user: 'willow'
#   password: 'willow'
#   name: 'household_objects'
#   module: 'tabletop'
string db

================================================================================
MSG: shape_msgs/SolidPrimitive
# Define box, sphere, cylinder, cone 
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type


# The dimensions of the shape
float64[] dimensions

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array

# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
# sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2


# For the SPHERE type, only one component is used, and it gives the radius of
# the sphere.
uint8 SPHERE_RADIUS=0


# For the CYLINDER and CONE types, the center line is oriented along
# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
# of dimensions gives the height of the cylinder (cone).  The
# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
# radius of the base of the cylinder (cone).  Cone and cylinder
# primitives are defined to be circular. The tip of the cone is
# pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: shape_msgs/Mesh
# Definition of a mesh

# list of triangles; the index values refer to positions in vertices[]
MeshTriangle[] triangles

# the actual vertices that make up the mesh
geometry_msgs/Point[] vertices

================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices
uint32[3] vertex_indices

================================================================================
MSG: shape_msgs/Plane
# Representation of a plane, using the plane equation ax + by + cz + d = 0

# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]

float64[4] coef
---
bool success


")

(setf (get xarm_pick_place::AddObjectResponse :md5sum-) "4688b67f368aa70bf4c9081a2a2d142a")
(setf (get xarm_pick_place::AddObjectResponse :datatype-) "xarm_pick_place/AddObjectResponse")
(setf (get xarm_pick_place::AddObjectResponse :definition-)
      "moveit_msgs/CollisionObject collision_object

================================================================================
MSG: moveit_msgs/CollisionObject
# a header, used for interpreting the poses
Header header

# the id of the object (name used in MoveIt)
string id

# The object type in a database of known objects
object_recognition_msgs/ObjectType type

# the the collision geometries associated with the object;
# their poses are with respect to the specified header

# solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses

# Adds the object to the planning scene. If the object previously existed, it is replaced.
byte ADD=0

# Removes the object from the environment entirely (everything that matches the specified id)
byte REMOVE=1

# Append to an object that already exists in the planning scene. If the does not exist, it is added.
byte APPEND=2

# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
# if solely moving the object is desired
byte MOVE=3

# Operation to be performed
byte operation

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: object_recognition_msgs/ObjectType
################################################## OBJECT ID #########################################################

# Contains information about the type of a found object. Those two sets of parameters together uniquely define an
# object

# The key of the found object: the unique identifier in the given db
string key

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
# The object_recognition_core as a generic DB type that can read those fields
# Current examples:
# For CouchDB:
#   type: 'CouchDB'
#   root: 'http://localhost:5984'
#   collection: 'object_recognition'
# For SQL household database:
#   type: 'SqlHousehold'
#   host: 'wgs36'
#   port: 5432
#   user: 'willow'
#   password: 'willow'
#   name: 'household_objects'
#   module: 'tabletop'
string db

================================================================================
MSG: shape_msgs/SolidPrimitive
# Define box, sphere, cylinder, cone 
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type


# The dimensions of the shape
float64[] dimensions

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array

# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
# sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2


# For the SPHERE type, only one component is used, and it gives the radius of
# the sphere.
uint8 SPHERE_RADIUS=0


# For the CYLINDER and CONE types, the center line is oriented along
# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
# of dimensions gives the height of the cylinder (cone).  The
# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
# radius of the base of the cylinder (cone).  Cone and cylinder
# primitives are defined to be circular. The tip of the cone is
# pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: shape_msgs/Mesh
# Definition of a mesh

# list of triangles; the index values refer to positions in vertices[]
MeshTriangle[] triangles

# the actual vertices that make up the mesh
geometry_msgs/Point[] vertices

================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices
uint32[3] vertex_indices

================================================================================
MSG: shape_msgs/Plane
# Representation of a plane, using the plane equation ax + by + cz + d = 0

# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]

float64[4] coef
---
bool success


")



(provide :xarm_pick_place/AddObject "4688b67f368aa70bf4c9081a2a2d142a")


