;; Auto-generated. Do not edit!


(when (boundp 'xarm_pick_place::TargetPickPose)
  (if (not (find-package "XARM_PICK_PLACE"))
    (make-package "XARM_PICK_PLACE"))
  (shadow 'TargetPickPose (find-package "XARM_PICK_PLACE")))
(unless (find-package "XARM_PICK_PLACE::TARGETPICKPOSE")
  (make-package "XARM_PICK_PLACE::TARGETPICKPOSE"))
(unless (find-package "XARM_PICK_PLACE::TARGETPICKPOSEREQUEST")
  (make-package "XARM_PICK_PLACE::TARGETPICKPOSEREQUEST"))
(unless (find-package "XARM_PICK_PLACE::TARGETPICKPOSERESPONSE")
  (make-package "XARM_PICK_PLACE::TARGETPICKPOSERESPONSE"))

(in-package "ROS")





(defclass xarm_pick_place::TargetPickPoseRequest
  :super ros::object
  :slots (_x _y _z ))

(defmethod xarm_pick_place::TargetPickPoseRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_pick_place::TargetPickPoseResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_pick_place::TargetPickPoseResponse
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

(defclass xarm_pick_place::TargetPickPose
  :super ros::object
  :slots ())

(setf (get xarm_pick_place::TargetPickPose :md5sum-) "58d59f258ca9f2d2ba375d9428a7f1de")
(setf (get xarm_pick_place::TargetPickPose :datatype-) "xarm_pick_place/TargetPickPose")
(setf (get xarm_pick_place::TargetPickPose :request) xarm_pick_place::TargetPickPoseRequest)
(setf (get xarm_pick_place::TargetPickPose :response) xarm_pick_place::TargetPickPoseResponse)

(defmethod xarm_pick_place::TargetPickPoseRequest
  (:response () (instance xarm_pick_place::TargetPickPoseResponse :init)))

(setf (get xarm_pick_place::TargetPickPoseRequest :md5sum-) "58d59f258ca9f2d2ba375d9428a7f1de")
(setf (get xarm_pick_place::TargetPickPoseRequest :datatype-) "xarm_pick_place/TargetPickPoseRequest")
(setf (get xarm_pick_place::TargetPickPoseRequest :definition-)
      "float32 x
float32 y
float32 z
---
bool success


")

(setf (get xarm_pick_place::TargetPickPoseResponse :md5sum-) "58d59f258ca9f2d2ba375d9428a7f1de")
(setf (get xarm_pick_place::TargetPickPoseResponse :datatype-) "xarm_pick_place/TargetPickPoseResponse")
(setf (get xarm_pick_place::TargetPickPoseResponse :definition-)
      "float32 x
float32 y
float32 z
---
bool success


")



(provide :xarm_pick_place/TargetPickPose "58d59f258ca9f2d2ba375d9428a7f1de")


