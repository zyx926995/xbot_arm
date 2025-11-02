;; Auto-generated. Do not edit!


(when (boundp 'xarm_driver::JointLocation)
  (if (not (find-package "XARM_DRIVER"))
    (make-package "XARM_DRIVER"))
  (shadow 'JointLocation (find-package "XARM_DRIVER")))
(unless (find-package "XARM_DRIVER::JOINTLOCATION")
  (make-package "XARM_DRIVER::JOINTLOCATION"))

(in-package "ROS")
;;//! \htmlinclude JointLocation.msg.html


(defclass xarm_driver::JointLocation
  :super ros::object
  :slots (_gripper _arm_1 _arm_2 _arm_3 _arm_4 _arm_5 _arm_6 ))

(defmethod xarm_driver::JointLocation
  (:init
   (&key
    ((:gripper __gripper) 0.0)
    ((:arm_1 __arm_1) 0.0)
    ((:arm_2 __arm_2) 0.0)
    ((:arm_3 __arm_3) 0.0)
    ((:arm_4 __arm_4) 0.0)
    ((:arm_5 __arm_5) 0.0)
    ((:arm_6 __arm_6) 0.0)
    )
   (send-super :init)
   (setq _gripper (float __gripper))
   (setq _arm_1 (float __arm_1))
   (setq _arm_2 (float __arm_2))
   (setq _arm_3 (float __arm_3))
   (setq _arm_4 (float __arm_4))
   (setq _arm_5 (float __arm_5))
   (setq _arm_6 (float __arm_6))
   self)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:arm_1
   (&optional __arm_1)
   (if __arm_1 (setq _arm_1 __arm_1)) _arm_1)
  (:arm_2
   (&optional __arm_2)
   (if __arm_2 (setq _arm_2 __arm_2)) _arm_2)
  (:arm_3
   (&optional __arm_3)
   (if __arm_3 (setq _arm_3 __arm_3)) _arm_3)
  (:arm_4
   (&optional __arm_4)
   (if __arm_4 (setq _arm_4 __arm_4)) _arm_4)
  (:arm_5
   (&optional __arm_5)
   (if __arm_5 (setq _arm_5 __arm_5)) _arm_5)
  (:arm_6
   (&optional __arm_6)
   (if __arm_6 (setq _arm_6 __arm_6)) _arm_6)
  (:serialization-length
   ()
   (+
    ;; float64 _gripper
    8
    ;; float64 _arm_1
    8
    ;; float64 _arm_2
    8
    ;; float64 _arm_3
    8
    ;; float64 _arm_4
    8
    ;; float64 _arm_5
    8
    ;; float64 _arm_6
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _arm_1
       (sys::poke _arm_1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _arm_2
       (sys::poke _arm_2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _arm_3
       (sys::poke _arm_3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _arm_4
       (sys::poke _arm_4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _arm_5
       (sys::poke _arm_5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _arm_6
       (sys::poke _arm_6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _gripper
     (setq _gripper (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _arm_1
     (setq _arm_1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _arm_2
     (setq _arm_2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _arm_3
     (setq _arm_3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _arm_4
     (setq _arm_4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _arm_5
     (setq _arm_5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _arm_6
     (setq _arm_6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get xarm_driver::JointLocation :md5sum-) "bfa33ef15f3f0cc9e792db497bca66aa")
(setf (get xarm_driver::JointLocation :datatype-) "xarm_driver/JointLocation")
(setf (get xarm_driver::JointLocation :definition-)
      "float64 gripper
float64 arm_1
float64 arm_2
float64 arm_3
float64 arm_4
float64 arm_5
float64 arm_6

")



(provide :xarm_driver/JointLocation "bfa33ef15f3f0cc9e792db497bca66aa")


