;; Auto-generated. Do not edit!


(when (boundp 'xarm_pick_place::ArmControl)
  (if (not (find-package "XARM_PICK_PLACE"))
    (make-package "XARM_PICK_PLACE"))
  (shadow 'ArmControl (find-package "XARM_PICK_PLACE")))
(unless (find-package "XARM_PICK_PLACE::ARMCONTROL")
  (make-package "XARM_PICK_PLACE::ARMCONTROL"))
(unless (find-package "XARM_PICK_PLACE::ARMCONTROLREQUEST")
  (make-package "XARM_PICK_PLACE::ARMCONTROLREQUEST"))
(unless (find-package "XARM_PICK_PLACE::ARMCONTROLRESPONSE")
  (make-package "XARM_PICK_PLACE::ARMCONTROLRESPONSE"))

(in-package "ROS")





(defclass xarm_pick_place::ArmControlRequest
  :super ros::object
  :slots (_joints_values ))

(defmethod xarm_pick_place::ArmControlRequest
  (:init
   (&key
    ((:joints_values __joints_values) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joints_values __joints_values)
   self)
  (:joints_values
   (&optional __joints_values)
   (if __joints_values (setq _joints_values __joints_values)) _joints_values)
  (:serialization-length
   ()
   (+
    ;; float32[6] _joints_values
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[6] _joints_values
     (dotimes (i 6)
       (sys::poke (elt _joints_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[6] _joints_values
   (dotimes (i (length _joints_values))
     (setf (elt _joints_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass xarm_pick_place::ArmControlResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_pick_place::ArmControlResponse
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

(defclass xarm_pick_place::ArmControl
  :super ros::object
  :slots ())

(setf (get xarm_pick_place::ArmControl :md5sum-) "bdae41a092f6ee8c19a55dc64ce0bff1")
(setf (get xarm_pick_place::ArmControl :datatype-) "xarm_pick_place/ArmControl")
(setf (get xarm_pick_place::ArmControl :request) xarm_pick_place::ArmControlRequest)
(setf (get xarm_pick_place::ArmControl :response) xarm_pick_place::ArmControlResponse)

(defmethod xarm_pick_place::ArmControlRequest
  (:response () (instance xarm_pick_place::ArmControlResponse :init)))

(setf (get xarm_pick_place::ArmControlRequest :md5sum-) "bdae41a092f6ee8c19a55dc64ce0bff1")
(setf (get xarm_pick_place::ArmControlRequest :datatype-) "xarm_pick_place/ArmControlRequest")
(setf (get xarm_pick_place::ArmControlRequest :definition-)
      "float32[6] joints_values
---
bool success


")

(setf (get xarm_pick_place::ArmControlResponse :md5sum-) "bdae41a092f6ee8c19a55dc64ce0bff1")
(setf (get xarm_pick_place::ArmControlResponse :datatype-) "xarm_pick_place/ArmControlResponse")
(setf (get xarm_pick_place::ArmControlResponse :definition-)
      "float32[6] joints_values
---
bool success


")



(provide :xarm_pick_place/ArmControl "bdae41a092f6ee8c19a55dc64ce0bff1")


