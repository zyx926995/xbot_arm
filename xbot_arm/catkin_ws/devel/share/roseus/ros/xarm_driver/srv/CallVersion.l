;; Auto-generated. Do not edit!


(when (boundp 'xarm_driver::CallVersion)
  (if (not (find-package "XARM_DRIVER"))
    (make-package "XARM_DRIVER"))
  (shadow 'CallVersion (find-package "XARM_DRIVER")))
(unless (find-package "XARM_DRIVER::CALLVERSION")
  (make-package "XARM_DRIVER::CALLVERSION"))
(unless (find-package "XARM_DRIVER::CALLVERSIONREQUEST")
  (make-package "XARM_DRIVER::CALLVERSIONREQUEST"))
(unless (find-package "XARM_DRIVER::CALLVERSIONRESPONSE")
  (make-package "XARM_DRIVER::CALLVERSIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass xarm_driver::CallVersionRequest
  :super ros::object
  :slots (_empty ))

(defmethod xarm_driver::CallVersionRequest
  (:init
   (&key
    ((:empty __empty) (instance std_msgs::Empty :init))
    )
   (send-super :init)
   (setq _empty __empty)
   self)
  (:empty
   (&rest __empty)
   (if (keywordp (car __empty))
       (send* _empty __empty)
     (progn
       (if __empty (setq _empty (car __empty)))
       _empty)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Empty _empty
    (send _empty :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Empty _empty
       (send _empty :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Empty _empty
     (send _empty :deserialize buf ptr-) (incf ptr- (send _empty :serialization-length))
   ;;
   self)
  )

(defclass xarm_driver::CallVersionResponse
  :super ros::object
  :slots (_version ))

(defmethod xarm_driver::CallVersionResponse
  (:init
   (&key
    ((:version __version) "")
    )
   (send-super :init)
   (setq _version (string __version))
   self)
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:serialization-length
   ()
   (+
    ;; string _version
    4 (length _version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _version
       (write-long (length _version) s) (princ _version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_driver::CallVersion
  :super ros::object
  :slots ())

(setf (get xarm_driver::CallVersion :md5sum-) "bdd34b709b1964ee2118cbfc3e1ee913")
(setf (get xarm_driver::CallVersion :datatype-) "xarm_driver/CallVersion")
(setf (get xarm_driver::CallVersion :request) xarm_driver::CallVersionRequest)
(setf (get xarm_driver::CallVersion :response) xarm_driver::CallVersionResponse)

(defmethod xarm_driver::CallVersionRequest
  (:response () (instance xarm_driver::CallVersionResponse :init)))

(setf (get xarm_driver::CallVersionRequest :md5sum-) "bdd34b709b1964ee2118cbfc3e1ee913")
(setf (get xarm_driver::CallVersionRequest :datatype-) "xarm_driver/CallVersionRequest")
(setf (get xarm_driver::CallVersionRequest :definition-)
      "std_msgs/Empty empty

================================================================================
MSG: std_msgs/Empty
---
string version


")

(setf (get xarm_driver::CallVersionResponse :md5sum-) "bdd34b709b1964ee2118cbfc3e1ee913")
(setf (get xarm_driver::CallVersionResponse :datatype-) "xarm_driver/CallVersionResponse")
(setf (get xarm_driver::CallVersionResponse :definition-)
      "std_msgs/Empty empty

================================================================================
MSG: std_msgs/Empty
---
string version


")



(provide :xarm_driver/CallVersion "bdd34b709b1964ee2118cbfc3e1ee913")


