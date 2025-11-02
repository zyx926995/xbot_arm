;; Auto-generated. Do not edit!


(when (boundp 'xarm_pick_place::RemoveObjects)
  (if (not (find-package "XARM_PICK_PLACE"))
    (make-package "XARM_PICK_PLACE"))
  (shadow 'RemoveObjects (find-package "XARM_PICK_PLACE")))
(unless (find-package "XARM_PICK_PLACE::REMOVEOBJECTS")
  (make-package "XARM_PICK_PLACE::REMOVEOBJECTS"))
(unless (find-package "XARM_PICK_PLACE::REMOVEOBJECTSREQUEST")
  (make-package "XARM_PICK_PLACE::REMOVEOBJECTSREQUEST"))
(unless (find-package "XARM_PICK_PLACE::REMOVEOBJECTSRESPONSE")
  (make-package "XARM_PICK_PLACE::REMOVEOBJECTSRESPONSE"))

(in-package "ROS")





(defclass xarm_pick_place::RemoveObjectsRequest
  :super ros::object
  :slots (_objects_id ))

(defmethod xarm_pick_place::RemoveObjectsRequest
  (:init
   (&key
    ((:objects_id __objects_id) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _objects_id __objects_id)
   self)
  (:objects_id
   (&optional __objects_id)
   (if __objects_id (setq _objects_id __objects_id)) _objects_id)
  (:serialization-length
   ()
   (+
    ;; string[] _objects_id
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _objects_id)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _objects_id
     (write-long (length _objects_id) s)
     (dolist (elem _objects_id)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _objects_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects_id (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _objects_id i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass xarm_pick_place::RemoveObjectsResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_pick_place::RemoveObjectsResponse
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

(defclass xarm_pick_place::RemoveObjects
  :super ros::object
  :slots ())

(setf (get xarm_pick_place::RemoveObjects :md5sum-) "1c917203483ea92cdcc10db8ea0c542a")
(setf (get xarm_pick_place::RemoveObjects :datatype-) "xarm_pick_place/RemoveObjects")
(setf (get xarm_pick_place::RemoveObjects :request) xarm_pick_place::RemoveObjectsRequest)
(setf (get xarm_pick_place::RemoveObjects :response) xarm_pick_place::RemoveObjectsResponse)

(defmethod xarm_pick_place::RemoveObjectsRequest
  (:response () (instance xarm_pick_place::RemoveObjectsResponse :init)))

(setf (get xarm_pick_place::RemoveObjectsRequest :md5sum-) "1c917203483ea92cdcc10db8ea0c542a")
(setf (get xarm_pick_place::RemoveObjectsRequest :datatype-) "xarm_pick_place/RemoveObjectsRequest")
(setf (get xarm_pick_place::RemoveObjectsRequest :definition-)
      "string[] objects_id
---
bool success


")

(setf (get xarm_pick_place::RemoveObjectsResponse :md5sum-) "1c917203483ea92cdcc10db8ea0c542a")
(setf (get xarm_pick_place::RemoveObjectsResponse :datatype-) "xarm_pick_place/RemoveObjectsResponse")
(setf (get xarm_pick_place::RemoveObjectsResponse :definition-)
      "string[] objects_id
---
bool success


")



(provide :xarm_pick_place/RemoveObjects "1c917203483ea92cdcc10db8ea0c542a")


