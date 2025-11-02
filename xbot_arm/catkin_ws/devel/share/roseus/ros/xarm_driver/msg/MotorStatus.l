;; Auto-generated. Do not edit!


(when (boundp 'xarm_driver::MotorStatus)
  (if (not (find-package "XARM_DRIVER"))
    (make-package "XARM_DRIVER"))
  (shadow 'MotorStatus (find-package "XARM_DRIVER")))
(unless (find-package "XARM_DRIVER::MOTORSTATUS")
  (make-package "XARM_DRIVER::MOTORSTATUS"))

(in-package "ROS")
;;//! \htmlinclude MotorStatus.msg.html


(defclass xarm_driver::MotorStatus
  :super ros::object
  :slots (_location _speed _load _voltage _temperature _current _error ))

(defmethod xarm_driver::MotorStatus
  (:init
   (&key
    ((:location __location) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:speed __speed) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:load __load) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:voltage __voltage) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:temperature __temperature) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:current __current) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error __error) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _location __location)
   (setq _speed __speed)
   (setq _load __load)
   (setq _voltage __voltage)
   (setq _temperature __temperature)
   (setq _current __current)
   (setq _error __error)
   self)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; float32[] _location
    (* 4    (length _location)) 4
    ;; float32[] _speed
    (* 4    (length _speed)) 4
    ;; float32[] _load
    (* 4    (length _load)) 4
    ;; float32[] _voltage
    (* 4    (length _voltage)) 4
    ;; float32[] _temperature
    (* 4    (length _temperature)) 4
    ;; float32[] _current
    (* 4    (length _current)) 4
    ;; uint8[] _error
    (* 1    (length _error)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _location
     (write-long (length _location) s)
     (dotimes (i (length _location))
       (sys::poke (elt _location i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _speed
     (write-long (length _speed) s)
     (dotimes (i (length _speed))
       (sys::poke (elt _speed i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _load
     (write-long (length _load) s)
     (dotimes (i (length _load))
       (sys::poke (elt _load i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _voltage
     (write-long (length _voltage) s)
     (dotimes (i (length _voltage))
       (sys::poke (elt _voltage i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _temperature
     (write-long (length _temperature) s)
     (dotimes (i (length _temperature))
       (sys::poke (elt _temperature i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _current
     (write-long (length _current) s)
     (dotimes (i (length _current))
       (sys::poke (elt _current i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8[] _error
     (write-long (length _error) s)
     (princ _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _location
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _location (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _location i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _speed
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speed (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _speed i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _load
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _load (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _load i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _voltage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _voltage (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _voltage i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _temperature
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temperature (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _temperature i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _current i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint8[] _error
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _error (make-array n :element-type :char))
     (replace _error buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get xarm_driver::MotorStatus :md5sum-) "b35e70483b9ebb379f12dfaee0874ca1")
(setf (get xarm_driver::MotorStatus :datatype-) "xarm_driver/MotorStatus")
(setf (get xarm_driver::MotorStatus :definition-)
      "float32[] location
float32[] speed
float32[] load
float32[] voltage
float32[] temperature
float32[] current
uint8[] error

")



(provide :xarm_driver/MotorStatus "b35e70483b9ebb379f12dfaee0874ca1")


