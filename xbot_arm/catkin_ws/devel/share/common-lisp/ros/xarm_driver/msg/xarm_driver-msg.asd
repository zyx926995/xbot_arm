
(cl:in-package :asdf)

(defsystem "xarm_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointLocation" :depends-on ("_package_JointLocation"))
    (:file "_package_JointLocation" :depends-on ("_package"))
    (:file "MotorStatus" :depends-on ("_package_MotorStatus"))
    (:file "_package_MotorStatus" :depends-on ("_package"))
    (:file "SingleJointControl" :depends-on ("_package_SingleJointControl"))
    (:file "_package_SingleJointControl" :depends-on ("_package"))
  ))