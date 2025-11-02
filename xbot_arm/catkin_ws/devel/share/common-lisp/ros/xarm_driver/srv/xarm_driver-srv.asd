
(cl:in-package :asdf)

(defsystem "xarm_driver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CallVersion" :depends-on ("_package_CallVersion"))
    (:file "_package_CallVersion" :depends-on ("_package"))
    (:file "CommandJoint" :depends-on ("_package_CommandJoint"))
    (:file "_package_CommandJoint" :depends-on ("_package"))
    (:file "CommandPose" :depends-on ("_package_CommandPose"))
    (:file "_package_CommandPose" :depends-on ("_package"))
  ))