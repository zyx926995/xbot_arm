
(cl:in-package :asdf)

(defsystem "xarm_pick_place-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :moveit_msgs-msg
)
  :components ((:file "_package")
    (:file "AddObject" :depends-on ("_package_AddObject"))
    (:file "_package_AddObject" :depends-on ("_package"))
    (:file "ArmControl" :depends-on ("_package_ArmControl"))
    (:file "_package_ArmControl" :depends-on ("_package"))
    (:file "CommandPose" :depends-on ("_package_CommandPose"))
    (:file "_package_CommandPose" :depends-on ("_package"))
    (:file "LeftArmControl" :depends-on ("_package_LeftArmControl"))
    (:file "_package_LeftArmControl" :depends-on ("_package"))
    (:file "RemoveObjects" :depends-on ("_package_RemoveObjects"))
    (:file "_package_RemoveObjects" :depends-on ("_package"))
    (:file "TargetPickPose" :depends-on ("_package_TargetPickPose"))
    (:file "_package_TargetPickPose" :depends-on ("_package"))
    (:file "TargetPlacePose" :depends-on ("_package_TargetPlacePose"))
    (:file "_package_TargetPlacePose" :depends-on ("_package"))
  ))