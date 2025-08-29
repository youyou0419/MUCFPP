
(cl:in-package :asdf)

(defsystem "uav_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
               :uav_control-msg
)
  :components ((:file "_package")
    (:file "PathPlan" :depends-on ("_package_PathPlan"))
    (:file "_package_PathPlan" :depends-on ("_package"))
    (:file "PathTracking" :depends-on ("_package_PathTracking"))
    (:file "_package_PathTracking" :depends-on ("_package"))
    (:file "SimplePathPlan" :depends-on ("_package_SimplePathPlan"))
    (:file "_package_SimplePathPlan" :depends-on ("_package"))
  ))