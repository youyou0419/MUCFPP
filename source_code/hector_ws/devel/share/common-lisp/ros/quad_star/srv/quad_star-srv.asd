
(cl:in-package :asdf)

(defsystem "quad_star-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PathPlan" :depends-on ("_package_PathPlan"))
    (:file "_package_PathPlan" :depends-on ("_package"))
    (:file "SimplePathPlan" :depends-on ("_package_SimplePathPlan"))
    (:file "_package_SimplePathPlan" :depends-on ("_package"))
  ))