
(cl:in-package :asdf)

(defsystem "uav_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DeadlockInfo" :depends-on ("_package_DeadlockInfo"))
    (:file "_package_DeadlockInfo" :depends-on ("_package"))
    (:file "LDRCompletion" :depends-on ("_package_LDRCompletion"))
    (:file "_package_LDRCompletion" :depends-on ("_package"))
    (:file "LDRInfo" :depends-on ("_package_LDRInfo"))
    (:file "_package_LDRInfo" :depends-on ("_package"))
    (:file "LDRInfoArray" :depends-on ("_package_LDRInfoArray"))
    (:file "_package_LDRInfoArray" :depends-on ("_package"))
    (:file "ResolutionStatus" :depends-on ("_package_ResolutionStatus"))
    (:file "_package_ResolutionStatus" :depends-on ("_package"))
    (:file "ResolvedPaths" :depends-on ("_package_ResolvedPaths"))
    (:file "_package_ResolvedPaths" :depends-on ("_package"))
    (:file "UAVPath" :depends-on ("_package_UAVPath"))
    (:file "_package_UAVPath" :depends-on ("_package"))
  ))