
(cl:in-package :asdf)

(defsystem "orca_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BLockCondition" :depends-on ("_package_BLockCondition"))
    (:file "_package_BLockCondition" :depends-on ("_package"))
  ))