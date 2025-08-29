;; Auto-generated. Do not edit!


(when (boundp 'orca_navigation::BLockCondition)
  (if (not (find-package "ORCA_NAVIGATION"))
    (make-package "ORCA_NAVIGATION"))
  (shadow 'BLockCondition (find-package "ORCA_NAVIGATION")))
(unless (find-package "ORCA_NAVIGATION::BLOCKCONDITION")
  (make-package "ORCA_NAVIGATION::BLOCKCONDITION"))

(in-package "ROS")
;;//! \htmlinclude BLockCondition.msg.html


(defclass orca_navigation::BLockCondition
  :super ros::object
  :slots (_blocker_id _blocked_id _stamp ))

(defmethod orca_navigation::BLockCondition
  (:init
   (&key
    ((:blocker_id __blocker_id) "")
    ((:blocked_id __blocked_id) "")
    ((:stamp __stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _blocker_id (string __blocker_id))
   (setq _blocked_id (string __blocked_id))
   (setq _stamp __stamp)
   self)
  (:blocker_id
   (&optional __blocker_id)
   (if __blocker_id (setq _blocker_id __blocker_id)) _blocker_id)
  (:blocked_id
   (&optional __blocked_id)
   (if __blocked_id (setq _blocked_id __blocked_id)) _blocked_id)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:serialization-length
   ()
   (+
    ;; string _blocker_id
    4 (length _blocker_id)
    ;; string _blocked_id
    4 (length _blocked_id)
    ;; time _stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _blocker_id
       (write-long (length _blocker_id) s) (princ _blocker_id s)
     ;; string _blocked_id
       (write-long (length _blocked_id) s) (princ _blocked_id s)
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _blocker_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _blocker_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _blocked_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _blocked_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get orca_navigation::BLockCondition :md5sum-) "159eaf14713728dcfaf35256e7ad2445")
(setf (get orca_navigation::BLockCondition :datatype-) "orca_navigation/BLockCondition")
(setf (get orca_navigation::BLockCondition :definition-)
      "string blocker_id
string blocked_id
time stamp
")



(provide :orca_navigation/BLockCondition "159eaf14713728dcfaf35256e7ad2445")


