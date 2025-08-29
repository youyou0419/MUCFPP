;; Auto-generated. Do not edit!


(when (boundp 'uav_control::DeadlockInfo)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'DeadlockInfo (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::DEADLOCKINFO")
  (make-package "UAV_CONTROL::DEADLOCKINFO"))

(in-package "ROS")
;;//! \htmlinclude DeadlockInfo.msg.html


(defclass uav_control::DeadlockInfo
  :super ros::object
  :slots (_blocker_id _blocked_id _is_bilateral ))

(defmethod uav_control::DeadlockInfo
  (:init
   (&key
    ((:blocker_id __blocker_id) "")
    ((:blocked_id __blocked_id) "")
    ((:is_bilateral __is_bilateral) nil)
    )
   (send-super :init)
   (setq _blocker_id (string __blocker_id))
   (setq _blocked_id (string __blocked_id))
   (setq _is_bilateral __is_bilateral)
   self)
  (:blocker_id
   (&optional __blocker_id)
   (if __blocker_id (setq _blocker_id __blocker_id)) _blocker_id)
  (:blocked_id
   (&optional __blocked_id)
   (if __blocked_id (setq _blocked_id __blocked_id)) _blocked_id)
  (:is_bilateral
   (&optional (__is_bilateral :null))
   (if (not (eq __is_bilateral :null)) (setq _is_bilateral __is_bilateral)) _is_bilateral)
  (:serialization-length
   ()
   (+
    ;; string _blocker_id
    4 (length _blocker_id)
    ;; string _blocked_id
    4 (length _blocked_id)
    ;; bool _is_bilateral
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _blocker_id
       (write-long (length _blocker_id) s) (princ _blocker_id s)
     ;; string _blocked_id
       (write-long (length _blocked_id) s) (princ _blocked_id s)
     ;; bool _is_bilateral
       (if _is_bilateral (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _blocker_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _blocker_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _blocked_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _blocked_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_bilateral
     (setq _is_bilateral (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get uav_control::DeadlockInfo :md5sum-) "38f3084008c30c3c1c815f245eba16b8")
(setf (get uav_control::DeadlockInfo :datatype-) "uav_control/DeadlockInfo")
(setf (get uav_control::DeadlockInfo :definition-)
      "string blocker_id    # 造成阻塞的无人机ID
string blocked_id   # 被阻塞的无人机ID
bool is_bilateral   # true表示双向死锁，false表示单向阻塞
")



(provide :uav_control/DeadlockInfo "38f3084008c30c3c1c815f245eba16b8")


