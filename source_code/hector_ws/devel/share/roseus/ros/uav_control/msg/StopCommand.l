;; Auto-generated. Do not edit!


(when (boundp 'uav_control::StopCommand)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'StopCommand (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::STOPCOMMAND")
  (make-package "UAV_CONTROL::STOPCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude StopCommand.msg.html


(defclass uav_control::StopCommand
  :super ros::object
  :slots (_stop _blocker_id ))

(defmethod uav_control::StopCommand
  (:init
   (&key
    ((:stop __stop) nil)
    ((:blocker_id __blocker_id) "")
    )
   (send-super :init)
   (setq _stop __stop)
   (setq _blocker_id (string __blocker_id))
   self)
  (:stop
   (&optional (__stop :null))
   (if (not (eq __stop :null)) (setq _stop __stop)) _stop)
  (:blocker_id
   (&optional __blocker_id)
   (if __blocker_id (setq _blocker_id __blocker_id)) _blocker_id)
  (:serialization-length
   ()
   (+
    ;; bool _stop
    1
    ;; string _blocker_id
    4 (length _blocker_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _stop
       (if _stop (write-byte -1 s) (write-byte 0 s))
     ;; string _blocker_id
       (write-long (length _blocker_id) s) (princ _blocker_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _stop
     (setq _stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _blocker_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _blocker_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get uav_control::StopCommand :md5sum-) "1ee931530b17c39eb5a3b97eb6933cbd")
(setf (get uav_control::StopCommand :datatype-) "uav_control/StopCommand")
(setf (get uav_control::StopCommand :definition-)
      "bool stop
string blocker_id  # 导致死锁的无人机ID
")



(provide :uav_control/StopCommand "1ee931530b17c39eb5a3b97eb6933cbd")


