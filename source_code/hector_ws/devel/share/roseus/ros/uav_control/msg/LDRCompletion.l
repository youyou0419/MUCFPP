;; Auto-generated. Do not edit!


(when (boundp 'uav_control::LDRCompletion)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'LDRCompletion (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::LDRCOMPLETION")
  (make-package "UAV_CONTROL::LDRCOMPLETION"))

(in-package "ROS")
;;//! \htmlinclude LDRCompletion.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uav_control::LDRCompletion
  :super ros::object
  :slots (_header _ldr_id _uav_ids _message ))

(defmethod uav_control::LDRCompletion
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ldr_id __ldr_id) 0)
    ((:uav_ids __uav_ids) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:message __message) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _ldr_id (round __ldr_id))
   (setq _uav_ids __uav_ids)
   (setq _message (string __message))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ldr_id
   (&optional __ldr_id)
   (if __ldr_id (setq _ldr_id __ldr_id)) _ldr_id)
  (:uav_ids
   (&optional __uav_ids)
   (if __uav_ids (setq _uav_ids __uav_ids)) _uav_ids)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _ldr_id
    4
    ;; string[] _uav_ids
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _uav_ids)) 4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _ldr_id
       (write-long _ldr_id s)
     ;; string[] _uav_ids
     (write-long (length _uav_ids) s)
     (dolist (elem _uav_ids)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _ldr_id
     (setq _ldr_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _uav_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _uav_ids (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _uav_ids i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get uav_control::LDRCompletion :md5sum-) "b6ac87e9767065ecacf77bb14cf02fd4")
(setf (get uav_control::LDRCompletion :datatype-) "uav_control/LDRCompletion")
(setf (get uav_control::LDRCompletion :definition-)
      "# LDR完成广播消息
Header header
int32 ldr_id
string[] uav_ids
string message

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :uav_control/LDRCompletion "b6ac87e9767065ecacf77bb14cf02fd4")


