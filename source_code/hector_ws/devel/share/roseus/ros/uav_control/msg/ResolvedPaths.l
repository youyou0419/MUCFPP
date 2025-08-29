;; Auto-generated. Do not edit!


(when (boundp 'uav_control::ResolvedPaths)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'ResolvedPaths (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::RESOLVEDPATHS")
  (make-package "UAV_CONTROL::RESOLVEDPATHS"))

(in-package "ROS")
;;//! \htmlinclude ResolvedPaths.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uav_control::ResolvedPaths
  :super ros::object
  :slots (_header _ldr_id _uav_ids _uav_paths ))

(defmethod uav_control::ResolvedPaths
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ldr_id __ldr_id) 0)
    ((:uav_ids __uav_ids) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:uav_paths __uav_paths) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _ldr_id (round __ldr_id))
   (setq _uav_ids __uav_ids)
   (setq _uav_paths __uav_paths)
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
  (:uav_paths
   (&rest __uav_paths)
   (if (keywordp (car __uav_paths))
       (send* _uav_paths __uav_paths)
     (progn
       (if __uav_paths (setq _uav_paths (car __uav_paths)))
       _uav_paths)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _ldr_id
    4
    ;; string[] _uav_ids
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _uav_ids)) 4
    ;; uav_control/UAVPath[] _uav_paths
    (apply #'+ (send-all _uav_paths :serialization-length)) 4
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
     ;; uav_control/UAVPath[] _uav_paths
     (write-long (length _uav_paths) s)
     (dolist (elem _uav_paths)
       (send elem :serialize s)
       )
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
   ;; uav_control/UAVPath[] _uav_paths
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _uav_paths (let (r) (dotimes (i n) (push (instance uav_control::UAVPath :init) r)) r))
     (dolist (elem- _uav_paths)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get uav_control::ResolvedPaths :md5sum-) "66074861de72e89556e85750791c9c7b")
(setf (get uav_control::ResolvedPaths :datatype-) "uav_control/ResolvedPaths")
(setf (get uav_control::ResolvedPaths :definition-)
      "# 自定义消息类型：LDR解决后的路径信息
# 文件路径: uav_control/msg/ResolvedPaths.msg

Header header
int32 ldr_id
string[] uav_ids
UAVPath[] uav_paths
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

================================================================================
MSG: uav_control/UAVPath
# 单个无人机的路径信息
# 文件路径: uav_control/msg/UAVPath.msg

string uav_id
geometry_msgs/Point[] path_points
int32[] time_indices
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uav_control/ResolvedPaths "66074861de72e89556e85750791c9c7b")


