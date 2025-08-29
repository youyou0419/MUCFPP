;; Auto-generated. Do not edit!


(when (boundp 'uav_control::UAVPath)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'UAVPath (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::UAVPATH")
  (make-package "UAV_CONTROL::UAVPATH"))

(in-package "ROS")
;;//! \htmlinclude UAVPath.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uav_control::UAVPath
  :super ros::object
  :slots (_uav_id _path_points _time_indices ))

(defmethod uav_control::UAVPath
  (:init
   (&key
    ((:uav_id __uav_id) "")
    ((:path_points __path_points) ())
    ((:time_indices __time_indices) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _uav_id (string __uav_id))
   (setq _path_points __path_points)
   (setq _time_indices __time_indices)
   self)
  (:uav_id
   (&optional __uav_id)
   (if __uav_id (setq _uav_id __uav_id)) _uav_id)
  (:path_points
   (&rest __path_points)
   (if (keywordp (car __path_points))
       (send* _path_points __path_points)
     (progn
       (if __path_points (setq _path_points (car __path_points)))
       _path_points)))
  (:time_indices
   (&optional __time_indices)
   (if __time_indices (setq _time_indices __time_indices)) _time_indices)
  (:serialization-length
   ()
   (+
    ;; string _uav_id
    4 (length _uav_id)
    ;; geometry_msgs/Point[] _path_points
    (apply #'+ (send-all _path_points :serialization-length)) 4
    ;; int32[] _time_indices
    (* 4    (length _time_indices)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _uav_id
       (write-long (length _uav_id) s) (princ _uav_id s)
     ;; geometry_msgs/Point[] _path_points
     (write-long (length _path_points) s)
     (dolist (elem _path_points)
       (send elem :serialize s)
       )
     ;; int32[] _time_indices
     (write-long (length _time_indices) s)
     (dotimes (i (length _time_indices))
       (write-long (elt _time_indices i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _uav_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _uav_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point[] _path_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _path_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32[] _time_indices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time_indices (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _time_indices i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get uav_control::UAVPath :md5sum-) "11d1db57573332d6454c162bb2bcf71b")
(setf (get uav_control::UAVPath :datatype-) "uav_control/UAVPath")
(setf (get uav_control::UAVPath :definition-)
      "# 单个无人机的路径信息
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



(provide :uav_control/UAVPath "11d1db57573332d6454c162bb2bcf71b")


