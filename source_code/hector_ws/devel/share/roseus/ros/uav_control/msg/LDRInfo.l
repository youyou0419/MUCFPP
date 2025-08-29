;; Auto-generated. Do not edit!


(when (boundp 'uav_control::LDRInfo)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'LDRInfo (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::LDRINFO")
  (make-package "UAV_CONTROL::LDRINFO"))

(in-package "ROS")
;;//! \htmlinclude LDRInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uav_control::LDRInfo
  :super ros::object
  :slots (_uav_ids _positions _min_x _max_x _min_y _max_y ))

(defmethod uav_control::LDRInfo
  (:init
   (&key
    ((:uav_ids __uav_ids) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:positions __positions) ())
    ((:min_x __min_x) 0.0)
    ((:max_x __max_x) 0.0)
    ((:min_y __min_y) 0.0)
    ((:max_y __max_y) 0.0)
    )
   (send-super :init)
   (setq _uav_ids __uav_ids)
   (setq _positions __positions)
   (setq _min_x (float __min_x))
   (setq _max_x (float __max_x))
   (setq _min_y (float __min_y))
   (setq _max_y (float __max_y))
   self)
  (:uav_ids
   (&optional __uav_ids)
   (if __uav_ids (setq _uav_ids __uav_ids)) _uav_ids)
  (:positions
   (&rest __positions)
   (if (keywordp (car __positions))
       (send* _positions __positions)
     (progn
       (if __positions (setq _positions (car __positions)))
       _positions)))
  (:min_x
   (&optional __min_x)
   (if __min_x (setq _min_x __min_x)) _min_x)
  (:max_x
   (&optional __max_x)
   (if __max_x (setq _max_x __max_x)) _max_x)
  (:min_y
   (&optional __min_y)
   (if __min_y (setq _min_y __min_y)) _min_y)
  (:max_y
   (&optional __max_y)
   (if __max_y (setq _max_y __max_y)) _max_y)
  (:serialization-length
   ()
   (+
    ;; string[] _uav_ids
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _uav_ids)) 4
    ;; geometry_msgs/Point[] _positions
    (apply #'+ (send-all _positions :serialization-length)) 4
    ;; float64 _min_x
    8
    ;; float64 _max_x
    8
    ;; float64 _min_y
    8
    ;; float64 _max_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _uav_ids
     (write-long (length _uav_ids) s)
     (dolist (elem _uav_ids)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Point[] _positions
     (write-long (length _positions) s)
     (dolist (elem _positions)
       (send elem :serialize s)
       )
     ;; float64 _min_x
       (sys::poke _min_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_x
       (sys::poke _max_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _min_y
       (sys::poke _min_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_y
       (sys::poke _max_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _uav_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _uav_ids (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _uav_ids i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Point[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _positions)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _min_x
     (setq _min_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_x
     (setq _max_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _min_y
     (setq _min_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_y
     (setq _max_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uav_control::LDRInfo :md5sum-) "7f8c645ab0d7e706bc2196262140227e")
(setf (get uav_control::LDRInfo :datatype-) "uav_control/LDRInfo")
(setf (get uav_control::LDRInfo :definition-)
      "string[] uav_ids
geometry_msgs/Point[] positions
float64 min_x
float64 max_x
float64 min_y
float64 max_y
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uav_control/LDRInfo "7f8c645ab0d7e706bc2196262140227e")


