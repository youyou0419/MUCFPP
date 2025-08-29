;; Auto-generated. Do not edit!


(when (boundp 'uav_control::ResolutionStatus)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'ResolutionStatus (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::RESOLUTIONSTATUS")
  (make-package "UAV_CONTROL::RESOLUTIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ResolutionStatus.msg.html


(defclass uav_control::ResolutionStatus
  :super ros::object
  :slots (_ldr_id _uav_ids _in_resolution _aabb_min_x _aabb_max_x _aabb_min_y _aabb_max_y ))

(defmethod uav_control::ResolutionStatus
  (:init
   (&key
    ((:ldr_id __ldr_id) 0)
    ((:uav_ids __uav_ids) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:in_resolution __in_resolution) nil)
    ((:aabb_min_x __aabb_min_x) 0.0)
    ((:aabb_max_x __aabb_max_x) 0.0)
    ((:aabb_min_y __aabb_min_y) 0.0)
    ((:aabb_max_y __aabb_max_y) 0.0)
    )
   (send-super :init)
   (setq _ldr_id (round __ldr_id))
   (setq _uav_ids __uav_ids)
   (setq _in_resolution __in_resolution)
   (setq _aabb_min_x (float __aabb_min_x))
   (setq _aabb_max_x (float __aabb_max_x))
   (setq _aabb_min_y (float __aabb_min_y))
   (setq _aabb_max_y (float __aabb_max_y))
   self)
  (:ldr_id
   (&optional __ldr_id)
   (if __ldr_id (setq _ldr_id __ldr_id)) _ldr_id)
  (:uav_ids
   (&optional __uav_ids)
   (if __uav_ids (setq _uav_ids __uav_ids)) _uav_ids)
  (:in_resolution
   (&optional (__in_resolution :null))
   (if (not (eq __in_resolution :null)) (setq _in_resolution __in_resolution)) _in_resolution)
  (:aabb_min_x
   (&optional __aabb_min_x)
   (if __aabb_min_x (setq _aabb_min_x __aabb_min_x)) _aabb_min_x)
  (:aabb_max_x
   (&optional __aabb_max_x)
   (if __aabb_max_x (setq _aabb_max_x __aabb_max_x)) _aabb_max_x)
  (:aabb_min_y
   (&optional __aabb_min_y)
   (if __aabb_min_y (setq _aabb_min_y __aabb_min_y)) _aabb_min_y)
  (:aabb_max_y
   (&optional __aabb_max_y)
   (if __aabb_max_y (setq _aabb_max_y __aabb_max_y)) _aabb_max_y)
  (:serialization-length
   ()
   (+
    ;; int32 _ldr_id
    4
    ;; string[] _uav_ids
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _uav_ids)) 4
    ;; bool _in_resolution
    1
    ;; float64 _aabb_min_x
    8
    ;; float64 _aabb_max_x
    8
    ;; float64 _aabb_min_y
    8
    ;; float64 _aabb_max_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _ldr_id
       (write-long _ldr_id s)
     ;; string[] _uav_ids
     (write-long (length _uav_ids) s)
     (dolist (elem _uav_ids)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool _in_resolution
       (if _in_resolution (write-byte -1 s) (write-byte 0 s))
     ;; float64 _aabb_min_x
       (sys::poke _aabb_min_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _aabb_max_x
       (sys::poke _aabb_max_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _aabb_min_y
       (sys::poke _aabb_min_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _aabb_max_y
       (sys::poke _aabb_max_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _ldr_id
     (setq _ldr_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _uav_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _uav_ids (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _uav_ids i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool _in_resolution
     (setq _in_resolution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _aabb_min_x
     (setq _aabb_min_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _aabb_max_x
     (setq _aabb_max_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _aabb_min_y
     (setq _aabb_min_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _aabb_max_y
     (setq _aabb_max_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uav_control::ResolutionStatus :md5sum-) "68d9e3003f30b6e4832b7d80035318cd")
(setf (get uav_control::ResolutionStatus :datatype-) "uav_control/ResolutionStatus")
(setf (get uav_control::ResolutionStatus :definition-)
      "# ResolutionStatus.msg
# LDR Resolution状态消息定义


# LDR标识符
int32 ldr_id

# 涉及的无人机ID列表
string[] uav_ids

# Resolution状态：true表示进入Resolution阶段，false表示退出
bool in_resolution


# LDR区域边界信息
float64 aabb_min_x
float64 aabb_max_x  
float64 aabb_min_y
float64 aabb_max_y
")



(provide :uav_control/ResolutionStatus "68d9e3003f30b6e4832b7d80035318cd")


