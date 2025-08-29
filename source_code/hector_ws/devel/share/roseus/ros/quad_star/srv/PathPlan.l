;; Auto-generated. Do not edit!


(when (boundp 'quad_star::PathPlan)
  (if (not (find-package "QUAD_STAR"))
    (make-package "QUAD_STAR"))
  (shadow 'PathPlan (find-package "QUAD_STAR")))
(unless (find-package "QUAD_STAR::PATHPLAN")
  (make-package "QUAD_STAR::PATHPLAN"))
(unless (find-package "QUAD_STAR::PATHPLANREQUEST")
  (make-package "QUAD_STAR::PATHPLANREQUEST"))
(unless (find-package "QUAD_STAR::PATHPLANRESPONSE")
  (make-package "QUAD_STAR::PATHPLANRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quad_star::PathPlanRequest
  :super ros::object
  :slots (_start_x _start_y _goal_x _goal_y ))

(defmethod quad_star::PathPlanRequest
  (:init
   (&key
    ((:start_x __start_x) 0.0)
    ((:start_y __start_y) 0.0)
    ((:goal_x __goal_x) 0.0)
    ((:goal_y __goal_y) 0.0)
    )
   (send-super :init)
   (setq _start_x (float __start_x))
   (setq _start_y (float __start_y))
   (setq _goal_x (float __goal_x))
   (setq _goal_y (float __goal_y))
   self)
  (:start_x
   (&optional __start_x)
   (if __start_x (setq _start_x __start_x)) _start_x)
  (:start_y
   (&optional __start_y)
   (if __start_y (setq _start_y __start_y)) _start_y)
  (:goal_x
   (&optional __goal_x)
   (if __goal_x (setq _goal_x __goal_x)) _goal_x)
  (:goal_y
   (&optional __goal_y)
   (if __goal_y (setq _goal_y __goal_y)) _goal_y)
  (:serialization-length
   ()
   (+
    ;; float32 _start_x
    4
    ;; float32 _start_y
    4
    ;; float32 _goal_x
    4
    ;; float32 _goal_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _start_x
       (sys::poke _start_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _start_y
       (sys::poke _start_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _goal_x
       (sys::poke _goal_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _goal_y
       (sys::poke _goal_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _start_x
     (setq _start_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _start_y
     (setq _start_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _goal_x
     (setq _goal_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _goal_y
     (setq _goal_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass quad_star::PathPlanResponse
  :super ros::object
  :slots (_success _message _path ))

(defmethod quad_star::PathPlanResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:path __path) ())
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _path __path)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; geometry_msgs/Point[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; geometry_msgs/Point[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass quad_star::PathPlan
  :super ros::object
  :slots ())

(setf (get quad_star::PathPlan :md5sum-) "c9c9ee362512ec5be6042c67e74ac40f")
(setf (get quad_star::PathPlan :datatype-) "quad_star/PathPlan")
(setf (get quad_star::PathPlan :request) quad_star::PathPlanRequest)
(setf (get quad_star::PathPlan :response) quad_star::PathPlanResponse)

(defmethod quad_star::PathPlanRequest
  (:response () (instance quad_star::PathPlanResponse :init)))

(setf (get quad_star::PathPlanRequest :md5sum-) "c9c9ee362512ec5be6042c67e74ac40f")
(setf (get quad_star::PathPlanRequest :datatype-) "quad_star/PathPlanRequest")
(setf (get quad_star::PathPlanRequest :definition-)
      "float32 start_x
float32 start_y
float32 goal_x
float32 goal_y
---
bool success
string message
geometry_msgs/Point[] path

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get quad_star::PathPlanResponse :md5sum-) "c9c9ee362512ec5be6042c67e74ac40f")
(setf (get quad_star::PathPlanResponse :datatype-) "quad_star/PathPlanResponse")
(setf (get quad_star::PathPlanResponse :definition-)
      "float32 start_x
float32 start_y
float32 goal_x
float32 goal_y
---
bool success
string message
geometry_msgs/Point[] path

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :quad_star/PathPlan "c9c9ee362512ec5be6042c67e74ac40f")


