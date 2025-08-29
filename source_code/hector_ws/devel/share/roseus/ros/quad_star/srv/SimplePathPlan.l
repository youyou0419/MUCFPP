;; Auto-generated. Do not edit!


(when (boundp 'quad_star::SimplePathPlan)
  (if (not (find-package "QUAD_STAR"))
    (make-package "QUAD_STAR"))
  (shadow 'SimplePathPlan (find-package "QUAD_STAR")))
(unless (find-package "QUAD_STAR::SIMPLEPATHPLAN")
  (make-package "QUAD_STAR::SIMPLEPATHPLAN"))
(unless (find-package "QUAD_STAR::SIMPLEPATHPLANREQUEST")
  (make-package "QUAD_STAR::SIMPLEPATHPLANREQUEST"))
(unless (find-package "QUAD_STAR::SIMPLEPATHPLANRESPONSE")
  (make-package "QUAD_STAR::SIMPLEPATHPLANRESPONSE"))

(in-package "ROS")





(defclass quad_star::SimplePathPlanRequest
  :super ros::object
  :slots (_start_x _start_y _goal_x _goal_y ))

(defmethod quad_star::SimplePathPlanRequest
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

(defclass quad_star::SimplePathPlanResponse
  :super ros::object
  :slots (_success _message ))

(defmethod quad_star::SimplePathPlanResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass quad_star::SimplePathPlan
  :super ros::object
  :slots ())

(setf (get quad_star::SimplePathPlan :md5sum-) "cc0fa8f370b2645f29bc836e78ed2538")
(setf (get quad_star::SimplePathPlan :datatype-) "quad_star/SimplePathPlan")
(setf (get quad_star::SimplePathPlan :request) quad_star::SimplePathPlanRequest)
(setf (get quad_star::SimplePathPlan :response) quad_star::SimplePathPlanResponse)

(defmethod quad_star::SimplePathPlanRequest
  (:response () (instance quad_star::SimplePathPlanResponse :init)))

(setf (get quad_star::SimplePathPlanRequest :md5sum-) "cc0fa8f370b2645f29bc836e78ed2538")
(setf (get quad_star::SimplePathPlanRequest :datatype-) "quad_star/SimplePathPlanRequest")
(setf (get quad_star::SimplePathPlanRequest :definition-)
      "# 请求消息
float32 start_x
float32 start_y
float32 goal_x
float32 goal_y
---
# 响应消息
bool success
string message
")

(setf (get quad_star::SimplePathPlanResponse :md5sum-) "cc0fa8f370b2645f29bc836e78ed2538")
(setf (get quad_star::SimplePathPlanResponse :datatype-) "quad_star/SimplePathPlanResponse")
(setf (get quad_star::SimplePathPlanResponse :definition-)
      "# 请求消息
float32 start_x
float32 start_y
float32 goal_x
float32 goal_y
---
# 响应消息
bool success
string message
")



(provide :quad_star/SimplePathPlan "cc0fa8f370b2645f29bc836e78ed2538")


