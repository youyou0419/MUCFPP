;; Auto-generated. Do not edit!


(when (boundp 'uav_control::LDRInfoArray)
  (if (not (find-package "UAV_CONTROL"))
    (make-package "UAV_CONTROL"))
  (shadow 'LDRInfoArray (find-package "UAV_CONTROL")))
(unless (find-package "UAV_CONTROL::LDRINFOARRAY")
  (make-package "UAV_CONTROL::LDRINFOARRAY"))

(in-package "ROS")
;;//! \htmlinclude LDRInfoArray.msg.html


(defclass uav_control::LDRInfoArray
  :super ros::object
  :slots (_ldr_infos ))

(defmethod uav_control::LDRInfoArray
  (:init
   (&key
    ((:ldr_infos __ldr_infos) ())
    )
   (send-super :init)
   (setq _ldr_infos __ldr_infos)
   self)
  (:ldr_infos
   (&rest __ldr_infos)
   (if (keywordp (car __ldr_infos))
       (send* _ldr_infos __ldr_infos)
     (progn
       (if __ldr_infos (setq _ldr_infos (car __ldr_infos)))
       _ldr_infos)))
  (:serialization-length
   ()
   (+
    ;; uav_control/LDRInfo[] _ldr_infos
    (apply #'+ (send-all _ldr_infos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uav_control/LDRInfo[] _ldr_infos
     (write-long (length _ldr_infos) s)
     (dolist (elem _ldr_infos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uav_control/LDRInfo[] _ldr_infos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ldr_infos (let (r) (dotimes (i n) (push (instance uav_control::LDRInfo :init) r)) r))
     (dolist (elem- _ldr_infos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get uav_control::LDRInfoArray :md5sum-) "9e49ba5991a82a17025c4ca6b278a4e1")
(setf (get uav_control::LDRInfoArray :datatype-) "uav_control/LDRInfoArray")
(setf (get uav_control::LDRInfoArray :definition-)
      "uav_control/LDRInfo[] ldr_infos
================================================================================
MSG: uav_control/LDRInfo
string[] uav_ids
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



(provide :uav_control/LDRInfoArray "9e49ba5991a82a17025c4ca6b278a4e1")


