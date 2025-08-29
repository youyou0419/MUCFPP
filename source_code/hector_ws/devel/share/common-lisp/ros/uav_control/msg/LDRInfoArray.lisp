; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude LDRInfoArray.msg.html

(cl:defclass <LDRInfoArray> (roslisp-msg-protocol:ros-message)
  ((ldr_infos
    :reader ldr_infos
    :initarg :ldr_infos
    :type (cl:vector uav_control-msg:LDRInfo)
   :initform (cl:make-array 0 :element-type 'uav_control-msg:LDRInfo :initial-element (cl:make-instance 'uav_control-msg:LDRInfo))))
)

(cl:defclass LDRInfoArray (<LDRInfoArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LDRInfoArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LDRInfoArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<LDRInfoArray> is deprecated: use uav_control-msg:LDRInfoArray instead.")))

(cl:ensure-generic-function 'ldr_infos-val :lambda-list '(m))
(cl:defmethod ldr_infos-val ((m <LDRInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:ldr_infos-val is deprecated.  Use uav_control-msg:ldr_infos instead.")
  (ldr_infos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LDRInfoArray>) ostream)
  "Serializes a message object of type '<LDRInfoArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ldr_infos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ldr_infos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LDRInfoArray>) istream)
  "Deserializes a message object of type '<LDRInfoArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ldr_infos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ldr_infos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'uav_control-msg:LDRInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LDRInfoArray>)))
  "Returns string type for a message object of type '<LDRInfoArray>"
  "uav_control/LDRInfoArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LDRInfoArray)))
  "Returns string type for a message object of type 'LDRInfoArray"
  "uav_control/LDRInfoArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LDRInfoArray>)))
  "Returns md5sum for a message object of type '<LDRInfoArray>"
  "9e49ba5991a82a17025c4ca6b278a4e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LDRInfoArray)))
  "Returns md5sum for a message object of type 'LDRInfoArray"
  "9e49ba5991a82a17025c4ca6b278a4e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LDRInfoArray>)))
  "Returns full string definition for message of type '<LDRInfoArray>"
  (cl:format cl:nil "uav_control/LDRInfo[] ldr_infos~%================================================================================~%MSG: uav_control/LDRInfo~%string[] uav_ids~%geometry_msgs/Point[] positions~%float64 min_x~%float64 max_x~%float64 min_y~%float64 max_y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LDRInfoArray)))
  "Returns full string definition for message of type 'LDRInfoArray"
  (cl:format cl:nil "uav_control/LDRInfo[] ldr_infos~%================================================================================~%MSG: uav_control/LDRInfo~%string[] uav_ids~%geometry_msgs/Point[] positions~%float64 min_x~%float64 max_x~%float64 min_y~%float64 max_y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LDRInfoArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ldr_infos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LDRInfoArray>))
  "Converts a ROS message object to a list"
  (cl:list 'LDRInfoArray
    (cl:cons ':ldr_infos (ldr_infos msg))
))
