; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude LDRInfo.msg.html

(cl:defclass <LDRInfo> (roslisp-msg-protocol:ros-message)
  ((uav_ids
    :reader uav_ids
    :initarg :uav_ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (positions
    :reader positions
    :initarg :positions
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (min_x
    :reader min_x
    :initarg :min_x
    :type cl:float
    :initform 0.0)
   (max_x
    :reader max_x
    :initarg :max_x
    :type cl:float
    :initform 0.0)
   (min_y
    :reader min_y
    :initarg :min_y
    :type cl:float
    :initform 0.0)
   (max_y
    :reader max_y
    :initarg :max_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass LDRInfo (<LDRInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LDRInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LDRInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<LDRInfo> is deprecated: use uav_control-msg:LDRInfo instead.")))

(cl:ensure-generic-function 'uav_ids-val :lambda-list '(m))
(cl:defmethod uav_ids-val ((m <LDRInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:uav_ids-val is deprecated.  Use uav_control-msg:uav_ids instead.")
  (uav_ids m))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <LDRInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:positions-val is deprecated.  Use uav_control-msg:positions instead.")
  (positions m))

(cl:ensure-generic-function 'min_x-val :lambda-list '(m))
(cl:defmethod min_x-val ((m <LDRInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:min_x-val is deprecated.  Use uav_control-msg:min_x instead.")
  (min_x m))

(cl:ensure-generic-function 'max_x-val :lambda-list '(m))
(cl:defmethod max_x-val ((m <LDRInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:max_x-val is deprecated.  Use uav_control-msg:max_x instead.")
  (max_x m))

(cl:ensure-generic-function 'min_y-val :lambda-list '(m))
(cl:defmethod min_y-val ((m <LDRInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:min_y-val is deprecated.  Use uav_control-msg:min_y instead.")
  (min_y m))

(cl:ensure-generic-function 'max_y-val :lambda-list '(m))
(cl:defmethod max_y-val ((m <LDRInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:max_y-val is deprecated.  Use uav_control-msg:max_y instead.")
  (max_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LDRInfo>) ostream)
  "Serializes a message object of type '<LDRInfo>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'uav_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'uav_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'positions))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LDRInfo>) istream)
  "Deserializes a message object of type '<LDRInfo>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'uav_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'uav_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LDRInfo>)))
  "Returns string type for a message object of type '<LDRInfo>"
  "uav_control/LDRInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LDRInfo)))
  "Returns string type for a message object of type 'LDRInfo"
  "uav_control/LDRInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LDRInfo>)))
  "Returns md5sum for a message object of type '<LDRInfo>"
  "7f8c645ab0d7e706bc2196262140227e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LDRInfo)))
  "Returns md5sum for a message object of type 'LDRInfo"
  "7f8c645ab0d7e706bc2196262140227e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LDRInfo>)))
  "Returns full string definition for message of type '<LDRInfo>"
  (cl:format cl:nil "string[] uav_ids~%geometry_msgs/Point[] positions~%float64 min_x~%float64 max_x~%float64 min_y~%float64 max_y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LDRInfo)))
  "Returns full string definition for message of type 'LDRInfo"
  (cl:format cl:nil "string[] uav_ids~%geometry_msgs/Point[] positions~%float64 min_x~%float64 max_x~%float64 min_y~%float64 max_y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LDRInfo>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uav_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LDRInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'LDRInfo
    (cl:cons ':uav_ids (uav_ids msg))
    (cl:cons ':positions (positions msg))
    (cl:cons ':min_x (min_x msg))
    (cl:cons ':max_x (max_x msg))
    (cl:cons ':min_y (min_y msg))
    (cl:cons ':max_y (max_y msg))
))
