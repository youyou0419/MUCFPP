; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude UAVPath.msg.html

(cl:defclass <UAVPath> (roslisp-msg-protocol:ros-message)
  ((uav_id
    :reader uav_id
    :initarg :uav_id
    :type cl:string
    :initform "")
   (path_points
    :reader path_points
    :initarg :path_points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (time_indices
    :reader time_indices
    :initarg :time_indices
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass UAVPath (<UAVPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UAVPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UAVPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<UAVPath> is deprecated: use uav_control-msg:UAVPath instead.")))

(cl:ensure-generic-function 'uav_id-val :lambda-list '(m))
(cl:defmethod uav_id-val ((m <UAVPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:uav_id-val is deprecated.  Use uav_control-msg:uav_id instead.")
  (uav_id m))

(cl:ensure-generic-function 'path_points-val :lambda-list '(m))
(cl:defmethod path_points-val ((m <UAVPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:path_points-val is deprecated.  Use uav_control-msg:path_points instead.")
  (path_points m))

(cl:ensure-generic-function 'time_indices-val :lambda-list '(m))
(cl:defmethod time_indices-val ((m <UAVPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:time_indices-val is deprecated.  Use uav_control-msg:time_indices instead.")
  (time_indices m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UAVPath>) ostream)
  "Serializes a message object of type '<UAVPath>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'uav_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'uav_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'time_indices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'time_indices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UAVPath>) istream)
  "Deserializes a message object of type '<UAVPath>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uav_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'uav_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'time_indices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'time_indices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UAVPath>)))
  "Returns string type for a message object of type '<UAVPath>"
  "uav_control/UAVPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UAVPath)))
  "Returns string type for a message object of type 'UAVPath"
  "uav_control/UAVPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UAVPath>)))
  "Returns md5sum for a message object of type '<UAVPath>"
  "11d1db57573332d6454c162bb2bcf71b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UAVPath)))
  "Returns md5sum for a message object of type 'UAVPath"
  "11d1db57573332d6454c162bb2bcf71b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UAVPath>)))
  "Returns full string definition for message of type '<UAVPath>"
  (cl:format cl:nil "# 单个无人机的路径信息~%# 文件路径: uav_control/msg/UAVPath.msg~%~%string uav_id~%geometry_msgs/Point[] path_points~%int32[] time_indices~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UAVPath)))
  "Returns full string definition for message of type 'UAVPath"
  (cl:format cl:nil "# 单个无人机的路径信息~%# 文件路径: uav_control/msg/UAVPath.msg~%~%string uav_id~%geometry_msgs/Point[] path_points~%int32[] time_indices~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UAVPath>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'uav_id))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'time_indices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UAVPath>))
  "Converts a ROS message object to a list"
  (cl:list 'UAVPath
    (cl:cons ':uav_id (uav_id msg))
    (cl:cons ':path_points (path_points msg))
    (cl:cons ':time_indices (time_indices msg))
))
