; Auto-generated. Do not edit!


(cl:in-package uav_control-srv)


;//! \htmlinclude PathTracking-request.msg.html

(cl:defclass <PathTracking-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ldr_id
    :reader ldr_id
    :initarg :ldr_id
    :type cl:integer
    :initform 0)
   (uav_ids
    :reader uav_ids
    :initarg :uav_ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (uav_paths
    :reader uav_paths
    :initarg :uav_paths
    :type (cl:vector uav_control-msg:UAVPath)
   :initform (cl:make-array 0 :element-type 'uav_control-msg:UAVPath :initial-element (cl:make-instance 'uav_control-msg:UAVPath))))
)

(cl:defclass PathTracking-request (<PathTracking-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathTracking-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathTracking-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-srv:<PathTracking-request> is deprecated: use uav_control-srv:PathTracking-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PathTracking-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:header-val is deprecated.  Use uav_control-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'ldr_id-val :lambda-list '(m))
(cl:defmethod ldr_id-val ((m <PathTracking-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:ldr_id-val is deprecated.  Use uav_control-srv:ldr_id instead.")
  (ldr_id m))

(cl:ensure-generic-function 'uav_ids-val :lambda-list '(m))
(cl:defmethod uav_ids-val ((m <PathTracking-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:uav_ids-val is deprecated.  Use uav_control-srv:uav_ids instead.")
  (uav_ids m))

(cl:ensure-generic-function 'uav_paths-val :lambda-list '(m))
(cl:defmethod uav_paths-val ((m <PathTracking-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:uav_paths-val is deprecated.  Use uav_control-srv:uav_paths instead.")
  (uav_paths m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathTracking-request>) ostream)
  "Serializes a message object of type '<PathTracking-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'ldr_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'uav_paths))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'uav_paths))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathTracking-request>) istream)
  "Deserializes a message object of type '<PathTracking-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ldr_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  (cl:setf (cl:slot-value msg 'uav_paths) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'uav_paths)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'uav_control-msg:UAVPath))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathTracking-request>)))
  "Returns string type for a service object of type '<PathTracking-request>"
  "uav_control/PathTrackingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathTracking-request)))
  "Returns string type for a service object of type 'PathTracking-request"
  "uav_control/PathTrackingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathTracking-request>)))
  "Returns md5sum for a message object of type '<PathTracking-request>"
  "0ff8a4c9efe935b06b349c3681633f16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathTracking-request)))
  "Returns md5sum for a message object of type 'PathTracking-request"
  "0ff8a4c9efe935b06b349c3681633f16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathTracking-request>)))
  "Returns full string definition for message of type '<PathTracking-request>"
  (cl:format cl:nil "# PathTracking.srv~%# 路径跟踪服务定义~%~%# Request - 发送路径信息~%std_msgs/Header header~%int32 ldr_id~%string[] uav_ids~%uav_control/UAVPath[] uav_paths~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: uav_control/UAVPath~%# 单个无人机的路径信息~%# 文件路径: uav_control/msg/UAVPath.msg~%~%string uav_id~%geometry_msgs/Point[] path_points~%int32[] time_indices~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathTracking-request)))
  "Returns full string definition for message of type 'PathTracking-request"
  (cl:format cl:nil "# PathTracking.srv~%# 路径跟踪服务定义~%~%# Request - 发送路径信息~%std_msgs/Header header~%int32 ldr_id~%string[] uav_ids~%uav_control/UAVPath[] uav_paths~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: uav_control/UAVPath~%# 单个无人机的路径信息~%# 文件路径: uav_control/msg/UAVPath.msg~%~%string uav_id~%geometry_msgs/Point[] path_points~%int32[] time_indices~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathTracking-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uav_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uav_paths) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathTracking-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PathTracking-request
    (cl:cons ':header (header msg))
    (cl:cons ':ldr_id (ldr_id msg))
    (cl:cons ':uav_ids (uav_ids msg))
    (cl:cons ':uav_paths (uav_paths msg))
))
;//! \htmlinclude PathTracking-response.msg.html

(cl:defclass <PathTracking-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (execution_time
    :reader execution_time
    :initarg :execution_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass PathTracking-response (<PathTracking-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathTracking-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathTracking-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-srv:<PathTracking-response> is deprecated: use uav_control-srv:PathTracking-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PathTracking-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:success-val is deprecated.  Use uav_control-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PathTracking-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:message-val is deprecated.  Use uav_control-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'execution_time-val :lambda-list '(m))
(cl:defmethod execution_time-val ((m <PathTracking-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:execution_time-val is deprecated.  Use uav_control-srv:execution_time instead.")
  (execution_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathTracking-response>) ostream)
  "Serializes a message object of type '<PathTracking-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'execution_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathTracking-response>) istream)
  "Deserializes a message object of type '<PathTracking-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'execution_time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathTracking-response>)))
  "Returns string type for a service object of type '<PathTracking-response>"
  "uav_control/PathTrackingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathTracking-response)))
  "Returns string type for a service object of type 'PathTracking-response"
  "uav_control/PathTrackingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathTracking-response>)))
  "Returns md5sum for a message object of type '<PathTracking-response>"
  "0ff8a4c9efe935b06b349c3681633f16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathTracking-response)))
  "Returns md5sum for a message object of type 'PathTracking-response"
  "0ff8a4c9efe935b06b349c3681633f16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathTracking-response>)))
  "Returns full string definition for message of type '<PathTracking-response>"
  (cl:format cl:nil "~%# Response - 返回执行结果~%bool success~%string message~%float64 execution_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathTracking-response)))
  "Returns full string definition for message of type 'PathTracking-response"
  (cl:format cl:nil "~%# Response - 返回执行结果~%bool success~%string message~%float64 execution_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathTracking-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathTracking-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PathTracking-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':execution_time (execution_time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PathTracking)))
  'PathTracking-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PathTracking)))
  'PathTracking-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathTracking)))
  "Returns string type for a service object of type '<PathTracking>"
  "uav_control/PathTracking")