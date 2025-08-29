; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude LDRCompletion.msg.html

(cl:defclass <LDRCompletion> (roslisp-msg-protocol:ros-message)
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
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass LDRCompletion (<LDRCompletion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LDRCompletion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LDRCompletion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<LDRCompletion> is deprecated: use uav_control-msg:LDRCompletion instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LDRCompletion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:header-val is deprecated.  Use uav_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ldr_id-val :lambda-list '(m))
(cl:defmethod ldr_id-val ((m <LDRCompletion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:ldr_id-val is deprecated.  Use uav_control-msg:ldr_id instead.")
  (ldr_id m))

(cl:ensure-generic-function 'uav_ids-val :lambda-list '(m))
(cl:defmethod uav_ids-val ((m <LDRCompletion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:uav_ids-val is deprecated.  Use uav_control-msg:uav_ids instead.")
  (uav_ids m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <LDRCompletion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:message-val is deprecated.  Use uav_control-msg:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LDRCompletion>) ostream)
  "Serializes a message object of type '<LDRCompletion>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LDRCompletion>) istream)
  "Deserializes a message object of type '<LDRCompletion>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LDRCompletion>)))
  "Returns string type for a message object of type '<LDRCompletion>"
  "uav_control/LDRCompletion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LDRCompletion)))
  "Returns string type for a message object of type 'LDRCompletion"
  "uav_control/LDRCompletion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LDRCompletion>)))
  "Returns md5sum for a message object of type '<LDRCompletion>"
  "b6ac87e9767065ecacf77bb14cf02fd4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LDRCompletion)))
  "Returns md5sum for a message object of type 'LDRCompletion"
  "b6ac87e9767065ecacf77bb14cf02fd4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LDRCompletion>)))
  "Returns full string definition for message of type '<LDRCompletion>"
  (cl:format cl:nil "# LDR完成广播消息~%Header header~%int32 ldr_id~%string[] uav_ids~%string message~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LDRCompletion)))
  "Returns full string definition for message of type 'LDRCompletion"
  (cl:format cl:nil "# LDR完成广播消息~%Header header~%int32 ldr_id~%string[] uav_ids~%string message~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LDRCompletion>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uav_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LDRCompletion>))
  "Converts a ROS message object to a list"
  (cl:list 'LDRCompletion
    (cl:cons ':header (header msg))
    (cl:cons ':ldr_id (ldr_id msg))
    (cl:cons ':uav_ids (uav_ids msg))
    (cl:cons ':message (message msg))
))
