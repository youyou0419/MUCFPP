; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude DeadlockInfo.msg.html

(cl:defclass <DeadlockInfo> (roslisp-msg-protocol:ros-message)
  ((blocker_id
    :reader blocker_id
    :initarg :blocker_id
    :type cl:string
    :initform "")
   (blocked_id
    :reader blocked_id
    :initarg :blocked_id
    :type cl:string
    :initform "")
   (is_bilateral
    :reader is_bilateral
    :initarg :is_bilateral
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DeadlockInfo (<DeadlockInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeadlockInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeadlockInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<DeadlockInfo> is deprecated: use uav_control-msg:DeadlockInfo instead.")))

(cl:ensure-generic-function 'blocker_id-val :lambda-list '(m))
(cl:defmethod blocker_id-val ((m <DeadlockInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:blocker_id-val is deprecated.  Use uav_control-msg:blocker_id instead.")
  (blocker_id m))

(cl:ensure-generic-function 'blocked_id-val :lambda-list '(m))
(cl:defmethod blocked_id-val ((m <DeadlockInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:blocked_id-val is deprecated.  Use uav_control-msg:blocked_id instead.")
  (blocked_id m))

(cl:ensure-generic-function 'is_bilateral-val :lambda-list '(m))
(cl:defmethod is_bilateral-val ((m <DeadlockInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:is_bilateral-val is deprecated.  Use uav_control-msg:is_bilateral instead.")
  (is_bilateral m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeadlockInfo>) ostream)
  "Serializes a message object of type '<DeadlockInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'blocker_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'blocker_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'blocked_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'blocked_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_bilateral) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeadlockInfo>) istream)
  "Deserializes a message object of type '<DeadlockInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'blocker_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'blocker_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'blocked_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'blocked_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'is_bilateral) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeadlockInfo>)))
  "Returns string type for a message object of type '<DeadlockInfo>"
  "uav_control/DeadlockInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeadlockInfo)))
  "Returns string type for a message object of type 'DeadlockInfo"
  "uav_control/DeadlockInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeadlockInfo>)))
  "Returns md5sum for a message object of type '<DeadlockInfo>"
  "38f3084008c30c3c1c815f245eba16b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeadlockInfo)))
  "Returns md5sum for a message object of type 'DeadlockInfo"
  "38f3084008c30c3c1c815f245eba16b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeadlockInfo>)))
  "Returns full string definition for message of type '<DeadlockInfo>"
  (cl:format cl:nil "string blocker_id    # 造成阻塞的无人机ID~%string blocked_id   # 被阻塞的无人机ID~%bool is_bilateral   # true表示双向死锁，false表示单向阻塞~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeadlockInfo)))
  "Returns full string definition for message of type 'DeadlockInfo"
  (cl:format cl:nil "string blocker_id    # 造成阻塞的无人机ID~%string blocked_id   # 被阻塞的无人机ID~%bool is_bilateral   # true表示双向死锁，false表示单向阻塞~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeadlockInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'blocker_id))
     4 (cl:length (cl:slot-value msg 'blocked_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeadlockInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'DeadlockInfo
    (cl:cons ':blocker_id (blocker_id msg))
    (cl:cons ':blocked_id (blocked_id msg))
    (cl:cons ':is_bilateral (is_bilateral msg))
))
