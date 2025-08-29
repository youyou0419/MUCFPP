; Auto-generated. Do not edit!


(cl:in-package orca_navigation-msg)


;//! \htmlinclude BLockCondition.msg.html

(cl:defclass <BLockCondition> (roslisp-msg-protocol:ros-message)
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
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0))
)

(cl:defclass BLockCondition (<BLockCondition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BLockCondition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BLockCondition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name orca_navigation-msg:<BLockCondition> is deprecated: use orca_navigation-msg:BLockCondition instead.")))

(cl:ensure-generic-function 'blocker_id-val :lambda-list '(m))
(cl:defmethod blocker_id-val ((m <BLockCondition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader orca_navigation-msg:blocker_id-val is deprecated.  Use orca_navigation-msg:blocker_id instead.")
  (blocker_id m))

(cl:ensure-generic-function 'blocked_id-val :lambda-list '(m))
(cl:defmethod blocked_id-val ((m <BLockCondition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader orca_navigation-msg:blocked_id-val is deprecated.  Use orca_navigation-msg:blocked_id instead.")
  (blocked_id m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <BLockCondition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader orca_navigation-msg:stamp-val is deprecated.  Use orca_navigation-msg:stamp instead.")
  (stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BLockCondition>) ostream)
  "Serializes a message object of type '<BLockCondition>"
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
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BLockCondition>) istream)
  "Deserializes a message object of type '<BLockCondition>"
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
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BLockCondition>)))
  "Returns string type for a message object of type '<BLockCondition>"
  "orca_navigation/BLockCondition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BLockCondition)))
  "Returns string type for a message object of type 'BLockCondition"
  "orca_navigation/BLockCondition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BLockCondition>)))
  "Returns md5sum for a message object of type '<BLockCondition>"
  "159eaf14713728dcfaf35256e7ad2445")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BLockCondition)))
  "Returns md5sum for a message object of type 'BLockCondition"
  "159eaf14713728dcfaf35256e7ad2445")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BLockCondition>)))
  "Returns full string definition for message of type '<BLockCondition>"
  (cl:format cl:nil "string blocker_id~%string blocked_id~%time stamp~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BLockCondition)))
  "Returns full string definition for message of type 'BLockCondition"
  (cl:format cl:nil "string blocker_id~%string blocked_id~%time stamp~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BLockCondition>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'blocker_id))
     4 (cl:length (cl:slot-value msg 'blocked_id))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BLockCondition>))
  "Converts a ROS message object to a list"
  (cl:list 'BLockCondition
    (cl:cons ':blocker_id (blocker_id msg))
    (cl:cons ':blocked_id (blocked_id msg))
    (cl:cons ':stamp (stamp msg))
))
