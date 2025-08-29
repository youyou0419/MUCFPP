; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude StopCommand.msg.html

(cl:defclass <StopCommand> (roslisp-msg-protocol:ros-message)
  ((stop
    :reader stop
    :initarg :stop
    :type cl:boolean
    :initform cl:nil)
   (blocker_id
    :reader blocker_id
    :initarg :blocker_id
    :type cl:string
    :initform ""))
)

(cl:defclass StopCommand (<StopCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<StopCommand> is deprecated: use uav_control-msg:StopCommand instead.")))

(cl:ensure-generic-function 'stop-val :lambda-list '(m))
(cl:defmethod stop-val ((m <StopCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:stop-val is deprecated.  Use uav_control-msg:stop instead.")
  (stop m))

(cl:ensure-generic-function 'blocker_id-val :lambda-list '(m))
(cl:defmethod blocker_id-val ((m <StopCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:blocker_id-val is deprecated.  Use uav_control-msg:blocker_id instead.")
  (blocker_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopCommand>) ostream)
  "Serializes a message object of type '<StopCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'blocker_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'blocker_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopCommand>) istream)
  "Deserializes a message object of type '<StopCommand>"
    (cl:setf (cl:slot-value msg 'stop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'blocker_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'blocker_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopCommand>)))
  "Returns string type for a message object of type '<StopCommand>"
  "uav_control/StopCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopCommand)))
  "Returns string type for a message object of type 'StopCommand"
  "uav_control/StopCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopCommand>)))
  "Returns md5sum for a message object of type '<StopCommand>"
  "1ee931530b17c39eb5a3b97eb6933cbd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopCommand)))
  "Returns md5sum for a message object of type 'StopCommand"
  "1ee931530b17c39eb5a3b97eb6933cbd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopCommand>)))
  "Returns full string definition for message of type '<StopCommand>"
  (cl:format cl:nil "bool stop~%string blocker_id  # 导致死锁的无人机ID~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopCommand)))
  "Returns full string definition for message of type 'StopCommand"
  (cl:format cl:nil "bool stop~%string blocker_id  # 导致死锁的无人机ID~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopCommand>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'blocker_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'StopCommand
    (cl:cons ':stop (stop msg))
    (cl:cons ':blocker_id (blocker_id msg))
))
