; Auto-generated. Do not edit!


(cl:in-package uav_control-srv)


;//! \htmlinclude PathPlan-request.msg.html

(cl:defclass <PathPlan-request> (roslisp-msg-protocol:ros-message)
  ((start_x
    :reader start_x
    :initarg :start_x
    :type cl:float
    :initform 0.0)
   (start_y
    :reader start_y
    :initarg :start_y
    :type cl:float
    :initform 0.0)
   (goal_x
    :reader goal_x
    :initarg :goal_x
    :type cl:float
    :initform 0.0)
   (goal_y
    :reader goal_y
    :initarg :goal_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass PathPlan-request (<PathPlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathPlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathPlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-srv:<PathPlan-request> is deprecated: use uav_control-srv:PathPlan-request instead.")))

(cl:ensure-generic-function 'start_x-val :lambda-list '(m))
(cl:defmethod start_x-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:start_x-val is deprecated.  Use uav_control-srv:start_x instead.")
  (start_x m))

(cl:ensure-generic-function 'start_y-val :lambda-list '(m))
(cl:defmethod start_y-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:start_y-val is deprecated.  Use uav_control-srv:start_y instead.")
  (start_y m))

(cl:ensure-generic-function 'goal_x-val :lambda-list '(m))
(cl:defmethod goal_x-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:goal_x-val is deprecated.  Use uav_control-srv:goal_x instead.")
  (goal_x m))

(cl:ensure-generic-function 'goal_y-val :lambda-list '(m))
(cl:defmethod goal_y-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:goal_y-val is deprecated.  Use uav_control-srv:goal_y instead.")
  (goal_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathPlan-request>) ostream)
  "Serializes a message object of type '<PathPlan-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'start_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'start_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goal_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goal_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathPlan-request>) istream)
  "Deserializes a message object of type '<PathPlan-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathPlan-request>)))
  "Returns string type for a service object of type '<PathPlan-request>"
  "uav_control/PathPlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathPlan-request)))
  "Returns string type for a service object of type 'PathPlan-request"
  "uav_control/PathPlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathPlan-request>)))
  "Returns md5sum for a message object of type '<PathPlan-request>"
  "c9c9ee362512ec5be6042c67e74ac40f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathPlan-request)))
  "Returns md5sum for a message object of type 'PathPlan-request"
  "c9c9ee362512ec5be6042c67e74ac40f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathPlan-request>)))
  "Returns full string definition for message of type '<PathPlan-request>"
  (cl:format cl:nil "float32 start_x~%float32 start_y~%float32 goal_x~%float32 goal_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathPlan-request)))
  "Returns full string definition for message of type 'PathPlan-request"
  (cl:format cl:nil "float32 start_x~%float32 start_y~%float32 goal_x~%float32 goal_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathPlan-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathPlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PathPlan-request
    (cl:cons ':start_x (start_x msg))
    (cl:cons ':start_y (start_y msg))
    (cl:cons ':goal_x (goal_x msg))
    (cl:cons ':goal_y (goal_y msg))
))
;//! \htmlinclude PathPlan-response.msg.html

(cl:defclass <PathPlan-response> (roslisp-msg-protocol:ros-message)
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
   (path
    :reader path
    :initarg :path
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass PathPlan-response (<PathPlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathPlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathPlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-srv:<PathPlan-response> is deprecated: use uav_control-srv:PathPlan-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PathPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:success-val is deprecated.  Use uav_control-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PathPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:message-val is deprecated.  Use uav_control-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <PathPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-srv:path-val is deprecated.  Use uav_control-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathPlan-response>) ostream)
  "Serializes a message object of type '<PathPlan-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathPlan-response>) istream)
  "Deserializes a message object of type '<PathPlan-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathPlan-response>)))
  "Returns string type for a service object of type '<PathPlan-response>"
  "uav_control/PathPlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathPlan-response)))
  "Returns string type for a service object of type 'PathPlan-response"
  "uav_control/PathPlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathPlan-response>)))
  "Returns md5sum for a message object of type '<PathPlan-response>"
  "c9c9ee362512ec5be6042c67e74ac40f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathPlan-response)))
  "Returns md5sum for a message object of type 'PathPlan-response"
  "c9c9ee362512ec5be6042c67e74ac40f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathPlan-response>)))
  "Returns full string definition for message of type '<PathPlan-response>"
  (cl:format cl:nil "bool success~%string message~%geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathPlan-response)))
  "Returns full string definition for message of type 'PathPlan-response"
  (cl:format cl:nil "bool success~%string message~%geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathPlan-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathPlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PathPlan-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PathPlan)))
  'PathPlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PathPlan)))
  'PathPlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathPlan)))
  "Returns string type for a service object of type '<PathPlan>"
  "uav_control/PathPlan")