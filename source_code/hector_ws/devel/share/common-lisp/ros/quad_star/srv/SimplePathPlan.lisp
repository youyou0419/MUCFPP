; Auto-generated. Do not edit!


(cl:in-package quad_star-srv)


;//! \htmlinclude SimplePathPlan-request.msg.html

(cl:defclass <SimplePathPlan-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SimplePathPlan-request (<SimplePathPlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimplePathPlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimplePathPlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_star-srv:<SimplePathPlan-request> is deprecated: use quad_star-srv:SimplePathPlan-request instead.")))

(cl:ensure-generic-function 'start_x-val :lambda-list '(m))
(cl:defmethod start_x-val ((m <SimplePathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_star-srv:start_x-val is deprecated.  Use quad_star-srv:start_x instead.")
  (start_x m))

(cl:ensure-generic-function 'start_y-val :lambda-list '(m))
(cl:defmethod start_y-val ((m <SimplePathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_star-srv:start_y-val is deprecated.  Use quad_star-srv:start_y instead.")
  (start_y m))

(cl:ensure-generic-function 'goal_x-val :lambda-list '(m))
(cl:defmethod goal_x-val ((m <SimplePathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_star-srv:goal_x-val is deprecated.  Use quad_star-srv:goal_x instead.")
  (goal_x m))

(cl:ensure-generic-function 'goal_y-val :lambda-list '(m))
(cl:defmethod goal_y-val ((m <SimplePathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_star-srv:goal_y-val is deprecated.  Use quad_star-srv:goal_y instead.")
  (goal_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimplePathPlan-request>) ostream)
  "Serializes a message object of type '<SimplePathPlan-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimplePathPlan-request>) istream)
  "Deserializes a message object of type '<SimplePathPlan-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimplePathPlan-request>)))
  "Returns string type for a service object of type '<SimplePathPlan-request>"
  "quad_star/SimplePathPlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimplePathPlan-request)))
  "Returns string type for a service object of type 'SimplePathPlan-request"
  "quad_star/SimplePathPlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimplePathPlan-request>)))
  "Returns md5sum for a message object of type '<SimplePathPlan-request>"
  "cc0fa8f370b2645f29bc836e78ed2538")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimplePathPlan-request)))
  "Returns md5sum for a message object of type 'SimplePathPlan-request"
  "cc0fa8f370b2645f29bc836e78ed2538")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimplePathPlan-request>)))
  "Returns full string definition for message of type '<SimplePathPlan-request>"
  (cl:format cl:nil "# 请求消息~%float32 start_x~%float32 start_y~%float32 goal_x~%float32 goal_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimplePathPlan-request)))
  "Returns full string definition for message of type 'SimplePathPlan-request"
  (cl:format cl:nil "# 请求消息~%float32 start_x~%float32 start_y~%float32 goal_x~%float32 goal_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimplePathPlan-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimplePathPlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SimplePathPlan-request
    (cl:cons ':start_x (start_x msg))
    (cl:cons ':start_y (start_y msg))
    (cl:cons ':goal_x (goal_x msg))
    (cl:cons ':goal_y (goal_y msg))
))
;//! \htmlinclude SimplePathPlan-response.msg.html

(cl:defclass <SimplePathPlan-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SimplePathPlan-response (<SimplePathPlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimplePathPlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimplePathPlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_star-srv:<SimplePathPlan-response> is deprecated: use quad_star-srv:SimplePathPlan-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SimplePathPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_star-srv:success-val is deprecated.  Use quad_star-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SimplePathPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_star-srv:message-val is deprecated.  Use quad_star-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimplePathPlan-response>) ostream)
  "Serializes a message object of type '<SimplePathPlan-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimplePathPlan-response>) istream)
  "Deserializes a message object of type '<SimplePathPlan-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimplePathPlan-response>)))
  "Returns string type for a service object of type '<SimplePathPlan-response>"
  "quad_star/SimplePathPlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimplePathPlan-response)))
  "Returns string type for a service object of type 'SimplePathPlan-response"
  "quad_star/SimplePathPlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimplePathPlan-response>)))
  "Returns md5sum for a message object of type '<SimplePathPlan-response>"
  "cc0fa8f370b2645f29bc836e78ed2538")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimplePathPlan-response)))
  "Returns md5sum for a message object of type 'SimplePathPlan-response"
  "cc0fa8f370b2645f29bc836e78ed2538")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimplePathPlan-response>)))
  "Returns full string definition for message of type '<SimplePathPlan-response>"
  (cl:format cl:nil "# 响应消息~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimplePathPlan-response)))
  "Returns full string definition for message of type 'SimplePathPlan-response"
  (cl:format cl:nil "# 响应消息~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimplePathPlan-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimplePathPlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SimplePathPlan-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SimplePathPlan)))
  'SimplePathPlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SimplePathPlan)))
  'SimplePathPlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimplePathPlan)))
  "Returns string type for a service object of type '<SimplePathPlan>"
  "quad_star/SimplePathPlan")