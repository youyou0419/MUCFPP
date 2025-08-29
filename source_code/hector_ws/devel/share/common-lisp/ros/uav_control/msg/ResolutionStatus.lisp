; Auto-generated. Do not edit!


(cl:in-package uav_control-msg)


;//! \htmlinclude ResolutionStatus.msg.html

(cl:defclass <ResolutionStatus> (roslisp-msg-protocol:ros-message)
  ((ldr_id
    :reader ldr_id
    :initarg :ldr_id
    :type cl:integer
    :initform 0)
   (uav_ids
    :reader uav_ids
    :initarg :uav_ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (in_resolution
    :reader in_resolution
    :initarg :in_resolution
    :type cl:boolean
    :initform cl:nil)
   (aabb_min_x
    :reader aabb_min_x
    :initarg :aabb_min_x
    :type cl:float
    :initform 0.0)
   (aabb_max_x
    :reader aabb_max_x
    :initarg :aabb_max_x
    :type cl:float
    :initform 0.0)
   (aabb_min_y
    :reader aabb_min_y
    :initarg :aabb_min_y
    :type cl:float
    :initform 0.0)
   (aabb_max_y
    :reader aabb_max_y
    :initarg :aabb_max_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass ResolutionStatus (<ResolutionStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResolutionStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResolutionStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_control-msg:<ResolutionStatus> is deprecated: use uav_control-msg:ResolutionStatus instead.")))

(cl:ensure-generic-function 'ldr_id-val :lambda-list '(m))
(cl:defmethod ldr_id-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:ldr_id-val is deprecated.  Use uav_control-msg:ldr_id instead.")
  (ldr_id m))

(cl:ensure-generic-function 'uav_ids-val :lambda-list '(m))
(cl:defmethod uav_ids-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:uav_ids-val is deprecated.  Use uav_control-msg:uav_ids instead.")
  (uav_ids m))

(cl:ensure-generic-function 'in_resolution-val :lambda-list '(m))
(cl:defmethod in_resolution-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:in_resolution-val is deprecated.  Use uav_control-msg:in_resolution instead.")
  (in_resolution m))

(cl:ensure-generic-function 'aabb_min_x-val :lambda-list '(m))
(cl:defmethod aabb_min_x-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:aabb_min_x-val is deprecated.  Use uav_control-msg:aabb_min_x instead.")
  (aabb_min_x m))

(cl:ensure-generic-function 'aabb_max_x-val :lambda-list '(m))
(cl:defmethod aabb_max_x-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:aabb_max_x-val is deprecated.  Use uav_control-msg:aabb_max_x instead.")
  (aabb_max_x m))

(cl:ensure-generic-function 'aabb_min_y-val :lambda-list '(m))
(cl:defmethod aabb_min_y-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:aabb_min_y-val is deprecated.  Use uav_control-msg:aabb_min_y instead.")
  (aabb_min_y m))

(cl:ensure-generic-function 'aabb_max_y-val :lambda-list '(m))
(cl:defmethod aabb_max_y-val ((m <ResolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_control-msg:aabb_max_y-val is deprecated.  Use uav_control-msg:aabb_max_y instead.")
  (aabb_max_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResolutionStatus>) ostream)
  "Serializes a message object of type '<ResolutionStatus>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'in_resolution) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'aabb_min_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'aabb_max_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'aabb_min_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'aabb_max_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResolutionStatus>) istream)
  "Deserializes a message object of type '<ResolutionStatus>"
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
    (cl:setf (cl:slot-value msg 'in_resolution) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aabb_min_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aabb_max_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aabb_min_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aabb_max_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResolutionStatus>)))
  "Returns string type for a message object of type '<ResolutionStatus>"
  "uav_control/ResolutionStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResolutionStatus)))
  "Returns string type for a message object of type 'ResolutionStatus"
  "uav_control/ResolutionStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResolutionStatus>)))
  "Returns md5sum for a message object of type '<ResolutionStatus>"
  "68d9e3003f30b6e4832b7d80035318cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResolutionStatus)))
  "Returns md5sum for a message object of type 'ResolutionStatus"
  "68d9e3003f30b6e4832b7d80035318cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResolutionStatus>)))
  "Returns full string definition for message of type '<ResolutionStatus>"
  (cl:format cl:nil "# ResolutionStatus.msg~%# LDR Resolution状态消息定义~%~%~%# LDR标识符~%int32 ldr_id~%~%# 涉及的无人机ID列表~%string[] uav_ids~%~%# Resolution状态：true表示进入Resolution阶段，false表示退出~%bool in_resolution~%~%~%# LDR区域边界信息~%float64 aabb_min_x~%float64 aabb_max_x  ~%float64 aabb_min_y~%float64 aabb_max_y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResolutionStatus)))
  "Returns full string definition for message of type 'ResolutionStatus"
  (cl:format cl:nil "# ResolutionStatus.msg~%# LDR Resolution状态消息定义~%~%~%# LDR标识符~%int32 ldr_id~%~%# 涉及的无人机ID列表~%string[] uav_ids~%~%# Resolution状态：true表示进入Resolution阶段，false表示退出~%bool in_resolution~%~%~%# LDR区域边界信息~%float64 aabb_min_x~%float64 aabb_max_x  ~%float64 aabb_min_y~%float64 aabb_max_y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResolutionStatus>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uav_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResolutionStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ResolutionStatus
    (cl:cons ':ldr_id (ldr_id msg))
    (cl:cons ':uav_ids (uav_ids msg))
    (cl:cons ':in_resolution (in_resolution msg))
    (cl:cons ':aabb_min_x (aabb_min_x msg))
    (cl:cons ':aabb_max_x (aabb_max_x msg))
    (cl:cons ':aabb_min_y (aabb_min_y msg))
    (cl:cons ':aabb_max_y (aabb_max_y msg))
))
