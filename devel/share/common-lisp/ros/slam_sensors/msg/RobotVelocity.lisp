; Auto-generated. Do not edit!


(cl:in-package slam_sensors-msg)


;//! \htmlinclude RobotVelocity.msg.html

(cl:defclass <RobotVelocity> (roslisp-msg-protocol:ros-message)
  ((angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type cl:float
    :initform 0.0)
   (linear_velocity
    :reader linear_velocity
    :initarg :linear_velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass RobotVelocity (<RobotVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name slam_sensors-msg:<RobotVelocity> is deprecated: use slam_sensors-msg:RobotVelocity instead.")))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <RobotVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_sensors-msg:angular_velocity-val is deprecated.  Use slam_sensors-msg:angular_velocity instead.")
  (angular_velocity m))

(cl:ensure-generic-function 'linear_velocity-val :lambda-list '(m))
(cl:defmethod linear_velocity-val ((m <RobotVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_sensors-msg:linear_velocity-val is deprecated.  Use slam_sensors-msg:linear_velocity instead.")
  (linear_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotVelocity>) ostream)
  "Serializes a message object of type '<RobotVelocity>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotVelocity>) istream)
  "Deserializes a message object of type '<RobotVelocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotVelocity>)))
  "Returns string type for a message object of type '<RobotVelocity>"
  "slam_sensors/RobotVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotVelocity)))
  "Returns string type for a message object of type 'RobotVelocity"
  "slam_sensors/RobotVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotVelocity>)))
  "Returns md5sum for a message object of type '<RobotVelocity>"
  "37340f3f2186e83a4c0b06837d532fcc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotVelocity)))
  "Returns md5sum for a message object of type 'RobotVelocity"
  "37340f3f2186e83a4c0b06837d532fcc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotVelocity>)))
  "Returns full string definition for message of type '<RobotVelocity>"
  (cl:format cl:nil "float64 angular_velocity~%float64 linear_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotVelocity)))
  "Returns full string definition for message of type 'RobotVelocity"
  (cl:format cl:nil "float64 angular_velocity~%float64 linear_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotVelocity>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotVelocity
    (cl:cons ':angular_velocity (angular_velocity msg))
    (cl:cons ':linear_velocity (linear_velocity msg))
))
