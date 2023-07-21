; Auto-generated. Do not edit!


(cl:in-package dynamixel_current_2port-srv)


;//! \htmlinclude Select_Motion-request.msg.html

(cl:defclass <Select_Motion-request> (roslisp-msg-protocol:ros-message)
  ((finish
    :reader finish
    :initarg :finish
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Select_Motion-request (<Select_Motion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Select_Motion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Select_Motion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_current_2port-srv:<Select_Motion-request> is deprecated: use dynamixel_current_2port-srv:Select_Motion-request instead.")))

(cl:ensure-generic-function 'finish-val :lambda-list '(m))
(cl:defmethod finish-val ((m <Select_Motion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_current_2port-srv:finish-val is deprecated.  Use dynamixel_current_2port-srv:finish instead.")
  (finish m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Select_Motion-request>) ostream)
  "Serializes a message object of type '<Select_Motion-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finish) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Select_Motion-request>) istream)
  "Deserializes a message object of type '<Select_Motion-request>"
    (cl:setf (cl:slot-value msg 'finish) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Select_Motion-request>)))
  "Returns string type for a service object of type '<Select_Motion-request>"
  "dynamixel_current_2port/Select_MotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Select_Motion-request)))
  "Returns string type for a service object of type 'Select_Motion-request"
  "dynamixel_current_2port/Select_MotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Select_Motion-request>)))
  "Returns md5sum for a message object of type '<Select_Motion-request>"
  "cc9905de9a643bc5d56477c60453b2c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Select_Motion-request)))
  "Returns md5sum for a message object of type 'Select_Motion-request"
  "cc9905de9a643bc5d56477c60453b2c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Select_Motion-request>)))
  "Returns full string definition for message of type '<Select_Motion-request>"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Select_Motion-request)))
  "Returns full string definition for message of type 'Select_Motion-request"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Select_Motion-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Select_Motion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Select_Motion-request
    (cl:cons ':finish (finish msg))
))
;//! \htmlinclude Select_Motion-response.msg.html

(cl:defclass <Select_Motion-response> (roslisp-msg-protocol:ros-message)
  ((Select_Motion
    :reader Select_Motion
    :initarg :Select_Motion
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Select_Motion-response (<Select_Motion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Select_Motion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Select_Motion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_current_2port-srv:<Select_Motion-response> is deprecated: use dynamixel_current_2port-srv:Select_Motion-response instead.")))

(cl:ensure-generic-function 'Select_Motion-val :lambda-list '(m))
(cl:defmethod Select_Motion-val ((m <Select_Motion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_current_2port-srv:Select_Motion-val is deprecated.  Use dynamixel_current_2port-srv:Select_Motion instead.")
  (Select_Motion m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Select_Motion-response>) ostream)
  "Serializes a message object of type '<Select_Motion-response>"
  (cl:let* ((signed (cl:slot-value msg 'Select_Motion)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Select_Motion-response>) istream)
  "Deserializes a message object of type '<Select_Motion-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Select_Motion) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Select_Motion-response>)))
  "Returns string type for a service object of type '<Select_Motion-response>"
  "dynamixel_current_2port/Select_MotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Select_Motion-response)))
  "Returns string type for a service object of type 'Select_Motion-response"
  "dynamixel_current_2port/Select_MotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Select_Motion-response>)))
  "Returns md5sum for a message object of type '<Select_Motion-response>"
  "cc9905de9a643bc5d56477c60453b2c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Select_Motion-response)))
  "Returns md5sum for a message object of type 'Select_Motion-response"
  "cc9905de9a643bc5d56477c60453b2c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Select_Motion-response>)))
  "Returns full string definition for message of type '<Select_Motion-response>"
  (cl:format cl:nil "int8 Select_Motion~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Select_Motion-response)))
  "Returns full string definition for message of type 'Select_Motion-response"
  (cl:format cl:nil "int8 Select_Motion~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Select_Motion-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Select_Motion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Select_Motion-response
    (cl:cons ':Select_Motion (Select_Motion msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Select_Motion)))
  'Select_Motion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Select_Motion)))
  'Select_Motion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Select_Motion)))
  "Returns string type for a service object of type '<Select_Motion>"
  "dynamixel_current_2port/Select_Motion")