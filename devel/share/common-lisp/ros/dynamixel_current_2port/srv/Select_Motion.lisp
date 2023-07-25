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
  "83c87f0c95a7dce9842bd2a407ff039e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Select_Motion-request)))
  "Returns md5sum for a message object of type 'Select_Motion-request"
  "83c87f0c95a7dce9842bd2a407ff039e")
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
  ((select_motion
    :reader select_motion
    :initarg :select_motion
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Select_Motion-response (<Select_Motion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Select_Motion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Select_Motion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_current_2port-srv:<Select_Motion-response> is deprecated: use dynamixel_current_2port-srv:Select_Motion-response instead.")))

(cl:ensure-generic-function 'select_motion-val :lambda-list '(m))
(cl:defmethod select_motion-val ((m <Select_Motion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_current_2port-srv:select_motion-val is deprecated.  Use dynamixel_current_2port-srv:select_motion instead.")
  (select_motion m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Select_Motion-response>) ostream)
  "Serializes a message object of type '<Select_Motion-response>"
  (cl:let* ((signed (cl:slot-value msg 'select_motion)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Select_Motion-response>) istream)
  "Deserializes a message object of type '<Select_Motion-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'select_motion) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
  "83c87f0c95a7dce9842bd2a407ff039e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Select_Motion-response)))
  "Returns md5sum for a message object of type 'Select_Motion-response"
  "83c87f0c95a7dce9842bd2a407ff039e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Select_Motion-response>)))
  "Returns full string definition for message of type '<Select_Motion-response>"
  (cl:format cl:nil "int8 select_motion~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Select_Motion-response)))
  "Returns full string definition for message of type 'Select_Motion-response"
  (cl:format cl:nil "int8 select_motion~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Select_Motion-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Select_Motion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Select_Motion-response
    (cl:cons ':select_motion (select_motion msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Select_Motion)))
  'Select_Motion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Select_Motion)))
  'Select_Motion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Select_Motion)))
  "Returns string type for a service object of type '<Select_Motion>"
  "dynamixel_current_2port/Select_Motion")