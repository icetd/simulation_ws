; Auto-generated. Do not edit!


(cl:in-package manager_msgs-msg)


;//! \htmlinclude Type.msg.html

(cl:defclass <Type> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Type (<Type>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Type>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Type)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manager_msgs-msg:<Type> is deprecated: use manager_msgs-msg:Type instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manager_msgs-msg:status-val is deprecated.  Use manager_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Type>)))
    "Constants for message type '<Type>"
  '((:STOP . 0)
    (:PAUSE . 1)
    (:MOVE . 2)
    (:ACTION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Type)))
    "Constants for message type 'Type"
  '((:STOP . 0)
    (:PAUSE . 1)
    (:MOVE . 2)
    (:ACTION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Type>) ostream)
  "Serializes a message object of type '<Type>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Type>) istream)
  "Deserializes a message object of type '<Type>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Type>)))
  "Returns string type for a message object of type '<Type>"
  "manager_msgs/Type")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Type)))
  "Returns string type for a message object of type 'Type"
  "manager_msgs/Type")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Type>)))
  "Returns md5sum for a message object of type '<Type>"
  "41aa323be3c944320ef6a43a04b1d136")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Type)))
  "Returns md5sum for a message object of type 'Type"
  "41aa323be3c944320ef6a43a04b1d136")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Type>)))
  "Returns full string definition for message of type '<Type>"
  (cl:format cl:nil "uint8 STOP = 0~%uint8 PAUSE = 1~%uint8 MOVE = 2~%uint8 ACTION = 3~%uint8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Type)))
  "Returns full string definition for message of type 'Type"
  (cl:format cl:nil "uint8 STOP = 0~%uint8 PAUSE = 1~%uint8 MOVE = 2~%uint8 ACTION = 3~%uint8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Type>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Type>))
  "Converts a ROS message object to a list"
  (cl:list 'Type
    (cl:cons ':status (status msg))
))
