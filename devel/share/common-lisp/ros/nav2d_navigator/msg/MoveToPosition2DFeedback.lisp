; Auto-generated. Do not edit!


(cl:in-package nav2d_navigator-msg)


;//! \htmlinclude MoveToPosition2DFeedback.msg.html

(cl:defclass <MoveToPosition2DFeedback> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveToPosition2DFeedback (<MoveToPosition2DFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPosition2DFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPosition2DFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nav2d_navigator-msg:<MoveToPosition2DFeedback> is deprecated: use nav2d_navigator-msg:MoveToPosition2DFeedback instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <MoveToPosition2DFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav2d_navigator-msg:distance-val is deprecated.  Use nav2d_navigator-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPosition2DFeedback>) ostream)
  "Serializes a message object of type '<MoveToPosition2DFeedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPosition2DFeedback>) istream)
  "Deserializes a message object of type '<MoveToPosition2DFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPosition2DFeedback>)))
  "Returns string type for a message object of type '<MoveToPosition2DFeedback>"
  "nav2d_navigator/MoveToPosition2DFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition2DFeedback)))
  "Returns string type for a message object of type 'MoveToPosition2DFeedback"
  "nav2d_navigator/MoveToPosition2DFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPosition2DFeedback>)))
  "Returns md5sum for a message object of type '<MoveToPosition2DFeedback>"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPosition2DFeedback)))
  "Returns md5sum for a message object of type 'MoveToPosition2DFeedback"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPosition2DFeedback>)))
  "Returns full string definition for message of type '<MoveToPosition2DFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float32 distance~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPosition2DFeedback)))
  "Returns full string definition for message of type 'MoveToPosition2DFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float32 distance~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPosition2DFeedback>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPosition2DFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPosition2DFeedback
    (cl:cons ':distance (distance msg))
))
