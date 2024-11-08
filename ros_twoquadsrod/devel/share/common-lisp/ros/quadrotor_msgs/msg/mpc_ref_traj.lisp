; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude mpc_ref_traj.msg.html

(cl:defclass <mpc_ref_traj> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mpc_ref_points
    :reader mpc_ref_points
    :initarg :mpc_ref_points
    :type (cl:vector quadrotor_msgs-msg:mpc_ref_point)
   :initform (cl:make-array 0 :element-type 'quadrotor_msgs-msg:mpc_ref_point :initial-element (cl:make-instance 'quadrotor_msgs-msg:mpc_ref_point)))
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass mpc_ref_traj (<mpc_ref_traj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mpc_ref_traj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mpc_ref_traj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<mpc_ref_traj> is deprecated: use quadrotor_msgs-msg:mpc_ref_traj instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <mpc_ref_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mpc_ref_points-val :lambda-list '(m))
(cl:defmethod mpc_ref_points-val ((m <mpc_ref_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:mpc_ref_points-val is deprecated.  Use quadrotor_msgs-msg:mpc_ref_points instead.")
  (mpc_ref_points m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <mpc_ref_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:goal-val is deprecated.  Use quadrotor_msgs-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mpc_ref_traj>) ostream)
  "Serializes a message object of type '<mpc_ref_traj>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mpc_ref_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'mpc_ref_points))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mpc_ref_traj>) istream)
  "Deserializes a message object of type '<mpc_ref_traj>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mpc_ref_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mpc_ref_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'quadrotor_msgs-msg:mpc_ref_point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mpc_ref_traj>)))
  "Returns string type for a message object of type '<mpc_ref_traj>"
  "quadrotor_msgs/mpc_ref_traj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_ref_traj)))
  "Returns string type for a message object of type 'mpc_ref_traj"
  "quadrotor_msgs/mpc_ref_traj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mpc_ref_traj>)))
  "Returns md5sum for a message object of type '<mpc_ref_traj>"
  "4faae5b6cb706021d9657ec34fedc421")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mpc_ref_traj)))
  "Returns md5sum for a message object of type 'mpc_ref_traj"
  "4faae5b6cb706021d9657ec34fedc421")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mpc_ref_traj>)))
  "Returns full string definition for message of type '<mpc_ref_traj>"
  (cl:format cl:nil "Header header~%mpc_ref_point[] mpc_ref_points~%geometry_msgs/Vector3 goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/mpc_ref_point~%Header header~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mpc_ref_traj)))
  "Returns full string definition for message of type 'mpc_ref_traj"
  (cl:format cl:nil "Header header~%mpc_ref_point[] mpc_ref_points~%geometry_msgs/Vector3 goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/mpc_ref_point~%Header header~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mpc_ref_traj>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mpc_ref_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mpc_ref_traj>))
  "Converts a ROS message object to a list"
  (cl:list 'mpc_ref_traj
    (cl:cons ':header (header msg))
    (cl:cons ':mpc_ref_points (mpc_ref_points msg))
    (cl:cons ':goal (goal msg))
))
