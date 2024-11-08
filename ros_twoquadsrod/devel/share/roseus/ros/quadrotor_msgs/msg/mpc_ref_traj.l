;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::mpc_ref_traj)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'mpc_ref_traj (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::MPC_REF_TRAJ")
  (make-package "QUADROTOR_MSGS::MPC_REF_TRAJ"))

(in-package "ROS")
;;//! \htmlinclude mpc_ref_traj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::mpc_ref_traj
  :super ros::object
  :slots (_header _mpc_ref_points _goal ))

(defmethod quadrotor_msgs::mpc_ref_traj
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mpc_ref_points __mpc_ref_points) ())
    ((:goal __goal) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _mpc_ref_points __mpc_ref_points)
   (setq _goal __goal)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mpc_ref_points
   (&rest __mpc_ref_points)
   (if (keywordp (car __mpc_ref_points))
       (send* _mpc_ref_points __mpc_ref_points)
     (progn
       (if __mpc_ref_points (setq _mpc_ref_points (car __mpc_ref_points)))
       _mpc_ref_points)))
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; quadrotor_msgs/mpc_ref_point[] _mpc_ref_points
    (apply #'+ (send-all _mpc_ref_points :serialization-length)) 4
    ;; geometry_msgs/Vector3 _goal
    (send _goal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; quadrotor_msgs/mpc_ref_point[] _mpc_ref_points
     (write-long (length _mpc_ref_points) s)
     (dolist (elem _mpc_ref_points)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3 _goal
       (send _goal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; quadrotor_msgs/mpc_ref_point[] _mpc_ref_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mpc_ref_points (let (r) (dotimes (i n) (push (instance quadrotor_msgs::mpc_ref_point :init) r)) r))
     (dolist (elem- _mpc_ref_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3 _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::mpc_ref_traj :md5sum-) "4faae5b6cb706021d9657ec34fedc421")
(setf (get quadrotor_msgs::mpc_ref_traj :datatype-) "quadrotor_msgs/mpc_ref_traj")
(setf (get quadrotor_msgs::mpc_ref_traj :definition-)
      "Header header
mpc_ref_point[] mpc_ref_points
geometry_msgs/Vector3 goal

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: quadrotor_msgs/mpc_ref_point
Header header
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :quadrotor_msgs/mpc_ref_traj "4faae5b6cb706021d9657ec34fedc421")


