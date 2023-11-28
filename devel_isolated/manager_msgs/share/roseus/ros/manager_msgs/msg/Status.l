;; Auto-generated. Do not edit!


(when (boundp 'manager_msgs::Status)
  (if (not (find-package "MANAGER_MSGS"))
    (make-package "MANAGER_MSGS"))
  (shadow 'Status (find-package "MANAGER_MSGS")))
(unless (find-package "MANAGER_MSGS::STATUS")
  (make-package "MANAGER_MSGS::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STOP*" (find-package "MANAGER_MSGS::STATUS"))
(shadow '*STOP* (find-package "MANAGER_MSGS::STATUS"))
(defconstant manager_msgs::Status::*STOP* 0)
(intern "*START*" (find-package "MANAGER_MSGS::STATUS"))
(shadow '*START* (find-package "MANAGER_MSGS::STATUS"))
(defconstant manager_msgs::Status::*START* 1)
(intern "*CANCEL*" (find-package "MANAGER_MSGS::STATUS"))
(shadow '*CANCEL* (find-package "MANAGER_MSGS::STATUS"))
(defconstant manager_msgs::Status::*CANCEL* 2)
(intern "*CONTINUE*" (find-package "MANAGER_MSGS::STATUS"))
(shadow '*CONTINUE* (find-package "MANAGER_MSGS::STATUS"))
(defconstant manager_msgs::Status::*CONTINUE* 3)
(intern "*DELETEALL*" (find-package "MANAGER_MSGS::STATUS"))
(shadow '*DELETEALL* (find-package "MANAGER_MSGS::STATUS"))
(defconstant manager_msgs::Status::*DELETEALL* 4)
(intern "*MAPSAVE*" (find-package "MANAGER_MSGS::STATUS"))
(shadow '*MAPSAVE* (find-package "MANAGER_MSGS::STATUS"))
(defconstant manager_msgs::Status::*MAPSAVE* 5)

(defun manager_msgs::Status-to-symbol (const)
  (cond
        ((= const 0) 'manager_msgs::Status::*STOP*)
        ((= const 1) 'manager_msgs::Status::*START*)
        ((= const 2) 'manager_msgs::Status::*CANCEL*)
        ((= const 3) 'manager_msgs::Status::*CONTINUE*)
        ((= const 4) 'manager_msgs::Status::*DELETEALL*)
        ((= const 5) 'manager_msgs::Status::*MAPSAVE*)
        (t nil)))

(defclass manager_msgs::Status
  :super ros::object
  :slots (_header _status ))

(defmethod manager_msgs::Status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _status (round __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get manager_msgs::Status :md5sum-) "593a64654818a9bdd4d99c681cbdf720")
(setf (get manager_msgs::Status :datatype-) "manager_msgs/Status")
(setf (get manager_msgs::Status :definition-)
      "Header header
uint8 STOP = 0
uint8 START = 1
uint8 CANCEL = 2
uint8 CONTINUE = 3
uint8 DELETEALL = 4
uint8 MAPSAVE = 5
uint8 status
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

")



(provide :manager_msgs/Status "593a64654818a9bdd4d99c681cbdf720")


