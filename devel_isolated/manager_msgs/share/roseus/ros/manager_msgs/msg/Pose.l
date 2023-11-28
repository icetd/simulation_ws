;; Auto-generated. Do not edit!


(when (boundp 'manager_msgs::Pose)
  (if (not (find-package "MANAGER_MSGS"))
    (make-package "MANAGER_MSGS"))
  (shadow 'Pose (find-package "MANAGER_MSGS")))
(unless (find-package "MANAGER_MSGS::POSE")
  (make-package "MANAGER_MSGS::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html


(defclass manager_msgs::Pose
  :super ros::object
  :slots (_position _orientation ))

(defmethod manager_msgs::Pose
  (:init
   (&key
    ((:position __position) (instance manager_msgs::Point :init))
    ((:orientation __orientation) (instance manager_msgs::Quaternion :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _orientation __orientation)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:serialization-length
   ()
   (+
    ;; manager_msgs/Point _position
    (send _position :serialization-length)
    ;; manager_msgs/Quaternion _orientation
    (send _orientation :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; manager_msgs/Point _position
       (send _position :serialize s)
     ;; manager_msgs/Quaternion _orientation
       (send _orientation :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; manager_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; manager_msgs/Quaternion _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;;
   self)
  )

(setf (get manager_msgs::Pose :md5sum-) "e45d45a5a1ce597b249e23fb30fc871f")
(setf (get manager_msgs::Pose :datatype-) "manager_msgs/Pose")
(setf (get manager_msgs::Pose :definition-)
      "Point position
Quaternion orientation

================================================================================
MSG: manager_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: manager_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :manager_msgs/Pose "e45d45a5a1ce597b249e23fb30fc871f")


