;; Auto-generated. Do not edit!


(when (boundp 'manager_msgs::Type)
  (if (not (find-package "MANAGER_MSGS"))
    (make-package "MANAGER_MSGS"))
  (shadow 'Type (find-package "MANAGER_MSGS")))
(unless (find-package "MANAGER_MSGS::TYPE")
  (make-package "MANAGER_MSGS::TYPE"))

(in-package "ROS")
;;//! \htmlinclude Type.msg.html


(intern "*STOP*" (find-package "MANAGER_MSGS::TYPE"))
(shadow '*STOP* (find-package "MANAGER_MSGS::TYPE"))
(defconstant manager_msgs::Type::*STOP* 0)
(intern "*PAUSE*" (find-package "MANAGER_MSGS::TYPE"))
(shadow '*PAUSE* (find-package "MANAGER_MSGS::TYPE"))
(defconstant manager_msgs::Type::*PAUSE* 1)
(intern "*MOVE*" (find-package "MANAGER_MSGS::TYPE"))
(shadow '*MOVE* (find-package "MANAGER_MSGS::TYPE"))
(defconstant manager_msgs::Type::*MOVE* 2)
(intern "*ACTION*" (find-package "MANAGER_MSGS::TYPE"))
(shadow '*ACTION* (find-package "MANAGER_MSGS::TYPE"))
(defconstant manager_msgs::Type::*ACTION* 3)

(defun manager_msgs::Type-to-symbol (const)
  (cond
        ((= const 0) 'manager_msgs::Type::*STOP*)
        ((= const 1) 'manager_msgs::Type::*PAUSE*)
        ((= const 2) 'manager_msgs::Type::*MOVE*)
        ((= const 3) 'manager_msgs::Type::*ACTION*)
        (t nil)))

(defclass manager_msgs::Type
  :super ros::object
  :slots (_status ))

(defmethod manager_msgs::Type
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get manager_msgs::Type :md5sum-) "41aa323be3c944320ef6a43a04b1d136")
(setf (get manager_msgs::Type :datatype-) "manager_msgs/Type")
(setf (get manager_msgs::Type :definition-)
      "uint8 STOP = 0
uint8 PAUSE = 1
uint8 MOVE = 2
uint8 ACTION = 3
uint8 status
")



(provide :manager_msgs/Type "41aa323be3c944320ef6a43a04b1d136")


