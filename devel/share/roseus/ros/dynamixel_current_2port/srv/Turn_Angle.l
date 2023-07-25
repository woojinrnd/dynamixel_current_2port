;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_current_2port::Turn_Angle)
  (if (not (find-package "DYNAMIXEL_CURRENT_2PORT"))
    (make-package "DYNAMIXEL_CURRENT_2PORT"))
  (shadow 'Turn_Angle (find-package "DYNAMIXEL_CURRENT_2PORT")))
(unless (find-package "DYNAMIXEL_CURRENT_2PORT::TURN_ANGLE")
  (make-package "DYNAMIXEL_CURRENT_2PORT::TURN_ANGLE"))
(unless (find-package "DYNAMIXEL_CURRENT_2PORT::TURN_ANGLEREQUEST")
  (make-package "DYNAMIXEL_CURRENT_2PORT::TURN_ANGLEREQUEST"))
(unless (find-package "DYNAMIXEL_CURRENT_2PORT::TURN_ANGLERESPONSE")
  (make-package "DYNAMIXEL_CURRENT_2PORT::TURN_ANGLERESPONSE"))

(in-package "ROS")





(defclass dynamixel_current_2port::Turn_AngleRequest
  :super ros::object
  :slots (_finish ))

(defmethod dynamixel_current_2port::Turn_AngleRequest
  (:init
   (&key
    ((:finish __finish) nil)
    )
   (send-super :init)
   (setq _finish __finish)
   self)
  (:finish
   (&optional (__finish :null))
   (if (not (eq __finish :null)) (setq _finish __finish)) _finish)
  (:serialization-length
   ()
   (+
    ;; bool _finish
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _finish
       (if _finish (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _finish
     (setq _finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dynamixel_current_2port::Turn_AngleResponse
  :super ros::object
  :slots (_turn_angle ))

(defmethod dynamixel_current_2port::Turn_AngleResponse
  (:init
   (&key
    ((:turn_angle __turn_angle) 0)
    )
   (send-super :init)
   (setq _turn_angle (round __turn_angle))
   self)
  (:turn_angle
   (&optional __turn_angle)
   (if __turn_angle (setq _turn_angle __turn_angle)) _turn_angle)
  (:serialization-length
   ()
   (+
    ;; int8 _turn_angle
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _turn_angle
       (write-byte _turn_angle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _turn_angle
     (setq _turn_angle (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _turn_angle 127) (setq _turn_angle (- _turn_angle 256)))
   ;;
   self)
  )

(defclass dynamixel_current_2port::Turn_Angle
  :super ros::object
  :slots ())

(setf (get dynamixel_current_2port::Turn_Angle :md5sum-) "6bcc5ac74e5ed33763d25bed29ca1508")
(setf (get dynamixel_current_2port::Turn_Angle :datatype-) "dynamixel_current_2port/Turn_Angle")
(setf (get dynamixel_current_2port::Turn_Angle :request) dynamixel_current_2port::Turn_AngleRequest)
(setf (get dynamixel_current_2port::Turn_Angle :response) dynamixel_current_2port::Turn_AngleResponse)

(defmethod dynamixel_current_2port::Turn_AngleRequest
  (:response () (instance dynamixel_current_2port::Turn_AngleResponse :init)))

(setf (get dynamixel_current_2port::Turn_AngleRequest :md5sum-) "6bcc5ac74e5ed33763d25bed29ca1508")
(setf (get dynamixel_current_2port::Turn_AngleRequest :datatype-) "dynamixel_current_2port/Turn_AngleRequest")
(setf (get dynamixel_current_2port::Turn_AngleRequest :definition-)
      "bool finish
---
int8 turn_angle
")

(setf (get dynamixel_current_2port::Turn_AngleResponse :md5sum-) "6bcc5ac74e5ed33763d25bed29ca1508")
(setf (get dynamixel_current_2port::Turn_AngleResponse :datatype-) "dynamixel_current_2port/Turn_AngleResponse")
(setf (get dynamixel_current_2port::Turn_AngleResponse :definition-)
      "bool finish
---
int8 turn_angle
")



(provide :dynamixel_current_2port/Turn_Angle "6bcc5ac74e5ed33763d25bed29ca1508")


