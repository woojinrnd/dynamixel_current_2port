;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_current_2port::Select_Motion)
  (if (not (find-package "DYNAMIXEL_CURRENT_2PORT"))
    (make-package "DYNAMIXEL_CURRENT_2PORT"))
  (shadow 'Select_Motion (find-package "DYNAMIXEL_CURRENT_2PORT")))
(unless (find-package "DYNAMIXEL_CURRENT_2PORT::SELECT_MOTION")
  (make-package "DYNAMIXEL_CURRENT_2PORT::SELECT_MOTION"))
(unless (find-package "DYNAMIXEL_CURRENT_2PORT::SELECT_MOTIONREQUEST")
  (make-package "DYNAMIXEL_CURRENT_2PORT::SELECT_MOTIONREQUEST"))
(unless (find-package "DYNAMIXEL_CURRENT_2PORT::SELECT_MOTIONRESPONSE")
  (make-package "DYNAMIXEL_CURRENT_2PORT::SELECT_MOTIONRESPONSE"))

(in-package "ROS")





(defclass dynamixel_current_2port::Select_MotionRequest
  :super ros::object
  :slots (_finish ))

(defmethod dynamixel_current_2port::Select_MotionRequest
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

(defclass dynamixel_current_2port::Select_MotionResponse
  :super ros::object
  :slots (_Select_Motion ))

(defmethod dynamixel_current_2port::Select_MotionResponse
  (:init
   (&key
    ((:Select_Motion __Select_Motion) 0)
    )
   (send-super :init)
   (setq _Select_Motion (round __Select_Motion))
   self)
  (:Select_Motion
   (&optional __Select_Motion)
   (if __Select_Motion (setq _Select_Motion __Select_Motion)) _Select_Motion)
  (:serialization-length
   ()
   (+
    ;; int8 _Select_Motion
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _Select_Motion
       (write-byte _Select_Motion s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _Select_Motion
     (setq _Select_Motion (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _Select_Motion 127) (setq _Select_Motion (- _Select_Motion 256)))
   ;;
   self)
  )

(defclass dynamixel_current_2port::Select_Motion
  :super ros::object
  :slots ())

(setf (get dynamixel_current_2port::Select_Motion :md5sum-) "cc9905de9a643bc5d56477c60453b2c8")
(setf (get dynamixel_current_2port::Select_Motion :datatype-) "dynamixel_current_2port/Select_Motion")
(setf (get dynamixel_current_2port::Select_Motion :request) dynamixel_current_2port::Select_MotionRequest)
(setf (get dynamixel_current_2port::Select_Motion :response) dynamixel_current_2port::Select_MotionResponse)

(defmethod dynamixel_current_2port::Select_MotionRequest
  (:response () (instance dynamixel_current_2port::Select_MotionResponse :init)))

(setf (get dynamixel_current_2port::Select_MotionRequest :md5sum-) "cc9905de9a643bc5d56477c60453b2c8")
(setf (get dynamixel_current_2port::Select_MotionRequest :datatype-) "dynamixel_current_2port/Select_MotionRequest")
(setf (get dynamixel_current_2port::Select_MotionRequest :definition-)
      "bool finish
---
int8 Select_Motion
")

(setf (get dynamixel_current_2port::Select_MotionResponse :md5sum-) "cc9905de9a643bc5d56477c60453b2c8")
(setf (get dynamixel_current_2port::Select_MotionResponse :datatype-) "dynamixel_current_2port/Select_MotionResponse")
(setf (get dynamixel_current_2port::Select_MotionResponse :definition-)
      "bool finish
---
int8 Select_Motion
")



(provide :dynamixel_current_2port/Select_Motion "cc9905de9a643bc5d56477c60453b2c8")


