
(cl:in-package :asdf)

(defsystem "dynamixel_current_2port-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Select_Motion" :depends-on ("_package_Select_Motion"))
    (:file "_package_Select_Motion" :depends-on ("_package"))
    (:file "Turn_Angle" :depends-on ("_package_Turn_Angle"))
    (:file "_package_Turn_Angle" :depends-on ("_package"))
  ))