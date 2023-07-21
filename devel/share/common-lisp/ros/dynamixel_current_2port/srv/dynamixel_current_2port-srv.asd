
(cl:in-package :asdf)

(defsystem "dynamixel_current_2port-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Select_Motion" :depends-on ("_package_Select_Motion"))
    (:file "_package_Select_Motion" :depends-on ("_package"))
  ))