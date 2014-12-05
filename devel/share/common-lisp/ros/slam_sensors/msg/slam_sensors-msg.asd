
(cl:in-package :asdf)

(defsystem "slam_sensors-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RobotVelocity" :depends-on ("_package_RobotVelocity"))
    (:file "_package_RobotVelocity" :depends-on ("_package"))
  ))