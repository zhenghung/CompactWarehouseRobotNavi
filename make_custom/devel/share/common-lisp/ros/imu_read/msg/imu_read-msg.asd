
(cl:in-package :asdf)

(defsystem "imu_read-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "imu_read" :depends-on ("_package_imu_read"))
    (:file "_package_imu_read" :depends-on ("_package"))
  ))