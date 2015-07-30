
(cl:in-package :asdf)

(defsystem "diagnostic-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Feedback" :depends-on ("_package_Feedback"))
    (:file "_package_Feedback" :depends-on ("_package"))
    (:file "LaserScan" :depends-on ("_package_LaserScan"))
    (:file "_package_LaserScan" :depends-on ("_package"))
    (:file "Status" :depends-on ("_package_Status"))
    (:file "_package_Status" :depends-on ("_package"))
  ))