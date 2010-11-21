
(in-package :asdf)

(defsystem "playground-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Encoder" :depends-on ("_package"))
    (:file "_package_Encoder" :depends-on ("_package"))
    ))
