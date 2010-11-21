; Auto-generated. Do not edit!


(in-package playground-msg)


;//! \htmlinclude Encoder.msg.html

(defclass <Encoder> (ros-message)
  ((x
    :reader x-val
    :initarg :x
    :type fixnum
    :initform 0)
   (y
    :reader y-val
    :initarg :y
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Encoder>) ostream)
  "Serializes a message object of type '<Encoder>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'y)) ostream)
)
(defmethod deserialize ((msg <Encoder>) istream)
  "Deserializes a message object of type '<Encoder>"
  (setf (ldb (byte 8 0) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'y)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Encoder>)))
  "Returns string type for a message object of type '<Encoder>"
  "playground/Encoder")
(defmethod md5sum ((type (eql '<Encoder>)))
  "Returns md5sum for a message object of type '<Encoder>"
  "6d78a6b8c9650c754bf0432d3d1707c3")
(defmethod message-definition ((type (eql '<Encoder>)))
  "Returns full string definition for message of type '<Encoder>"
  (format nil "int16 x~%int16 y~%~%~%"))
(defmethod serialization-length ((msg <Encoder>))
  (+ 0
     2
     2
))
(defmethod ros-message-to-list ((msg <Encoder>))
  "Converts a ROS message object to a list"
  (list '<Encoder>
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
))
