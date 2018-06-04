;; Auto-generated. Do not edit!


(when (boundp 'rslidar_msgs::rslidarPic)
  (if (not (find-package "RSLIDAR_MSGS"))
    (make-package "RSLIDAR_MSGS"))
  (shadow 'rslidarPic (find-package "RSLIDAR_MSGS")))
(unless (find-package "RSLIDAR_MSGS::RSLIDARPIC")
  (make-package "RSLIDAR_MSGS::RSLIDARPIC"))

(in-package "ROS")
;;//! \htmlinclude rslidarPic.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rslidar_msgs::rslidarPic
  :super ros::object
  :slots (_header _col _distance _intensity _azimuthforeachP ))

(defmethod rslidar_msgs::rslidarPic
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:col __col) 0)
    ((:distance __distance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:intensity __intensity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:azimuthforeachP __azimuthforeachP) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _col (round __col))
   (setq _distance __distance)
   (setq _intensity __intensity)
   (setq _azimuthforeachP __azimuthforeachP)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:col
   (&optional __col)
   (if __col (setq _col __col)) _col)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:azimuthforeachP
   (&optional __azimuthforeachP)
   (if __azimuthforeachP (setq _azimuthforeachP __azimuthforeachP)) _azimuthforeachP)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _col
    4
    ;; float32[] _distance
    (* 4    (length _distance)) 4
    ;; float32[] _intensity
    (* 4    (length _intensity)) 4
    ;; float32[] _azimuthforeachP
    (* 4    (length _azimuthforeachP)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _col
       (write-long _col s)
     ;; float32[] _distance
     (write-long (length _distance) s)
     (dotimes (i (length _distance))
       (sys::poke (elt _distance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _intensity
     (write-long (length _intensity) s)
     (dotimes (i (length _intensity))
       (sys::poke (elt _intensity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _azimuthforeachP
     (write-long (length _azimuthforeachP) s)
     (dotimes (i (length _azimuthforeachP))
       (sys::poke (elt _azimuthforeachP i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _col
     (setq _col (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _distance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _intensity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _intensity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _intensity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _azimuthforeachP
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _azimuthforeachP (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _azimuthforeachP i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rslidar_msgs::rslidarPic :md5sum-) "f6082ae0a03112c5dd200b5cae6f683d")
(setf (get rslidar_msgs::rslidarPic :datatype-) "rslidar_msgs/rslidarPic")
(setf (get rslidar_msgs::rslidarPic :definition-)
      "Header		    header
uint32               col
float32[]           distance
float32[]           intensity
float32[]           azimuthforeachP

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :rslidar_msgs/rslidarPic "f6082ae0a03112c5dd200b5cae6f683d")


