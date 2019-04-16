(require :pa10 "package://hrpsys_ros_bridge_tutorials/models/pa10.l")

(defun main
    ()
  (setq *robot* (instance hrp1-robot :init))
  (objects (list *robot*))

  (warning-message 2 "== robot information ==~%")
  (warning-message 7 "dof: ~a~%" (length (send *robot* :joint-list)))
  (warning-message 7 "root-link name: ~a~%" (send (car (send *robot* :links)) :name))
  (warning-message 7 "root-link pos: ~a~%" (send (car (send *robot* :links)) :worldpos))
  (warning-message 7 "root-link rot:~%")
  (format-array (send (car (send *robot* :links)) :worldrot))

  (warning-message 2 "== joint path information ==~%")
  (setq *link-list*
        (send *robot* :link-list (send *robot* :link "J7_LINK")))
  (setq *end-coords*
        (make-cascoords :parent (car (last *link-list*))
                        :coords (send (car (last *link-list*)) :copy-worldcoords)
                        ))
  (dotimes (i (length *link-list*))
    (warning-message 7 "joint [~a]: ~a~%" i (send (elt *link-list* i) :joint :name)))
  (warning-message 7 "end-coords: ~a~%" (send (send *end-coords* :parent) :name))

  (warning-message 2 "== FK ==~%")
  (dotimes (i (length *link-list*))
    (send (elt *link-list* i) :joint :joint-angle (rad2deg 0.5))
    (warning-message 7 "joint angle [~a]: ~a~%" i (deg2rad (send (elt *link-list* i) :joint :joint-angle)))
    )
  (warning-message 7 "end-coords pos: ~a~%" (send *end-coords* :worldpos))
  (warning-message 7 "end-coords rot:~%")
  (format-array (send *end-coords* :worldrot))

  (warning-message 2 "== IK ==~%")
  (send *robot* :inverse-kinematics
        (send (send *end-coords* :copy-worldcoords) :translate (float-vector 0 0 -10) :world)
        :move-target *end-coords*
        :link-list *link-list*)
  (dotimes (i (length *link-list*))
    (warning-message 7 "joint angle [~a]: ~a~%" i (deg2rad (send (elt *link-list* i) :joint :joint-angle)))
    )
  (warning-message 7 "end-coords pos: ~a~%" (send *end-coords* :worldpos))
  (warning-message 7 "end-coords rot:~%")
  (format-array (send *end-coords* :worldrot))
  )
(main)
