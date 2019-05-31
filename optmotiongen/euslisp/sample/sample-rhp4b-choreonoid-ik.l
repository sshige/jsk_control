(load "../eus-choreonoid-converter.l")

;; right hand forward

;;TODO this is not work yet
(defun rhp4b-choreonoid-ik
    ()
  (require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")
  (setq *robot* (instance rhp4b-robot :init))
  (send *robot* :move-coords
        (make-coords :pos #f(0 0 0))
        (send (car (send *robot* :links)) :worldcoords))
  (send *robot* :init-pose)
  (generate-cnoid-model :robot *robot*) ;; make choreonoid model
  (setq *coords* (send (send (send *robot* :link "R_HAND_LINK") :copy-worldcoords)
                       :translate #f(100 0 0) :world))
  (send *robot* :cnoid-inverse-kinematics *coords* :move-target-link (send *robot* :link "R_HAND_LINK"))
  (send *robot* :cnoid->eus)
  (objects (list *robot*))
  )
