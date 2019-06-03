(load "../eus-choreonoid-converter.l")

;; right hand forward

(defun rhp4b-choreonoid-ik
    ()
  (require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")
  (setq *robot* (instance rhp4b-robot :init))
  (send *robot* :move-coords
        (make-coords :pos #f(0 0 0))
        (send (car (send *robot* :links)) :worldcoords))
  (send *robot* :init-pose)
  (generate-cnoid-model :robot *robot*) ;; make choreonoid model
  (send *robot* :reset-pose)
  (send *robot* :eus->cnoid)
  (setq *coords* (send (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                       :translate #f(0 0 200) :world))
  (send *robot* :cnoid-inverse-kinematics *coords* :move-target-link (send *robot* :link "R_ANKLE_P_LINK"))
  (setq *coords* (send (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                       :translate #f(200 0 0) :world))
  (send *robot* :cnoid-inverse-kinematics *coords* :move-target-link (send *robot* :link "R_WRIST_Y_LINK"))
  (send *robot* :cnoid->eus)
  (objects (list *robot*))
  )
(warning-message 2 "(rhp4b-choreonoid-ik)~%")
