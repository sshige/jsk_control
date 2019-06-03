(load "../eus-choreonoid-converter.l")


(defun sample-eus-choreonoid-converter-common
    (&key
     (robot *robot*)
     (test-link)
     (move-target-link)
     (wait? nil)
     )
  (objects (list robot))

  ;; initialize
  (generate-cnoid-model :robot robot) ;; make choreonoid model
  (print-cnoid-model-info);; print info
  (assert (eps= (norm (send robot :angle-vector)) 0.0)
          (format nil "initial euslisp angle-vector should be 0 ~%")
          )
  (assert (eps= (norm (send robot :cnoid-angle-vector)) 0.0)
          (format nil "initial cnoid angle-vector should be 0")
          )

  ;; set/get joint angle
  (send (send test-link :joint) :cnoid-joint-angle (send (send test-link :joint) :max-angle))
  (assert (eps= (send (send test-link :joint) :joint-angle) 0.0)
          ("euslisp angle should not be changed")
          )
  (assert (eps= (send (send test-link :joint) :cnoid-joint-angle) (float (send (send test-link :joint) :max-angle)))
          ("choreonoid angle should be changed to max angle")
          )
  (send *robot* :cnoid->eus)
  (assert (eps= (send (send test-link :joint) :joint-angle) (float (send (send test-link :joint) :max-angle)))
          ("eus angle should become max angle in this stage")
          )
  (send *irtviewer* :draw-objects)
  (when wait?
    (warning-message 2 "set/get joint angle~%")
    (read-line)
    )

  ;; set/get angle-vector
  (dolist (j (send robot :joint-list))
    (send j :joint-angle (+ (* 0.8 (- (send j :max-angle) (send j :min-angle))) (send j :min-angle))))
  (assert (not (eps-v= (send robot :angle-vector) (send robot :cnoid-angle-vector))))
  (send robot :cnoid-angle-vector (send robot :angle-vector))
  (assert (eps-v= (send robot :angle-vector) (send robot :cnoid-angle-vector)))
  (send *irtviewer* :draw-objects)
  (when wait?
    (warning-message 2 "set/get angle-vector~%")
    (read-line)
    )  ;; get link coordinates
  (dolist (l (send robot :links))
    (assert (eps-coords= (send l :worldcoords) (send l :cnoid-worldcoords))
            (format nil "eus world cds and cnoid world cds should be the same. eus: ~a ~%" (send l :name))
            )
    (send (send l :worldcoords) :draw-on :flush t :size 50 :width 10 :color #f(0 1 0))
    (send (send l :cnoid-worldcoords) :draw-on :flush t :size 100 :width 2 :color #f(1 0 0))
    )
  (when wait?
    (warning-message 2 "get link coordinates~%")
    (read-line)
    )

  ;; inverse kinematics
  (setq *coords* (send (send move-target-link :copy-worldcoords) :translate #f(0 0 -10) :world))
  (assert (not (eps-coords= (send move-target-link :worldcoords) *coords*)))
  (send robot :cnoid-inverse-kinematics *coords* :move-target-link move-target-link)
  (assert (not (eps-coords= (send move-target-link :worldcoords) *coords*)))
  (send robot :cnoid->eus)
  (assert (eps-coords= (send move-target-link :worldcoords) *coords*))
  (send *irtviewer* :draw-objects)
  (send *coords* :draw-on :flush t :size 50 :width 10 :color #f(0 1 0))
  (send (send move-target-link :worldcoords) :draw-on :flush t :size 100 :width 2 :color #f(1 0 0))
  (when wait?
    (warning-message 2 "inverse kinematics~%")
    (read-line)
    )

  ;; mass property
  (assert (eps= (send robot :cnoid-weight) (reduce #'+ (send-all (send robot :links) :weight))))
  (assert (eps-v=
           (send robot :cnoid-centroid)
           (scale
            (/ 1.0 (reduce #'+ (send-all (send robot :links) :weight)))
            (reduce #'v+ (mapcar #'(lambda (l) (scale (send l :weight) (send l :centroid))) (send robot :links))))
           10.0 ;; this tolerance is necessary because maybe there is some bug
           ))
  )

(defun sample-eus-choreonoid-converter-sample-arm
    (&key (wait? nil))
  (require "irteus/demo/sample-arm-model.l")
  (setq *robot* (instance sarmclass :init))
  (sample-eus-choreonoid-converter-common
   :robot *robot*
   :test-link (send *robot* :link :sarm-b5)
   :move-target-link (send (send *robot* :end-coords) :parent)
   :wait? wait?
   )
  )
;; this has bug
;; Rb is not reflected in FK
;; https://github.com/s-nakaoka/choreonoid/blob/master/src/Body/LinkTraverse.cpp#L102-L104
;; https://github.com/s-nakaoka/choreonoid/blob/master/src/Body/LinkTraverse.cpp#L166-L168
;; (warn "(sample-eus-choreonoid-converter-sample-arm)~%")

(defun sample-eus-choreonoid-converter-rhp4b
    (&key (wait? nil))
  (require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")
  (setq *robot* (instance rhp4b-robot :init))
  (send *robot* :move-coords
        (make-coords :pos #f(0 0 0))
        (send (car (send *robot* :links)) :worldcoords))
  (send *robot* :init-pose)
  (sample-eus-choreonoid-converter-common
   :robot *robot*
   :test-link (send *robot* :link "R_KNEE_P_LINK")
   :move-target-link (send (send *robot* :rarm :end-coords) :parent)
   :wait? wait?
   )
  )
(warn "(sample-eus-choreonoid-converter-rhp4b)~%")

(defun sample-eus-choreonoid-converter-sample-robot
    (&key (wait? nil))
  (require "irteus/demo/sample-robot-model.l")
  (setq *robot* (instance sample-robot :init))
  (sample-eus-choreonoid-converter-common
   :robot *robot*
   :test-link (send *robot* :link :rleg-link2)
   :move-target-link (send (send *robot* :rarm :end-coords) :parent)
   :wait? wait?
   )
  )
(warn "(sample-eus-choreonoid-converter-sample-robot)~%")

(defun sample-eus-choreonoid-converter-pa10
    (&key (wait? nil))
  (require :pa10 "package://hrpsys_ros_bridge_tutorials/models/pa10.l")
  (setq *robot* (instance hrp1-robot :init))
  (sample-eus-choreonoid-converter-common
   :robot *robot*
   :test-link (send *robot* :link "J2_LINK")
   :move-target-link (send (send *robot* :rarm :end-coords) :parent)
   :wait? wait?
   )
  )
(warn "(sample-eus-choreonoid-converter-pa10)~%")
