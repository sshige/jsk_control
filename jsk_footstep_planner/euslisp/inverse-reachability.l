;; this is temporary
;; (require :jaxon-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")

;; (unless (boundp '*robot*)
;;   (jaxon-init)
;;   (setq *robot* *jaxon*))

;; usage example
;; (inverse-reachability-with-given-coords *robot* base-coords-list :rarm :inverse-kinematics target-coords)
(defun inverse-reachability-with-given-coords
  (robot base-coords-list
         &rest args)
  (let* ((ik-command-index (or (position :inverse-kinematics args) (position :fullbody-inverse-kinematics args)))
         (target-coords (elt args (+ ik-command-index 1)))
         (initial-angle-vector (send robot :angle-vector))
         (target-coords-transformed nil)
         (result-coords-list))
    (dotimes (i (length base-coords-list))
      ;; method consp = not (atom object)
      ;; if there exists target coords
      (print args)
      ;;if there exists target coords
      (if (consp target-coords)
          (progn
            ;; repeat below processes for all target coords
            (dotimes (j (length target-coords))
              (setq target-coords-transformed
                    (append target-coords-transformed
                            ;; method transformation = make relationship between the subject coords and give coords
                            ;; make local relationiship coords between target coords and base coords
                            (send (send (send (elt target-coords j) :copy-worldcoords) :transformation (elt base-coords-list i) :local)
                                  :transformation (make-coords) :local)))))
        ;; if this is first
        (setq target-coords-transformed
              (send (send (send target-coords :copy-worldcoords) :transformation (elt base-coords-list i) :local)
                    :transformation (make-coords) :local)))
      ;; set the relationship to the args
      (setf (elt args (+ ik-command-index 1)) target-coords-transformed)
      (if (forward-message-to robot args)
          (setq result-coords-list (append result-coords-list (list (elt base-coords-list i)))))
      (send robot :angle-vector initial-angle-vector))
    result-coords-list))

(defun make-coords-list-for-inverse-reachability ; by args sent to go-pos
  (&key (max-x 0.5) (min-x -0.5) (dx 0.2) (max-y 0.5) (min-y -0.5) (dy 0.2) (max-d 30) (min-d -30) (dd 15))
  (let ((output-coords-list nil))
    (dotimes (ix (+ 1 (round (/ (- max-x min-x) dx))))
      (let ((x-index (+ min-x (* ix dx))))
        (dotimes (iy (+ 1 (round (/ (- max-y min-y) dy))))
          (let ((y-index (+ min-y (* iy dy))))
            (dotimes (id (+ 1 (round (/ (- max-d min-d) dd))))
              (let ((d-index (+ min-d (* id dd))))
                (setq output-coords-list (append output-coords-list (list
                                                                     (make-coords :pos (float-vector (* x-index 1000) (* y-index 1000) 0)
                                                                                  :rpy (float-vector (deg2rad d-index) 0 0)))))))))))
    output-coords-list))

(defun prioritize-coords
  (cds-lst)
  (let ((pos-vec-lst) ;; list of all position vectors
        (pos-vec-cnt-lst) ;; list of how many times the vectors appear
        (pos-vec-and-cnt-lst)
        )
    (dotimes (idx (length cds-lst))
      (if (consp pos-vec-lst)
          (if (setq tmp-lst (member (send (elt cds-lst idx) :pos) pos-vec-lst :test v=))
              (incf (elt pos-vec-cnt-lst (- (length pos-vec-lst) (length tmp-lst))))
              (progn
                (setq pos-vec-lst (list* (send (elt cds-lst idx) :pos) pos-vec-lst))
                (setq pos-vec-cnt-lst (list* 1 pos-vec-cnt-lst))))
        (progn
          (setq pos-vec-lst (send (elt cds-lst idx) :pos))
          (setq pos-vec-cnt-lst '(1)))))
    (print (length pos-vec-lst))
    (print (length pos-vec-cnt-lst))
    (dotimes (idx (length pos-vec-lst))
      (if (eq idx 0)
          (setq pos-vec-cnt-lst (list (elt pos-vec-lst idx) (elt pos-vec-cnt-lst idx)))
        (setq pos-vec-and-cnt-lst (list* (list (elt pos-vec-lst idx) (elt pos-vec-cnt-lst idx)) pos-vec-and-cnt-lst))))
    (sort pos-vec-and-cnt-lst #'<= :key #'cdr)
    pos-vec-and-cnt-lst))
