(unless (find-package "CNOID") (make-package "CNOID" :nicknames '("CNOID")))
(in-package "CNOID")

(defvar *eus-choreonoid-converter-lib*
  (labels
      ((library_search
        (str &key colon-pos lib-path)
        (cond
         ((eq (length str) 0)
          (format t "~% libEusChoreonoidConverter.so not found~%")
          (exit -1))
         ((and (setq colon-pos (or (position #\: str) (length str)))
               (setq lib-path (subseq str 0 colon-pos))
               (setq lib-path
                     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
                         (subseq lib-path 0 (- (length lib-path) 1))
                       lib-path))
               (probe-file (setq lib-path
                                 (format nil "~A/libEusChoreonoidConverter.so" lib-path))))
          (load-foreign lib-path))
         (t
          (library_search (subseq str (min (length str) (+ colon-pos 1))))))))
    (library_search (format nil "~A" (unix:getenv "LD_LIBRARY_PATH")))))

(defconstant *joint-type-rotational* 0)
(defconstant *joint-type-linear* 1)

(defun joint->joint-type
    (jnt)
  (cond ((derivedp jnt user::rotational-joint)
         cnoid::*joint-type-rotational*
         )
        ((derivedp jnt user::linear-joint)
         cnoid::*joint-type-rotational*
         )
        (t
         (error "unsupported joint class: ~a~%" jnt)
         ))
  )

(defun axis->vector
    (axis)
  (case axis
        (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
        (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
        (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
        (t axis))
  )

;; long callInitializeBodyGeneration(char *bodyName)
(defforeign initialize-body-generation
  *eus-choreonoid-converter-lib*
  "callInitializeBodyGeneration"
  ()
  :integer
  )

;; long callAddLinkFromLinkInfo(long convAddr,
;;                              char *name, char *parentName,
;;                              double *pos, double *quat,
;;                              long jointId, long jointType, double* jointAxis,
;;                              double lower, double upper,
;;                              double mass, double *com, double *inertia)
(defforeign add-link-from-link-info
  *eus-choreonoid-converter-lib*
  "callAddLinkFromLinkInfo"
  ()
  :integer
  )

;; long callFinalizeBodyGeneration(long convAddr)
(defforeign finalize-body-generation
  *eus-choreonoid-converter-lib*
  "callFinalizeBodyGeneration"
  ()
  :integer
  )

;; long callPrintBodyInfo(long convAddr)
(defforeign print-body-info
  *eus-choreonoid-converter-lib*
  "callPrintBodyInfo"
  ()
  :integer
  )

;; long callSetJointAngle(long convAddr, char *name, double angle)
(defforeign set-joint-angle
  *eus-choreonoid-converter-lib*
  "callSetJointAngle"
  ()
  :integer
  )

;; long callGetJointAngle(long convAddr, char *name, double *angle)
(defforeign get-joint-angle
  *eus-choreonoid-converter-lib*
  "callGetJointAngle"
  ()
  :integer
  )

;; long callSetJointAngleAll(long convAddr, double *angleAllArr)
(defforeign set-joint-angle-all
  *eus-choreonoid-converter-lib*
  "callSetJointAngleAll"
  ()
  :integer
  )

;; long callGetJointAngleAll(long convAddr, double *angleAllArr)
(defforeign get-joint-angle-all
  *eus-choreonoid-converter-lib*
  "callGetJointAngleAll"
  ()
  :integer
  )

;; long callGetLinkPose(long convAddr, char *name, double *pos, double *quat)
(defforeign get-link-pose
  *eus-choreonoid-converter-lib*
  "callGetLinkPose"
  ()
  :integer
  )

;; long callCalcInverseKinematics(long convAddr, char *startLinkName, char *endLinkName, double *pos, double *quat, double *angleAllArr)
(defforeign calc-inverse-kinematics
  *eus-choreonoid-converter-lib*
  "callCalcInverseKinematics"
  ()
  :integer
  )

;; long callGetMassProp(long convAddr, double *mass, double *comArr)
(defforeign get-mass-prop
  *eus-choreonoid-converter-lib*
  "callGetMassProp"
  ()
  :integer
  )


(in-package "USER")

(defun generate-cnoid-model
    (&key (robot *robot*))
  "
Generate choreonoid model from euslisp cascaded-link instance.
Pointer of choreonoid body is stored, and passed when calling other functions.
  "
  (send robot :put :cnoid-model (cnoid::initialize-body-generation (string (send robot :name))))
  (let (jnt)
    (dolist (link (send robot :links))
      (setq jnt (send link :joint))
      (cnoid::add-link-from-link-info
       (send robot :get :cnoid-model)
       (string (send link :name)) ;; name
       (if (send link :parent-link) (string (send (send link :parent-link) :name)) "") ;; parentName
       (scale 1e-3 (send link :pos)) ;; pos
       (matrix2quaternion (send link :rot)) ;; quat
       (if jnt (position jnt (send robot :joint-list)) -1) ;; jointId
       (if jnt (cnoid::joint->joint-type jnt) -1) ;; jointType
       (if jnt (cnoid::axis->vector (send jnt :get-val 'axis)) (float-vector 0 0 0)) ;; jointAxis
       (if jnt (deg2rad (send jnt :min-angle)) 0.0) ;; lower
       (if jnt (deg2rad (send jnt :max-angle)) 0.0) ;; upper
       (* 1e-3 (send link :weight)) ;; mass
       (scale 1e-3 (send link :get-val 'acentroid)) ;; com
       (scale 1e-9 (array-entity (send link :get-val 'inertia-tensor))) ;; inertia
       )
      ))
  (cnoid::finalize-body-generation (send robot :get :cnoid-model))
  )

(defun print-cnoid-model-info
    (&key (robot *robot*))
  "
Print choreonoid model information.
  "
  (unless (send robot :get :cnoid-model)
    (warning-message 1 "cnoid model is not initialized.~%")
    (return-from print-cnoid-model-info nil)
    )
  (cnoid::print-body-info (send robot :get :cnoid-model))
  )

(defmethod joint
  (:cnoid-joint-angle
   (&optional
    (angle)
    &key
    (angle-buffer)
    (robot *robot*)
    (cnoid-model (send robot :get :cnoid-model))
    )
   "
Set/get joint angle of choreonoid model. Euslisp model does not change.
   "
   (cond (angle
          (cnoid::set-joint-angle cnoid-model (string (send child-link :name)) (deg2rad angle))
          )
         (t
          (unless angle-buffer
            (setq angle-buffer (float-vector 0)))
          (cnoid::get-joint-angle cnoid-model (string (send child-link :name)) angle-buffer)
          (rad2deg (elt angle-buffer 0))
          ))
   )
  )

(defmethod bodyset-link
  (:cnoid-worldcoords
   (&key
    (pos-buffer (float-vector 0 0 0))
    (quat-buffer (float-vector 0 0 0 0))
    (robot *robot*)
    (cnoid-model (send robot :get :cnoid-model))
    )
   "
Get link coordinates of choreonoid model. Euslisp model does not change.
   "
   (if (= (cnoid::get-link-pose cnoid-model (string (send self :name)) pos-buffer quat-buffer) 0)
       (make-coords :pos (scale 1e3 pos-buffer) :rot (quaternion2matrix quat-buffer)))
   )
  )

(defmethod cascaded-link
  (:cnoid-angle-vector
   (&optional
    (av)
    &key
    (av-buffer) ;; only for getting angle-vector
    )
   "
Set/get angle-vector of choreonoid model. Euslisp model does not change.
   "
   (cond (av
          (cnoid::set-joint-angle-all
           (send self :get :cnoid-model)
           (map float-vector #'deg2rad av))
          )
         (t
          (unless av-buffer
            (setq av-buffer (make-array (length joint-list) :element-type :float)))
          (cnoid::get-joint-angle-all (send self :get :cnoid-model) av-buffer)
          (dotimes (i (length joint-list))
            (setf (elt av-buffer i) (rad2deg (elt av-buffer i))))
          av-buffer
          ))
   )
  (:eus->cnoid
   ()
   "
Apply angle-vector from euslisp model to choreonoid model.
   "
   (send self :cnoid-angle-vector (send self :angle-vector))
   )
  (:cnoid->eus
   ()
   "
Apply angle-vector from choreonoid model to euslisp model.
   "
   (send self :angle-vector (send self :cnoid-angle-vector))
   )
  (:cnoid-inverse-kinematics
   (target-coords
    &key
    (move-target-link)
    (root-link (car (send self :links)))
    (av-buffer (make-array (length joint-list) :element-type :float))
    )
   "
Solve inverse-kinematics with choreonoid model. Euslisp model does not change.
   "
   (cond ((= (cnoid::calc-inverse-kinematics
              (send self :get :cnoid-model)
              (string (send root-link :name))
              (string (send move-target-link :name))
              (scale 1e-3 (send target-coords :worldpos))
              (matrix2quaternion (send target-coords :worldrot))
              av-buffer)
             0)
          (dotimes (i (length joint-list))
            (setf (elt av-buffer i) (rad2deg (elt av-buffer i))))
          av-buffer
          )
         (t
          nil
          ))
   )
  (:cnoid-weight
   (&key
    (mass-buffer (float-vector 0))
    (com-buffer (float-vector 0 0 0))
    )
   "
Get weight of choreonoid model.
   "
   (cnoid::get-mass-prop (send self :get :cnoid-model) mass-buffer com-buffer)
   (* 1e3 (elt mass-buffer 0))
   )
  (:cnoid-centroid
   (&key
    (mass-buffer (float-vector 0))
    (com-buffer (float-vector 0 0 0))
    )
   "
Get centroid of choreonoid model.
   "
   (cnoid::get-mass-prop (send self :get :cnoid-model) mass-buffer com-buffer)
   (scale 1e3 com-buffer)
   )
  )
