CHOREONOID_IK_PLUGIN_PATH=/home/riku/catkin_ws/jaxon_tutorial/devel/.private/optmotiongen/lib

# use the method of https://stackoverflow.com/questions/1396066/detect-if-users-path-has-a-specific-directory-in-it/1397020#1397020
if [[ ! ":$CNOID_PLUGIN_PATH:" == *":$CHOREONOID_IK_PLUGIN_PATH:"* ]]; then
    export CNOID_PLUGIN_PATH=/home/riku/catkin_ws/jaxon_tutorial/devel/.private/optmotiongen/lib:${CNOID_PLUGIN_PATH}
fi
