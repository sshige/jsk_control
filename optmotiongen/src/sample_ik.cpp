#include <iostream>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>
#include <cnoid/FileUtil>
#include <ros/package.h>


using namespace std;
using namespace cnoid;
using namespace Eigen;

int main(int argc, char** argv)
{
  // load robot model
  string modelfile = getNativePathString(boost::filesystem::path(ros::package::getPath("choreonoid_ik")) / "models/PA10/PA10.body");
  BodyLoader loader;
  BodyPtr robot = loader.load(modelfile);
  if(!robot){
    cout << modelfile << " cannot be loaded." << endl;
    return 1;
  }

  // print robot information
  cout << "== robot information ==" << endl;
  cout << "dof: " << robot->numJoints() << endl;
  cout << "base link name: " << robot->rootLink()->name() << endl;
  cout << "base link pos: \n" << robot->rootLink()->p() << endl;
  cout << "base link rot: \n" << robot->rootLink()->R() << endl;

  // joint path information
  cout << "== joint path information ==" << endl;
  JointPathPtr jointPathRootToEnd = getCustomJointPath(robot, robot->rootLink(), robot->link("J7"));
  Link *linkEnd = jointPathRootToEnd->endLink();
  for (int i = 0; i < jointPathRootToEnd->numJoints(); i++) {
    cout << "joint(" << i << ")->name(): " << jointPathRootToEnd->joint(i)->name() << endl;
  }
  cout << "linkEnd->name(): " << linkEnd->name() << endl;

  // calculate FK
  cout << "== FK ==" << endl;
  for (int i = 0; i < jointPathRootToEnd->numJoints(); i++) {
    jointPathRootToEnd->joint(i)->q() = 0.5;
    cout << "joint(" << i << ")->q(): " << jointPathRootToEnd->joint(i)->q() << endl;
  }
  jointPathRootToEnd->calcForwardKinematics();
  Position currentEndCoords = linkEnd->T();
  cout << "currentEndPos: \n" << currentEndCoords.translation() << endl;
  cout << "currentEndRot: \n" << currentEndCoords.linear() << endl;

  // calculate IK
  cout << "== IK ==" << endl;
 Position targetEndCoords = currentEndCoords;
  targetEndCoords.translation().z() -= 0.01;
  if (jointPathRootToEnd->calcInverseKinematics(targetEndCoords)) {
    for (int i = 0; i < jointPathRootToEnd->numJoints(); i++) {
      cout << "joint(" << i << ")->q(): " << jointPathRootToEnd->joint(i)->q() << endl;
    }
    Position resultEndCoords = linkEnd->T();
    cout << "resultEndPos: \n" << resultEndCoords.translation() << endl;
    cout << "resultEndRot: \n" << resultEndCoords.linear() << endl;
  } else {
    cout << "failed to solve IK" << endl;
  }

  return 0;
}
