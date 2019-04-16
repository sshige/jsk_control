#include <iostream>
#include <cnoid/Plugin>
#include <cnoid/ToolBar>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>
#include <ros/package.h>


using namespace std;
using namespace cnoid;
using namespace Eigen;

class SampleIKPlugin : public Plugin
{
public:

  SampleIKPlugin() : Plugin("Sample1")
  {
    require("Body");
  }

  virtual bool initialize()
  {
    // initialize GUI
    {
      auto bar = new ToolBar("Sample1");
      auto button0 = bar->addButton("Load robot");
      button0->sigClicked().connect([&](){ onInitButtonClicked(); });
      auto button1 = bar->addButton("Increment Z");
      button1->sigClicked().connect([&](){ onUpdateButtonClicked(0.04); });
      auto button2 = bar->addButton("Decrement Z");
      button2->sigClicked().connect([&](){ onUpdateButtonClicked(-0.04); });
      bar->setVisibleByDefault(true);
      addToolBar(bar);
    }

    return true;
  }

private:

  void onInitButtonClicked()
  {
    auto bodyItems = ItemTreeView::instance()->selectedItems<BodyItem>();
    if (bodyItems.size() == 0) {
      cerr << "failed to load robot because no body item is selected." << endl;
      return;
    }
    bodyItem_ = bodyItems[0];
    robot_ = bodyItem_->body();

    // print robot information
    cout << "== robot information ==" << endl;
    cout << "dof: " << robot_->numJoints() << endl;
    cout << "base link name: " << robot_->rootLink()->name() << endl;
    cout << "base link pos: \n" << robot_->rootLink()->p() << endl;
    cout << "base link rot: \n" << robot_->rootLink()->R() << endl;

    // joint path information
    cout << "== joint path information ==" << endl;
    jointPathRootToEnd_ = getCustomJointPath(robot_, robot_->rootLink(), robot_->link("J7"));
    linkEnd_ = jointPathRootToEnd_->endLink();
    for (int i = 0; i < jointPathRootToEnd_->numJoints(); i++) {
      cout << "joint(" << i << ")->name(): " << jointPathRootToEnd_->joint(i)->name() << endl;
    }
    cout << "linkEnd->name(): " << linkEnd_->name() << endl;

    // initialize joint angle
    for (int i = 0; i < jointPathRootToEnd_->numJoints(); i++) {
      jointPathRootToEnd_->joint(i)->q() = 0.5;
    }
    bodyItem_->notifyKinematicStateChange(true);
  }

  void onUpdateButtonClicked(double dz)
  {
    if (!robot_) {
      cerr << "robot is not initialized yet." << endl;
      return;
    }

    // calculate FK
    cout << "== FK ==" << endl;
    jointPathRootToEnd_->calcForwardKinematics();
    Position currentEndCoords = linkEnd_->T();
    cout << "currentEndPos: \n" << currentEndCoords.translation() << endl;
    cout << "currentEndRot: \n" << currentEndCoords.linear() << endl;

    // calculate IK
    cout << "== IK ==" << endl;
    Position targetEndCoords = currentEndCoords;
    targetEndCoords.translation().z() += dz;
    if (jointPathRootToEnd_->calcInverseKinematics(targetEndCoords)) {
      cout << "succeeded to solve IK" << endl;
    } else {
      cout << "failed to solve IK" << endl;
    }

    bodyItem_->notifyKinematicStateChange(true);
  }

  BodyPtr robot_;
  BodyItem* bodyItem_;
  JointPathPtr jointPathRootToEnd_;
  Link* linkEnd_;
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SampleIKPlugin)


int main(int argc, char** argv)
{


  return 0;
}
