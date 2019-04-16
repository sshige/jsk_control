#include <iostream>
#include "SimpleBodyGenerator.h"


using namespace cnoid;
using namespace std;

void SimpleBodyGenerator::initializeBodyGeneration(const string &bodyName)
{
  DEBUG_PRINT("initialize body. name: " << bodyName);

  nameLinkInfoMap.clear();
  body = new Body;
  body->setName(bodyName);
  body->setModelName(bodyName);
}

void SimpleBodyGenerator::addLinkFromLinkInfo(LinkInfo &linkInfo)
{
  DEBUG_PRINT("add link. name: " << linkInfo.name);

  Link* link = body->createLink();

  // basic info
  link->setName(linkInfo.name);
  link->setOffsetTranslation(linkInfo.pose.translation());
  link->setOffsetRotation(linkInfo.pose.linear());

  // joint info
  if (linkInfo.jointId >= 0) {
    DEBUG_PRINT("add joint. name: " << link->name());
    link->setJointId(linkInfo.jointId);
    link->setJointType(linkInfo.jointType);
    link->setJointAxis(linkInfo.jointAxis);
    link->setJointRange(linkInfo.lower, linkInfo.upper);
  }

  // physical info
  link->setCenterOfMass(linkInfo.com);
  link->setMass(linkInfo.mass);
  link->setInertia(linkInfo.inertia);

  linkInfo.link = link;
  nameLinkInfoMap.insert(make_pair(linkInfo.name, linkInfo));
}

void SimpleBodyGenerator::finalizeBodyGeneration()
{
  DEBUG_PRINT("finalize.");

  Link* rootLink = nullptr;
  for (auto nameLinkInfo : nameLinkInfoMap) {
    Link* link = nameLinkInfo.second.link;
    string parentName = nameLinkInfo.second.parentName;
    if (nameLinkInfoMap.find(parentName) == nameLinkInfoMap.end()) {
      DEBUG_PRINT("Root link is " << nameLinkInfo.first);
      if (rootLink != nullptr) {
        cerr << ";; [SimpleBodyGenerator] multiple root link found. " << rootLink->name() << ", " << link->name() << endl;
      }
      rootLink = link;
    } else {
      DEBUG_PRINT("Parent link of " << nameLinkInfo.first << " is " << parentName);
      Link* parentLink = nameLinkInfoMap[parentName].link;
      parentLink->appendChild(link);
    }
  }
  body->setRootLink(rootLink);

  body->expandLinkOffsetRotations();
  // body->installCustomizer();
}

void SimpleBodyGenerator::generateBodyFromLinkInfo(const string &bodyName, const vector<LinkInfo> &linkInfoList)
{
  initializeBodyGeneration(bodyName);
  for (auto linkInfo : linkInfoList) {
    addLinkFromLinkInfo(linkInfo);
  }
  finalizeBodyGeneration();
}

void SimpleBodyGenerator::printBodyInfo()
{
  cout << "====== body information ======" << endl;

  // FK is necessary to get coords and com of link in world frame.
  body->calcForwardKinematics();

  cout << "== basic information ==" << endl;
  cout << "name: " << body->name() << endl;
  cout << "dof: " << body->numJoints() << endl;

  cout << "== root information ==" << endl;
  cout << "root link name: " << body->rootLink()->name() << endl;
  cout << "root link pos: \n" << body->rootLink()->p() << endl;
  cout << "root link rot: \n" << body->rootLink()->R() << endl;

  cout << "== link information ==" << endl;
  cout << "num of links: " << body->numLinks() << endl;
  for (int i = 0; i < body->numLinks(); i++) {
    Link* link = body->link(i);
    cout << "link[" << i << "] name: " << link->name() << endl;
    cout << "link[" << i << "] pos (in local frame): \n" << link->offsetTranslation() << endl;
    cout << "link[" << i << "] rot (in local frame): \n" << link->offsetRotation() << endl;
    cout << "link[" << i << "] pos (in world frame): \n" << link->p() << endl;
    cout << "link[" << i << "] rot (in world frame): \n" << link->R() << endl;
  }

  cout << "== joint information ==" << endl;
  cout << "num of joints: " << body->numJoints() << endl;
  for (int i = 0; i < body->numJoints(); i++) {
    Link* joint = body->joint(i);
    cout << "joint[" << i << "] name: " << joint->name() << endl;
    cout << "joint[" << i << "] angle: " << joint->q() << endl;
    cout << "joint[" << i << "] joint type: " << joint->jointType() << endl;
    cout << "joint[" << i << "] axis: \n" << joint->jointAxis() << endl;
  }

  cout << "== physical information ==" << endl;
  cout << "whole mass: " << body->mass() << endl;
  cout << "whole com: \n" << body->calcCenterOfMass() << endl;
  for (int i = 0; i < body->numLinks(); i++) {
    Link* link = body->link(i);
    cout << "link[" << i << "] mass: " << link->mass() << endl;
    cout << "link[" << i << "] com (in world frame): \n" << link->wc() << endl;
  }
}

int SimpleBodyGenerator::setJointAngle(const string &name, const double &angle)
{
  auto link = body->link(name);
  if (!link) {
    cerr << "link " << name << " not found." << endl;
    return -1;
  }
  body->link(name)->q() = angle;
  return 0;
}

int SimpleBodyGenerator::getJointAngle(const string &name, double &angle)
{
  auto link = body->link(name);
  if (!link) {
    cerr << "link " << name << " not found." << endl;
    return -1;
  }
  angle = body->link(name)->q();
  return 0;
}

void SimpleBodyGenerator::setJointAngleAll(const vector<double> &angleAll)
{
  for (int i = 0; i < body->numJoints(); i++) {
    body->joint(i)->q() = angleAll[i];
  }
}

void SimpleBodyGenerator::getJointAngleAll(vector<double> &angleAll)
{
  for (int i = 0; i < body->numJoints(); i++) {
    angleAll[i] = body->joint(i)->q();
  }
}

int SimpleBodyGenerator::getLinkPose(const string &name, Position &pose)
{
  auto link = body->link(name);
  if (!link) {
    cerr << "link " << name << " not found." << endl;
    return -1;
  }
  body->calcForwardKinematics();
  pose = link->T();
  return 0;
}

int SimpleBodyGenerator::calcInverseKinematics(const string &startLinkName, const string &endLinkName,
                                               const Position &targetPose,
                                               vector<double> &angleAll)
{
  DEBUG_PRINT("IK.");

  // generate joint path
  Link* startLink = body->rootLink();
  Link* endLink = body->link(endLinkName);
  if (! startLinkName.empty()) {
    startLink = body->link(startLinkName);
    if (!startLink) {
      cerr << "link " << startLinkName << " not found." << endl;
      return -1;
    }
  }
  if (!endLink) {
    cerr << "link " << endLinkName << " not found." << endl;
    return -1;
  }
  DEBUG_PRINT("start link name: " << startLink->name());
  DEBUG_PRINT("end link name: " << endLink->name());
  JointPathPtr jointPath = getCustomJointPath(body, startLink, endLink);
  for (int i = 0; i < jointPath->numJoints(); i++) {
    DEBUG_PRINT("joint[" << i << "] name: " << jointPath->joint(i)->name());
  }

  // solve IK and set result
  jointPath->calcForwardKinematics();
  if (jointPath->calcInverseKinematics(targetPose)) {
    DEBUG_PRINT("succeeded to solve IK.");
    angleAll.resize(jointPath->numJoints());
    for (int i = 0; i < jointPath->numJoints(); i++) {
      DEBUG_PRINT("joint[" << i << "] angle: " << jointPath->joint(i)->q());
      angleAll[i] = jointPath->joint(i)->q();
    }
  } else {
    DEBUG_PRINT("failed to solve IK.");
    return 1;
  }
  return 0;
}
