#include <iostream>
#include "SimpleBodyGenerator.h"

using namespace cnoid;
using namespace std;

extern "C" {
  /** \brief Initialize body generation.
   *
   * New SimpleBodyGenerator instance is generated and its pointer is returned.
   * The returned adress needs to be passed when calling other functions.
   */
  long callInitializeBodyGeneration(char *bodyName)
  {
    SimpleBodyGenerator *pconv = new SimpleBodyGenerator;
    pconv->initializeBodyGeneration(string(bodyName));
    return (long)pconv;
  }

  /** \brief Add link to body.
   *
   * LinkInfo instance is generated from arguments.
   */
  long callAddLinkFromLinkInfo(long convAddr,
                               char *name, char *parentName,
                               double *pos, double *quat,
                               long jointId, long jointType, double* jointAxis,
                               double lower, double upper,
                               double mass, double *com, double *inertia)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    Position pose;
    pose.translation() = Vector3(pos);
    pose.linear() = Quat(quat[0], quat[1], quat[2], quat[3]).normalized().toRotationMatrix();
    SimpleBodyGenerator::LinkInfo linkInfo = {
      // basic info
      .name = string(name),
      .parentName = string(parentName),
      .pose = pose,
      // joint info
      .jointId = (int)jointId,
      .jointType = (Link::JointType)jointType,
      .jointAxis = Vector3(jointAxis),
      .lower = lower,
      .upper = upper,
      // physical info
      .mass = mass,
      .com = Vector3(com),
      .inertia = Matrix3(inertia),
    };
    pconv->addLinkFromLinkInfo(linkInfo);
    return 0;
  }

  /** \brief Finalize body generation. */
  long callFinalizeBodyGeneration(long convAddr)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    pconv->finalizeBodyGeneration();
    return 0;
  }

  /** \brief Print body information. */
  long callPrintBodyInfo(long convAddr)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    pconv->printBodyInfo();
    return 0;
  }

  /** \brief Set joint angle of joint with the specified name. */
  long callSetJointAngle(long convAddr, char *name, double angle)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    return pconv->setJointAngle(string(name), angle);
  }

  /** \brief Get joint angle of joint with the specified name. */
  long callGetJointAngle(long convAddr, char *name, double *angle)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    return pconv->getJointAngle(string(name), *angle);
  }

  /** \brief Set joint angles of all joints in body. */
  long callSetJointAngleAll(long convAddr, double *angleAllArr)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    vector<double> angleAll(pconv->body->numJoints());
    for (int i = 0; i < angleAll.size(); i++) {
      angleAll[i] = angleAllArr[i];
    }
    pconv->setJointAngleAll(angleAll);
    return 0;
  }

  /** \brief Get joint angles of all joints in body. */
  long callGetJointAngleAll(long convAddr, double *angleAllArr)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    vector<double> angleAll(pconv->body->numJoints());
    pconv->getJointAngleAll(angleAll);
    for (int i = 0; i < angleAll.size(); i++) {
      angleAllArr[i] = angleAll[i];
    }
    return 0;
  }

  /** \brief Get pose of link with the specified name.
   *
   * Forward kinematics is calculated inside.
   */
  long callGetLinkPose(long convAddr, char *name, double *pos, double *quat)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    Position pose;
    long ret = pconv->getLinkPose(string(name), pose);
    pos[0] = pose.translation().x();
    pos[1] = pose.translation().y();
    pos[2] = pose.translation().z();
    Quat eigenQuat(pose.linear());
    quat[0] = eigenQuat.w();
    quat[1] = eigenQuat.x();
    quat[2] = eigenQuat.y();
    quat[3] = eigenQuat.z();
    return ret;
  }

  /** \brief Get joint angles such that specified link reaches to the specified pose.
   *
   * Inverse kinematics is calculated inside.
   */
  long callCalcInverseKinematics(long convAddr, char *startLinkName, char *endLinkName, double *pos, double *quat, double *angleAllArr)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    Position targetPose;
    targetPose.translation() = Vector3(pos);
    targetPose.linear() = Quat(quat[0], quat[1], quat[2], quat[3]).normalized().toRotationMatrix();
    vector<double> angleAll;
    int ret = pconv->calcInverseKinematics(string(startLinkName), string(endLinkName), targetPose, angleAll);
    for (int i = 0; i < angleAll.size(); i++) {
      angleAllArr[i] = angleAll[i];
    }
    return ret;
  }

  /** \brief Get mass and center of mass. */
  long callGetMassProp(long convAddr, double *mass, double *comArr)
  {
    SimpleBodyGenerator *pconv = (SimpleBodyGenerator*)convAddr;
    *mass = pconv->body->mass();
    Vector3 com = pconv->body->calcCenterOfMass();
    comArr[0] = com[0];
    comArr[1] = com[1];
    comArr[2] = com[2];
    return 0;
  }
}
