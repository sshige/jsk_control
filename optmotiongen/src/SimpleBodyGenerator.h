#ifndef __SIMPLE_BODY_GENERATOR__
#define __SIMPLE_BODY_GENERATOR__

#include <map>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>

// #define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(str) std::cout << "[" << __FUNCTION__ << "] " << str << std::endl;
#else
#define DEBUG_PRINT(str)
#endif

#define PRINT_VARIABLE(X) std::cout << (#X) << ":" << (X) << std::endl;

using std::string;
using std::vector;
using std::map;

namespace cnoid {

  // using Link::JointType;
  class SimpleBodyGenerator
  {
  public:
    struct LinkInfo
    {
      // basic info
      string name;
      string parentName;
      Position pose;

      // joint info
      int jointId;
      Link::JointType jointType;
      Vector3 jointAxis;
      double lower;
      double upper;

      // physical info
      double mass;
      Vector3 com;
      Matrix3 inertia;

      Link* link;
    };

    /** \brief Default constructor.
     *
     * Do nothing.
     */
    SimpleBodyGenerator(){};

    /** \brief Initialize body generation.
     *
     * Generate new Body instance and set name.
     */
    void initializeBodyGeneration(const string &bodyName);

    /** \brief Add link to body.
     *
     * Generate new Link from LinkInfo and add to body. Joint is added at the same time.
     */
    void addLinkFromLinkInfo(LinkInfo &linkInfo);

    /** \brief Finalize body generation.
     *
     * Set parent-child relation between links. Call other finalization function.
     */
    void finalizeBodyGeneration();

    /** \brief Generate body with one function.
     *
     * Initialize, add link, and finalize body generation.
     */
    void generateBodyFromLinkInfo(const string &bodyName, const vector<LinkInfo> &linkInfoList);

    /** \brief Print body information. */
    void printBodyInfo();

    /** \brief Set joint angle of joint with the specified name. */
    int setJointAngle(const string &name, const double &angle);

    /** \brief Get joint angle of joint with the specified name. */
    int getJointAngle(const string &name, double &angle);

    /** \brief Set joint angles of all joints in body. */
    void setJointAngleAll(const vector<double> &angleAll);

    /** \brief Get joint angles of all joints in body. */
    void getJointAngleAll(vector<double> &angleAll);

    /** \brief Get pose of link with the specified name.
     *
     * Forward kinematics is calculated inside.
     */
    int getLinkPose(const string &name, Position &pose);

    /** \brief Get joint angles such that specified link reaches to the specified pose.
     *
     * Inverse kinematics is calculated inside.
     */
    int calcInverseKinematics(const string &startLinkName, const string &endLinkName,
                              const Position &targetPose, vector<double> &angleAll);

    Body* body;

  private:
    map<string, LinkInfo> nameLinkInfoMap;
  };
}

#endif
