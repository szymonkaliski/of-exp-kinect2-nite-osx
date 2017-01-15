#include "ofApp.h"

void ofApp::setup() {
  nite::Status niteRc = nite::NiTE::initialize();

  if (niteRc != nite::STATUS_OK) {
    ofLogError() << "Couldn't start NiTE";
    return;
  }

  niteRc = userTracker.create();

  if (niteRc != nite::STATUS_OK) {
    ofLogError() << "Couldn't create user tracker";
    return;
  }

  depthPixels.allocate(0, 0, 1);
  depthTexture.clear();
}

void ofApp::update() {
  nite::UserTrackerFrameRef userTrackerFrame;

  // get raw depth frame
  nite::Status niteRc = userTracker.readFrame(&userTrackerFrame);

  // get user labels map
  const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
  const nite::UserId* pixelLabels = userLabels.getPixels();

  // process users
  const nite::Array<nite::UserData>& usersData = userTrackerFrame.getUsers();

  for (int i = 0; i < usersData.getSize(); ++i) {
    const nite::UserData& user = usersData[i];

    int id = user.getId();

    if (user.isNew()) {
      userTracker.startSkeletonTracking(id);
    }
    else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED && id < MAX_USERS - 1) {
      users[id].visible       = user.isVisible();

      users[id].head          = getJointInDepthCoordinates(user, nite::JOINT_HEAD);
      users[id].neck          = getJointInDepthCoordinates(user, nite::JOINT_NECK);
      users[id].leftShoulder  = getJointInDepthCoordinates(user, nite::JOINT_LEFT_SHOULDER);
      users[id].rightShoulder = getJointInDepthCoordinates(user, nite::JOINT_RIGHT_SHOULDER);
      users[id].leftElbow     = getJointInDepthCoordinates(user, nite::JOINT_LEFT_ELBOW);
      users[id].rightElbow    = getJointInDepthCoordinates(user, nite::JOINT_RIGHT_ELBOW);
      users[id].leftHand      = getJointInDepthCoordinates(user, nite::JOINT_LEFT_HAND);
      users[id].rightHand     = getJointInDepthCoordinates(user, nite::JOINT_RIGHT_HAND);
      users[id].torso         = getJointInDepthCoordinates(user, nite::JOINT_TORSO);
      users[id].leftHip       = getJointInDepthCoordinates(user, nite::JOINT_LEFT_HIP);
      users[id].rightHip      = getJointInDepthCoordinates(user, nite::JOINT_RIGHT_HIP);
      users[id].leftKnee      = getJointInDepthCoordinates(user, nite::JOINT_LEFT_KNEE);
      users[id].rightKnee     = getJointInDepthCoordinates(user, nite::JOINT_RIGHT_KNEE);
      users[id].leftFoot      = getJointInDepthCoordinates(user, nite::JOINT_LEFT_FOOT);
      users[id].rightFoot     = getJointInDepthCoordinates(user, nite::JOINT_RIGHT_FOOT);
    }
  }

  // get depth frame data
  openni::VideoFrameRef frame = userTrackerFrame.getDepthFrame();
  calculateHistogram(depthHist, MAX_DEPTH, frame);

  const openni::DepthPixel* depthRow = (const openni::DepthPixel*)frame.getData();

  int resX = frame.getVideoMode().getResolutionX();
  int resY = frame.getVideoMode().getResolutionY();

  depthPixels.allocate(resX, resY, 1);

  for (int y = 0; y < resY; y++) {
    for (int x = 0; x < resX; x++, depthRow++, pixelLabels++) {
      float value = depthHist[*depthRow];

      // filter out everything that's not a user
      if (*pixelLabels == 0) {
        value = 0;
      }

      depthPixels.setColor(x, y, value);
    }
  }

  if (!depthTexture.isAllocated() || depthTexture.getWidth() != resX || depthTexture.getHeight() != resY) {
    depthTexture.allocate(depthPixels);
  }

  depthTexture.loadData(depthPixels);
}

void ofApp::draw() {
  ofBackground(0);

  if (depthTexture.isAllocated()) {
    ofSetColor(ofColor::white);
    depthTexture.draw(0, 0);
  }

  ofSetColor(ofColor(255, 0, 0));
  for (int i = 0; i < MAX_USERS; ++i) {
    if (users[i].visible) {
      drawUser(users[i]);
    }
  }
}

// helpers

void ofApp::calculateHistogram(float* histogram, int histogramSize, const openni::VideoFrameRef& frame) {
  const openni::DepthPixel* depth = (const openni::DepthPixel*)frame.getData();

  memset(histogram, 0, histogramSize * sizeof(float));

  int restOfRow = frame.getStrideInBytes() / sizeof(openni::DepthPixel) - frame.getWidth();
  int height    = frame.getHeight();
  int width     = frame.getWidth();

  unsigned int pointsNum = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++depth) {
      if (*depth != 0) {
        histogram[*depth]++;
        pointsNum++;
      }
    }

    depth += restOfRow;
  }

  for (int i = 1; i < histogramSize; i++) {
    histogram[i] += histogram[i-1];
  }

  if (pointsNum) {
    for (int i = 1; i < histogramSize; i++) {
      histogram[i] = (256 * (1.0f - (histogram[i] / pointsNum)));
    }
  }
}

ofVec2f ofApp::getJointInDepthCoordinates(nite::UserData user, nite::JointType jointType) {
  const nite::SkeletonJoint& joint = user.getSkeleton().getJoint(jointType);
  float x, y;

  userTracker.convertJointCoordinatesToDepth(joint.getPosition().x, joint.getPosition().y, joint.getPosition().z, &x, &y);

  return ofVec2f(x, y);
}

void ofApp::drawUser(user_t user) {
  float r = 3;

  ofDrawCircle(user.head,          r);
  ofDrawCircle(user.neck,          r);
  ofDrawCircle(user.leftShoulder,  r);
  ofDrawCircle(user.rightShoulder, r);
  ofDrawCircle(user.leftElbow,     r);
  ofDrawCircle(user.rightElbow,    r);
  ofDrawCircle(user.leftHand,      r);
  ofDrawCircle(user.rightHand,     r);
  ofDrawCircle(user.torso,         r);
  ofDrawCircle(user.leftHip,       r);
  ofDrawCircle(user.rightHip,      r);
  ofDrawCircle(user.leftKnee,      r);
  ofDrawCircle(user.rightKnee,     r);
  ofDrawCircle(user.leftFoot,      r);
  ofDrawCircle(user.rightFoot,     r);

  ofDrawLine(user.head,            user.neck);
  ofDrawLine(user.leftShoulder,    user.rightShoulder);
  ofDrawLine(user.leftShoulder,    user.torso);
  ofDrawLine(user.rightShoulder,   user.torso);

  ofDrawLine(user.leftShoulder,    user.leftElbow);
  ofDrawLine(user.leftElbow,       user.leftHand);

  ofDrawLine(user.rightShoulder,   user.rightElbow);
  ofDrawLine(user.rightElbow,      user.rightHand);

  ofDrawLine(user.torso,           user.leftHip);
  ofDrawLine(user.torso,           user.rightHip);

  ofDrawLine(user.leftHip,         user.leftKnee);
  ofDrawLine(user.leftKnee,        user.leftFoot);

  ofDrawLine(user.rightHip,        user.rightKnee);
  ofDrawLine(user.rightKnee,       user.rightFoot);
}
