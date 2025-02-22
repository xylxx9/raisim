// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char **argv) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const int loopN = 200000000;
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  raisim::World world;
  raisim::RaisimServer server(&world);

  auto checkerBoard = world.addGround(0.0, "glass");
  auto sphere = world.addSphere(0.2, 5);
  sphere->setPosition(2, 0, 0.5);

  Eigen::VectorXd jointConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointVel(18), jointPgain(18), jointDgain(18);

  jointPgain.setZero();
  jointPgain.tail(12).setConstant(200.0);

  jointDgain.setZero();
  jointDgain.tail(12).setConstant(10.0);

  jointVelocityTarget.setZero();

  jointConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  auto anymal = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal_sensored.urdf");
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setName("Anymal");

  auto depthSensor1 = anymal->getSensor<raisim::DepthCamera>("depth_camera_front_camera_parent:depth");
  depthSensor1->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
//  depthSensor1->setMeasurementSource(raisim::Sensor::MeasurementSource::RAISIM); // uncomment this line if you want to update the sensor using Raisim (CPU)

  auto rgbCamera1 = anymal->getSensor<raisim::RGBCamera>("depth_camera_front_camera_parent:color");
  rgbCamera1->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);

  auto depthSensor2 = anymal->getSensor<raisim::DepthCamera>("depth_camera_rear_camera_parent:depth");
  depthSensor2->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
  auto rgbCamera2 = anymal->getSensor<raisim::RGBCamera>("depth_camera_rear_camera_parent:color");
  rgbCamera2->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
  auto imu = anymal->getSensor<raisim::InertialMeasurementUnit>("depth_camera_front_camera_parent:imu");

  auto dummySphere1 = server.addVisualSphere("dummy1", 0.05, 1, 0, 0, 1);
  auto dummySphere2 = server.addVisualSphere("dummy2", 0.05, 1, 0, 0, 1);
  std::vector<raisim::Vec<3>> pointCloudFromConversion;

  server.launchServer();
  for (int k = 0; k < loopN; k++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();

    depthSensor1->lockMutex();
    const auto &depth = depthSensor1->getDepthArray();
    depthSensor1->depthToPointCloud(depth, pointCloudFromConversion, false); // this method lets you convert depth values to 3D points
    auto& posFromRaisim = depthSensor1->get3DPoints(); // this method returns garbage if the update is done by the visualizer
    depthSensor1->unlockMutex();

    dummySphere1->setPosition(posFromRaisim[0].e()); // this doesn't work if the update is done by Raisim Unreal
    dummySphere2->setPosition(pointCloudFromConversion[400].e());
    std::cout<<"Imu orientation\n" << imu->getOrientation()<<std::endl;
  }

  server.killServer();
  return 0;
}