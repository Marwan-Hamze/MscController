#include "MscController.h"

MscController::MscController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  rightFootTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("RightFoot", robots(), 0, 2, 500);
  leftFootTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFoot", robots(), 0, 2, 500);

/*   Gain = Gain.Zero();
  rightFootTask_->setGains(Gain,Gain);
  leftFootTask_->setGains(Gain,Gain); */

  solver().addTask(postureTask);
  solver().addTask(comTask);


 stab_.reset(new mc_tasks::msc_stabilizer::Stabilizer(robots(), realRobots(), robots().robot().robotIndex()));

  mc_rtc::log::success("MscController init done ");
  gui()->addElement({"Here","Main"}, mc_rtc::gui::Button("Push", []() {std::cout << "Test!" << std::endl;}));

}

bool MscController::run()
{
  stab_->config_ = stab_->configure(robots());
  stab_->x_ref_ = stab_->reference(stab_->robots_);
  stab_->K_ = stab_->computeGain(stab_->x_ref_, stab_->config_); 
  stab_->feedback_ = stab_->getFeedback(realRobots());
  stab_->error_ = stab_->computeError(stab_->x_ref_, stab_->feedback_, stab_->config_);
  stab_->accelerations_ = stab_->computeAccelerations(stab_->K_, stab_->feedback_, stab_->x_ref_, stab_->config_, stab_->error_, robots());

  comAcc = stab_->accelerations_.acc;
  comTask->refAccel(comAcc);
  uRF = sva::MotionVecd(stab_->accelerations_.RF_angAcc, stab_->accelerations_.RF_linAcc).Zero();
  uLF = sva::MotionVecd(stab_->accelerations_.LF_angAcc, stab_->accelerations_.LF_linAcc).Zero();

  //solver().addTask(rightFootTask_);
  //solver().addTask(leftFootTask_);
/*   rightFootTask_->refAccel(uRF);
  leftFootTask_->refAccel(uLF); */

  //stab_->run();
  
/*   uRF = sva::MotionVecd(stab_->accelerations_.RF_angAcc, stab_->accelerations_.RF_linAcc);
  uLF = sva::MotionVecd(stab_->accelerations_.LF_angAcc, stab_->accelerations_.LF_linAcc);

  comAcc = stab_->accelerations_.acc;
  
  comTask->refAccel(comAcc);
  rightFootTask_->refAccel(uRF);
  leftFootTask_->refAccel(uLF); */

  //mc_rtc::log::info("The test value is: \n {} \n", realRobots().robot().bodyPosW("L_ANKLE_R_S").translation());
  //mc_rtc::log::info("The reference value is: \n {} \n", stab_->x_ref_.CoM.R);
  //mc_rtc::log::info("The feedback value is: \n {} \n", stab_->feedback_.x.CoM.R);
  //mc_rtc::log::info("The test value is: \n {} \n", stab_->feedback_.x.rightFoot.fc - stab_->x_ref_.rightFoot.fc);
  //mc_rtc::log::info("The test value is: \n {} \n", stab_->feedback_.x.rightFoot.tc - stab_->x_ref_.rightFoot.tc);

  return mc_control::fsm::Controller::run();
}

void MscController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  comTask->reset();
  rightFootTask_->reset();
  leftFootTask_->reset();
  
  const auto & observerp = observerPipeline(observerPipelineName_);
  
  if(observerp.success())
  {
    mc_rtc::log::success("Pipeline \"{}\" for real robot observation loaded!", observerPipelineName_);
  }

}

void MscController::logs_robot(mc_rbdyn::Robots &robots){

/*   All of this works
  mc_rtc::log::info("Robot Weight: {}", robots.robot().mass());

    mc_rtc::log::info("Right Foot Wrench: {}", robots.robot().bodyWrench("rfsensor"));
    mc_rtc::log::info("Left Foot Wrench: {}", robots.robot().bodyWrench("lfsensor"));

    mc_rtc::log::info("Robot CoM Position: {}", robots.robot().com());
    mc_rtc::log::info("Robot Orientation: {}", robots.robot().posW().rotation().transpose());

    mc_rtc::log::info("Right Foot Orientation in the Base Frame: {}", robots.robot().X_b1_b2("base_link","R_ANKLE_R_S").rotation().transpose());
    mc_rtc::log::info("Left Foot Orientation in the Base Frame: {}", robots.robot().X_b1_b2("base_link","L_ANKLE_R_S").rotation().transpose());
   
    mc_rtc::log::info("The state vector x: {}", stabilizer_.getFeedback(robots).x);
    mc_rtc::log::info("The test value is: {}", stabilizer_.test_);
    mc_rtc::log::info("The test value is: {}", stabilizer_.K);

    mc_rtc::log::info("The test value is: {}", stab_->K);
    mc_rtc::log::info("The test value is: {}", stab_->feedback_.com_dd);
    mc_rtc::log::info("The test value is: {}", stab_->X_ref);
    
*/ 
}

