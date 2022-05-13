#include "MscController.h"
#include <mc_rtc/gui/plot.h>

MscController::MscController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);
  
  solver().addTask(postureTask);
  postureTask->weight(1000);

  comTask_ = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 0, 10);
  baseTask_ = std::make_shared<mc_tasks::OrientationTask>("base_link", robots(), robots().robot().robotIndex(), 0, 10);

  rightFoot_PosTask_ = std::make_shared<mc_tasks::PositionTask>("R_ANKLE_P_S", robots(), robots().robot().robotIndex(), 0, 500);
  rightFoot_OrTask_ = std::make_shared<mc_tasks::OrientationTask>("R_ANKLE_P_S", robots(), robots().robot().robotIndex(), 0, 500);

  leftFoot_PosTask_ = std::make_shared<mc_tasks::PositionTask>("L_ANKLE_P_S", robots(), robots().robot().robotIndex(), 0, 500);
  leftFoot_OrTask_ = std::make_shared<mc_tasks::OrientationTask>("L_ANKLE_P_S", robots(), robots().robot().robotIndex(), 0, 500);

  stab_.reset(new msc_stabilizer::Stabilizer(robots(), realRobots(), robots().robot().robotIndex()));

  mc_rtc::log::success("MscController init done ");

  fRF_ = fRF_.Zero();
  tRF_ = tRF_.Zero();
  fLF_ = fLF_.Zero();
  tLF_ = tLF_.Zero();

  logger().addLogEntry("Error_Force_RightFoot", [this]() { return fRF_; });
  logger().addLogEntry("Error_Moment_RightFoot", [this]() { return tRF_; });
  logger().addLogEntry("Error_Force_LeftFoot", [this]() { return fLF_; });
  logger().addLogEntry("Error_Moment_LeftFoot", [this]() { return tLF_; });

}

bool MscController::run()
{

  if(!stabilizer && !flip) {
    
  gui()->removeElement({"Stabilizer","Main"}, "Disable");
  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Enable", [this]() {
    
    stabilizer = true;

    solver().addTask(comTask_);
    solver().addTask(baseTask_);

    solver().addTask(rightFoot_PosTask_);
    //solver().addTask(rightFoot_OrTask_);

    solver().addTask(leftFoot_PosTask_);
    //solver().addTask(leftFoot_OrTask_);


    mc_rtc::log::success("Msc Stabilizer Enabled");
    }));

  // Confirue Q GUI

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::Label("Configure Q:", []() { return "Set the Weights corresponding to:"; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("com_p", [this]() {
    return stab_->config_.qcom_p; }, [this](Eigen::Vector3d com_p){ stab_->config_.qcom_p = com_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("com_r", [this]() {
    return stab_->config_.qcom_R; }, [this](Eigen::Vector3d com_r){ stab_->config_.qcom_R = com_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("com_v", [this]() {
    return stab_->config_.qcom_vel; }, [this](Eigen::Vector3d com_vel){ stab_->config_.qcom_vel = com_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("com_angvel", [this]() {
    return stab_->config_.qcom_angvel; }, [this](Eigen::Vector3d com_angvel){ stab_->config_.qcom_angvel = com_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_p", [this]() {
    return stab_->config_.qRF_p; }, [this](Eigen::Vector3d RF_p){ stab_->config_.qRF_p = RF_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_r", [this]() {
    return stab_->config_.qRF_R; }, [this](Eigen::Vector3d RF_r){ stab_->config_.qRF_R = RF_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_v", [this]() {
    return stab_->config_.qRF_vel; }, [this](Eigen::Vector3d RF_vel){ stab_->config_.qRF_vel = RF_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_angvel", [this]() {
    return stab_->config_.qRF_angvel; }, [this](Eigen::Vector3d RF_angvel){ stab_->config_.qRF_angvel = RF_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_p", [this]() {
    return stab_->config_.qLF_p; }, [this](Eigen::Vector3d LF_p){ stab_->config_.qLF_p = LF_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_r", [this]() {
    return stab_->config_.qLF_R; }, [this](Eigen::Vector3d LF_r){ stab_->config_.qLF_R = LF_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_v", [this]() {
    return stab_->config_.qLF_vel; }, [this](Eigen::Vector3d LF_vel){ stab_->config_.qLF_vel = LF_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_angvel", [this]() {
    return stab_->config_.qLF_angvel; }, [this](Eigen::Vector3d LF_angvel){ stab_->config_.qLF_angvel = LF_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::Button("Check Q", [this]() {
    
    mc_rtc::log::info("Q = \n{}\n" , stab_->config_.Q);
    }));

  // Confirue R GUI

  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::Label("Configure R:", []() { return "Set the Weights corresponding to:"; }));

  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("RF_lacc", [this]() {
    return stab_->config_.rRF_lacc; }, [this](Eigen::Vector3d RF_lacc){ stab_->config_.rRF_lacc = RF_lacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("RF_aacc", [this]() {
    return stab_->config_.rRF_aacc; }, [this](Eigen::Vector3d RF_aacc){ stab_->config_.rRF_aacc = RF_aacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("LF_lacc", [this]() {
    return stab_->config_.rLF_lacc; }, [this](Eigen::Vector3d LF_lacc){ stab_->config_.rLF_lacc = LF_lacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("LF_aacc", [this]() {
    return stab_->config_.rLF_aacc; }, [this](Eigen::Vector3d LF_aacc){ stab_->config_.rLF_aacc = LF_aacc; }));

  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::Button("Check R", [this]() {
    
    mc_rtc::log::info("R = \n{}\n" , stab_->config_.R);
    }));

  // Confirue W GUI

  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::Label("Configure W:", []() { return "Set the Weights (values between 0 and 1) corresponding to:"; }));

  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::ArrayInput("fc_RF", [this]() {
    return stab_->config_.wf_RF; }, [this](Eigen::Vector3d fc_RF){ stab_->config_.wf_RF = fc_RF; }));
  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::ArrayInput("tc_RF", [this]() {
    return stab_->config_.wt_RF; }, [this](Eigen::Vector3d tc_RF){ stab_->config_.wt_RF = tc_RF; }));
  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::ArrayInput("fc_LF", [this]() {
    return stab_->config_.wf_LF; }, [this](Eigen::Vector3d fc_LF){ stab_->config_.wf_LF = fc_LF; }));
  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::ArrayInput("tc_LF", [this]() {
    return stab_->config_.wt_LF; }, [this](Eigen::Vector3d tc_LF){ stab_->config_.wt_LF = tc_LF; }));

  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::Button("Check W", [this]() {
    
    mc_rtc::log::info("W = \n{}\n" , stab_->config_.W);
    }));

  flip = true;
   }

  else if (stabilizer && flip) {

  gui()->removeElement({"Stabilizer","Main"}, "Enable");
  gui()->removeCategory({"Stabilizer","Tuning Q"});
  gui()->removeCategory({"Stabilizer","Tuning R"});
  gui()->removeCategory({"Stabilizer","Tuning W"});

  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Disable", [this]() {
    
    stabilizer = false;

    solver().removeTask(comTask_);
    solver().removeTask(baseTask_);

    solver().removeTask(rightFoot_PosTask_);
    //solver().removeTask(rightFoot_OrTask_);
    
    solver().removeTask(leftFoot_PosTask_);
    //solver().removeTask(leftFoot_OrTask_);


    mc_rtc::log::success("Msc Stabilizer Disabled");
    }));

  flip = false;
  }

  stab_->reconfigure(stab_->config_);
  stab_->K_ = stab_->computeGain(stab_->linearMatrix_, stab_->config_); 
  stab_->feedback_ = stab_->getFeedback(realRobots());
  stab_->error_ = stab_->computeError(stab_->x_ref_, stab_->feedback_, stab_->config_);
  stab_->accelerations_ = stab_->computeAccelerations(stab_->K_, stab_->feedback_, stab_->x_ref_, stab_->config_, stab_->error_, realRobots());

  comTask_->refAccel(stab_->accelerations_.ddcom);
  baseTask_->refAccel(stab_->accelerations_.dwb);

  rightFoot_PosTask_->refAccel(stab_->accelerations_.RF_linAcc);
  rightFoot_OrTask_->refAccel(stab_->accelerations_.RF_angAcc);
  
  leftFoot_PosTask_->refAccel(stab_->accelerations_.LF_linAcc);
  leftFoot_OrTask_->refAccel(stab_->accelerations_.LF_angAcc);

  fRF_ = stab_->f_delta_.block(0,0,3,1);
  tRF_ = stab_->f_delta_.block(3,0,3,1);
  fLF_ = stab_->f_delta_.block(6,0,3,1);
  tLF_ = stab_->f_delta_.block(9,0,3,1);

  
  /* mc_rtc::log::info("From Ankle: \n {} \n", realRobots().robot().bodyWrench("L_ANKLE_P_S").moment());
  mc_rtc::log::info("From Surface: \n {} \n", realRobots().robot().surfaceWrench("LeftFoot").moment()); */

  return mc_control::fsm::Controller::run();
}

void MscController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  comTask_->reset();
  baseTask_->reset();

  rightFoot_PosTask_->reset();
  rightFoot_OrTask_->reset();

  leftFoot_PosTask_->reset();
  leftFoot_OrTask_->reset();

  const auto & observerp = observerPipeline(observerPipelineName_);
  
  if(observerp.success())
  {
    mc_rtc::log::success("Pipeline \"{}\" for real robot observation loaded!", observerPipelineName_);
  }

  stab_->config_ = stab_->configure(realRobots());
  stab_->x_ref_ = stab_->reference(realRobots());
  stab_->linearMatrix_ = stab_->computeMatrix(stab_->x_ref_, stab_->config_);

  mc_rtc::log::info("Reference obtained from the Robot\n");

  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check Accelerations", [this]() {
    
    mc_rtc::log::info("CoM Linear Acceleration = \n{}\n" , stab_->accelerations_.ddcom);
    mc_rtc::log::info("Base Angular Acceleration = \n{}\n" , stab_->accelerations_.dwb);
   
    mc_rtc::log::info("RightFoot Linear Acceleration = \n{}\n" , stab_->accelerations_.RF_linAcc);
    mc_rtc::log::info("RightFoot Angular Acceleration = \n{}\n" , stab_->accelerations_.RF_angAcc);

    mc_rtc::log::info("LeftFoot Linear Acceleration = \n{}\n" , stab_->accelerations_.LF_linAcc);
    mc_rtc::log::info("LeftFoot Angular Acceleration = \n{}\n" , stab_->accelerations_.LF_angAcc);

    }));

  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check The Error Vector", [this]() {
    
    mc_rtc::log::info("Error = \n{}\n" , stab_->error_);

    }));

  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check The State Error", [this]() {
    
    mc_rtc::log::info("State Error = \n{}\n" , stab_->x_delta_);

    }));

  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check The Force Error", [this]() {
    
    mc_rtc::log::info("Force Error = \n{}\n" , stab_->f_delta_);

    }));

}
