#include "MscController.h"

MscController::MscController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);

  Activedof_ = config("active_dof");
  
  solver().addConstraintSet(kinematicsConstraint);

  comTask_ = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robot().robotIndex(), 0, 1e7);
  baseTask_ = std::make_shared<mc_tasks::OrientationTask>("BODY", robots(), robots().robot().robotIndex(), 0, 1e7);

  rightFoot_PosTask_ = std::make_shared<mc_tasks::PositionTask>("RLEG_LINK5", robots(), robots().robot().robotIndex(), 0, 1e9);
  rightFoot_OrTask_ = std::make_shared<mc_tasks::OrientationTask>("RLEG_LINK5", robots(), robots().robot().robotIndex(), 0, 1e9);

  leftFoot_PosTask_ = std::make_shared<mc_tasks::PositionTask>("LLEG_LINK5", robots(), robots().robot().robotIndex(), 0, 1e9);
  leftFoot_OrTask_ = std::make_shared<mc_tasks::OrientationTask>("LLEG_LINK5", robots(), robots().robot().robotIndex(), 0, 1e9);

  rightHand_PosTask_ = std::make_shared<mc_tasks::PositionTask>("RARM_LINK7", robots(), robots().robot().robotIndex(), 0, 1e9);
  rightHand_OrTask_ = std::make_shared<mc_tasks::OrientationTask>("RARM_LINK7", robots(), robots().robot().robotIndex(), 0, 1e9);

  stab_.reset(new msc_stabilizer::Stabilizer(robots(), realRobots(), robots().robot().robotIndex())); 

  // Setting the anchor frame for the Kinematic Inertial estimator

  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                          [this](const mc_rbdyn::Robot & robot)
                          {
                            return sva::interpolate(robot.surfacePose("LeftFoot"),
                                                    robot.surfacePose("RightFoot"),
                                                    leftFootRatio);
                          });

  mc_rtc::log::success("MscController initialization from Constructor done ");

  // These dof vectors are used to fix/free the contacts when adding/removing the right hand contact 

  dof << 0, 0, 0, 0, 0, 0;
  dof_full << 1, 1, 1, 1, 1, 1;

  // Setting Logger Entries for error signals coming from the state variables of the LQR controller

  com_ = com_.Zero();
  theta_ = theta_.Zero();
  comd_ = comd_.Zero();
  om_ = om_.Zero();

  pRF_ = pRF_.Zero();
  thetaRF_ = thetaRF_.Zero();
  vRF_ = vRF_.Zero();
  omRF_ = omRF_.Zero();
  fRF_ = fRF_.Zero();
  tRF_ = tRF_.Zero();

  pLF_ = pLF_.Zero();
  thetaLF_ = thetaLF_.Zero();
  vLF_ = vLF_.Zero();
  omLF_ = omLF_.Zero();
  fLF_ = fLF_.Zero();
  tLF_ = tLF_.Zero();

  pRH_ = pRH_.Zero();
  thetaRH_ = thetaRH_.Zero();
  vRH_ = vRH_.Zero();
  omRH_ = omRH_.Zero();
  fRH_ = fRH_.Zero();
  tRH_ = tRH_.Zero();

  fr_x_RF_ = 0.0;
  fr_y_RF_ = 0.0;

  fr_z_RH_ = 0.0;
  fr_y_RH_ = 0.0;

  logger().addLogEntry("Error_com_Position", [this]() { return com_; });
  logger().addLogEntry("Error_com_Orientation", [this]() { return theta_; });
  logger().addLogEntry("Error_com_Velocity", [this]() { return comd_; });
  logger().addLogEntry("Error_com_AngVelocity", [this]() { return om_; });

  logger().addLogEntry("Error_RightFoot_Position", [this]() { return pRF_; });
  logger().addLogEntry("Error_RightFoot_Orientation", [this]() { return thetaRF_; });
  logger().addLogEntry("Error_RightFoot_Velocity", [this]() { return vRF_; });
  logger().addLogEntry("Error_RightFoot_AngVelocity", [this]() { return omRF_; });
  logger().addLogEntry("Error_RightFoot_Force", [this]() { return fRF_; });
  logger().addLogEntry("Error_RightFoot_Moment", [this]() { return tRF_; });

  logger().addLogEntry("Error_LeftFoot_Position", [this]() { return pLF_; });
  logger().addLogEntry("Error_LeftFoot_Orientation", [this]() { return thetaLF_; });
  logger().addLogEntry("Error_LeftFoot_Velocity", [this]() { return vLF_; });
  logger().addLogEntry("Error_LeftFoot_AngVelocity", [this]() { return omLF_; });
  logger().addLogEntry("Error_LeftFoot_Force", [this]() { return fLF_; });
  logger().addLogEntry("Error_LeftFoot_Moment", [this]() { return tLF_; });

  logger().addLogEntry("Error_RightHand_Position", [this]() { return pRH_; });
  logger().addLogEntry("Error_RightHand_Orientation", [this]() { return thetaRH_; });
  logger().addLogEntry("Error_RightHand_Velocity", [this]() { return vRH_; });
  logger().addLogEntry("Error_RightHand_AngVelocity", [this]() { return omRH_; });
  logger().addLogEntry("Error_RightHand_Force", [this]() { return fRH_; });
  logger().addLogEntry("Error_RightHand_Moment", [this]() { return tRH_; });

  // Logging the desired accelerations in the world frame sent to the QP 

  logger().addLogEntry("Accelerations_RightFoot_Linear", [this]() { return stab_->accelerations_.RF_linAcc;});
  logger().addLogEntry("Accelerations_RightFoot_Angular", [this]() { return stab_->accelerations_.RF_angAcc;});
  logger().addLogEntry("Accelerations_LeftFoot_Linear", [this]() { return stab_->accelerations_.LF_linAcc;});
  logger().addLogEntry("Accelerations_LeftFoot_Angular", [this]() { return stab_->accelerations_.LF_angAcc;});
  logger().addLogEntry("Accelerations_RightHand_Linear", [this]() { return stab_->accelerations_.RH_linAcc;});
  logger().addLogEntry("Accelerations_RightHand_Angular", [this]() { return stab_->accelerations_.RH_angAcc;});

  // Logging the Cop of each foot

  logger().addLogEntry("CoP_RightFoot", [this]() {return realRobots().robot().cop("RightFoot");});
  logger().addLogEntry("CoP_LeftFoot", [this]() {return realRobots().robot().cop("LeftFoot");});
  logger().addLogEntry("CoP_RightHand", [this]() {return realRobots().robot().cop("RightHand");});

  // Logging the Friction at the Right Foot and Right Hand

  logger().addLogEntry("Friction_RightFoot_x", [this]() {return fr_x_RF_;});
  logger().addLogEntry("Friction_RightFoot_y", [this]() {return fr_y_RF_;});

/*   logger().addLogEntry("Friction_RightHand_x", [this]() {return fr_z_RH_;});
  logger().addLogEntry("Friction_RightHand_y", [this]() {return fr_y_RH_;}); */

}

bool MscController::run()
{
    // GUI related code for the Compute Button

    if(init && !compute) {

      gui()->removeElement({"Stabilizer","Initialization"}, "Initialize");
      gui()->addElement({"Stabilizer","Initialization"}, mc_rtc::gui::Button("Compute", [this]() {
      
      stab_->config_ = stab_->configure(robots());
      stab_->x_ref_ = stab_->reference(realRobots());
      stab_->linearMatrix_ = stab_->computeMatrix(stab_->x_ref_, stab_->config_);
      stab_->K_ = stab_->computeGain(stab_->linearMatrix_, stab_->config_); 

      mc_rtc::log::info("Configuration Initialized\n");
      mc_rtc::log::info("Reference obtained from the Robot\n");
      mc_rtc::log::info("LQR Gain Calculated\n");

      ref = true;
      }));

    compute = true;

    }

    // GUI related code after clicking Compute

    if (ref && !main) {

    gui()->removeCategory({"Stabilizer","Initialization"});

    gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check Accelerations", [this]() {
      
      mc_rtc::log::info("CoM Linear Acceleration = \n{}\n" , stab_->accelerations_.ddcom);
      mc_rtc::log::info("Base Angular Acceleration = \n{}\n" , stab_->accelerations_.dwb);
    
      mc_rtc::log::info("RightFoot Linear Acceleration = \n{}\n" , stab_->accelerations_.RF_linAcc);
      mc_rtc::log::info("RightFoot Angular Acceleration = \n{}\n" , stab_->accelerations_.RF_angAcc);

      mc_rtc::log::info("LeftFoot Linear Acceleration = \n{}\n" , stab_->accelerations_.LF_linAcc);
      mc_rtc::log::info("LeftFoot Angular Acceleration = \n{}\n" , stab_->accelerations_.LF_angAcc);

      mc_rtc::log::info("RightHand Linear Acceleration = \n{}\n" , stab_->accelerations_.RH_linAcc);
      mc_rtc::log::info("RightHand Angular Acceleration = \n{}\n" , stab_->accelerations_.RH_angAcc);

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

    gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check A, B, Q and R", [this]() {
      
      mc_rtc::log::info("Matrix A = \n{}\n" , stab_->Ay);
      mc_rtc::log::info("Matrix B = \n{}\n" , stab_->By);
      mc_rtc::log::info("Matrix Q = \n{}\n" , stab_->Qy);
      mc_rtc::log::info("Matrix R = \n{}\n" , stab_->config_.R);

      }));

    gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Check The LQR Gain", [this]() {
      
      mc_rtc::log::info("LQR Gain K = \n{}\n" , stab_->K_);

      }));

    main = true;
  }

  // GUI related code when the tasks aren't loaded into the QP via the enable button

  if(!stabilizer && !flip && ref) {
    
  gui()->removeElement({"Stabilizer","Main"}, "Disable");
  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Enable", [this]() {
    
    stabilizer = true;

    solver().addTask(rightFoot_PosTask_);
    solver().addTask(rightFoot_OrTask_);

    solver().addTask(leftFoot_PosTask_);
    solver().addTask(leftFoot_OrTask_);

    solver().addTask(rightHand_PosTask_);
    solver().addTask(rightHand_OrTask_);


    mc_rtc::log::success("Msc Stabilizer Enabled");
    }));
    
  // GUI related code to the Stop button

  gui()->addElement({"Stabilizer", "Stop"}, mc_rtc::gui::Button("Stop", [this](){

    ref = false;

    gui()->removeCategory({"Stabilizer","Main"});
    gui()->removeCategory({"Stabilizer","Tuning Q"});
    gui()->removeCategory({"Stabilizer","Tuning R"});
    gui()->removeCategory({"Stabilizer","Tuning W"});
    gui()->removeCategory({"Stabilizer","Tuning Compliance"});

    removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
    removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});

    addContact({robot().name(), "ground", "RightFoot", "AllGround", mc_rbdyn::Contact::defaultFriction, dof_full});
    addContact({robot().name(), "ground", "LeftFoot", "AllGround", mc_rbdyn::Contact::defaultFriction, dof_full});

/*     solver().removeConstraintSet(kinematicsConstraint);
    solver().addConstraintSet(dynamicsConstraint); */

    mc_rtc::log::info("Now the Right Hand can be moved back\n");

    gui()->removeCategory({"Stabilizer","Stop"});

  }));

  // Confirue Q GUI

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::Label("Configure Q:", []() { return "Set the Weights corresponding to:"; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("com_pos", [this]() {
    return stab_->config_.qcom_p; }, [this](Eigen::Vector3d com_p){ stab_->config_.qcom_p = com_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("base_rot", [this]() {
    return stab_->config_.qcom_R; }, [this](Eigen::Vector3d com_r){ stab_->config_.qcom_R = com_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("com_vel", [this]() {
    return stab_->config_.qcom_vel; }, [this](Eigen::Vector3d com_vel){ stab_->config_.qcom_vel = com_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("base_angvel", [this]() {
    return stab_->config_.qcom_angvel; }, [this](Eigen::Vector3d com_angvel){ stab_->config_.qcom_angvel = com_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_pos", [this]() {
    return stab_->config_.qRF_p; }, [this](Eigen::Vector3d RF_p){ stab_->config_.qRF_p = RF_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_rot", [this]() {
    return stab_->config_.qRF_R; }, [this](Eigen::Vector3d RF_r){ stab_->config_.qRF_R = RF_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_vel", [this]() {
    return stab_->config_.qRF_vel; }, [this](Eigen::Vector3d RF_vel){ stab_->config_.qRF_vel = RF_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RF_angvel", [this]() {
    return stab_->config_.qRF_angvel; }, [this](Eigen::Vector3d RF_angvel){ stab_->config_.qRF_angvel = RF_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_pos", [this]() {
    return stab_->config_.qLF_p; }, [this](Eigen::Vector3d LF_p){ stab_->config_.qLF_p = LF_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_rot", [this]() {
    return stab_->config_.qLF_R; }, [this](Eigen::Vector3d LF_r){ stab_->config_.qLF_R = LF_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_vel", [this]() {
    return stab_->config_.qLF_vel; }, [this](Eigen::Vector3d LF_vel){ stab_->config_.qLF_vel = LF_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("LF_angvel", [this]() {
    return stab_->config_.qLF_angvel; }, [this](Eigen::Vector3d LF_angvel){ stab_->config_.qLF_angvel = LF_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RH_pos", [this]() {
    return stab_->config_.qRH_p; }, [this](Eigen::Vector3d RH_p){ stab_->config_.qRH_p = RH_p; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RH_rot", [this]() {
    return stab_->config_.qRH_R; }, [this](Eigen::Vector3d RH_r){ stab_->config_.qRH_R = RH_r; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RH_vel", [this]() {
    return stab_->config_.qRH_vel; }, [this](Eigen::Vector3d RH_vel){ stab_->config_.qRH_vel = RH_vel; }));
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::ArrayInput("RH_angvel", [this]() {
    return stab_->config_.qRH_angvel; }, [this](Eigen::Vector3d RH_angvel){ stab_->config_.qRH_angvel = RH_angvel; }));

  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::Button("Update", [this]() {
    
    stab_->reconfigure(stab_->config_);
    stab_->K_ = stab_->computeGain(stab_->linearMatrix_, stab_->config_);

    mc_rtc::log::info("The Matrix Q and the LQR Gain are Updated \n");

    }));
  
  gui()->addElement({"Stabilizer","Tuning Q"}, mc_rtc::gui::Button("Check Q", [this]() {
    
    mc_rtc::log::info("Q = \n{}\n" , stab_->config_.Q);
    }));

  // Confirue R GUI

  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::Label("Configure R:", []() { return "Set the Weights corresponding to:"; }));

  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("RF_linear_acc", [this]() {
    return stab_->config_.rRF_lacc; }, [this](Eigen::Vector3d RF_lacc){ stab_->config_.rRF_lacc = RF_lacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("RF_angular_acc", [this]() {
    return stab_->config_.rRF_aacc; }, [this](Eigen::Vector3d RF_aacc){ stab_->config_.rRF_aacc = RF_aacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("LF_linear_acc", [this]() {
    return stab_->config_.rLF_lacc; }, [this](Eigen::Vector3d LF_lacc){ stab_->config_.rLF_lacc = LF_lacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("LF_angular_acc", [this]() {
    return stab_->config_.rLF_aacc; }, [this](Eigen::Vector3d LF_aacc){ stab_->config_.rLF_aacc = LF_aacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("RH_linear_acc", [this]() {
    return stab_->config_.rRH_lacc; }, [this](Eigen::Vector3d RH_lacc){ stab_->config_.rRH_lacc = RH_lacc; }));
  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::ArrayInput("RH_angular_acc", [this]() {
    return stab_->config_.rRH_aacc; }, [this](Eigen::Vector3d RH_aacc){ stab_->config_.rRH_aacc = RH_aacc; }));

  gui()->addElement({"Stabilizer","Tuning R"}, mc_rtc::gui::Button("Update", [this]() {
    
    stab_->reconfigure(stab_->config_);
    stab_->K_ = stab_->computeGain(stab_->linearMatrix_, stab_->config_);

    mc_rtc::log::info("The Matrix R and the LQR Gain are Updated \n");

    }));

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
  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::ArrayInput("fc_RH", [this]() {
    return stab_->config_.wf_RH; }, [this](Eigen::Vector3d fc_RH){ stab_->config_.wf_RH = fc_RH; }));
  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::ArrayInput("tc_RH", [this]() {
    return stab_->config_.wt_RH; }, [this](Eigen::Vector3d tc_RH){ stab_->config_.wt_RH = tc_RH; }));

  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::Button("Update", [this]() {
    
    stab_->reconfigure(stab_->config_);
    stab_->K_ = stab_->computeGain(stab_->linearMatrix_, stab_->config_);

    mc_rtc::log::info("The Matrix W and the LQR Gain are Updated \n");

    }));

  gui()->addElement({"Stabilizer","Tuning W"}, mc_rtc::gui::Button("Check W", [this]() {
    
    mc_rtc::log::info("W = \n{}\n" , stab_->config_.W);
    }));

// Configuration of the Stiffness and Damping of the Contacts GUI

  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::Label("Configure Compliance:", []() { return "Set the Values corresponding to:"; }));

  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Linear_Stiffness_RF", [this]() {
    return stab_->config_.kfp_rf; }, [this](Eigen::Vector3d kfp_rf){ stab_->config_.kfp_rf = kfp_rf; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Linear_Damping_RF", [this]() {
    return stab_->config_.kfd_rf; }, [this](Eigen::Vector3d kfd_rf){ stab_->config_.kfd_rf = kfd_rf; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Angular_Stiffness_RF", [this]() {
    return stab_->config_.ktp_rf; }, [this](Eigen::Vector3d ktp_rf){ stab_->config_.ktp_rf = ktp_rf; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Angular_Damping_RF", [this]() {
    return stab_->config_.ktd_rf;  }, [this](Eigen::Vector3d ktd_rf){ stab_->config_.ktd_rf = ktd_rf; }));

  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Linear_Stiffness_LF", [this]() {
    return stab_->config_.kfp_lf; }, [this](Eigen::Vector3d kfp_lf){ stab_->config_.kfp_lf = kfp_lf; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Linear_Damping_LF", [this]() {
    return stab_->config_.kfd_lf; }, [this](Eigen::Vector3d kfd_lf){ stab_->config_.kfd_lf = kfd_lf; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Angular_Stiffness_LF", [this]() {
    return stab_->config_.ktp_lf; }, [this](Eigen::Vector3d ktp_lf){ stab_->config_.ktp_lf = ktp_lf; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Angular_Damping_LF", [this]() {
    return stab_->config_.ktd_lf;  }, [this](Eigen::Vector3d ktd_lf){ stab_->config_.ktd_lf = ktd_lf; }));

  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Linear_Stiffness_RH", [this]() {
    return stab_->config_.kfp_rh; }, [this](Eigen::Vector3d kfp_rh){ stab_->config_.kfp_rh = kfp_rh; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Linear_Damping_RH", [this]() {
    return stab_->config_.kfd_rh; }, [this](Eigen::Vector3d kfd_rh){ stab_->config_.kfd_rh = kfd_rh; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Angular_Stiffness_RH", [this]() {
    return stab_->config_.ktp_rh; }, [this](Eigen::Vector3d ktp_rh){ stab_->config_.ktp_rh = ktp_rh; }));
  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::ArrayInput("Angular_Damping_RH", [this]() {
    return stab_->config_.ktd_rh;  }, [this](Eigen::Vector3d ktd_rh){ stab_->config_.ktd_rh = ktd_rh; }));

  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::Button("Update", [this]() {
    
    stab_->reconfigure(stab_->config_);
    stab_->linearMatrix_ = stab_->computeMatrix(stab_->x_ref_, stab_->config_);
    stab_->K_ = stab_->computeGain(stab_->linearMatrix_, stab_->config_);

    mc_rtc::log::info("The LQR Gain is Updated \n");

    }));

  gui()->addElement({"Stabilizer","Tuning Compliance"}, mc_rtc::gui::Button("Check Compliance", [this]() {
    
    mc_rtc::log::info("KFP_RF = \n{}\n" , stab_->config_.KFP_RF);
    mc_rtc::log::info("KFD_RF = \n{}\n" , stab_->config_.KFD_RF);
    mc_rtc::log::info("KTP_RF = \n{}\n" , stab_->config_.KTP_RF);
    mc_rtc::log::info("KTD_RF = \n{}\n" , stab_->config_.KTD_RF);

    mc_rtc::log::info("KFP_LF = \n{}\n" , stab_->config_.KFP_LF);
    mc_rtc::log::info("KFD_LF = \n{}\n" , stab_->config_.KFD_LF);
    mc_rtc::log::info("KTP_LF = \n{}\n" , stab_->config_.KTP_LF);
    mc_rtc::log::info("KTD_LF = \n{}\n" , stab_->config_.KTD_LF);

    mc_rtc::log::info("KFP_RH = \n{}\n" , stab_->config_.KFP_RH);
    mc_rtc::log::info("KFD_RH = \n{}\n" , stab_->config_.KFD_RH);
    mc_rtc::log::info("KTP_RH = \n{}\n" , stab_->config_.KTP_RH);
    mc_rtc::log::info("KTD_RH = \n{}\n" , stab_->config_.KTD_RH);

    }));

  flip = true;
   }

  // GUI related code when the tasks are loaded into the QP via the enable button

  else if (stabilizer && flip && ref) {

  gui()->removeElement({"Stabilizer","Main"}, "Enable");
  gui()->removeCategory({"Stabilizer","Tuning Q"});
  gui()->removeCategory({"Stabilizer","Tuning R"});
  gui()->removeCategory({"Stabilizer","Tuning W"});
  gui()->removeCategory({"Stabilizer","Tuning Compliance"});
  gui()->removeCategory({"Stabilizer","Stop"});


  gui()->addElement({"Stabilizer","Main"}, mc_rtc::gui::Button("Disable", [this]() {
    
    stabilizer = false;

    solver().removeTask(rightFoot_PosTask_);
    solver().removeTask(rightFoot_OrTask_);
    
    solver().removeTask(leftFoot_PosTask_);
    solver().removeTask(leftFoot_OrTask_);

    solver().removeTask(rightHand_PosTask_);
    solver().removeTask(rightHand_OrTask_);

    mc_rtc::log::success("Msc Stabilizer Disabled");
    }));

  flip = false;
  }

  // Code that is running all the time during the simulation after clicking the compute button

  if (ref) {

    // Here the accelerations are calculated

    stab_->feedback_ = stab_->getFeedback(robots(), realRobots());
    stab_->error_ = stab_->computeError(stab_->x_ref_, stab_->feedback_, stab_->config_);
    stab_->accelerations_ = stab_->computeAccelerations(stab_->K_, stab_->feedback_, stab_->x_ref_, stab_->config_, stab_->error_);

    // Here the accelerations are loaded as reference accelerations for the respective tasks

    comTask_->refAccel(stab_->accelerations_.ddcom);
    baseTask_->refAccel(stab_->accelerations_.dwb);

    rightFoot_PosTask_->refAccel(stab_->accelerations_.RF_linAcc);
    rightFoot_OrTask_->refAccel(stab_->accelerations_.RF_angAcc);
    
    leftFoot_PosTask_->refAccel(stab_->accelerations_.LF_linAcc);
    leftFoot_OrTask_->refAccel(stab_->accelerations_.LF_angAcc);

    rightHand_PosTask_->refAccel(stab_->accelerations_.RH_linAcc);
    rightHand_OrTask_->refAccel(stab_->accelerations_.RH_angAcc);

    // To log the error of the state vector and forces

    com_ = stab_->x_delta_.block(0,0,3,1);
    theta_ = stab_->x_delta_.block(3,0,3,1);
    comd_ = stab_->x_delta_.block(6,0,3,1);
    om_ = stab_->x_delta_.block(9,0,3,1);

    pRF_ = stab_->x_delta_.block(12,0,3,1);
    thetaRF_ = stab_->x_delta_.block(15,0,3,1);
    vRF_ = stab_->x_delta_.block(18,0,3,1);
    omRF_ = stab_->x_delta_.block(21,0,3,1);
    fRF_ = stab_->f_delta_.block(0,0,3,1);
    tRF_ = stab_->f_delta_.block(3,0,3,1);

    pLF_ = stab_->x_delta_.block(24,0,3,1);
    thetaLF_ = stab_->x_delta_.block(27,0,3,1);
    vLF_ = stab_->x_delta_.block(30,0,3,1);
    omLF_ = stab_->x_delta_.block(33,0,3,1);
    fLF_ = stab_->f_delta_.block(6,0,3,1);
    tLF_ = stab_->f_delta_.block(9,0,3,1);

    pRH_ = stab_->x_delta_.block(36,0,3,1);
    thetaRH_ = stab_->x_delta_.block(39,0,3,1);
    vRH_ = stab_->x_delta_.block(42,0,3,1);
    omRH_ = stab_->x_delta_.block(45,0,3,1);
    fRH_ = stab_->f_delta_.block(12,0,3,1);
    tRH_ = stab_->f_delta_.block(15,0,3,1);

    // To log the Friction at the Right Foot and Right Hand
    // To review for the RH since you're not choosing the normal force correctly

    f_x_RF_ = realRobots().robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(realRobots().robot()).force().x();
    f_y_RF_ = realRobots().robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(realRobots().robot()).force().y();
    f_z_RF_ = realRobots().robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(realRobots().robot()).force().z();

    f_x_RH_ = realRobots().robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(realRobots().robot()).force().x();
    f_y_RH_ = realRobots().robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(realRobots().robot()).force().y();
    f_z_RH_ = realRobots().robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(realRobots().robot()).force().z();

    if(f_z_RF_ == 0.0 ) {
    
    fr_x_RF_ = 0.0;
    fr_y_RF_ = 0.0;

    }

    else {

    fr_x_RF_ = f_x_RF_/f_z_RF_;
    fr_y_RF_ = f_y_RF_/f_z_RF_;

    }

    if(f_x_RH_ == 0.0 ) {
    
    fr_z_RH_ = 0.0;
    fr_y_RH_ = 0.0;

    }

    else {

    fr_z_RH_ = f_z_RH_/f_x_RH_;
    fr_y_RH_ = f_y_RH_/f_x_RH_;

  }

  }

   t_ += timeStep;

  return mc_control::fsm::Controller::run();
}

// Reset function for the mc_rtc controller

void MscController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  comTask_->reset();
  baseTask_->reset();

  rightFoot_PosTask_->reset();
  rightFoot_OrTask_->reset();

  leftFoot_PosTask_->reset();
  leftFoot_OrTask_->reset();

  rightHand_PosTask_->reset();
  rightHand_OrTask_->reset();

  // Add the CoM and Base Tasks permanently to not cause the drifting of the base of the robot when the controller is disabled

  comTask_->selectActiveJoints(Activedof_);
  baseTask_->selectActiveJoints(Activedof_);

  solver().addTask(comTask_);
  solver().addTask(baseTask_);

  const auto & observerp = observerPipeline(observerPipelineName_);
  
  if(observerp.success())
  {
    mc_rtc::log::info("Pipeline \"{}\" for real robot observation loaded!", observerPipelineName_);
  }

  // GUI related code for the Initialize button

  if (!init) {
    gui()->addElement({"Stabilizer","Initialization"}, mc_rtc::gui::Button("Initialize", [this]() {

/*       solver().removeConstraintSet(dynamicsConstraint);
      solver().addConstraintSet(kinematicsConstraint); */

      removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
      removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});

      addContact({robot().name(), "ground", "RightFoot", "AllGround", mc_rbdyn::Contact::defaultFriction, dof});
      addContact({robot().name(), "ground", "LeftFoot", "AllGround", mc_rbdyn::Contact::defaultFriction, dof});

      mc_rtc::log::info("Feet Contacts are now free to move\n");

      init = true;

   }));
  }

  // Plot related code in RViz

  using Color = mc_rtc::gui::Color;

  gui()->addPlot(
      "Right Foot CoP_x(t)", mc_rtc::gui::plot::X("t",  [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "CoP(x)", [this]() { return realRobots().robot().cop("RightFoot").x(); }, Color::Blue));

  gui()->addPlot(
      "Right Foot CoP_y(t)", mc_rtc::gui::plot::X("t",  [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "CoP(y)", [this]() { return realRobots().robot().cop("RightFoot").y(); }, Color::Blue));

  gui()->addPlot(
      "Robot's CoM_x(t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "CoM(x)", [this]() { return realRobots().robot().com().x(); }, Color::Red));

  gui()->addPlot(
      "Robot's CoM_y(t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "CoM(y)", [this]() { return realRobots().robot().com().y(); }, Color::Red));

   gui()->addPlot(
      "Right Hand Force (t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "f_RH(z)", [this]() { return realRobots().robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(realRobots().robot()).force().z(); }, Color::Red), 
      mc_rtc::gui::plot::Y(
          "f_RH(x)", [this]() { return realRobots().robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(realRobots().robot()).force().x(); }, Color::Green),
      mc_rtc::gui::plot::Y(
          "f_RH(y)", [this]() { return realRobots().robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(realRobots().robot()).force().y(); }, Color::Blue)); 

}