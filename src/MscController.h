#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/OrientationTask.h>
#include "msc_stabilizer.h"

#include "api.h"

using Eigen::Matrix;
using Eigen::Vector3d;

// Constructor for the mc_rtc FSM controller

struct MscController_DLLAPI MscController : public mc_control::fsm::Controller
{
    MscController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

protected:
  
  double t_ = 0; ///< Elapsed time since the controller started

private:

    mc_rtc::Configuration config_;

    // The booleans below are used for the GUI customization

    bool stabilizer = false;
    bool flip = false;
    bool init = false;
    bool main = false;
    bool ref = false;
    bool compute = false;
    
    Eigen::Vector6d dof, dof_full;

    // Vectors to add error signals to the logs

    Vector3d com_, theta_, comd_, om_, pRF_, thetaRF_, vRF_, omRF_, fRF_, tRF_, pLF_, thetaLF_, vLF_, omLF_, fLF_, tLF_, pRH_, thetaRH_, vRH_, omRH_, fRH_, tRH_;

    // Variables to calculate and Log the Contact Friction 

    double f_x_RF_, f_y_RF_, f_z_RF_, fr_x_RF_, fr_y_RF_;
    double f_x_RH_, f_y_RH_, f_z_RH_, fr_z_RH_, fr_y_RH_;

    // Accelerations by Finite Differences to check the accelerations.

    Vector3d RF_linear_acc, RF_angular_acc, LF_linear_acc, LF_angular_acc, RH_linear_acc, RH_angular_acc;
    Vector3d comdd_, omegad_;

    // Old velocities for the Finite Difference Method

    Vector3d v_RF_old_ = v_RF_old_.Zero();
    Vector3d w_RF_old_ = w_RF_old_.Zero();
    Vector3d v_RH_old_ = v_RH_old_.Zero();
    Vector3d w_RH_old_ = w_RH_old_.Zero();
    Vector3d v_LF_old_ = v_LF_old_.Zero();
    Vector3d w_LF_old_ = w_LF_old_.Zero();
    Vector3d v_com_old_ = v_com_old_.Zero();
    Vector3d w_base_old_ = w_base_old_.Zero();

    // Tasks and Stabilizer Pointers

    std::shared_ptr<msc_stabilizer::Stabilizer> stab_;

    std::shared_ptr<mc_tasks::CoMTask> comTask_;
    std::shared_ptr<mc_tasks::OrientationTask> baseTask_;

    std::shared_ptr<mc_tasks::PositionTask> rightFoot_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> rightFoot_OrTask_;

    std::shared_ptr<mc_tasks::PositionTask> leftFoot_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> leftFoot_OrTask_;

    std::shared_ptr<mc_tasks::PositionTask> rightHand_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> rightHand_OrTask_;

    // Observer Pipeline
    
    std::string observerPipelineName_ = "MscControllerObserverPipeline";
};