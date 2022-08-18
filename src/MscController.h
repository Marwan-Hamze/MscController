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
  
  // Elapsed time since the controller started, used for plotting curves in RViz
  double t_ = 0;

private:

    mc_rtc::Configuration config_;

    // The booleans below are used for the GUI customization

    bool stabilizer = false;
    bool flip = false;
    bool main = false;
    bool ref = false;
    bool compute = false;
    
    // The 6d vectors below are used to free or fix the contacts to the ground. They have no effect in this branch, their use is in the multi-contact situation
    Eigen::Vector6d dof, dof_full;

    // Vectors to add error signals to the logs
    Vector3d com_, theta_, comd_, om_, pRF_, thetaRF_, vRF_, omRF_, fRF_, tRF_, pLF_, thetaLF_, vLF_, omLF_, fLF_, tLF_;

    std::shared_ptr<msc_stabilizer::Stabilizer> stab_;

    std::shared_ptr<mc_tasks::CoMTask> comTask_;
    std::shared_ptr<mc_tasks::OrientationTask> baseTask_;

    std::shared_ptr<mc_tasks::PositionTask> rightFoot_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> rightFoot_OrTask_;

    std::shared_ptr<mc_tasks::PositionTask> leftFoot_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> leftFoot_OrTask_;
    
    std::string observerPipelineName_ = "MscControllerObserverPipeline";
};