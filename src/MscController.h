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

struct MscController_DLLAPI MscController : public mc_control::fsm::Controller
{
    MscController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

protected:
  
  double t_ = 0; ///< Elapsed time since the controller started

private:

    mc_rtc::Configuration config_;

    bool stabilizer = false;
    bool flip = false;
    bool init = false;
    bool main = false;
    bool ref = false;
    bool compute = false;
    
    Eigen::Vector6d dof, dof_full;

    Vector3d com_, theta_, comd_, om_, pRF_, thetaRF_, vRF_, omRF_, fRF_, tRF_, pLF_, thetaLF_, vLF_, omLF_, fLF_, tLF_, pRH_, thetaRH_, vRH_, omRH_, fRH_, tRH_;

    std::shared_ptr<msc_stabilizer::Stabilizer> stab_;

    std::shared_ptr<mc_tasks::CoMTask> comTask_;
    std::shared_ptr<mc_tasks::OrientationTask> baseTask_;

    std::shared_ptr<mc_tasks::PositionTask> rightFoot_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> rightFoot_OrTask_;

    std::shared_ptr<mc_tasks::PositionTask> leftFoot_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> leftFoot_OrTask_;

    std::shared_ptr<mc_tasks::PositionTask> rightHand_PosTask_;
    std::shared_ptr<mc_tasks::OrientationTask> rightHand_OrTask_;
    
    std::string observerPipelineName_ = "MscControllerObserverPipeline";
};