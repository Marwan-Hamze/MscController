#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/Robots.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "msc_stabilizer.h"

#include "api.h"

using Eigen::Matrix;

struct MscController_DLLAPI MscController : public mc_control::fsm::Controller
{
    MscController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void logs_robot(mc_rbdyn::Robots &robots);

private:
    mc_rtc::Configuration config_;
    Eigen::VectorXd comAcc;
    sva::MotionVecd Gain;
    sva::MotionVecd uRF, uLF, uB;
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::msc_stabilizer::Stabilizer> stab_;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> rightFootTask_;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> leftFootTask_;
    std::string observerPipelineName_ = "MscControllerObserverPipeline";
};