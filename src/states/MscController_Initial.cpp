#include "MscController_Initial.h"

#include "../MscController.h"

void MscController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void MscController_Initial::start(mc_control::fsm::Controller & ctl_)
{
    ctl_.gui()->addElement({"Stabilizer","FSM"}, mc_rtc::gui::Button("Move Right Hand", [this]() { start_ = true; }));

}

bool MscController_Initial::run(mc_control::fsm::Controller & ctl_)
{
 if(start_)
  {
    output("OK");
    return true;
  }
  return false;
}

void MscController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MscController &>(ctl_);
  ctl_.gui()->removeElement({"Stabilizer","FSM"},"Move Right Hand");
}

EXPORT_SINGLE_STATE("MscController_Initial", MscController_Initial)
