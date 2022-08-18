#include "RemoveHand.h"

#include <mc_control/fsm/Controller.h>

void RemoveHand::configure(const mc_rtc::Configuration & config)
{
}

void RemoveHand::start(mc_control::fsm::Controller & ctl_)
{
    // ctl_.gui()->addElement({"Stabilizer","FSM"}, mc_rtc::gui::Button("Remove Right Hand", [this]() { start_ = true; }));

}

bool RemoveHand::run(mc_control::fsm::Controller & ctl_)
{
 if(start_)
  {
    output("OK");
    return true;
  }
  return false;
}

void RemoveHand::teardown(mc_control::fsm::Controller & ctl_)
{
   // ctl_.gui()->removeElement({"Stabilizer","FSM"},"Remove Right Hand");
}

EXPORT_SINGLE_STATE("RemoveHand", RemoveHand)
