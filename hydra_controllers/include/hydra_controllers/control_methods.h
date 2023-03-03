# pragma once

#include <hydra_controllers/robot_data_container.h>
#include <hydra_controllers/cached_controller_data.h>

namespace hydra_controllers {

void taskPriorityControl(ZaDataContainer& arm_data,
                         CachedModelData& input,
                         const TPCControllerParameters& context);

void coordinatedTaskPriorityControl(ZaDataContainer& arm_data,
                                    const CTPCControllerParameters& context);

void positionerControl(PositionerDataContainer& positioner_data,
                       CachedControllerData& controller_data);

void cacheControllerData(std::map<std::string, ZaDataContainer>& arms_data,
                         CachedControllerData& data);
} // namespace hydra_controllers