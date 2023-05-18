# pragma once

#include <hydra_controllers/robot_data_container.h>
#include <hydra_controllers/cached_controller_data.h>
#include <hydra_hw/hydra_model_interface.h>

namespace hydra_controllers {

typedef std::map<std::string, ZaDataContainer> ArmDataMap;

void taskPriorityControl(ZaDataContainer& arm_data,
                         CachedModelData& input,
                         const ControllerParameters& context);

void coordinatedTaskPriorityControl(ZaDataContainer& arm_data,
                                    CachedModelData& input,
                                    const ControllerParameters& context,
                                    double positioner_cmd);

void positionerControl(PositionerDataContainer& positioner_data,
                       const ArmDataMap& arm_data,
                       CachedControllerData& controller_data,
                       const ControllerParameters& context);

void cacheControllerData(ArmDataMap& arms_data,
                         const std::unique_ptr<hydra_hw::HydraModelHandle>& model_handle,
                         CachedControllerData& data);
} // namespace hydra_controllers