#ifndef FLIGHT_CONTROL_NODE_H
#define FLIGHT_CONTROL_NODE_H

namespace DroneNodes
{
  enum class ActionStatus {VIRGIN, REJECTED, PROCESSING, SUCCEEDED, ABORTED, CANCELED, UNKNOWN};
}  

#endif  // FLIGHT_CONTROL_NODE_H