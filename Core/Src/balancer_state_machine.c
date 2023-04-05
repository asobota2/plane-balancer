#include "balancer_state_machine.h"

StateMachineHandle StateMachine_Create() {
  StateMachineHandle handle;
  handle.state = Init;
  return handle;
}

BalancerState StateMachine_Init_GetNextState(const BS_State* system_state);
BalancerState StateMachine_WaitForData_GetNextState(const BS_State* system_state);
BalancerState StateMachine_UpdatePID_GetNextState(const BS_State* system_state);
BalancerState StateMachine_CalcPlatformAngle_GetNextState(const BS_State* system_state);
BalancerState StateMachine_SetServos_GetNextState(const BS_State* system_state);

void StateMachine_GetNextState(StateMachineHandle* handle, const BS_State* system_state) {
  switch (handle->state) {
    case Init:
      handle->state = StateMachine_Init_GetNextState(system_state);
      break;
    case WaitForData:
      handle->state = StateMachine_WaitForData_GetNextState(system_state);
      break;
    case UpdatePID:
      handle->state = StateMachine_UpdatePID_GetNextState(system_state);
      break;
    case CalcPlatformAngle:
      handle->state = StateMachine_CalcPlatformAngle_GetNextState(system_state);
      break;
    case SetServos:
      handle->state = StateMachine_SetServos_GetNextState(system_state);
      break;
    default:
      // TODO - Report error
      handle->state = Init;
  }
}

BalancerState StateMachine_Init_GetNextState(const BS_State* system_state) {
  return WaitForData;
}

BalancerState StateMachine_WaitForData_GetNextState(const BS_State* system_state) {
  if (*system_state->new_ball_pos_data) {
    return UpdatePID;
  }

  if (*system_state->new_accel_gyro_data) {
    return CalcPlatformAngle;
  }

  return WaitForData;
}

BalancerState StateMachine_UpdatePID_GetNextState(const BS_State* system_state) {
  if (*system_state->new_accel_gyro_data) {
    return CalcPlatformAngle;
  }

  return WaitForData;
}

BalancerState StateMachine_CalcPlatformAngle_GetNextState(const BS_State* system_state) {
  return SetServos;
}

BalancerState StateMachine_SetServos_GetNextState(const BS_State* system_state) {
  return WaitForData;
}
