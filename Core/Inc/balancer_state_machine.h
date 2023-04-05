#ifndef BALANCER_STATE_MACHINE_H_
#define BALANCER_STATE_MACHINE_H_

#include "balancer_system_state.h"

typedef enum {
  Init,
  WaitForData,
  UpdatePID,
  CalcPlatformAngle,
  SetServos
} BalancerState;

typedef struct {
  BalancerState state;
} StateMachineHandle;

/**
 * @brief Create state machine handle
 * 
 * @return StateMachineHandle 
 */
StateMachineHandle StateMachine_Create();

/**
 * @brief Set next state of state machine base on system state
 * 
 * @param handle 
 * @param system_state 
 */
void StateMachine_GetNextState(StateMachineHandle* handle, const BS_State* system_state);

#endif  // BALANCER_STATE_MACHINE_H_