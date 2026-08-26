#include "machine_state.h"

#include <stdio.h>

static int CheckCommand(const char *command) {
  MachineMotionPlan init_plan;
  MachineMotionPlan plan;

  MachineState_Init();
  if (MachineState_BuildRotatePlan("INIT", &init_plan) !=
      MACHINE_PLAN_OK) {
    printf("INIT rejected before %s\n", command);
    return 1;
  }
  MachineState_CommitRotate(&init_plan);

  if (MachineState_BuildRotatePlan(command, &plan) != MACHINE_PLAN_OK) {
    printf("%s rejected\n", command);
    return 1;
  }
  if (plan.stage_count == 0U) {
    printf("%s produced no action\n", command);
    return 1;
  }
  return 0;
}

int main(void) {
  static const char *const commands[] = {
      "R", "R_", "R2", "L", "L_", "L2", "U", "U_", "U2",
      "D", "D_", "D2", "F", "F_", "F2",
  };
  size_t index;

  for (index = 0U; index < sizeof(commands) / sizeof(commands[0]); index++) {
    if (CheckCommand(commands[index]) != 0) {
      return 1;
    }
  }

  puts("original actions: OK");
  return 0;
}
