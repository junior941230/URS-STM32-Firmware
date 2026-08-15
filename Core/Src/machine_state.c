#include "machine_state.h"

#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static MachineState machine_state;

static FaceModule *MachineState_GetFaceModule(MachineFace face) {
  switch (face) {
  case MACHINE_FACE_LEFT:
    return &machine_state.left_module;
  case MACHINE_FACE_RIGHT:
    return &machine_state.right_module;
  case MACHINE_FACE_UP:
    return &machine_state.up_module;
  case MACHINE_FACE_DOWN:
    return &machine_state.down_module;
  case MACHINE_FACE_FRONT:
    return &machine_state.front_module;
  case MACHINE_FACE_BACK:
    return &machine_state.back_module;
  default:
    return NULL;
  }
}

static uint8_t MachineState_ParseCommand(const char *text,
                                         Rotate_Command_Type *command) {
  static const struct {
    const char *name;
    Rotate_Command_Type command;
  } command_table[] = {
      {"R", ROTATE_COMMAND_R},   {"R_", ROTATE_COMMAND_R_PRIME},
      {"L", ROTATE_COMMAND_L},   {"L_", ROTATE_COMMAND_L_PRIME},
      {"U", ROTATE_COMMAND_U},   {"U_", ROTATE_COMMAND_U_PRIME},
      {"D", ROTATE_COMMAND_D},   {"D_", ROTATE_COMMAND_D_PRIME},
      {"F", ROTATE_COMMAND_F},   {"F_", ROTATE_COMMAND_F_PRIME},
      {"B", ROTATE_COMMAND_B},   {"B_", ROTATE_COMMAND_B_PRIME},
      {"RW", ROTATE_COMMAND_RW}, {"RW_", ROTATE_COMMAND_RW_PRIME},
      {"LW", ROTATE_COMMAND_LW}, {"LW_", ROTATE_COMMAND_LW_PRIME},
      {"UW", ROTATE_COMMAND_UW}, {"UW_", ROTATE_COMMAND_UW_PRIME},
      {"DW", ROTATE_COMMAND_DW}, {"DW_", ROTATE_COMMAND_DW_PRIME},
      {"FW", ROTATE_COMMAND_FW}, {"FW_", ROTATE_COMMAND_FW_PRIME},
      {"BW", ROTATE_COMMAND_BW}, {"BW_", ROTATE_COMMAND_BW_PRIME}};
  char normalized[4] = {0};
  size_t length;
  size_t index;

  if ((text == NULL) || (command == NULL)) {
    return 0U;
  }

  length = strlen(text);
  if ((length == 0U) || (length >= sizeof(normalized))) {
    return 0U;
  }
  for (index = 0U; index < length; index++) {
    normalized[index] = (char)toupper((unsigned char)text[index]);
  }

  for (index = 0U; index < (sizeof(command_table) / sizeof(command_table[0]));
       index++) {
    if (strcmp(normalized, command_table[index].name) == 0) {
      *command = command_table[index].command;
      return 1U;
    }
  }
  return 0U;
}

static MachineFace MachineState_CommandFace(Rotate_Command_Type command) {
  switch (command) {
  case ROTATE_COMMAND_R:
  case ROTATE_COMMAND_R_PRIME:
  case ROTATE_COMMAND_RW:
  case ROTATE_COMMAND_RW_PRIME:
    return MACHINE_FACE_RIGHT;
  case ROTATE_COMMAND_L:
  case ROTATE_COMMAND_L_PRIME:
  case ROTATE_COMMAND_LW:
  case ROTATE_COMMAND_LW_PRIME:
    return MACHINE_FACE_LEFT;
  case ROTATE_COMMAND_U:
  case ROTATE_COMMAND_U_PRIME:
  case ROTATE_COMMAND_UW:
  case ROTATE_COMMAND_UW_PRIME:
    return MACHINE_FACE_UP;
  case ROTATE_COMMAND_D:
  case ROTATE_COMMAND_D_PRIME:
  case ROTATE_COMMAND_DW:
  case ROTATE_COMMAND_DW_PRIME:
    return MACHINE_FACE_DOWN;
  case ROTATE_COMMAND_F:
  case ROTATE_COMMAND_F_PRIME:
  case ROTATE_COMMAND_FW:
  case ROTATE_COMMAND_FW_PRIME:
    return MACHINE_FACE_FRONT;
  default:
    return MACHINE_FACE_BACK;
  }
}

static uint8_t MachineState_CommandIsPrime(Rotate_Command_Type command) {
  return ((uint8_t)command & 1U) ? 1U : 0U;
}

static uint8_t MachineState_AngleIsValid(double angle_degrees) {
  const double minimum_angle = 180.0 / 16384.0;
  const double maximum_angle = 8388607.0 * 360.0 / 16384.0;

  if ((angle_degrees != angle_degrees) || (angle_degrees > maximum_angle) ||
      (angle_degrees < -maximum_angle)) {
    return 0U;
  }
  return ((angle_degrees >= minimum_angle) || (angle_degrees <= -minimum_angle))
             ? 1U
             : 0U;
}

static double MachineState_MapFaceAngle(const FaceModule *module,
                                        double logical_angle_degrees) {
  return module->clockwise_axis_positive ? logical_angle_degrees
                                         : -logical_angle_degrees;
}

MachinePlanStatus MachineState_PlanAddStage(MachineMotionPlan *plan,
                                            uint8_t *stage_index) {
  if ((plan == NULL) || (stage_index == NULL)) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }
  if (plan->stage_count >= MACHINE_STATE_MAX_STAGES) {
    return MACHINE_PLAN_TOO_MANY_STAGES;
  }

  *stage_index = plan->stage_count;
  memset(&plan->stages[plan->stage_count], 0,
         sizeof(plan->stages[plan->stage_count]));
  plan->stage_count++;
  return MACHINE_PLAN_OK;
}

MachinePlanStatus MachineState_PlanAddMotion(MachineMotionPlan *plan,
                                             uint8_t stage_index, uint8_t bus,
                                             uint16_t id, uint16_t speed_rpm,
                                             uint8_t acceleration,
                                             double angle_degrees) {
  MachineMotionStage *stage;
  uint8_t index;

  if ((plan == NULL) || (stage_index >= plan->stage_count) || (bus < 1U) ||
      (bus > 2U) || (id < 1U) || (id > 0x7FFU) || (speed_rpm < 1U) ||
      (speed_rpm > 3000U) || (!MachineState_AngleIsValid(angle_degrees))) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }
  stage = &plan->stages[stage_index];
  if (stage->motion_count >= MACHINE_STATE_MAX_SYNC_MOTORS) {
    return MACHINE_PLAN_TOO_MANY_MOTORS;
  }

  /* 同一個 stage 不允許重複定址同一顆馬達。 */
  for (index = 0U; index < stage->motion_count; index++) {
    if ((stage->motions[index].bus == bus) &&
        (stage->motions[index].id == id)) {
      return MACHINE_PLAN_INVALID_ARGUMENT;
    }
  }

  stage->motions[stage->motion_count].bus = bus;
  stage->motions[stage->motion_count].id = id;
  stage->motions[stage->motion_count].speed_rpm = speed_rpm;
  stage->motions[stage->motion_count].acceleration = acceleration;
  stage->motions[stage->motion_count].angle_degrees = angle_degrees;
  stage->motion_count++;
  return MACHINE_PLAN_OK;
}

MachinePlanStatus
MachineState_PlanAddFaceMotion(MachineMotionPlan *plan, uint8_t stage_index,
                               MachineFace face, MachineMotorRole role,
                               uint16_t speed_rpm, uint8_t acceleration,
                               double angle_degrees) {
  FaceModule *module = MachineState_GetFaceModule(face);
  uint16_t id;

  if ((module == NULL) || (role > MACHINE_MOTOR_SMALL) ||
      (!MachineState_AngleIsValid(angle_degrees))) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }
  id = (role == MACHINE_MOTOR_BIG) ? module->big_motor_id
                                   : module->small_motor_id;
  return MachineState_PlanAddMotion(
      plan, stage_index, module->bus, id, speed_rpm, acceleration,
      MachineState_MapFaceAngle(module, angle_degrees));
}
static MachinePlanStatus FaceModule_Macro(MachineMotionPlan *plan,
                                          uint8_t stage, MachineFace face,
                                          uint16_t rpm, uint8_t acc,
                                          double angle_degrees) {
  MachinePlanStatus status;
  status = MachineState_PlanAddFaceMotion(
      plan, stage, face, MACHINE_MOTOR_SMALL, rpm, acc, angle_degrees);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }
  return MachineState_PlanAddFaceMotion(
      plan, stage, face, MACHINE_MOTOR_BIG, (uint16_t)(rpm * 2U),
      (uint8_t)(acc * 2U), angle_degrees * 2.0);
}

static MachinePlanStatus
MachineState_AddPreparationStages(Rotate_Command_Type command,
                                  MachineMotionPlan *plan) {
  /*
   * 在這裡編排「目標 face 轉動前」的準備 stage。
   * 同一個 stage 內可加入多顆馬達，它們會用 4AH/4BH 同步啟動。
   * 再呼叫一次 MachineState_PlanAddStage() 就是下一組，前一組完成才執行。
   *
   * R 的兩組準備動作範例（先用 MachineState_ConfigureFace() 設定 ID）：
   *   uint8_t stage;
   *   MachinePlanStatus status;
   *   status = MachineState_PlanAddStage(plan, &stage);       // 第 1 組
   *   if (status != MACHINE_PLAN_OK) return status;
   *   status = MachineState_PlanAddFaceMotion(
   *     plan, stage, MACHINE_FACE_LEFT, MACHINE_MOTOR_SMALL,
   *     200U, 2U, 20.0);
   *   if (status != MACHINE_PLAN_OK) return status;
   *   status = MachineState_PlanAddFaceMotion(
   *     plan, stage, MACHINE_FACE_UP, MACHINE_MOTOR_SMALL,
   *     200U, 2U, -20.0);                                    // 與 LEFT
   * 同步反向 if (status != MACHINE_PLAN_OK) return status; status =
   * MachineState_PlanAddStage(plan, &stage);       // 第 2 組 if (status !=
   * MACHINE_PLAN_OK) return status; return MachineState_PlanAddFaceMotion(
   *     plan, stage, MACHINE_FACE_FRONT, MACHINE_MOTOR_SMALL,
   *     200U, 2U, 20.0);
   * 最後會自動執行第 3 組：RIGHT big motor。
   */
  switch (command) {
  case ROTATE_COMMAND_R: {
    uint8_t stage;
    MachinePlanStatus status;

    status = MachineState_PlanAddStage(plan, &stage);
    if (status != MACHINE_PLAN_OK) {
      return status;
    }

    return FaceModule_Macro(plan, stage, MACHINE_FACE_LEFT, 200U, 2U, 20.0);
  }
  case ROTATE_COMMAND_R_PRIME:
  case ROTATE_COMMAND_L:
  case ROTATE_COMMAND_L_PRIME:
  case ROTATE_COMMAND_U:
  case ROTATE_COMMAND_U_PRIME:
  case ROTATE_COMMAND_D:
  case ROTATE_COMMAND_D_PRIME:
  
  case ROTATE_COMMAND_F:
  case ROTATE_COMMAND_F_PRIME:
  case ROTATE_COMMAND_B:
  case ROTATE_COMMAND_B_PRIME:
  case ROTATE_COMMAND_RW:
  case ROTATE_COMMAND_RW_PRIME:
  case ROTATE_COMMAND_LW:
  case ROTATE_COMMAND_LW_PRIME:
  case ROTATE_COMMAND_UW:
  case ROTATE_COMMAND_UW_PRIME:
  case ROTATE_COMMAND_DW:
  case ROTATE_COMMAND_DW_PRIME:
  case ROTATE_COMMAND_FW:
  case ROTATE_COMMAND_FW_PRIME:
  case ROTATE_COMMAND_BW:
  case ROTATE_COMMAND_BW_PRIME:
    return MACHINE_PLAN_OK;
  default:
    (void)plan;
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }
}

void MachineState_Init(void) {
  memset(&machine_state, 0, sizeof(machine_state));
  machine_state.left_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.right_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.up_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.down_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.front_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.back_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;

  /* 未另外設定時，Rubik 邏輯正角度對應 Servo42D 正 relAxis。 */
  machine_state.left_module.clockwise_axis_positive = 1U;
  machine_state.right_module.clockwise_axis_positive = 1U;
  machine_state.up_module.clockwise_axis_positive = 1U;
  machine_state.down_module.clockwise_axis_positive = 1U;
  machine_state.front_module.clockwise_axis_positive = 1U;
  machine_state.back_module.clockwise_axis_positive = 1U;

  /*
   * 請在這裡集中設定實際接線，例如：
   * MachineState_ConfigureFace(MACHINE_FACE_LEFT,  1U, 1U, 2U, 1U);
   * MachineState_ConfigureFace(MACHINE_FACE_RIGHT, 1U, 3U, 4U, 1U);
   * 參數依序是 face、bus、big motor ID、small motor ID、正角度軸向。
   */
  MachineState_ConfigureFace(MACHINE_FACE_FRONT, 1U, 12U, 13U, 1U);
  MachineState_ConfigureFace(MACHINE_FACE_BACK, 2U, 10U, 11U, 1U);
  MachineState_ConfigureFace(MACHINE_FACE_LEFT, 1U, 8U, 9U, 1U);
  MachineState_ConfigureFace(MACHINE_FACE_RIGHT, 2U, 4U, 5U, 1U);
  MachineState_ConfigureFace(MACHINE_FACE_UP, 1U, 2U, 3U, 1U);
  MachineState_ConfigureFace(MACHINE_FACE_DOWN, 2U, 6U, 7U, 1U);
}

void MachineState_ConfigureFace(MachineFace face, uint8_t bus,
                                uint16_t big_motor_id, uint16_t small_motor_id,
                                uint8_t clockwise_axis_positive) {
  FaceModule *module = MachineState_GetFaceModule(face);

  if ((module == NULL) || (bus < 1U) || (bus > 2U) || (big_motor_id < 1U) ||
      (big_motor_id > 0x7FFU) || (small_motor_id > 0x7FFU) ||
      (clockwise_axis_positive > 1U)) {
    return;
  }
  module->bus = bus;
  module->big_motor_id = big_motor_id;
  module->small_motor_id = small_motor_id;
  module->clockwise_axis_positive = clockwise_axis_positive;
}

MachinePlanStatus MachineState_BuildRotatePlan(uint8_t fallback_bus,
                                               uint16_t fallback_id,
                                               const char *command_text,
                                               MachineMotionPlan *plan) {
  Rotate_Command_Type command;
  FaceModule *module;
  MachinePlanStatus status;

  if ((plan == NULL) || (fallback_bus < 1U) || (fallback_bus > 2U) ||
      (fallback_id < 1U) || (fallback_id > 0x7FFU) ||
      (!MachineState_ParseCommand(command_text, &command))) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }

  memset(plan, 0, sizeof(*plan));
  plan->command = command;
  module = MachineState_GetFaceModule(MachineState_CommandFace(command));
  if (module == NULL) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }

  status = MachineState_AddPreparationStages(command, plan);
  return status;
}

void MachineState_CommitRotate(Rotate_Command_Type command) {
  FaceModule *module =
      MachineState_GetFaceModule(MachineState_CommandFace(command));
  int16_t delta;

  if (module == NULL) {
    return;
  }
  delta = MachineState_CommandIsPrime(command) ? -90 : 90;
  module->big_motor_angle_degrees =
      (int16_t)(module->big_motor_angle_degrees + delta);

  while (module->big_motor_angle_degrees >= 360) {
    module->big_motor_angle_degrees -= 360;
  }
  while (module->big_motor_angle_degrees < 0) {
    module->big_motor_angle_degrees += 360;
  }
}

const MachineState *MachineState_Get(void) { return &machine_state; }
