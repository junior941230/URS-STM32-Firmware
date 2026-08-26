#include "machine_state.h"

#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MAX_RPM 60U

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
                                         Rotate_Command_Type *command,
                                         uint8_t *command_multiplier) {
  static const struct {
    const char *name;
    Rotate_Command_Type command;
  } command_table[] = {
      {"R", ROTATE_COMMAND_R},      {"R_", ROTATE_COMMAND_R_PRIME},
      {"L", ROTATE_COMMAND_L},      {"L_", ROTATE_COMMAND_L_PRIME},
      {"U", ROTATE_COMMAND_U},      {"U_", ROTATE_COMMAND_U_PRIME},
      {"D", ROTATE_COMMAND_D},      {"D_", ROTATE_COMMAND_D_PRIME},
      {"F", ROTATE_COMMAND_F},      {"F_", ROTATE_COMMAND_F_PRIME},
      {"B", ROTATE_COMMAND_B},      {"B_", ROTATE_COMMAND_B_PRIME},
      {"RW", ROTATE_COMMAND_RW},    {"RW_", ROTATE_COMMAND_RW_PRIME},
      {"LW", ROTATE_COMMAND_LW},    {"LW_", ROTATE_COMMAND_LW_PRIME},
      {"UW", ROTATE_COMMAND_UW},    {"UW_", ROTATE_COMMAND_UW_PRIME},
      {"DW", ROTATE_COMMAND_DW},    {"DW_", ROTATE_COMMAND_DW_PRIME},
      {"FW", ROTATE_COMMAND_FW},    {"FW_", ROTATE_COMMAND_FW_PRIME},
      {"BW", ROTATE_COMMAND_BW},    {"BW_", ROTATE_COMMAND_BW_PRIME},
      {"INIT", ROTATE_COMMAND_INIT}};
  char normalized[6] = {0};
  size_t length;
  size_t index;
  uint8_t multiplier = 1U;

  if ((text == NULL) || (command == NULL) || (command_multiplier == NULL)) {
    return 0U;
  }

  length = strlen(text);
  if ((length == 0U) || (length >= sizeof(normalized))) {
    return 0U;
  }
  for (index = 0U; index < length; index++) {
    normalized[index] = (char)toupper((unsigned char)text[index]);
  }
  if ((length >= 2U) && (normalized[length - 1U] == '2')) {
    if (normalized[length - 2U] == '_') {
      return 0U;
    }
    normalized[length - 1U] = '\0';
    multiplier = 2U;
  }

  for (index = 0U; index < (sizeof(command_table) / sizeof(command_table[0]));
       index++) {
    if (strcmp(normalized, command_table[index].name) == 0) {
      if ((multiplier == 2U) &&
          (command_table[index].command == ROTATE_COMMAND_INIT)) {
        return 0U;
      }
      *command = command_table[index].command;
      *command_multiplier = multiplier;
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
                                          uint16_t big_rpm, uint8_t big_acc,
                                          int8_t angle_units) {
  FaceModule *module = MachineState_GetFaceModule(face);
  MachinePlanStatus status;
  uint8_t small_acc;
  uint16_t small_rpm;
  double angle;

  if ((plan == NULL) || (module == NULL)) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }
  if (angle_units == 0) {
    return MACHINE_PLAN_OK;
  }

  small_acc = (uint8_t)(256U - (256U - big_acc) / 0.5);
  small_rpm = (uint16_t)(big_rpm * 0.5);
  /* 一個 unit 是 90 度；big motor 維持既有的 2:1 動作比例。 */
  angle = (double)angle_units * 90.0;
  status = MachineState_PlanAddFaceMotion(
      plan, stage, face, MACHINE_MOTOR_SMALL, small_rpm, small_acc, angle);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }

  status = MachineState_PlanAddFaceMotion(plan, stage, face, MACHINE_MOTOR_BIG,
                                          big_rpm, big_acc, angle * 2.0);
  /* 奇數個 90 度 unit 會切換垂直/水平；偶數個 unit 回到相同姿態。 */
  if ((status == MACHINE_PLAN_OK) && ((angle_units % 2) != 0)) {
    plan->pending_big_states[(uint8_t)face] =
        (plan->pending_big_states[(uint8_t)face] == BIG_MOTOR_VERTICAL)
            ? BIG_MOTOR_HORIZON
            : BIG_MOTOR_VERTICAL;
    plan->pending_big_state_mask |= (uint8_t)(1U << (uint8_t)face);
  }
  return status;
}

MachinePlanStatus MachineState_PlanGrip(MachineMotionPlan *plan, uint8_t stage,
                                        MachineFace face, uint16_t rpm,
                                        uint8_t acc,
                                        SmallMotor_State small_state) {
  FaceModule *module = MachineState_GetFaceModule(face);
  MachinePlanStatus status;
  int16_t delta;

  if ((plan == NULL) || (module == NULL) ||
      (small_state > MOTOR_ANGLE_SHORT_SIDE)) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }
  if (small_state == plan->pending_small_states[(uint8_t)face]) {
    return MACHINE_PLAN_OK;
  }
  delta =
      (int16_t)small_state - (int16_t)plan->pending_small_states[(uint8_t)face];
  status = MachineState_PlanAddFaceMotion(
      plan, stage, face, MACHINE_MOTOR_SMALL, rpm, acc, (double)delta * 90.0);
  if (status == MACHINE_PLAN_OK) {
    plan->pending_small_states[(uint8_t)face] = small_state;
    plan->pending_small_state_mask |= (uint8_t)(1U << (uint8_t)face);
  }
  return status;
}

static MachinePlanStatus
MachineState_AddPreparationStages(Rotate_Command_Type command,
                                  MachineMotionPlan *plan) {
  Rotate_Command_Type action_command = command;
  uint8_t current_stage = 0U;
  const int8_t command_units = MachineState_CommandIsPrime(command)
                                   ? -(int8_t)plan->command_multiplier
                                   : (int8_t)plan->command_multiplier;

  /* Prime 使用相同準備動作，只反轉最後的旋轉方向。 */
  switch (command) {
  case ROTATE_COMMAND_R_PRIME:
    action_command = ROTATE_COMMAND_R;
    break;
  case ROTATE_COMMAND_L_PRIME:
    action_command = ROTATE_COMMAND_L;
    break;
  case ROTATE_COMMAND_U_PRIME:
    action_command = ROTATE_COMMAND_U;
    break;
  case ROTATE_COMMAND_D_PRIME:
    action_command = ROTATE_COMMAND_D;
    break;
  case ROTATE_COMMAND_F_PRIME:
    action_command = ROTATE_COMMAND_F;
    break;
  case ROTATE_COMMAND_B_PRIME:
    action_command = ROTATE_COMMAND_B;
    break;
  case ROTATE_COMMAND_RW_PRIME:
    action_command = ROTATE_COMMAND_RW;
    break;
  case ROTATE_COMMAND_LW_PRIME:
    action_command = ROTATE_COMMAND_LW;
    break;
  case ROTATE_COMMAND_UW_PRIME:
    action_command = ROTATE_COMMAND_UW;
    break;
  case ROTATE_COMMAND_DW_PRIME:
    action_command = ROTATE_COMMAND_DW;
    break;
  case ROTATE_COMMAND_FW_PRIME:
    action_command = ROTATE_COMMAND_FW;
    break;
  case ROTATE_COMMAND_BW_PRIME:
    action_command = ROTATE_COMMAND_BW;
    break;
  default:
    break;
  }

  /* 同一 stage 的 move 同步執行；nextStage 後等待上一組完成再執行。 */
#define nextStage()                                                            \
  do {                                                                         \
    MachinePlanStatus add_stage_status =                                       \
        MachineState_PlanAddStage(plan, &current_stage);                       \
    if (add_stage_status != MACHINE_PLAN_OK) {                                 \
      return add_stage_status;                                                 \
    }                                                                          \
  } while (0)
#define move(face, rpm, acc, units)                                            \
  do {                                                                         \
    MachinePlanStatus move_status =                                            \
        FaceModule_Macro(plan, current_stage, (face), (rpm), (acc), (units));  \
    if (move_status != MACHINE_PLAN_OK) {                                      \
      return move_status;                                                      \
    }                                                                          \
  } while (0)
#define grip(face, rpm, acc, state)                                            \
  do {                                                                         \
    MachinePlanStatus grip_status = MachineState_PlanGrip(                     \
        plan, current_stage, (face), (rpm), (acc), (state));                   \
    if (grip_status != MACHINE_PLAN_OK) {                                      \
      return grip_status;                                                      \
    }                                                                          \
  } while (0)

  /* 自訂義開始 */
  switch (action_command) {
  case ROTATE_COMMAND_INIT:
    nextStage();
    if (plan->pending_big_states[MACHINE_FACE_LEFT] != BIG_MOTOR_VERTICAL) {
      move(MACHINE_FACE_LEFT, MAX_RPM, 255U, -1);
    }
    if (plan->pending_big_states[MACHINE_FACE_RIGHT] != BIG_MOTOR_VERTICAL) {
      move(MACHINE_FACE_RIGHT, MAX_RPM, 255U, -1);
    }
    if (plan->pending_big_states[MACHINE_FACE_DOWN] != BIG_MOTOR_VERTICAL) {
      move(MACHINE_FACE_DOWN, MAX_RPM, 255U, -1);
    }
    nextStage();
    grip(MACHINE_FACE_LEFT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
    grip(MACHINE_FACE_RIGHT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
    grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
    break;

  case ROTATE_COMMAND_R:
    nextStage();
    grip(MACHINE_FACE_UP, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
    /* 讓跟下面不衝突的另外兩側準備固定方塊。 */
    if (plan->pending_small_states[MACHINE_FACE_DOWN] !=
        MOTOR_ANGLE_NONE_SIDE) {
      if (plan->pending_big_states[MACHINE_FACE_DOWN] == BIG_MOTOR_VERTICAL) {
        if (plan->pending_big_states[MACHINE_FACE_FRONT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_FRONT, MAX_RPM, 255U, -1);
        }
        if (plan->pending_big_states[MACHINE_FACE_BACK] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);h
          move(MACHINE_FACE_BACK, MAX_RPM, 255U, -1);
        }
        nextStage();
        grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        nextStage();
        grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
      } else {
        if (plan->pending_big_states[MACHINE_FACE_LEFT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_LEFT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_LEFT, MAX_RPM, 255U, -1);
        }
        if (plan->pending_big_states[MACHINE_FACE_RIGHT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_RIGHT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_RIGHT, MAX_RPM, 255U, -1);
        }
        nextStage();
        grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        nextStage();
        grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
      }
    }
    nextStage();
    move(MACHINE_FACE_RIGHT, MAX_RPM, 255U, command_units);
    break;

  case ROTATE_COMMAND_L:
    nextStage();
    grip(MACHINE_FACE_UP, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
    /* 讓跟下面不衝突的另外兩側準備固定方塊。 */
    if (plan->pending_small_states[MACHINE_FACE_DOWN] !=
        MOTOR_ANGLE_NONE_SIDE) {
      if (plan->pending_big_states[MACHINE_FACE_DOWN] == BIG_MOTOR_VERTICAL) {
        if (plan->pending_big_states[MACHINE_FACE_FRONT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_FRONT, MAX_RPM, 255U, -1);
        }
        if (plan->pending_big_states[MACHINE_FACE_BACK] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_BACK, MAX_RPM, 255U, -1);
        }
        nextStage();
        grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        nextStage();
        grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
      } else {
        if (plan->pending_big_states[MACHINE_FACE_LEFT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_LEFT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_LEFT, MAX_RPM, 255U, -1);
        }
        if (plan->pending_big_states[MACHINE_FACE_RIGHT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_RIGHT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_RIGHT, MAX_RPM, 255U, -1);
        }
        nextStage();
        grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        nextStage();
        grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
      }
    }
    nextStage();
    move(MACHINE_FACE_LEFT, MAX_RPM, 255U, command_units);
    break;

  case ROTATE_COMMAND_U:
  case ROTATE_COMMAND_D:
  case ROTATE_COMMAND_F:
    nextStage();
    grip(MACHINE_FACE_UP, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
    /* 讓跟下面不衝突的另外兩側準備固定方塊。 */
    if (plan->pending_small_states[MACHINE_FACE_DOWN] !=
        MOTOR_ANGLE_NONE_SIDE) {
      if (plan->pending_big_states[MACHINE_FACE_DOWN] == BIG_MOTOR_VERTICAL) {
        if (plan->pending_big_states[MACHINE_FACE_FRONT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_FRONT, MAX_RPM, 255U, -1);
        }
        if (plan->pending_big_states[MACHINE_FACE_BACK] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_BACK, MAX_RPM, 255U, -1);
        }
        nextStage();
        grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        nextStage();
        grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
      } else {
        if (plan->pending_big_states[MACHINE_FACE_LEFT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_LEFT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_LEFT, MAX_RPM, 255U, -1);
        }
        if (plan->pending_big_states[MACHINE_FACE_RIGHT] != BIG_MOTOR_HORIZON) {
          grip(MACHINE_FACE_RIGHT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
          move(MACHINE_FACE_RIGHT, MAX_RPM, 255U, -1);
        }
        nextStage();
        grip(MACHINE_FACE_FRONT, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        grip(MACHINE_FACE_BACK, MAX_RPM, 255U, MOTOR_ANGLE_SHORT_SIDE);
        nextStage();
        grip(MACHINE_FACE_DOWN, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
      }
    }
    nextStage();
    grip(MACHINE_FACE_LEFT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
    grip(MACHINE_FACE_RIGHT, MAX_RPM, 255U, MOTOR_ANGLE_NONE_SIDE);
    move(MACHINE_FACE_FRONT, MAX_RPM, 255U, command_units);
    break;

  case ROTATE_COMMAND_B:
  case ROTATE_COMMAND_RW:
  case ROTATE_COMMAND_LW:
  case ROTATE_COMMAND_UW:
  case ROTATE_COMMAND_DW:
  case ROTATE_COMMAND_FW:
  case ROTATE_COMMAND_BW:
    break;

  default:
#undef move
#undef grip
#undef nextStage
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }

#undef move
#undef grip
#undef nextStage
  return MACHINE_PLAN_OK;
}

void MachineState_Init(void) {
  memset(&machine_state, 0, sizeof(machine_state));
  machine_state.Last_command = ROTATE_COMMAND_INIT;
  machine_state.left_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.right_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.up_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.down_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.front_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.back_module.small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  machine_state.left_module.big_motor_state = BIG_MOTOR_VERTICAL;
  machine_state.right_module.big_motor_state = BIG_MOTOR_VERTICAL;
  machine_state.up_module.big_motor_state = BIG_MOTOR_VERTICAL;
  machine_state.down_module.big_motor_state = BIG_MOTOR_VERTICAL;
  machine_state.front_module.big_motor_state = BIG_MOTOR_VERTICAL;
  machine_state.back_module.big_motor_state = BIG_MOTOR_VERTICAL;

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

MachinePlanStatus MachineState_BuildRotatePlan(const char *command_text,
                                               MachineMotionPlan *plan) {
  Rotate_Command_Type command;
  uint8_t command_multiplier;
  FaceModule *module;
  MachinePlanStatus status;

  if ((plan == NULL) || (!MachineState_ParseCommand(command_text, &command,
                                                    &command_multiplier))) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }

  memset(plan, 0, sizeof(*plan));
  plan->command = command;
  plan->command_multiplier = command_multiplier;
  plan->pending_big_states[MACHINE_FACE_LEFT] =
      machine_state.left_module.big_motor_state;
  plan->pending_big_states[MACHINE_FACE_RIGHT] =
      machine_state.right_module.big_motor_state;
  plan->pending_big_states[MACHINE_FACE_UP] =
      machine_state.up_module.big_motor_state;
  plan->pending_big_states[MACHINE_FACE_DOWN] =
      machine_state.down_module.big_motor_state;
  plan->pending_big_states[MACHINE_FACE_FRONT] =
      machine_state.front_module.big_motor_state;
  plan->pending_big_states[MACHINE_FACE_BACK] =
      machine_state.back_module.big_motor_state;
  plan->pending_small_states[MACHINE_FACE_LEFT] =
      machine_state.left_module.small_motor_state;
  plan->pending_small_states[MACHINE_FACE_RIGHT] =
      machine_state.right_module.small_motor_state;
  plan->pending_small_states[MACHINE_FACE_UP] =
      machine_state.up_module.small_motor_state;
  plan->pending_small_states[MACHINE_FACE_DOWN] =
      machine_state.down_module.small_motor_state;
  plan->pending_small_states[MACHINE_FACE_FRONT] =
      machine_state.front_module.small_motor_state;
  plan->pending_small_states[MACHINE_FACE_BACK] =
      machine_state.back_module.small_motor_state;
  if (command != ROTATE_COMMAND_INIT) {
    module = MachineState_GetFaceModule(MachineState_CommandFace(command));
    if (module == NULL) {
      return MACHINE_PLAN_INVALID_ARGUMENT;
    }
  }

  status = MachineState_AddPreparationStages(command, plan);
  if (status == MACHINE_PLAN_OK) {
    uint8_t read_index;
    uint8_t write_index = 0U;

    /* 狀態已符合的 move 是 no-op；移除因此產生的空 stage。 */
    for (read_index = 0U; read_index < plan->stage_count; read_index++) {
      if (plan->stages[read_index].motion_count == 0U) {
        continue;
      }
      if (write_index != read_index) {
        plan->stages[write_index] = plan->stages[read_index];
      }
      write_index++;
    }
    plan->stage_count = write_index;
  }
  return status;
}

void MachineState_CommitRotate(const MachineMotionPlan *plan) {
  Rotate_Command_Type command;
  uint8_t command_multiplier;
  uint8_t face;

  if (plan == NULL) {
    return;
  }
  command = plan->command;
  command_multiplier = plan->command_multiplier;
  if ((command_multiplier < 1U) || (command_multiplier > 2U)) {
    return;
  }
  for (face = 0U; face < 6U; face++) {
    if ((plan->pending_big_state_mask & (uint8_t)(1U << face)) != 0U) {
      FaceModule *pending_module =
          MachineState_GetFaceModule((MachineFace)face);
      if (pending_module != NULL) {
        pending_module->big_motor_state = plan->pending_big_states[face];
      }
    }
    if ((plan->pending_small_state_mask & (uint8_t)(1U << face)) != 0U) {
      FaceModule *pending_module =
          MachineState_GetFaceModule((MachineFace)face);
      if (pending_module != NULL) {
        pending_module->small_motor_state = plan->pending_small_states[face];
      }
    }
  }
  machine_state.Last_command = command;
  if (command == ROTATE_COMMAND_INIT) {
    /* INIT 若會改變 logical state，可在這裡加入對應的狀態更新。 */
    return;
  }

  FaceModule *module =
      MachineState_GetFaceModule(MachineState_CommandFace(command));
  int16_t delta;

  if (module == NULL) {
    return;
  }
  delta = (int16_t)((MachineState_CommandIsPrime(command) ? -90 : 90) *
                    (int16_t)command_multiplier);
  module->big_motor_angle_degrees =
      (int16_t)(module->big_motor_angle_degrees + delta);

  while (module->big_motor_angle_degrees >= 360) {
    module->big_motor_angle_degrees -= 360;
  }
  while (module->big_motor_angle_degrees < 0) {
    module->big_motor_angle_degrees += 360;
  }
}

void MachineState_CommitInitHoming(void) {
  FaceModule *modules[] = {
      &machine_state.left_module,  &machine_state.right_module,
      &machine_state.up_module,    &machine_state.down_module,
      &machine_state.front_module, &machine_state.back_module};
  uint8_t index;

  for (index = 0U; index < (sizeof(modules) / sizeof(modules[0])); index++) {
    modules[index]->big_motor_angle_degrees = 0;
    modules[index]->big_motor_state = BIG_MOTOR_VERTICAL;
    modules[index]->small_motor_state = MOTOR_ANGLE_NONE_SIDE;
  }
  machine_state.Last_command = ROTATE_COMMAND_INIT;
}

const MachineState *MachineState_Get(void) { return &machine_state; }

const FaceModule *MachineState_GetModule(MachineFace face) {
  return MachineState_GetFaceModule(face);
}
