#include "machine_state.h"

#include <stddef.h>
#include <string.h>

#define MACHINE_ROTATE_SPEED_RPM 1000U
#define MACHINE_ROTATE_ACCELERATION 0U
#define MACHINE_ROTATE_BIG_ANGLE_RATIO 2.0
#define MACHINE_INIT_READY_ANGLE_DEGREES 90.0
#define MACHINE_ENCODER_COUNTS_PER_REVOLUTION 16384.0
#define MACHINE_MAX_RELATIVE_AXIS_COUNTS 8388607.0

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

static uint8_t MachineState_AngleIsValid(double angle_degrees) {
  const double minimum_angle = 180.0 / MACHINE_ENCODER_COUNTS_PER_REVOLUTION;
  const double maximum_angle = MACHINE_MAX_RELATIVE_AXIS_COUNTS * 360.0 /
                               MACHINE_ENCODER_COUNTS_PER_REVOLUTION;

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

void MachineState_Init(void) {
  memset(&machine_state, 0, sizeof(machine_state));

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
      (big_motor_id > 0x7FFU) || (small_motor_id < 1U) ||
      (small_motor_id > 0x7FFU) || (clockwise_axis_positive > 1U)) {
    return;
  }
  module->bus = bus;
  module->big_motor_id = big_motor_id;
  module->small_motor_id = small_motor_id;
  module->clockwise_axis_positive = clockwise_axis_positive;
}

uint8_t MachineState_IsRotateRequestValid(MachineFace face,
                                          MachineMotorRole role,
                                          double angle_degrees) {
  if ((MachineState_GetFaceModule(face) == NULL) ||
      (role > MACHINE_MOTOR_SMALL) ||
      (!MachineState_AngleIsValid(angle_degrees))) {
    return 0U;
  }
  if ((role == MACHINE_MOTOR_BIG) &&
      (!MachineState_AngleIsValid(angle_degrees *
                                  MACHINE_ROTATE_BIG_ANGLE_RATIO))) {
    return 0U;
  }
  return 1U;
}

MachinePlanStatus MachineState_BuildRotatePlan(MachineFace face,
                                               MachineMotorRole role,
                                               double angle_degrees,
                                               MachineMotionPlan *plan) {
  uint8_t stage_index;
  uint8_t small_acc;
  uint16_t small_rpm;
  MachinePlanStatus status;

  if ((plan == NULL) ||
      (!MachineState_IsRotateRequestValid(face, role, angle_degrees))) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }

  memset(plan, 0, sizeof(*plan));
  status = MachineState_PlanAddStage(plan, &stage_index);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }
  if (role == MACHINE_MOTOR_SMALL) {
    return MachineState_PlanAddFaceMotion(
        plan, stage_index, face, MACHINE_MOTOR_SMALL, MACHINE_ROTATE_SPEED_RPM,
        MACHINE_ROTATE_ACCELERATION, angle_degrees);
  }

  /*
   * 沿用舊 FaceModule_Macro 的 2:1 機構比例：small 走輸入角度，big
   * 走兩倍角度；small 速度減半，使兩顆馬達的理想運動時間相同。
   */
  small_acc = (uint8_t)(256U - (256U - MACHINE_ROTATE_ACCELERATION) / 0.5);
  small_rpm = (uint16_t)(MACHINE_ROTATE_SPEED_RPM * 0.5);
  status = MachineState_PlanAddFaceMotion(plan, stage_index, face,
                                          MACHINE_MOTOR_SMALL, small_rpm,
                                          small_acc, angle_degrees);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }
  return MachineState_PlanAddFaceMotion(
      plan, stage_index, face, MACHINE_MOTOR_BIG, MACHINE_ROTATE_SPEED_RPM,
      MACHINE_ROTATE_ACCELERATION,
      angle_degrees * MACHINE_ROTATE_BIG_ANGLE_RATIO);
}

MachinePlanStatus MachineState_BuildInitReadyPlan(MachineMotionPlan *plan) {
  static const MachineFace ready_faces[] = {MACHINE_FACE_LEFT,
                                            MACHINE_FACE_RIGHT};
  uint8_t stage_index;
  uint8_t index;
  MachinePlanStatus status;

  if (plan == NULL) {
    return MACHINE_PLAN_INVALID_ARGUMENT;
  }

  memset(plan, 0, sizeof(*plan));
  status = MachineState_PlanAddStage(plan, &stage_index);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }
  status = MachineState_PlanAddFaceMotion(
      plan, stage_index, MACHINE_FACE_DOWN, MACHINE_MOTOR_SMALL,
      MACHINE_ROTATE_SPEED_RPM, MACHINE_ROTATE_ACCELERATION,
      MACHINE_INIT_READY_ANGLE_DEGREES);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }
  status = MachineState_PlanAddStage(plan, &stage_index);
  if (status != MACHINE_PLAN_OK) {
    return status;
  }
  for (index = 0U;
       index < (uint8_t)(sizeof(ready_faces) / sizeof(ready_faces[0]));
       index++) {
    status = MachineState_PlanAddFaceMotion(
        plan, stage_index, ready_faces[index], MACHINE_MOTOR_SMALL,
        MACHINE_ROTATE_SPEED_RPM, MACHINE_ROTATE_ACCELERATION,
        MACHINE_INIT_READY_ANGLE_DEGREES);
    if (status != MACHINE_PLAN_OK) {
      return status;
    }
  }
  return MACHINE_PLAN_OK;
}

const MachineState *MachineState_Get(void) { return &machine_state; }

const FaceModule *MachineState_GetModule(MachineFace face) {
  return MachineState_GetFaceModule(face);
}
