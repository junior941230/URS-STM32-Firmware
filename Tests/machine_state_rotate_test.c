#include "machine_state.h"

#include <math.h>
#include <stdio.h>

typedef struct {
  MachineFace face;
  uint8_t bus;
  uint16_t big_id;
  uint16_t small_id;
} ExpectedFace;

static int CheckMotion(const MachineMotorMotion *motion, uint8_t bus,
                       uint16_t id, uint16_t speed_rpm, uint8_t acceleration,
                       double angle) {
  if ((motion->bus != bus) || (motion->id != id) ||
      (motion->speed_rpm != speed_rpm) ||
      (motion->acceleration != acceleration) ||
      (fabs(motion->angle_degrees - angle) > 0.000001)) {
    printf("motion mismatch: bus=%u id=%u rpm=%u acc=%u angle=%.6f\n",
           motion->bus, motion->id, motion->speed_rpm, motion->acceleration,
           motion->angle_degrees);
    return 1;
  }
  return 0;
}

static int CheckFaceAndRole(const ExpectedFace *expected,
                             MachineMotorRole role, double angle) {
  MachineMotionPlan plan;

  if (MachineState_BuildRotatePlan(expected->face, role, angle, &plan) !=
      MACHINE_PLAN_OK) {
    puts("valid rotate request rejected");
    return 1;
  }
  if (plan.stage_count != 1U) {
    puts("rotate plan is not exactly one synchronized stage");
    return 1;
  }
  if (role == MACHINE_MOTOR_SMALL) {
    if (plan.stages[0].motion_count != 1U) {
      puts("SMALL rotate plan does not contain exactly one motor");
      return 1;
    }
    return CheckMotion(&plan.stages[0].motions[0], expected->bus,
                       expected->small_id, 2000U, 0U, angle);
  }
  if (plan.stages[0].motion_count != 2U) {
    puts("BIG rotate plan does not contain synchronized small and big motors");
    return 1;
  }
  return CheckMotion(&plan.stages[0].motions[0], expected->bus,
                     expected->small_id, 1000U, 0U, angle) ||
         CheckMotion(&plan.stages[0].motions[1], expected->bus,
                     expected->big_id, 2000U, 0U, angle * 2.0);
}

static int CheckInitReadyPose(void) {
  MachineMotionPlan plan;

  if (MachineState_BuildInitReadyPlan(&plan) != MACHINE_PLAN_OK) {
    puts("INIT ready pose rejected");
    return 1;
  }
  if ((plan.stage_count != 2U) || (plan.stages[0].motion_count != 1U) ||
      (plan.stages[1].motion_count != 2U)) {
    puts("INIT ready pose is not DOWN then synchronized LEFT/RIGHT");
    return 1;
  }
  if (CheckMotion(&plan.stages[0].motions[0], 2U, 7U, 2000U, 0U, 90.0) ||
      CheckMotion(&plan.stages[1].motions[0], 1U, 9U, 2000U, 0U, 90.0) ||
      CheckMotion(&plan.stages[1].motions[1], 2U, 5U, 2000U, 0U, 90.0)) {
    return 1;
  }
  return 0;
}

static int CheckInvalidRequests(void) {
  MachineMotionPlan plan;
  const double maximum = 8388607.0 * 360.0 / 16384.0;
  const double too_large = 8388607.0 * 360.0 / 16384.0 + 1.0;
  const double big_too_large = maximum / 2.0 + 1.0;

  if ((MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 0.0,
                                    &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG,
                                    0.001, &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG,
                                    too_large, &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG,
                                    big_too_large, &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan((MachineFace)6, MACHINE_MOTOR_BIG, 90.0,
                                    &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, (MachineMotorRole)2,
                                    90.0, &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG,
                                    NAN, &plan) == MACHINE_PLAN_OK) ||
      (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG,
                                    90.0, NULL) == MACHINE_PLAN_OK)) {
    puts("invalid rotate request accepted");
    return 1;
  }
  if (MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_SMALL,
                                   big_too_large, &plan) != MACHINE_PLAN_OK) {
    puts("SMALL request incorrectly used BIG doubled-angle limit");
    return 1;
  }
  return 0;
}

static int CheckConfiguredDirection(void) {
  MachineMotionPlan plan;

  MachineState_ConfigureFace(MACHINE_FACE_RIGHT, 2U, 4U, 5U, 0U);
  if ((MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_SMALL,
                                     12.5, &plan) != MACHINE_PLAN_OK) ||
      CheckMotion(&plan.stages[0].motions[0], 2U, 5U, 2000U, 0U, -12.5)) {
    puts("configured face direction was not applied");
    return 1;
  }
  if ((MachineState_BuildRotatePlan(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG,
                                    12.5, &plan) != MACHINE_PLAN_OK) ||
      (plan.stages[0].motion_count != 2U) ||
      CheckMotion(&plan.stages[0].motions[0], 2U, 5U, 1000U, 0U, -12.5) ||
      CheckMotion(&plan.stages[0].motions[1], 2U, 4U, 2000U, 0U, -25.0)) {
    puts("configured face direction was not applied to BIG synchronized pair");
    return 1;
  }
  MachineState_ConfigureFace(MACHINE_FACE_RIGHT, 2U, 4U, 5U, 1U);
  return 0;
}

int main(void) {
  static const ExpectedFace expected_faces[] = {
      {MACHINE_FACE_LEFT, 1U, 8U, 9U},
      {MACHINE_FACE_RIGHT, 2U, 4U, 5U},
      {MACHINE_FACE_UP, 1U, 2U, 3U},
      {MACHINE_FACE_DOWN, 2U, 6U, 7U},
      {MACHINE_FACE_FRONT, 1U, 12U, 13U},
      {MACHINE_FACE_BACK, 2U, 10U, 11U},
  };
  size_t index;

  MachineState_Init();
  for (index = 0U; index <
                        sizeof(expected_faces) / sizeof(expected_faces[0]);
       index++) {
    if (CheckFaceAndRole(&expected_faces[index], MACHINE_MOTOR_BIG, 90.25) ||
        CheckFaceAndRole(&expected_faces[index], MACHINE_MOTOR_SMALL, -45.5)) {
      return 1;
    }
  }
  if (CheckInitReadyPose() || CheckInvalidRequests() ||
      CheckConfiguredDirection()) {
    return 1;
  }

  puts("machine_state rotate plans: OK");
  return 0;
}
