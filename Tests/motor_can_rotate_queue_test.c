/*
 * White-box host test：直接納入 motor_can.c，讓測試能驅動 static FIFO 與完成
 * transition；未觸及的 STM32/HAL 路徑由 --gc-sections 移除。
 */
#include "motor_can.h"
#include "main.h"

/* Host test 沒有 ARM interrupt registers；被測 FIFO 本身只在主迴圈使用。 */
#define __get_PRIMASK() (0U)
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

#include "../Core/Src/motor_can.c"

#include <stdio.h>

static uint8_t test_ems_active;
static uint32_t test_tick;

uint8_t EMS_IsStopActive(void) { return test_ems_active; }

uint8_t EMS_AreCommandsBlocked(void) { return test_ems_active; }

uint32_t HAL_GetTick(void) { return test_tick++; }

static int CheckActiveSmall(uint8_t bus, uint16_t id, double angle) {
  const MachineMotorMotion *motion;

  if ((motor_context.operation != MOTOR_CAN_OPERATION_ROTATE) ||
      (motor_context.rotate_plan.stage_count != 1U) ||
      (motor_context.rotate_plan.stages[0].motion_count != 1U)) {
    puts("expected one active SMALL ROTATE motion");
    return 1;
  }
  motion = &motor_context.rotate_plan.stages[0].motions[0];
  if ((motion->bus != bus) || (motion->id != id) ||
      (motion->speed_rpm != 60U) || (motion->acceleration != 255U) ||
      (motion->angle_degrees != angle)) {
    printf("active mismatch: bus=%u id=%u angle=%.3f\n", motion->bus,
           motion->id, motion->angle_degrees);
    return 1;
  }
  return 0;
}

static int CheckActiveBig(uint8_t bus, uint16_t big_id, uint16_t small_id,
                          double angle) {
  const MachineMotionStage *stage;
  const MachineMotorMotion *small;
  const MachineMotorMotion *big;

  if ((motor_context.operation != MOTOR_CAN_OPERATION_ROTATE) ||
      (motor_context.rotate_plan.stage_count != 1U) ||
      (motor_context.rotate_plan.stages[0].motion_count != 2U)) {
    puts("expected synchronized small and big ROTATE motions");
    return 1;
  }
  stage = &motor_context.rotate_plan.stages[0];
  small = &stage->motions[0];
  big = &stage->motions[1];
  if ((small->bus != bus) || (small->id != small_id) ||
      (small->speed_rpm != 30U) || (small->acceleration != 254U) ||
      (small->angle_degrees != angle) || (big->bus != bus) ||
      (big->id != big_id) || (big->speed_rpm != 60U) ||
      (big->acceleration != 255U) ||
      (big->angle_degrees != angle * 2.0)) {
    puts("active BIG synchronized pair mismatch");
    return 1;
  }
  return 0;
}

static void ResetTestState(void) {
  MachineState_Init();
  MotorCAN_ResetOperation();
  MotorCAN_ClearRotateCommandQueue();
  motor_can_ready = 1U;
  motor_can_initialized = 1U;
  motor_bus_online[0] = 1U;
  motor_bus_online[1] = 1U;
  motor_event_head = 0U;
  motor_event_tail = 0U;
  test_ems_active = 0U;
  test_tick = 0U;
}

static int CheckRotateCompletionEvent(uint8_t expected_pending) {
  MotorCAN_Event event = {0};

  if ((!MotorCAN_GetEvent(&event)) ||
      (event.type != MOTOR_CAN_EVENT_ROTATE_FINISHED) ||
      (event.pending_count != expected_pending)) {
    printf("completion event pending mismatch: expected=%u actual=%u\n",
           expected_pending, event.pending_count);
    return 1;
  }
  return 0;
}

static int CheckFifoOrder(void) {
  uint8_t pending;

  if ((MotorCAN_QueueRotate(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 90.0,
                             &pending) != MOTOR_CAN_STATUS_OK) ||
      (pending != 0U) || CheckActiveBig(2U, 4U, 5U, 90.0)) {
    puts("first ROTATE did not start immediately");
    return 1;
  }
  if ((MotorCAN_QueueRotate(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 90.0,
                            &pending) != MOTOR_CAN_STATUS_OK) ||
      (pending != 1U) ||
      (MotorCAN_QueueRotate(MACHINE_FACE_LEFT, MACHINE_MOTOR_SMALL, -45.5,
                            &pending) != MOTOR_CAN_STATUS_OK) ||
      (pending != 2U)) {
    puts("ROTATE requests were not appended to FIFO");
    return 1;
  }

  MotorCAN_CompleteRotate();
  if ((MotorCAN_GetRotateQueueDepth() != 1U) ||
      CheckActiveBig(2U, 4U, 5U, 90.0) || CheckRotateCompletionEvent(2U)) {
    puts("second ROTATE did not start after first completion");
    return 1;
  }
  MotorCAN_CompleteRotate();
  if ((MotorCAN_GetRotateQueueDepth() != 0U) ||
      CheckActiveSmall(1U, 9U, -45.5) || CheckRotateCompletionEvent(1U)) {
    puts("third ROTATE did not preserve FIFO order");
    return 1;
  }
  MotorCAN_CompleteRotate();
  if ((motor_context.operation != MOTOR_CAN_OPERATION_NONE) ||
      CheckRotateCompletionEvent(0U)) {
    puts("ROTATE queue did not return to idle");
    return 1;
  }
  return 0;
}

static int CheckDelayedRotateReplyIsolation(void) {
  MotorCAN_RxFrame frame = {0};

  frame.data[0] = 0xF4U;
  motor_context.state = MOTOR_STATE_ROTATE_WAIT_RUN;
  motor_context.rotate_status_pending_mask = 1U;
  if (MotorCAN_TakeExpectedRotateStatusReply(&frame, 0U) ||
      (motor_context.rotate_status_pending_mask != 1U)) {
    puts("delayed F4 completion was accepted as current status");
    return 1;
  }

  frame.data[0] = 0xF1U;
  motor_context.state = MOTOR_STATE_ROTATE_QUEUE_MOTIONS;
  if (MotorCAN_TakeExpectedRotateStatusReply(&frame, 0U)) {
    puts("F1 reply was accepted before the current motion started");
    return 1;
  }
  motor_context.state = MOTOR_STATE_ROTATE_WAIT_RUN;
  motor_context.rotate_status_pending_mask = 0U;
  if (MotorCAN_TakeExpectedRotateStatusReply(&frame, 0U)) {
    puts("unsolicited F1 reply was accepted");
    return 1;
  }
  motor_context.rotate_status_pending_mask = 1U;
  if ((!MotorCAN_TakeExpectedRotateStatusReply(&frame, 0U)) ||
      (motor_context.rotate_status_pending_mask != 0U) ||
      MotorCAN_TakeExpectedRotateStatusReply(&frame, 0U)) {
    puts("fresh F1 reply was not consumed exactly once");
    return 1;
  }
  return 0;
}

static int CheckCapacity(void) {
  uint8_t index;
  uint8_t pending;

  if (MOTOR_CAN_ROTATE_COMMAND_QUEUE_SIZE != 64U) {
    puts("ROTATE FIFO capacity is not 64");
    return 1;
  }
  if (MotorCAN_QueueRotate(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 1.0,
                           &pending) != MOTOR_CAN_STATUS_OK) {
    return 1;
  }
  for (index = 0U; index < MOTOR_CAN_ROTATE_COMMAND_QUEUE_SIZE; index++) {
    if ((MotorCAN_QueueRotate(MACHINE_FACE_LEFT, MACHINE_MOTOR_SMALL,
                              (double)index + 1.0,
                              &pending) != MOTOR_CAN_STATUS_OK) ||
        (pending != (uint8_t)(index + 1U))) {
      puts("FIFO filled before its documented capacity");
      return 1;
    }
  }
  if (MotorCAN_QueueRotate(MACHINE_FACE_UP, MACHINE_MOTOR_BIG, 1.0, NULL) !=
      MOTOR_CAN_STATUS_QUEUE_FULL) {
    puts("FIFO did not report QUEUE_FULL");
    return 1;
  }
  return 0;
}

static int CheckInitHandoff(void) {
  uint8_t pending;

  MotorCAN_ResetOperation();
  MotorCAN_ClearRotateCommandQueue();
  motor_can_initialized = 0U;
  motor_context.operation = MOTOR_CAN_OPERATION_INIT;
  if ((MotorCAN_QueueRotate(MACHINE_FACE_DOWN, MACHINE_MOTOR_SMALL, -30.0,
                            &pending) != MOTOR_CAN_STATUS_OK) ||
      (pending != 1U)) {
    puts("ROTATE was not accepted while INIT was running");
    return 1;
  }

  MotorCAN_CompleteInit();
  if ((!motor_can_initialized) || (MotorCAN_GetRotateQueueDepth() != 0U) ||
      CheckActiveSmall(2U, 7U, -30.0)) {
    puts("queued ROTATE did not start after INIT completion");
    return 1;
  }
  return 0;
}

static int CheckRejections(void) {
  uint8_t pending = 99U;

  MotorCAN_ResetOperation();
  MotorCAN_ClearRotateCommandQueue();
  motor_can_initialized = 1U;
  test_ems_active = 1U;
  if ((MotorCAN_QueueRotate(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 90.0,
                            &pending) != MOTOR_CAN_STATUS_EMS_ACTIVE) ||
      (MotorCAN_GetRotateQueueDepth() != 0U)) {
    puts("EMS-active ROTATE was not rejected");
    return 1;
  }
  test_ems_active = 0U;
  if (MotorCAN_QueueRotate(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 0.0,
                           &pending) != MOTOR_CAN_STATUS_INVALID_ARGUMENT) {
    puts("zero-angle ROTATE was not rejected");
    return 1;
  }
  motor_context.operation = MOTOR_CAN_OPERATION_HOME;
  if (MotorCAN_QueueRotate(MACHINE_FACE_RIGHT, MACHINE_MOTOR_BIG, 90.0,
                           &pending) != MOTOR_CAN_STATUS_BUSY) {
    puts("ROTATE did not report BUSY during another operation");
    return 1;
  }
  return 0;
}

int main(void) {
  ResetTestState();
  if (CheckFifoOrder()) {
    return 1;
  }
  ResetTestState();
  if (CheckCapacity()) {
    return 1;
  }
  ResetTestState();
  if (CheckInitHandoff() || CheckRejections()) {
    return 1;
  }
  ResetTestState();
  if (CheckDelayedRotateReplyIsolation()) {
    return 1;
  }

  puts("motor_can rotate FIFO: OK");
  return 0;
}
