#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MACHINE_STATE_MAX_STAGES 8U
#define MACHINE_STATE_MAX_SYNC_MOTORS 12U

typedef enum {
  MACHINE_FACE_LEFT = 0,
  MACHINE_FACE_RIGHT,
  MACHINE_FACE_UP,
  MACHINE_FACE_DOWN,
  MACHINE_FACE_FRONT,
  MACHINE_FACE_BACK
} MachineFace;

typedef enum { MACHINE_MOTOR_BIG = 0, MACHINE_MOTOR_SMALL } MachineMotorRole;

typedef struct {
  uint8_t bus;
  uint16_t big_motor_id;
  uint16_t small_motor_id;
  uint8_t clockwise_axis_positive;
} FaceModule;

typedef struct {
  FaceModule left_module;
  FaceModule right_module;
  FaceModule up_module;
  FaceModule down_module;
  FaceModule front_module;
  FaceModule back_module;
} MachineState;

typedef struct {
  uint8_t bus;
  uint16_t id;
  uint16_t speed_rpm;
  uint8_t acceleration;
  double angle_degrees;
} MachineMotorMotion;

typedef struct {
  uint8_t motion_count;
  MachineMotorMotion motions[MACHINE_STATE_MAX_SYNC_MOTORS];
} MachineMotionStage;

typedef struct {
  uint8_t stage_count;
  MachineMotionStage stages[MACHINE_STATE_MAX_STAGES];
} MachineMotionPlan;

typedef enum {
  MACHINE_PLAN_OK = 0,
  MACHINE_PLAN_INVALID_ARGUMENT,
  MACHINE_PLAN_TOO_MANY_MOTORS,
  MACHINE_PLAN_TOO_MANY_STAGES
} MachinePlanStatus;

/** @brief 在 motion plan 尾端新增一個同步 stage。 */
MachinePlanStatus MachineState_PlanAddStage(MachineMotionPlan *plan,
                                            uint8_t *stage_index);

/** @brief 在指定 stage 加入一顆馬達的 signed-angle 相對運動。 */
MachinePlanStatus MachineState_PlanAddMotion(MachineMotionPlan *plan,
                                             uint8_t stage_index, uint8_t bus,
                                             uint16_t id, uint16_t speed_rpm,
                                             uint8_t acceleration,
                                             double angle_degrees);

/** @brief 依 face 設定加入指定 big 或 small motor 的相對運動。 */
MachinePlanStatus
MachineState_PlanAddFaceMotion(MachineMotionPlan *plan, uint8_t stage_index,
                               MachineFace face, MachineMotorRole role,
                               uint16_t speed_rpm, uint8_t acceleration,
                               double angle_degrees);

/** @brief 初始化六面的 CAN bus、big motor ID、small motor ID 與正方向。 */
void MachineState_Init(void);

/** @brief 設定單一 Rubik face 對應的兩顆馬達。 */
void MachineState_ConfigureFace(MachineFace face, uint8_t bus,
                                uint16_t big_motor_id, uint16_t small_motor_id,
                                uint8_t clockwise_axis_positive);

/** @brief 驗證 face、motor role 與 Servo42D 可表示的 signed angle。 */
uint8_t MachineState_IsRotateRequestValid(MachineFace face,
                                          MachineMotorRole role,
                                          double angle_degrees);

/**
 * @brief 建立 ROTATE plan；SMALL 單獨轉，BIG 則依舊 macro 比例同步帶動 small。
 * @note BIG：small=30 RPM/acc 254/angle，big=60 RPM/acc 255/angle*2。
 */
MachinePlanStatus MachineState_BuildRotatePlan(MachineFace face,
                                               MachineMotorRole role,
                                               double angle_degrees,
                                               MachineMotionPlan *plan);

/** @brief 建立 INIT homing 後自動執行的舊 ROTATE INIT ready pose。 */
MachinePlanStatus MachineState_BuildInitReadyPlan(MachineMotionPlan *plan);

/** @brief 取得唯讀 machine mapping。 */
const MachineState *MachineState_Get(void);
const FaceModule *MachineState_GetModule(MachineFace face);

#ifdef __cplusplus
}
#endif

#endif /* MACHINE_STATE_H */
