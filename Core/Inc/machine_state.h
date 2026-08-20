#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MACHINE_STATE_MAX_STAGES      8U
#define MACHINE_STATE_MAX_SYNC_MOTORS 12U

typedef enum
{
  MOTOR_ANGLE_LONG_SIDE = 0,
  MOTOR_ANGLE_SHORT_SIDE,
  MOTOR_ANGLE_NONE_SIDE
} MotorAngle_State;

typedef enum
{
  ROTATE_COMMAND_R = 0,
  ROTATE_COMMAND_R_PRIME,
  ROTATE_COMMAND_L,
  ROTATE_COMMAND_L_PRIME,
  ROTATE_COMMAND_U,
  ROTATE_COMMAND_U_PRIME,
  ROTATE_COMMAND_D,
  ROTATE_COMMAND_D_PRIME,
  ROTATE_COMMAND_F,
  ROTATE_COMMAND_F_PRIME,
  ROTATE_COMMAND_B,
  ROTATE_COMMAND_B_PRIME,
  ROTATE_COMMAND_RW,
  ROTATE_COMMAND_RW_PRIME,
  ROTATE_COMMAND_LW,
  ROTATE_COMMAND_LW_PRIME,
  ROTATE_COMMAND_UW,
  ROTATE_COMMAND_UW_PRIME,
  ROTATE_COMMAND_DW,
  ROTATE_COMMAND_DW_PRIME,
  ROTATE_COMMAND_FW,
  ROTATE_COMMAND_FW_PRIME,
  ROTATE_COMMAND_BW,
  ROTATE_COMMAND_BW_PRIME,
  ROTATE_COMMAND_INIT
} Rotate_Command_Type;

typedef enum
{
  MACHINE_FACE_LEFT = 0,
  MACHINE_FACE_RIGHT,
  MACHINE_FACE_UP,
  MACHINE_FACE_DOWN,
  MACHINE_FACE_FRONT,
  MACHINE_FACE_BACK
} MachineFace;

typedef enum
{
  MACHINE_MOTOR_BIG = 0,
  MACHINE_MOTOR_SMALL
} MachineMotorRole;

typedef struct
{
  uint8_t bus;
  uint16_t big_motor_id;
  uint16_t small_motor_id;
  int16_t big_motor_angle_degrees;
  MotorAngle_State small_motor_state;
  uint8_t clockwise_axis_positive;
} FaceModule;

typedef struct
{
  FaceModule left_module;
  FaceModule right_module;
  FaceModule up_module;
  FaceModule down_module;
  FaceModule front_module;
  FaceModule back_module;
  Rotate_Command_Type Last_command;
} MachineState;

typedef struct
{
  uint8_t bus;
  uint16_t id;
  uint16_t speed_rpm;
  uint8_t acceleration;
  double angle_degrees;
} MachineMotorMotion;

typedef struct
{
  uint8_t motion_count;
  MachineMotorMotion motions[MACHINE_STATE_MAX_SYNC_MOTORS];
} MachineMotionStage;

typedef struct
{
  Rotate_Command_Type command;
  uint8_t command_multiplier;
  uint8_t stage_count;
  MachineMotionStage stages[MACHINE_STATE_MAX_STAGES];
} MachineMotionPlan;

typedef enum
{
  MACHINE_PLAN_OK = 0,
  MACHINE_PLAN_INVALID_ARGUMENT,
  MACHINE_PLAN_TOO_MANY_MOTORS,
  MACHINE_PLAN_TOO_MANY_STAGES
} MachinePlanStatus;

/** @brief 在 plan 尾端新增一組；同組動作會由 Synchronization mark 同步啟動。 */
MachinePlanStatus MachineState_PlanAddStage(MachineMotionPlan *plan,
                                             uint8_t *stage_index);

/** @brief 把一顆馬達加入指定 stage；不同 stage 會依序執行。 */
MachinePlanStatus MachineState_PlanAddMotion(MachineMotionPlan *plan,
                                              uint8_t stage_index,
                                              uint8_t bus, uint16_t id,
                                              uint16_t speed_rpm,
                                              uint8_t acceleration,
                                              double angle_degrees);

/** @brief 依 MachineState_ConfigureFace() 的設定加入指定 face 的大小馬達。 */
MachinePlanStatus MachineState_PlanAddFaceMotion(
  MachineMotionPlan *plan, uint8_t stage_index,
  MachineFace face, MachineMotorRole role, uint16_t speed_rpm,
  uint8_t acceleration, double angle_degrees);

/** @brief 初始化六個 face module 的狀態與馬達對應。 */
void MachineState_Init(void);

/**
  * @brief 設定一個 face module 的馬達對應與順時針方向。
  * @param face 要設定的 Rubik face。
  * @param bus 1-based Motor CAN bus。
  * @param big_motor_id 旋轉該面的主要馬達 ID。
  * @param small_motor_id 輔助馬達 ID；目前由自訂規劃邏輯使用。
  * @param clockwise_axis_positive 1 表示邏輯正角度對應正 relAxis；0 表示反向。
  */
void MachineState_ConfigureFace(MachineFace face, uint8_t bus,
                                uint16_t big_motor_id,
                                uint16_t small_motor_id,
                                uint8_t clockwise_axis_positive);

/**
  * @brief 解析 Rubik command，依目前 machine state 產生同步馬達動作清單。
  * @param fallback_bus face 尚未設定時使用的 bus。
  * @param fallback_id face 尚未設定時使用的主要馬達 ID。
  * @param command INIT、R、R2、R_、Rw、Rw2、Rw_ 等命令字串，不分大小寫。
  * @param plan 成功時接收完整動作清單。
  */
MachinePlanStatus MachineState_BuildRotatePlan(uint8_t fallback_bus,
                                               uint16_t fallback_id,
                                               const char *command,
                                               MachineMotionPlan *plan);

/** @brief 同步動作完成後，把 command 與倍數套用到 machine state。 */
void MachineState_CommitRotate(Rotate_Command_Type command,
                               uint8_t command_multiplier);

/** @brief 取得目前 machine state 的唯讀 view。 */
const MachineState *MachineState_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* MACHINE_STATE_H */
