#include "can_device.h"

#include "bsp_can.h"
#include "can.h"
#include "chassis_control.h"
#include "imu_can.h"
#include "imu_can_task.h"
#include "main.h"
#include "shoot_control.h"

/* CAN1 过滤器：接收所有帧，CAN1 占用 Filter Bank 0-13。 */
static const CAN_FilterTypeDef can1_filter = {
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO1,
    .FilterActivation = ENABLE,
    .FilterBank = 0,
    .SlaveStartFilterBank = 14};

/* CAN2 过滤器：接收所有帧，CAN2 仅能使用 Filter Bank 14-27。 */
static const CAN_FilterTypeDef can2_filter = {
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0,
    .FilterActivation = ENABLE,
    .FilterBank = 14,
    .SlaveStartFilterBank = 14};

/* CAN2：4 台底盘 M3508 与 DM IMU；CAN1：3 台发射机构电机。 */
static STM32CAN_t can2_instance;
static STM32CAN_t can1_instance;
/* IMU 对象由本模块持有，任务通过 getter 函数访问。 */
static IMUCAN_t imu_can_instance;

/**
 * @brief 统一检查初始化步骤的返回码。
 * @param status 任一步骤的返回码；非 OK 即进入 Error_Handler()。
 */
static void can_device_check(err_t status) {
  if (status != OK) {
    Error_Handler();
  }
}

/**
 * @brief 完成 CAN 对象绑定、过滤器配置、设备注册与总线启动。
 *
 * 须在 RTOS 调度器启动前调用一次。先绑定 HAL 句柄，因为
 * chassis_control_init() 通过 BSP 对象表反查 CAN2，必须在 STM32CAN_Init()
 * 之后才能成功；总线在全部设备注册完成后才允许启动。
 */
void can_device_Init(void) {
  can_device_check(STM32CAN_Init(&can2_instance, &hcan2));
  can_device_check(STM32CAN_Init(&can1_instance, &hcan1));
  can_device_check(STM32CAN_ConfigFilter(&can1_instance, &can1_filter));
  can_device_check(STM32CAN_ConfigFilter(&can2_instance, &can2_filter));

  can_device_check(CAN_RegisterAllDevices());
  can_device_check(CAN_StartAll());
}

err_t CAN_RegisterAllDevices(void) {
  err_t status;

  /* CAN2：IMU RX 回调 + 底盘总线回调 + 4 台 M3508。 */
  status = imu_can_init(&imu_can_instance, &can2_instance, 0x01u, 0x11u);
  if (status != OK) {
    return status;
  }
  status = chassis_control_init();
  if (status != OK) {
    return status;
  }
  /* CAN1：发射机构总线回调 + 2 台 M3508 + 1 台 M2006。 */
  status = shoot_control_init();
  if (status != OK) {
    return status;
  }
  return OK;
}

/**
 * @brief 获取 IMU CAN 设备对象指针。
 *
 * 供 IMU 任务访问已注册的设备对象，无需暴露全局变量。
 * @return IMU 对象指针，若未初始化可能返回未就绪的对象。
 */
IMUCAN_t* can_device_get_imu(void) { return &imu_can_instance; }

/**
 * @brief 启动全部 CAN 总线。
 *
 * 所有设备注册完成后才允许调用；调用后禁止新增 RX 订阅者。
 */
err_t CAN_StartAll(void) {
  err_t status = STM32CAN_Start(&can1_instance);
  if (status != OK) {
    return status;
  }
  return STM32CAN_Start(&can2_instance);
}
