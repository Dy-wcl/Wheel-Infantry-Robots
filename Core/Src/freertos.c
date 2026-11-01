/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for dr16 */
osThreadId_t dr16Handle;
const osThreadAttr_t dr16_attributes = {
    .name = "dr16",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for vt13 */
osThreadId_t vt13Handle;
const osThreadAttr_t vt13_attributes = {
    .name = "vt13",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for vofa */
osThreadId_t vofaHandle;
const osThreadAttr_t vofa_attributes = {
    .name = "vofa",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for imu_can */
osThreadId_t imu_canHandle;
const osThreadAttr_t imu_can_attributes = {
    .name = "imu_can",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for lx824 */
osThreadId_t lx824Handle;
const osThreadAttr_t lx824_attributes = {
    .name = "lx824",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for i6X */
osThreadId_t i6XHandle;
const osThreadAttr_t i6X_attributes = {
    .name = "i6X",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for imu_485 */
osThreadId_t imu_485Handle;
const osThreadAttr_t imu_485_attributes = {
    .name = "imu_485",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for gimbal */
osThreadId_t gimbalHandle;
const osThreadAttr_t gimbal_attributes = {
    .name = "gimbal",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for chassis */
osThreadId_t chassisHandle;
const osThreadAttr_t chassis_attributes = {
    .name = "chassis",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for game */
osThreadId_t gameHandle;
const osThreadAttr_t game_attributes = {
    .name = "game",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for shoot */
osThreadId_t shootHandle;
const osThreadAttr_t shoot_attributes = {
    .name = "shoot",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow7,
};
/* Definitions for super_power */
osThreadId_t super_powerHandle;
const osThreadAttr_t super_power_attributes = {
    .name = "super_power",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for version */
osThreadId_t versionHandle;
const osThreadAttr_t version_attributes = {
    .name = "version",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void dr16_task(void *argument);
void vt13_task(void *argument);
void vofa_task(void *argument);
void imu_can_task(void *argument);
void lx824_task(void *argument);
void i6X_task(void *argument);
void imu_485_task(void *argument);
void gimbal_task(void *argument);
void chassis_task(void *argument);
void game_task(void *argument);
void shoot_task(void *argument);
void super_power_task(void *argument);
void version_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {}

__weak unsigned long getRunTimeCounterValue(void) { return 0; }
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created. It is also called by various parts of the
  demo application. If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of dr16 */
  dr16Handle = osThreadNew(dr16_task, NULL, &dr16_attributes);

  /* creation of vt13 */
  vt13Handle = osThreadNew(vt13_task, NULL, &vt13_attributes);

  /* creation of vofa */
  vofaHandle = osThreadNew(vofa_task, NULL, &vofa_attributes);

  /* creation of imu_can */
  imu_canHandle = osThreadNew(imu_can_task, NULL, &imu_can_attributes);

  /* creation of lx824 */
  lx824Handle = osThreadNew(lx824_task, NULL, &lx824_attributes);

  /* creation of i6X */
  i6XHandle = osThreadNew(i6X_task, NULL, &i6X_attributes);

  /* creation of imu_485 */
  imu_485Handle = osThreadNew(imu_485_task, NULL, &imu_485_attributes);

  /* creation of gimbal */
  gimbalHandle = osThreadNew(gimbal_task, NULL, &gimbal_attributes);

  /* creation of chassis */
  chassisHandle = osThreadNew(chassis_task, NULL, &chassis_attributes);

  /* creation of game */
  gameHandle = osThreadNew(game_task, NULL, &game_attributes);

  /* creation of shoot */
  shootHandle = osThreadNew(shoot_task, NULL, &shoot_attributes);

  /* creation of super_power */
  super_powerHandle =
      osThreadNew(super_power_task, NULL, &super_power_attributes);

  /* creation of version */
  versionHandle = osThreadNew(version_task, NULL, &version_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    /*
     * The watchdog is started by MX_IWDG_Init() before the scheduler starts.
     * Keep the refresh in a scheduler task so a stalled scheduler or a task
     * that prevents normal scheduling can still cause an automatic reset.
     */
    (void)HAL_IWDG_Refresh(&hiwdg);
    osDelay(100u);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_dr16_task */
/**
 * @brief Function implementing the dr16 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_dr16_task */
__weak void dr16_task(void *argument) {
  /* USER CODE BEGIN dr16_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END dr16_task */
}

/* USER CODE BEGIN Header_vt13_task */
/**
 * @brief Function implementing the vt13 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vt13_task */
__weak void vt13_task(void *argument) {
  /* USER CODE BEGIN vt13_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vt13_task */
}

/* USER CODE BEGIN Header_vofa_task */
/**
 * @brief Function implementing the vofa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vofa_task */
__weak void vofa_task(void *argument) {
  /* USER CODE BEGIN vofa_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END vofa_task */
}

/* USER CODE BEGIN Header_imu_can_task */
/**
 * @brief Function implementing the imu_can thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_imu_can_task */
__weak void imu_can_task(void *argument) {
  /* USER CODE BEGIN imu_can_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END imu_can_task */
}

/* USER CODE BEGIN Header_lx824_task */
/**
 * @brief Function implementing the lx824 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_lx824_task */
__weak void lx824_task(void *argument) {
  /* USER CODE BEGIN lx824_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END lx824_task */
}

/* USER CODE BEGIN Header_i6X_task */
/**
 * @brief Function implementing the i6X thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_i6X_task */
__weak void i6X_task(void *argument) {
  /* USER CODE BEGIN i6X_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END i6X_task */
}

/* USER CODE BEGIN Header_imu_485_task */
/**
 * @brief Function implementing the imu_485 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_imu_485_task */
__weak void imu_485_task(void *argument) {
  /* USER CODE BEGIN imu_485_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END imu_485_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
 * @brief Function implementing the gimbal thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void *argument) {
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
 * @brief Function implementing the chassis thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument) {
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_game_task */
/**
 * @brief Function implementing the game thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_game_task */
__weak void game_task(void *argument) {
  /* USER CODE BEGIN game_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END game_task */
}

/* USER CODE BEGIN Header_shoot_task */
/**
 * @brief Function implementing the shoot thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_shoot_task */
__weak void shoot_task(void *argument) {
  /* USER CODE BEGIN shoot_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END shoot_task */
}

/* USER CODE BEGIN Header_super_power_task */
/**
 * @brief Function implementing the super_power thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_super_power_task */
__weak void super_power_task(void *argument) {
  /* USER CODE BEGIN super_power_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END super_power_task */
}

/* USER CODE BEGIN Header_version_task */
/**
 * @brief Function implementing the version thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_version_task */
__weak void version_task(void *argument) {
  /* USER CODE BEGIN version_task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END version_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
