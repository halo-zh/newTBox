/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MC20.h"
#include "can.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PubTopic "/sys/a1fkV4RfRSP/AQ_SCOOTER_TEST/thing/event/property/post"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t gprsStatus;
extern uint8_t gnssDataAval;

uint32_t eventData = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osTimerId ioControlTimerHandle;
osTimerId cloudTimerHandle;
osTimerId gpsTimerHandle;
osTimerId errorTimerHandle;
osTimerId readBatInfoTimerHandle;
osMessageQId cloudQueueHandle;
enum CLOUD_EVENT_ID
{
  CLOUD_NULL = 0,
  CLOUD_UPLOAD_INFO,
  CLOUD_UPLOAD_GPS,
  CLOUD_UPLOAD_BAT_ERR,
  CLOUD_UPLOAD_INVERTER_ERR,
  GPS_INIT,
  READ_GPS_DATA,

};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId g_mqtt_process_threadHandle;
osThreadId g_mqtt_recv_threadHandle;
osSemaphoreId sempRxmsgRdyHandle;
osStaticSemaphoreDef_t sempRxmsgRdyControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ioControlFunc(void const *argument);
void gpsTimerFunc(void const *argument);
void cloudTimerFunc(void const *argument);
void cloud_queue_exec(osEvent *osEvent);
void errorTimerFunc(void const *argument);
void readBatInfoFunc(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void mqtt_process_thread(void const *argument);
void mqtt_recv_thread(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook(void)
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook(void)
{
  /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of sempRxmsgRdy */
  osSemaphoreStaticDef(sempRxmsgRdy, &sempRxmsgRdyControlBlock);
  sempRxmsgRdyHandle = osSemaphoreCreate(osSemaphore(sempRxmsgRdy), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerDef(ioControlTimer, ioControlFunc);
  ioControlTimerHandle = osTimerCreate(osTimer(ioControlTimer), osTimerPeriodic, NULL);
  osTimerStart(ioControlTimerHandle, 100);

  osTimerDef(gpsTimer, gpsTimerFunc);
  gpsTimerHandle = osTimerCreate(osTimer(gpsTimer), osTimerPeriodic, NULL);

  osTimerDef(readBatTimer, readBatInfoFunc);
  readBatInfoTimerHandle = osTimerCreate(osTimer(readBatTimer), osTimerPeriodic, NULL);

  osTimerDef(cloudTimer, cloudTimerFunc);
  cloudTimerHandle = osTimerCreate(osTimer(cloudTimer), osTimerPeriodic, NULL);

  osTimerDef(errorTimer, errorTimerFunc);
  errorTimerHandle = osTimerCreate(osTimer(errorTimer), osTimerPeriodic, NULL);

  osMessageQDef(cloudQueue, 50, uint32_t);
  cloudQueueHandle = osMessageCreate(osMessageQ(cloudQueue), NULL);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 200);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of g_mqtt_process_thread */
  osThreadDef(g_mqtt_process_thread, mqtt_process_thread, osPriorityBelowNormal, 0, 128);
  g_mqtt_process_threadHandle = osThreadCreate(osThread(g_mqtt_process_thread), NULL);

  /* definition and creation of g_mqtt_recv_thread */
  osThreadDef(g_mqtt_recv_thread, mqtt_recv_thread, osPriorityAboveNormal, 0, 200);
  g_mqtt_recv_threadHandle = osThreadCreate(osThread(g_mqtt_recv_thread), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  osTimerStart(readBatInfoTimerHandle, 500);

  MC20_Init();
  GPSInit();
  MQTT_Start();
  mqttStatus = 1;

  osTimerStart(errorTimerHandle, 10000);
  osTimerStart(cloudTimerHandle, 5000);
  osTimerStart(gpsTimerHandle, 7000);

  HAL_GPIO_WritePin(DEBUG_EN_GPIO_Port, DEBUG_EN_Pin, ENABLE);
  /* Infinite loop */
  for (;;)
  {

    osEvent event;
    event = osMessageGet(cloudQueueHandle, 0);
    if (osEventMessage == event.status)
    {
      cloud_queue_exec(&event);
    }
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_mqtt_process_thread */
/**
* @brief Function implementing the g_mqtt_process_thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mqtt_process_thread */
void mqtt_process_thread(void const *argument)
{
  /* USER CODE BEGIN mqtt_process_thread */
  /* Infinite loop */
  for (;;)
  {

    osDelay(100);
  }
  /* USER CODE END mqtt_process_thread */
}

/* USER CODE BEGIN Header_mqtt_recv_thread */
/**
* @brief Function implementing the g_mqtt_recv_thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mqtt_recv_thread */
void mqtt_recv_thread(void const *argument)
{
  /* USER CODE BEGIN mqtt_recv_thread */
  /* Infinite loop */
  for (;;)
  {
    osDelay(10000);
  }
  /* USER CODE END mqtt_recv_thread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void stopAllTimer()
{

  osTimerStop(readBatInfoTimerHandle);
  osTimerStop(errorTimerHandle);
  osTimerStop(cloudTimerHandle);
  osTimerStop(gpsTimerHandle);
}

uint8_t volt12VStatus = 0;
void ioControlFunc(void const *argument)
{

  if (gprsStatus == 1)
  {
    osTimerStart(ioControlTimerHandle, 600);
    HAL_GPIO_TogglePin(LED_Y_GPIO_Port, LED_Y_Pin);
  }
  else
  {
    HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
  }
  if (gnssDataAval == 1)
  {
    osTimerStart(ioControlTimerHandle, 600);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  }
  else
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  }
}

void gpsTimerFunc(void const *argument)
{

  if (gnssDataAval == 0)
  {
    eventData = READ_GPS_DATA;
    osMessagePut(cloudQueueHandle, eventData, 0);
    return;
  }

  eventData = READ_GPS_DATA;
  osMessagePut(cloudQueueHandle, eventData, 0);
  osDelay(10);

  if (gnssDataAval == 1) // Data valid then upload.otherwise no upload
  {
    eventData = CLOUD_UPLOAD_GPS;
    osMessagePut(cloudQueueHandle, eventData, 0);
  }
}

CAN_TxHeaderTypeDef readVolt;

uint8_t reqData[8] = {
    0x46,
    0x16,
    0x1,
    0,
};
uint32_t txMailBox = 0;

HAL_StatusTypeDef sendCANMessage(uint8_t index, uint8_t len)
{
  reqData[3] = index;
  reqData[4] = len;
  reqData[5] = 0;

  for (uint8_t i = 0; i < 5; i++)
  {
    reqData[5] += reqData[i];
  }
  HAL_CAN_AddTxMessage(&hcan2, &readVolt, reqData, &txMailBox);
  HAL_CAN_AddTxMessage(&hcan1, &readVolt, reqData, &txMailBox);
  return 0;
}

uint8_t index = 0;
uint16_t batInsideVolt = 0;
uint8_t sleepCondition = 0;
uint8_t test = 0;
uint8_t externalPwrStatus = 0;
uint16_t delayCnt = 0;
#define DELAY_CNT 2 * 300
void readBatInfoFunc(void const *arg)
{
  /* 500ms periodic */
  uint16_t tempAdc = 0;
  uint16_t i = 0;

  HAL_ADC_Start(&hadc1);
  if (HAL_OK == HAL_ADC_PollForConversion(&hadc1, 2))
  {
    tempAdc = HAL_ADC_GetValue(&hadc1) * 330 * 2 / 4096;
    batInsideVolt = tempAdc;

    if (batInsideVolt < 330)
    {
      sleepCondition = 1;
    }
    else
    {
      sleepCondition = 0;
    }
  }

  externalPwrStatus = HAL_GPIO_ReadPin(Power_Monitor_GPIO_Port, Power_Monitor_Pin);
  if (externalPwrStatus == 1)
  {
    delayCnt = 0;
  }
  else
  {
    delayCnt++;
  }

  if (delayCnt > DELAY_CNT)
  {
    sleepCondition = 1;
  }
  else
  {
    sleepCondition = 0;
  }

  if (test == 1)
  {
    sleepCondition = test;
  }

  /*
   readVolt.DLC=6;
   readVolt.IDE = CAN_ID_STD;
   readVolt.StdId = 0x528;
   
   if((index%4)==0)
   {
     sendCANMessage(0x09,0x04); //read volt
   }
   else if((index%4)==1)
   {
     sendCANMessage(0x0a,0x04); //read volt
   }
   else if((index%4)==2)
   {
     sendCANMessage(0x0d,0x04); //read soc
   }
   else if((index%4)==3)
   {
     sendCANMessage(0x0e,0x04); //read soh
   }
   index++;
*/
}

void cloudTimerFunc(void const *argument)
{
  if (gprsStatus == 1)
  {
    eventData = CLOUD_UPLOAD_INFO;
    osMessagePut(cloudQueueHandle, eventData, 0);
  }
}

void errorTimerFunc(void const *argument)
{

  if (checkBatErr() == 1)
  {
    eventData = CLOUD_UPLOAD_BAT_ERR;
    osMessagePut(cloudQueueHandle, eventData, 0);
  }

  if (checkInverterErr() == 1)
  {
    eventData = CLOUD_UPLOAD_INVERTER_ERR;
    osMessagePut(cloudQueueHandle, eventData, 0);
  }
}

uint32_t event = 0;
void cloud_queue_exec(osEvent *osEvent)
{

  event = (uint32_t)(osEvent->value.v);
#if 0    
     if( (event & CLOUD_UPLOAD_INFO) !=0 )
     {
        pubLiveData();
     }
     if( (event & CLOUD_UPLOAD_GPS) !=0 )
     {
       pubGPSData();
     }
     if( (event & CLOUD_UPLOAD_INVERTER_ERR) !=0 )
     {
        pubInverterErrData();
     }
     if( (event & CLOUD_UPLOAD_BAT_ERR) !=0 )
     {
        pubBatErrData();
     }
     if( (event & READ_GPS_DATA) !=0 )
     {
       readGnssData();
     }

#endif

#if 1
  switch (event)
  {
  case CLOUD_UPLOAD_INFO:
    pubLiveData();
    break;
  case CLOUD_UPLOAD_GPS:
    pubGPSData();
    break;
  case CLOUD_UPLOAD_INVERTER_ERR:
    pubInverterErrData();
    break;
  case CLOUD_UPLOAD_BAT_ERR:
    pubBatErrData();
    break;
  case READ_GPS_DATA:
    readGnssData();
    break;

  default:
    break;
  }

#endif
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
