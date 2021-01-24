#include "idle.h"
#include "cmsis_os.h"


uint32_t timeStamp=0;
extern uint8_t sleepCondition;
extern void mc20PowerOff(void);

extern void closeGNSS();
extern void closePower();
extern uint8_t mc20Status;
extern osThreadId defaultTaskHandle;
extern void stopAllTimer();

enum IDLE_STATE
{
  PRE_SLEEP=0,
  ENTER_SLEEP,
  WAKEUP,
  IDLE,
};

enum IDLE_STATE state;

#define TIME_OUT 2000
void sleepCheck(void)
{
  if( sleepCondition == 1)
  {
     timeStamp = osKernelSysTick();
     state = ENTER_SLEEP; 
  }
  
}


void countToSleep()
{
  if(sleepCondition != 1)
    state = PRE_SLEEP;
  
  if(osKernelSysTick()-timeStamp >= TIME_OUT )
  {
    stopAllTimer();
    vTaskSuspend(defaultTaskHandle);
    
  //  closeGNSS();
  //  closePower();
    if(mc20Status == 1)
    {
      mc20PowerOff();
    }
    
    HAL_PWR_EnterSTANDBYMode();
  }
}


void wakeUp()
{
   NVIC_SystemReset();  
}


void vApplicationIdleHook(void)
{
  
  switch(state)
  {
  case PRE_SLEEP:
    sleepCheck();
    break;
  case ENTER_SLEEP:
    countToSleep();
    break;
  case WAKEUP:
    wakeUp();
    break;
  default:
    break;
       
  }
  
}



void vApplicationTickHook(void)
{
  
}