
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer; //Assigning xIdleTaskTCBBuffer: The function sets *ppxIdleTaskTCBBuffer to the address of xIdleTaskTCBBuffer. This pointer provides FreeRTOS with the memory for managing the idle task's state.
  *ppxIdleTaskStackBuffer = &xIdleStack[0]; //Assigning xIdleStack: The function sets *ppxIdleTaskStackBuffer to the base address of xIdleStack. This tells FreeRTOS where to place the idle task's stack.
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE; //Setting the Stack Size: The function assigns the value of configMINIMAL_STACK_SIZE to *pulIdleTaskStackSize. FreeRTOS uses this value to know how much stack memory it can use for the idle task.

}

