#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <avr/io.h>

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      ( ( uint32_t ) F_CPU )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    4
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 85 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 400 ) )
#define configMAX_TASK_NAME_LEN                 8
#define configUSE_16_BIT_TICKS                  1
#define configIDLE_SHOULD_YIELD                 1

// Mutexes, semaphores, and other features can be enabled as needed
#define configUSE_MUTEXES                       1
#define configUSE_COUNTING_SEMAPHORES           1

// Include assert functionality
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

#endif /* FREERTOS_CONFIG_H */
