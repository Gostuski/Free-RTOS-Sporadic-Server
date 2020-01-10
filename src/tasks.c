/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* FreeRTOS includes. */
#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "list.h"

/* Lint e9021, e961 and e750 are suppressed as a MISRA exception justified
because the MPU ports require MPU_WRAPPERS_INCLUDED_FROM_API_FILE to be defined
for the header files above, but not in this file, in order to generate the
correct privileged Vs unprivileged linkage and placement. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750 !e9021. */

/* Set configUSE_STATS_FORMATTING_FUNCTIONS to 2 to include the stats formatting
functions but without including stdio.h here. */

#if (configUSE_PREEMPTION == 0)
/* If the cooperative scheduler is being used then a yield should not be
    performed just because a higher priority task has been woken. */
#define taskYIELD_IF_USING_PREEMPTION()
#else
#define taskYIELD_IF_USING_PREEMPTION() portYIELD_WITHIN_API()
#endif

/* Values that can be assigned to the ucNotifyState member of the TCB. */
#define taskNOT_WAITING_NOTIFICATION ((uint8_t)0)
#define taskWAITING_NOTIFICATION ((uint8_t)1)
#define taskNOTIFICATION_RECEIVED ((uint8_t)2)

/*
 * The value used to fill the stack of a task when the task is created.  This
 * is used purely for checking the high water mark for tasks.
 */
#define tskSTACK_FILL_BYTE (0xa5U)

/* Bits used to record how a task's stack and TCB were allocated. */
#define tskDYNAMICALLY_ALLOCATED_STACK_AND_TCB ((uint8_t)0)
#define tskSTATICALLY_ALLOCATED_STACK_ONLY ((uint8_t)1)
#define tskSTATICALLY_ALLOCATED_STACK_AND_TCB ((uint8_t)2)

/* If any of the following are set then task stacks are filled with a known
value so the high water mark can be determined.  If none of the following are
set then don't fill the stack so there is no unnecessary dependency on memset. */
#if ((configCHECK_FOR_STACK_OVERFLOW > 1) || (configUSE_TRACE_FACILITY == 1) || (INCLUDE_uxTaskGetStackHighWaterMark == 1))
#define tskSET_NEW_STACKS_TO_KNOWN_VALUE 1
#else
#define tskSET_NEW_STACKS_TO_KNOWN_VALUE 0
#endif

/*
 * Macros used by vListTask to indicate which state a task is in.
 */
#define tskRUNNING_CHAR ('X')
#define tskBLOCKED_CHAR ('B')
#define tskREADY_CHAR ('R')
#define tskDELETED_CHAR ('D')
#define tskSUSPENDED_CHAR ('S')

/*
 * Some kernel aware debuggers require the data the debugger needs access to be
 * global, rather than file scope.
 */
#ifdef portREMOVE_STATIC_QUALIFIER
#define static
#endif

/* The name allocated to the Idle task.  This can be overridden by defining
configIDLE_TASK_NAME in FreeRTOSConfig.h. */
#ifndef configIDLE_TASK_NAME
#define configIDLE_TASK_NAME "IDLE"
#endif

#if (configUSE_PORT_OPTIMISED_TASK_SELECTION == 0)

/* If configUSE_PORT_OPTIMISED_TASK_SELECTION is 0 then task selection is
    performed in a generic way that is not optimised to any particular
    microcontroller architecture. */

/* uxTopReadyPriority holds the priority of the highest priority ready
    state task. */
#define taskRECORD_READY_PRIORITY(uxPriority)  \
    {                                          \
        if ((uxPriority) > uxTopReadyPriority) \
        {                                      \
            uxTopReadyPriority = (uxPriority); \
        }                                      \
    } /* taskRECORD_READY_PRIORITY */

/*-----------------------------------------------------------*/

#define taskSELECT_HIGHEST_PRIORITY_TASK()                                              \
    {                                                                                   \
        UBaseType_t uxTopPriority = uxTopReadyPriority;                                 \
                                                                                        \
        /* Find the highest priority queue that contains ready tasks. */                \
        while (listLIST_IS_EMPTY(&(pxReadyTasksLists[uxTopPriority])))                  \
        {                                                                               \
            configASSERT(uxTopPriority);                                                \
            --uxTopPriority;                                                            \
        }                                                                               \
                                                                                        \
        /* listGET_OWNER_OF_NEXT_ENTRY indexes through the list, so the tasks of        \
        the    same priority get an equal share of the processor time. */               \
        listGET_OWNER_OF_NEXT_ENTRY(pxCurrentTCB, &(pxReadyTasksLists[uxTopPriority])); \
        uxTopReadyPriority = uxTopPriority;                                             \
    } /* taskSELECT_HIGHEST_PRIORITY_TASK */

/*-----------------------------------------------------------*/

/* Define away taskRESET_READY_PRIORITY() and portRESET_READY_PRIORITY() as
    they are only required when a port optimised method of task selection is
    being used. */
#define taskRESET_READY_PRIORITY(uxPriority)
#define portRESET_READY_PRIORITY(uxPriority, uxTopReadyPriority)

#else /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* If configUSE_PORT_OPTIMISED_TASK_SELECTION is 1 then task selection is
    performed in a way that is tailored to the particular microcontroller
    architecture being used. */

/* A port optimised version is provided.  Call the port defined macros. */
#define taskRECORD_READY_PRIORITY(uxPriority) portRECORD_READY_PRIORITY(uxPriority, uxTopReadyPriority)

/*-----------------------------------------------------------*/

#define taskSELECT_HIGHEST_PRIORITY_TASK()                                              \
    {                                                                                   \
        UBaseType_t uxTopPriority;                                                      \
                                                                                        \
        /* Find the highest priority list that contains ready tasks. */                 \
        portGET_HIGHEST_PRIORITY(uxTopPriority, uxTopReadyPriority);                    \
        configASSERT(listCURRENT_LIST_LENGTH(&(pxReadyTasksLists[uxTopPriority])) > 0); \
        listGET_OWNER_OF_NEXT_ENTRY(pxCurrentTCB, &(pxReadyTasksLists[uxTopPriority])); \
    } /* taskSELECT_HIGHEST_PRIORITY_TASK() */

/*-----------------------------------------------------------*/

/* A port optimised version is provided, call it only if the TCB being reset
    is being referenced from a ready list.  If it is referenced from a delayed
    or suspended list then it won't be in a ready list. */
#define taskRESET_READY_PRIORITY(uxPriority)                                               \
    {                                                                                      \
        if (listCURRENT_LIST_LENGTH(&(pxReadyTasksLists[(uxPriority)])) == (UBaseType_t)0) \
        {                                                                                  \
            portRESET_READY_PRIORITY((uxPriority), (uxTopReadyPriority));                  \
        }                                                                                  \
    }

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

void (*print_string)(char *);
void (*print_number)(int);

void set_print_str(void (*print_str)(char *))
{
    print_string = print_str;
}

void set_print_num(void (*print_num)(int))
{
    print_number = print_num;
}

/*-----------------------------------------------------------*/

/* pxDelayedTaskList and pxOverflowDelayedTaskList are switched when the tick
count overflows. */
#define taskSWITCH_DELAYED_LISTS()                                                \
    {                                                                             \
        List_t *pxTemp;                                                           \
                                                                                  \
        /* The delayed tasks list should be empty when the lists are switched. */ \
        configASSERT((listLIST_IS_EMPTY(pxDelayedTaskList)));                     \
                                                                                  \
        pxTemp = pxDelayedTaskList;                                               \
        pxDelayedTaskList = pxOverflowDelayedTaskList;                            \
        pxOverflowDelayedTaskList = pxTemp;                                       \
        xNumOfOverflows++;                                                        \
        prvResetNextTaskUnblockTime();                                            \
    }

/*-----------------------------------------------------------*/

/*
 * Place the task represented by pxTCB into the appropriate ready list for
 * the task.  It is inserted at the end of the list.
 */
#define prvAddTaskToReadyList(pxTCB)                                                       \
    traceMOVED_TASK_TO_READY_STATE(pxTCB);                                                 \
    taskRECORD_READY_PRIORITY((pxTCB)->uxPriority);                                        \
    vListInsertEnd(&(pxReadyTasksLists[(pxTCB)->uxPriority]), &((pxTCB)->xStateListItem)); \
    tracePOST_MOVED_TASK_TO_READY_STATE(pxTCB)
/*-----------------------------------------------------------*/

/*
 * Several functions take an TaskHandle_t parameter that can optionally be NULL,
 * where NULL is used to indicate that the handle of the currently executing
 * task should be used in place of the parameter.  This macro simply checks to
 * see if the parameter is NULL and returns a pointer to the appropriate TCB.
 */
#define prvGetTCBFromHandle(pxHandle) (((pxHandle) == NULL) ? pxCurrentTCB : (pxHandle))

/* The item value of the event list item is normally used to hold the priority
of the task to which it belongs (coded to allow it to be held in reverse
priority order).  However, it is occasionally borrowed for other purposes.  It
is important its value is not updated due to a task priority change while it is
being used for another purpose.  The following bit definition is used to inform
the scheduler that the value should not be changed - in which case it is the
responsibility of whichever module is using the value to ensure it gets set back
to its original value when it is released. */
#if (configUSE_16_BIT_TICKS == 1)
#define taskEVENT_LIST_ITEM_VALUE_IN_USE 0x8000U
#else
#define taskEVENT_LIST_ITEM_VALUE_IN_USE 0x80000000UL
#endif

/*
 * Task control block.  A task control block (TCB) is allocated for each task,
 * and stores task state information, including a pointer to the task's context
 * (the task's run time environment, including register values)
 */

//MYTCB
typedef struct TaskControlBlock_t
{
    volatile StackType_t *pxTopOfStack; /*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

    ListItem_t xStateListItem;                                                                                                     /*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
    ListItem_t xEventListItem;                                                                                                     /*< Used to reference a task from an event list. */
    UBaseType_t uxPriority;                                                                                                        /*< The priority of the task.  0 is the lowest priority. */
    StackType_t *pxStack;                                                                                                          /*< Points to the start of the stack. */
    char pcTaskName[configMAX_TASK_NAME_LEN]; /*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

    char *pcName;

    void *pvParameters;

    configSTACK_DEPTH_TYPE stackDepth;

    TaskFunction_t taskCode;

    int cycle;

    TickType_t period, duration, arrival;

#if (configUSE_MUTEXES == 1)
    UBaseType_t uxBasePriority; /*< The priority last assigned to the task - used by the priority inheritance mechanism. */
    UBaseType_t uxMutexesHeld;
#endif

#if (configUSE_TASK_NOTIFICATIONS == 1)
    volatile uint32_t ulNotifiedValue;
    volatile uint8_t ucNotifyState;
#endif

/* See the comments in Arduino_FreeRTOS.h with the definition of
    tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
#if (tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0) /*lint !e731 !e9029 Macro has been consolidated for readability reasons. */
    uint8_t ucStaticallyAllocated;                   /*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
#endif

} tskTCB;

/* The old tskTCB name is maintained above then typedefed to the new TCB_t name
below to enable the use of older kernel aware debuggers. */
typedef tskTCB TCB_t;

TCB_t *restartTask = NULL;
/*lint -save -e956 A manual analysis and inspection has been used to determine
which static variables must be declared volatile. */
PRIVILEGED_DATA TCB_t *volatile pxCurrentTCB __attribute__((used)) = NULL;

/* Lists for ready and blocked tasks. --------------------
xDelayedTaskList1 and xDelayedTaskList2 could be move to function scople but
doing so breaks some kernel aware debuggers and debuggers that rely on removing
the static qualifier. */
PRIVILEGED_DATA static List_t pxReadyTasksLists[configMAX_PRIORITIES]; /*< Prioritised ready tasks. */
PRIVILEGED_DATA static List_t xDelayedTaskList1;                       /*< Delayed tasks. */
PRIVILEGED_DATA static List_t xDelayedTaskList2;                       /*< Delayed tasks (two lists are used - one for delays that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t *volatile pxDelayedTaskList;             /*< Points to the delayed task list currently being used. */
PRIVILEGED_DATA static List_t *volatile pxOverflowDelayedTaskList;     /*< Points to the delayed task list currently being used to hold tasks that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t xPendingReadyList;                       /*< Tasks that have been readied while the scheduler was suspended.  They will be moved to the ready list when the scheduler is resumed. */

#if (INCLUDE_vTaskDelete == 1)

PRIVILEGED_DATA static List_t xTasksWaitingTermination; /*< Tasks that have been deleted - but their memory not yet freed. */
PRIVILEGED_DATA static volatile UBaseType_t uxDeletedTasksWaitingCleanUp = (UBaseType_t)0U;

#endif

#if (INCLUDE_vTaskSuspend == 1)

PRIVILEGED_DATA static List_t xSuspendedTaskList; /*< Tasks that are currently suspended. */

#endif

/* Other file private variables. --------------------------------*/
PRIVILEGED_DATA static volatile UBaseType_t uxCurrentNumberOfTasks = (UBaseType_t)0U;
PRIVILEGED_DATA static volatile TickType_t xTickCount = (TickType_t)configINITIAL_TICK_COUNT;
PRIVILEGED_DATA static volatile UBaseType_t uxTopReadyPriority = tskIDLE_PRIORITY;
PRIVILEGED_DATA static volatile BaseType_t xSchedulerRunning = pdFALSE;
PRIVILEGED_DATA static volatile UBaseType_t uxPendedTicks = (UBaseType_t)0U;
PRIVILEGED_DATA static volatile BaseType_t xYieldPending = pdFALSE;
PRIVILEGED_DATA static volatile BaseType_t xNumOfOverflows = (BaseType_t)0;
PRIVILEGED_DATA static UBaseType_t uxTaskNumber = (UBaseType_t)0U;
PRIVILEGED_DATA static volatile TickType_t xNextTaskUnblockTime = (TickType_t)0U; /* Initialised to portMAX_DELAY before the scheduler starts. */
PRIVILEGED_DATA static TaskHandle_t xIdleTaskHandle = NULL;                       /*< Holds the handle of the idle task.  The idle task is created automatically when the scheduler is started. */

/* Context switches are held pending while the scheduler is suspended.  Also,
interrupts must not manipulate the xStateListItem of a TCB, or any of the
lists the xStateListItem can be referenced from, if the scheduler is suspended.
If an interrupt needs to unblock a task while the scheduler is suspended then it
moves the task's event list item into the xPendingReadyList, ready for the
kernel to move the task from the pending ready list into the real ready list
when the scheduler is unsuspended.  The pending ready list itself can only be
accessed from a critical section. */
PRIVILEGED_DATA static volatile UBaseType_t uxSchedulerSuspended = (UBaseType_t)pdFALSE;

/*lint -restore */

/*-----------------------------------------------------------*/

/* Callback function prototypes. --------------------------*/
#if (configCHECK_FOR_STACK_OVERFLOW > 0)

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

#endif

/* File private functions. --------------------------------*/

/**
 * Utility task that simply returns pdTRUE if the task referenced by xTask is
 * currently in the Suspended state, or pdFALSE if the task referenced by xTask
 * is in any other state.
 */
#if (INCLUDE_vTaskSuspend == 1)

static BaseType_t prvTaskIsTaskSuspended(const TaskHandle_t xTask) PRIVILEGED_FUNCTION;

#endif /* INCLUDE_vTaskSuspend */

/*
 * Utility to ready all the lists used by the scheduler.  This is called
 * automatically upon the creation of the first task.
 */
static void prvInitialiseTaskLists(void) PRIVILEGED_FUNCTION;

/*
 * The idle task, which as all tasks is implemented as a never ending loop.
 * The idle task is automatically created and added to the ready lists upon
 * creation of the first user task.
 *
 * The portTASK_FUNCTION_PROTO() macro is used to allow port/compiler specific
 * language extensions.  The equivalent prototype for this function is:
 *
 * void prvIdleTask( void *pvParameters );
 *
 */
static portTASK_FUNCTION_PROTO(prvIdleTask, pvParameters);

/*
 * Utility to free all memory allocated by the scheduler to hold a TCB,
 * including the stack pointed to by the TCB.
 *
 * This does not free memory allocated by the task itself (i.e. memory
 * allocated by calls to pvPortMalloc from within the tasks application code).
 */
#if (INCLUDE_vTaskDelete == 1)

static void prvDeleteTCB(TCB_t *pxTCB) PRIVILEGED_FUNCTION;

#endif

/*
 * Used only by the idle task.  This checks to see if anything has been placed
 * in the list of tasks waiting to be deleted.  If so the task is cleaned up
 * and its TCB deleted.
 */
static void prvCheckTasksWaitingTermination(void) PRIVILEGED_FUNCTION;

/*
 * The currently executing task is entering the Blocked state.  Add the task to
 * either the current or the overflow delayed task list.
 */
static void prvAddCurrentTaskToDelayedList(TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely) PRIVILEGED_FUNCTION;

/*
 * When a task is created, the stack of the task is filled with a known value.
 * This function determines the 'high water mark' of the task stack by
 * determining how much of the stack remains at the original preset value.
 */
#if ((configUSE_TRACE_FACILITY == 1) || (INCLUDE_uxTaskGetStackHighWaterMark == 1))

static configSTACK_DEPTH_TYPE prvTaskCheckFreeStackSpace(const uint8_t *pucStackByte) PRIVILEGED_FUNCTION;

#endif

/*
 * Set xNextTaskUnblockTime to the time at which the next Blocked state task
 * will exit the Blocked state.
 */
static void prvResetNextTaskUnblockTime(void);

#if ((configUSE_TRACE_FACILITY == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS > 0))

/*
     * Helper function used to pad task names with spaces when printing out
     * human readable tables of task information.
     */
static char *prvWriteNameToBuffer(char *pcBuffer, const char *pcTaskName) PRIVILEGED_FUNCTION;

#endif

/*
 * Called after a Task_t structure has been allocated either statically or
 * dynamically to fill in the structure's members.
 */
static void prvInitialiseNewTask(TaskFunction_t pxTaskCode,
                                 const char *const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                                 const configSTACK_DEPTH_TYPE ulStackDepth,
                                 void *const pvParameters,
                                 UBaseType_t uxPriority,
                                 TaskHandle_t *const pxCreatedTask,
                                 TCB_t *pxNewTCB,
                                 const MemoryRegion_t *const xRegions) PRIVILEGED_FUNCTION;

/*
 * Called after a new task has been created and initialised to place the task
 * under the control of the scheduler.
 */
static void prvAddNewTaskToReadyList(TCB_t *pxNewTCB) PRIVILEGED_FUNCTION;

/*
 * freertos_tasks_c_additions_init() should only be called if the user definable
 * macro FREERTOS_TASKS_C_ADDITIONS_INIT() is defined, as that is the only macro
 * called by the function.
 */
#ifdef FREERTOS_TASKS_C_ADDITIONS_INIT

static void freertos_tasks_c_additions_init(void) PRIVILEGED_FUNCTION;

#endif

#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)

BaseType_t xTaskCreatePeriodic(TaskFunction_t pxTaskCode,
                               const char *const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                               const configSTACK_DEPTH_TYPE usStackDepth,
                               void *const pvParameters,
                               UBaseType_t uxPriority,
                               TaskHandle_t *const pxCreatedTask, TickType_t arrival, TickType_t period, TickType_t duration)
{
    TCB_t *pxNewTCB;
    BaseType_t xReturn;

    StackType_t *pxStack;

    /* Allocate space for the stack used by the task being created. */
    pxStack = (StackType_t *)pvPortMalloc((((size_t)usStackDepth) * sizeof(StackType_t)));

    if (pxStack != NULL)
    {
        /* Allocate space for the TCB. */
        pxNewTCB = (TCB_t *)pvPortMalloc(sizeof(TCB_t));

        if (pxNewTCB != NULL)
        {
            /* Store the stack location in the TCB. */
            pxNewTCB->pxStack = pxStack;
        }
        else
        {
            /* The stack cannot be used as the TCB was not created.  Free
                        it again. */
            vPortFree(pxStack);
        }
    }
    else
    {
        pxNewTCB = NULL;
    }

    if (pxNewTCB != NULL)
    {
        prvInitialiseNewTask(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, pxNewTCB, NULL);
        pxNewTCB->arrival = arrival;
        pxNewTCB->duration = duration;
        pxNewTCB->period = period;
        pxNewTCB->cycle = 0;
        pxNewTCB->pvParameters = pvParameters;
        pxNewTCB->stackDepth = usStackDepth;
        pxNewTCB->taskCode = pxTaskCode;
        pxNewTCB->pcName = pcName;
        prvAddNewTaskToReadyList(pxNewTCB);
        xReturn = pdPASS;
    }
    else
    {
        xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }

    return xReturn;
}

BaseType_t xTaskCreate(TaskFunction_t pxTaskCode,
                       const char *const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                       const configSTACK_DEPTH_TYPE usStackDepth,
                       void *const pvParameters,
                       UBaseType_t uxPriority,
                       TaskHandle_t *const pxCreatedTask)
{
    TCB_t *pxNewTCB;
    BaseType_t xReturn;

    /* If the stack grows down then allocate the stack then the TCB so the stack
        does not grow into the TCB.  Likewise if the stack grows up then allocate
        the TCB then the stack. */
    {
        StackType_t *pxStack;

        /* Allocate space for the stack used by the task being created. */
        pxStack = (StackType_t *)pvPortMalloc((((size_t)usStackDepth) * sizeof(StackType_t))); /*lint !e9079 All values returned by pvPortMalloc() have at least the alignment required by the MCU's stack and this allocation is the stack. */

        if (pxStack != NULL)
        {
            /* Allocate space for the TCB. */
            pxNewTCB = (TCB_t *)pvPortMalloc(sizeof(TCB_t)); /*lint !e9087 !e9079 All values returned by pvPortMalloc() have at least the alignment required by the MCU's stack, and the first member of TCB_t is always a pointer to the task's stack. */

            if (pxNewTCB != NULL)
            {
                /* Store the stack location in the TCB. */
                pxNewTCB->pxStack = pxStack;
            }
            else
            {
                /* The stack cannot be used as the TCB was not created.  Free
                    it again. */
                vPortFree(pxStack);
            }
        }
        else
        {
            pxNewTCB = NULL;
        }
    }

    if (pxNewTCB != NULL)
    {
#if (tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0) /*lint !e9029 !e731 Macro has been consolidated for readability reasons. */
        {
            /* Tasks can be created statically or dynamically, so note this
                task was created dynamically in case it is later deleted. */
            pxNewTCB->ucStaticallyAllocated = tskDYNAMICALLY_ALLOCATED_STACK_AND_TCB;
        }
#endif /* tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE */

        prvInitialiseNewTask(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, pxNewTCB, NULL);
        prvAddNewTaskToReadyList(pxNewTCB);
        xReturn = pdPASS;
    }
    else
    {
        xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }

    return xReturn;
}

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

static void prvInitialiseNewTask(TaskFunction_t pxTaskCode,
                                 const char *const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                                 const configSTACK_DEPTH_TYPE ulStackDepth,
                                 void *const pvParameters,
                                 UBaseType_t uxPriority,
                                 TaskHandle_t *const pxCreatedTask,
                                 TCB_t *pxNewTCB,
                                 const MemoryRegion_t *const xRegions)
{
    StackType_t *pxTopOfStack;
    UBaseType_t x;

/* Avoid dependency on memset() if it is not required. */
#if (tskSET_NEW_STACKS_TO_KNOWN_VALUE == 1)
    {
        /* Fill the stack with a known value to assist debugging. */
        (void)memset(pxNewTCB->pxStack, (int)tskSTACK_FILL_BYTE, (size_t)ulStackDepth * sizeof(StackType_t));
    }
#endif /* tskSET_NEW_STACKS_TO_KNOWN_VALUE */

    /* Calculate the top of stack address.  This depends on whether the stack
    grows from high memory to low (as per the 80x86) or vice versa.
    portSTACK_GROWTH is used to make the result positive or negative as required
    by the port. */
    pxTopOfStack = &(pxNewTCB->pxStack[ulStackDepth - (configSTACK_DEPTH_TYPE)1]);
    pxTopOfStack = (StackType_t *)(((portPOINTER_SIZE_TYPE)pxTopOfStack) & (~((portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK))); /*lint !e923 !e9033 !e9078 MISRA exception.  Avoiding casts between pointers and integers is not practical.  Size differences accounted for using portPOINTER_SIZE_TYPE type.  Checked by assert(). */

    /* Check the alignment of the calculated top of stack is correct. */
    configASSERT((((portPOINTER_SIZE_TYPE)pxTopOfStack & (portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK) == 0UL));

    /* Store the task name in the TCB. */
    if (pcName != NULL)
    {
        for (x = (UBaseType_t)0; x < (UBaseType_t)configMAX_TASK_NAME_LEN; x++)
        {
            pxNewTCB->pcTaskName[x] = pcName[x];

            /* Don't copy all configMAX_TASK_NAME_LEN if the string is shorter than
        configMAX_TASK_NAME_LEN characters just in case the memory after the
        string is not accessible (extremely unlikely). */
            if (pcName[x] == (char)0x00)
            {
                break;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }

        /* Ensure the name string is terminated in the case that the string length
    was greater or equal to configMAX_TASK_NAME_LEN. */
        pxNewTCB->pcTaskName[configMAX_TASK_NAME_LEN - 1] = '\0';
    }
    else
    {
        /* The task has not been given a name, so just ensure there is a NULL
        terminator when it is read out. */
        pxNewTCB->pcTaskName[0] = 0x00;
    }

    /* This is used as an array index so must ensure it's not too large.  First
    remove the privilege bit if one is present. */
    if (uxPriority >= (UBaseType_t)configMAX_PRIORITIES)
    {
        uxPriority = (UBaseType_t)configMAX_PRIORITIES - (UBaseType_t)1U;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    pxNewTCB->uxPriority = uxPriority;
#if (configUSE_MUTEXES == 1)
    {
        pxNewTCB->uxBasePriority = uxPriority;
        pxNewTCB->uxMutexesHeld = 0;
    }
#endif /* configUSE_MUTEXES */

    vListInitialiseItem(&(pxNewTCB->xStateListItem));
    vListInitialiseItem(&(pxNewTCB->xEventListItem));

    /* Set the pxNewTCB as a link back from the ListItem_t.  This is so we can get
    back to    the containing TCB from a generic item in a list. */
    listSET_LIST_ITEM_OWNER(&(pxNewTCB->xStateListItem), pxNewTCB);

    /* Event lists are always in priority order. */
    listSET_LIST_ITEM_VALUE(&(pxNewTCB->xEventListItem), (TickType_t)configMAX_PRIORITIES - (TickType_t)uxPriority); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
    listSET_LIST_ITEM_OWNER(&(pxNewTCB->xEventListItem), pxNewTCB);

    /* Avoid compiler warning about unreferenced parameter. */
    (void)xRegions;

#if (configUSE_TASK_NOTIFICATIONS == 1)
    {
        pxNewTCB->ulNotifiedValue = 0;
        pxNewTCB->ucNotifyState = taskNOT_WAITING_NOTIFICATION;
    }
#endif

    pxNewTCB->pxTopOfStack = pxPortInitialiseStack(pxTopOfStack, pxTaskCode, pvParameters);

    if (pxCreatedTask != NULL)
    {
        /* Pass the handle out in an anonymous way.  The handle can be used to
        change the created task's priority, delete the created task, etc.*/
        *pxCreatedTask = (TaskHandle_t)pxNewTCB;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}
/*-----------------------------------------------------------*/

static void prvAddNewTaskToReadyList(TCB_t *pxNewTCB)
{
    /* Ensure interrupts don't access the task lists while the lists are being
    updated. */
    taskENTER_CRITICAL();
    {
        uxCurrentNumberOfTasks++;
        if (pxCurrentTCB == NULL)
        {
            /* There are no other tasks, or all the other tasks are in
            the suspended state - make this the current task. */
            pxCurrentTCB = pxNewTCB;

            if (uxCurrentNumberOfTasks == (UBaseType_t)1)
            {
                /* This is the first task to be created so do the preliminary
                initialisation required.  We will not recover if this call
                fails, but we will report the failure. */
                prvInitialiseTaskLists();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            /* If the scheduler is not already running, make this task the
            current task if it is the highest priority task to be created
            so far. */
            if (xSchedulerRunning == pdFALSE)
            {
                if (pxCurrentTCB->uxPriority <= pxNewTCB->uxPriority)
                {
                    // pxCurrentTCB = pxNewTCB;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }

        uxTaskNumber++;

        traceTASK_CREATE(pxNewTCB);

        prvAddTaskToReadyList(pxNewTCB);

        portSETUP_TCB(pxNewTCB);
    }
    taskEXIT_CRITICAL();

    if (xSchedulerRunning != pdFALSE)
    {
        /* If the created task is of a higher priority than the current task
        then it should run now. */
        if (pxCurrentTCB->uxPriority < pxNewTCB->uxPriority)
        {
            taskYIELD_IF_USING_PREEMPTION();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskDelete == 1)

void vTaskDeleteLogical()
{
    pxCurrentTCB->cycle += 1;
    restartTask = pxCurrentTCB;
    portYIELD_WITHIN_API();
}

void vTaskDelete(TaskHandle_t xTaskToDelete)
{
    TCB_t *pxTCB;
    taskENTER_CRITICAL();
    {
        /* If null is passed in here then it is the calling task that is
            being deleted. */
        pxTCB = prvGetTCBFromHandle(xTaskToDelete);

        /* Remove task from the ready list. */
        if (uxListRemove(&(pxTCB->xStateListItem)) == (UBaseType_t)0)
        {
            taskRESET_READY_PRIORITY(pxTCB->uxPriority);
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Is the task waiting on an event also? */
        if (listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) != NULL)
        {
            (void)uxListRemove(&(pxTCB->xEventListItem));
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Increment the uxTaskNumber also so kernel aware debuggers can
            detect that the task lists need re-generating.  This is done before
            portPRE_TASK_DELETE_HOOK() as in the Windows port that macro will
            not return. */
        uxTaskNumber++;

        if (pxTCB == pxCurrentTCB)
        {
            /* A task is deleting itself.  This cannot complete within the
                task itself, as a context switch to another task is required.
                Place the task in the termination list.  The idle task will
                check the termination list and free up any memory allocated by
                the scheduler for the TCB and stack of the deleted task. */
            vListInsertEnd(&xTasksWaitingTermination, &(pxTCB->xStateListItem));

            /* Increment the ucTasksDeleted variable so the idle task knows
                there is a task that has been deleted and that it should therefore
                check the xTasksWaitingTermination list. */
            ++uxDeletedTasksWaitingCleanUp;

            /* The pre-delete hook is primarily for the Windows simulator,
                in which Windows specific clean up operations are performed,
                after which it is not possible to yield away from this task -
                hence xYieldPending is used to latch that a context switch is
                required. */
            portPRE_TASK_DELETE_HOOK(pxTCB, &xYieldPending);
        }
        else
        {
            --uxCurrentNumberOfTasks;
            prvDeleteTCB(pxTCB);

            /* Reset the next expected unblock time in case it referred to
                the task that has just been deleted. */
            prvResetNextTaskUnblockTime();
        }

        traceTASK_DELETE(pxTCB);
    }
    taskEXIT_CRITICAL();

    /* Force a reschedule if it is the currently running task that has just
        been deleted. */
    if (xSchedulerRunning != pdFALSE)
    {
        if (pxTCB == pxCurrentTCB)
        {
            configASSERT(uxSchedulerSuspended == 0);
            portYIELD_WITHIN_API();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
}

#endif /* INCLUDE_vTaskDelete */
/*-----------------------------------------------------------*/
/* Project variables */

#define PERIODIC_TASK_PRIORITY 2
#define APERIODIC_TASK_PRIORITY 1
#define MAX_REFILLS 3
#define MAX_TASK_NAME_LENGTH 5

#define MAX_TASKS_INPUT 5

static TickType_t serverCapacity = 5;
static TickType_t serverPeriod = 10;

struct parameters
{
    BaseType_t taskType;
    TickType_t arrival;
    TickType_t period;
    TickType_t duration;
    BaseType_t create;
    char taskName[MAX_TASK_NAME_LENGTH];

} taskParameters[MAX_TASKS_INPUT];

struct capacityRefill
{
    TickType_t refillTick;
    TickType_t refillAmount;

} refills[MAX_REFILLS];

void initialiseServer(TickType_t capacity, TickType_t period)
{

    serverCapacity = capacity;
    serverPeriod = period;

    print_string("Server capacity: ");
    print_number(serverCapacity);
    print_string("\n");
    print_string("Server period: ");
    print_number(serverPeriod);
    print_string("\n");
    return;
}

void setRefill(TickType_t refill)
{

    unsigned char i;
    for (i = 0; i < MAX_REFILLS; i++)
    {
        if (refills[i].refillAmount == 0)
        {
            refills[i].refillAmount = refill;
            refills[i].refillTick = xTaskGetTickCount() + serverPeriod;
            return;
        }
    }
}
void taskAperiodicNumber(void *parameter)
{
    char *output = (char*)parameter;

    TickType_t counter = 0;
    TickType_t temp = xTaskGetTickCount() - 1;

    while (counter < pxCurrentTCB->duration)
    {
        if (temp != xTaskGetTickCount())
        {
            counter++;
            print_string(output);
            print_number(xTaskGetTickCount());
            print_string("\n");
            temp = xTaskGetTickCount();
            serverCapacity -= 1;
        }
    }

    while (temp == xTaskGetTickCount())
    {
    }

    vTaskDelete(NULL);
}
void taskAperiodic(void *parameter)
{
    char *output = (char*)parameter;

    TickType_t counter = 0;
    TickType_t temp = xTaskGetTickCount() - 1;

    while (counter < pxCurrentTCB->duration)
    {
        if (temp != xTaskGetTickCount())
        {
            counter++;
            print_string(output);
            print_number(xTaskGetTickCount());
            print_string("\n");
            temp = xTaskGetTickCount();
            serverCapacity -= 1;
        }
    }

    while (temp == xTaskGetTickCount())
    {
    }

    vTaskDelete(NULL);
}
void taskPeriodicNumber(void *parameter)
{
    char *output = (char*)parameter;

    TickType_t counter = 0;
    TickType_t temp = xTaskGetTickCount() - 1;

    while (counter < pxCurrentTCB->duration)
    {
        if (temp != xTaskGetTickCount())
        {
            counter++;
            print_string(output);
            print_number(xTaskGetTickCount());
            print_string("\n");
            temp = xTaskGetTickCount();
        }
    }

    while (temp == xTaskGetTickCount())
    {
    }

    vTaskDeleteLogical();
}
void taskPeriodic(void *parameter)
{
    char *output = (char*)parameter;

    TickType_t counter = 0;
    TickType_t temp = xTaskGetTickCount() - 1;

    while (counter < pxCurrentTCB->duration)
    {
        if (temp != xTaskGetTickCount())
        {
            counter++;
            // print_string(pxCurrentTCB->pcName);
            print_string(output);
            print_number(xTaskGetTickCount());
            print_string("\n");
            temp = xTaskGetTickCount();
        }
    }

    while (temp == xTaskGetTickCount())
    {
    }

    vTaskDeleteLogical();
}
void parseInput(char *input)
{
    char *token;
    // print_string("Parse\n");
    token = strtok(input, " ");

    if (strCmp(token, "delete"))
    {
        token = strtok(NULL, " ");
        deleteTask(token);
    }
    else if (strCmp(token, "periodic"))
    {
        char *taskName = pvPortMalloc(MAX_TASK_NAME_LENGTH * sizeof(char));
        strcpy(taskName, strtok(NULL, " "));
        TickType_t period;
        TickType_t duration;
        char *taskFunction;
        taskFunction = strtok(NULL, " ");
        char *taskParam = pvPortMalloc(MAX_TASK_NAME_LENGTH * sizeof(char));
        strcpy(taskParam, strtok(NULL, " "));
        token = strtok(NULL, " ");
        period = atoi(token);
        token = strtok(NULL, " ");
        duration = atoi(token);

        // print_string(period);
        // print_string(duration);

        if(strcmp(taskFunction, "word")){
            xTaskCreatePeriodic(taskPeriodic, taskName, 120, taskParam, PERIODIC_TASK_PRIORITY, NULL, xTaskGetTickCount(), period, duration);
        }else{
            xTaskCreatePeriodic(taskPeriodicNumber, taskName, 120, taskParam, PERIODIC_TASK_PRIORITY, NULL, xTaskGetTickCount(), period, duration);
        }

        // xTaskCreatePeriodic(taskPeriodic, taskName, 100, taskName, PERIODIC_TASK_PRIORITY, NULL, xTaskGetTickCount(), period, duration);
    }
    else if (strCmp(token, "aperiodic"))
    {
        char *taskName = pvPortMalloc(MAX_TASK_NAME_LENGTH * sizeof(char));
        strcpy(taskName, strtok(NULL, " "));
        char *taskFunction;
        taskFunction = strtok(NULL, " ");
        TickType_t arrival;
        TickType_t period;
        TickType_t duration;
        char *taskParam = pvPortMalloc(MAX_TASK_NAME_LENGTH * sizeof(char));
        strcpy(taskParam, strtok(NULL, " "));
        token = strtok(NULL, " ");
        arrival = atoi(token) + xTaskGetTickCount();
        token = strtok(NULL, " ");
        period = atoi(token);
        token = strtok(NULL, " ");
        duration = atoi(token);

        if(strcmp(taskFunction, "word")){
            xTaskCreatePeriodic(taskAperiodic, taskName, 120, taskParam, APERIODIC_TASK_PRIORITY, NULL, arrival, period, duration);
        }else{
            xTaskCreatePeriodic(taskAperiodicNumber, taskName, 120, taskParam, APERIODIC_TASK_PRIORITY, NULL, arrival, period, duration);
        }
    }
    else if (strCmp(token, "server"))
    {
        token = strtok(NULL, " ");
        TickType_t capacity = atoi(token);

        token = strtok(NULL, " ");
        TickType_t period = atoi(token);

        initialiseServer(capacity, period);
    }
    else if (strCmp(token, "capacity"))
    {

        char *k = strtok(NULL, " ");

        char *temp = strtok(k, "-");

        BaseType_t counter = 0;
        BaseType_t i;

        while (temp != NULL)
        {

            TickType_t arrival = atoi(temp);
            temp = strtok(NULL, "-");
            TickType_t period = atoi(temp);
            temp = strtok(NULL, "-");
            TickType_t duration = atoi(temp);
            temp = strtok(NULL, "-");

            taskParameters[counter].arrival = arrival;
            taskParameters[counter].period = period;
            taskParameters[counter].duration = duration;

            counter++;
        }

        for (i = 0; i < counter; i++)
        {
            print_number(taskParameters[i].arrival);
            print_string(" - ");
            print_number(taskParameters[i].period);
            print_string(" - ");
            print_number(taskParameters[i].duration);
            print_string("\n");

            //Remove from list;
            taskParameters[i].duration = 0;
        }
    }
    else if (strCmp(token, "batch"))
    {
        char *k = strtok(NULL, " ");

        char *temp = strtok(k, "-");

        double sum = 0;

        BaseType_t counter = 0;
        BaseType_t i;

        while (temp != NULL)
        {
            if (counter == MAX_TASKS_INPUT)
                break;

            if (strcmp(temp, "periodic") == 0)
            {
                taskParameters[counter].taskType = PERIODIC_TASK_PRIORITY;
            }
            else
            {
                taskParameters[counter].taskType = APERIODIC_TASK_PRIORITY;
            }

            temp = strtok(NULL, "-");
            strcpy(taskParameters[counter].taskName, temp);
            temp = strtok(NULL, "-");
            TickType_t arrival = atoi(temp) + xTaskGetTickCount();
            temp = strtok(NULL, "-");
            TickType_t period = atoi(temp);
            temp = strtok(NULL, "-");
            TickType_t duration = atoi(temp);
            temp = strtok(NULL, "-");

            taskParameters[counter].arrival = arrival;
            taskParameters[counter].period = period;
            taskParameters[counter].duration = duration;
            taskParameters[counter].create = 1;

            if (taskParameters[i].taskType == PERIODIC_TASK_PRIORITY)
            {
                sum += taskParameters[i].duration / (double)taskParameters[i].period;
            }

            counter++;
        }

        double x = counter * (pow(2, 1 / (double)counter) - 1);

        if (sum > x)
        {
            print_string("Batch is not schedulable");

            for (i = 0; i < counter; i++)
            {
                taskParameters[i].duration = 0;
                taskParameters[i].create = 0;
            }

            return;
        }

        for (i = 0; i < counter; i++)
        {

            if (taskParameters[i].taskType == PERIODIC_TASK_PRIORITY)
            {
                xTaskCreatePeriodic(taskPeriodic, taskParameters[i].taskName, 100, taskParameters[i].taskName, PERIODIC_TASK_PRIORITY, NULL, taskParameters[i].arrival, taskParameters[i].period, taskParameters[i].duration);
            }
            // print_number(taskParameters[i].taskType);
            // print_string(" - ");
            // print_string(taskParameters[i].taskName);
            // print_string(" - ");
            // print_number(taskParameters[i].arrival);
            // print_string(" - ");
            // print_number(taskParameters[i].period);
            // print_string(" - ");
            // print_number(taskParameters[i].duration);
            // print_string("\n");
        }
    }
}

#if (INCLUDE_vTaskDelayUntil == 1)

void vTaskDelayUntil(TickType_t *const pxPreviousWakeTime, const TickType_t xTimeIncrement)
{
    TickType_t xTimeToWake;
    BaseType_t xAlreadyYielded, xShouldDelay = pdFALSE;

    configASSERT(pxPreviousWakeTime);
    configASSERT((xTimeIncrement > 0U));
    configASSERT(uxSchedulerSuspended == 0);

    vTaskSuspendAll();
    {
        /* Minor optimisation.  The tick count cannot change in this
            block. */
        const TickType_t xConstTickCount = xTickCount;

        /* Generate the tick time at which the task wants to wake. */
        xTimeToWake = *pxPreviousWakeTime + xTimeIncrement;

        if (xConstTickCount < *pxPreviousWakeTime)
        {
            /* The tick count has overflowed since this function was
                lasted called.  In this case the only time we should ever
                actually delay is if the wake time has also    overflowed,
                and the wake time is greater than the tick time.  When this
                is the case it is as if neither time had overflowed. */
            if ((xTimeToWake < *pxPreviousWakeTime) && (xTimeToWake > xConstTickCount))
            {
                xShouldDelay = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            /* The tick time has not overflowed.  In this case we will
                delay if either the wake time has overflowed, and/or the
                tick time is less than the wake time. */
            if ((xTimeToWake < *pxPreviousWakeTime) || (xTimeToWake > xConstTickCount))
            {
                xShouldDelay = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }

        /* Update the wake time ready for the next call. */
        *pxPreviousWakeTime = xTimeToWake;

        if (xShouldDelay != pdFALSE)
        {
            traceTASK_DELAY_UNTIL(xTimeToWake);

            /* prvAddCurrentTaskToDelayedList() needs the block time, not
                the time to wake, so subtract the current tick count. */
            prvAddCurrentTaskToDelayedList(xTimeToWake - xConstTickCount, pdFALSE);
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    xAlreadyYielded = xTaskResumeAll();

    /* Force a reschedule if xTaskResumeAll has not already done so, we may
        have put ourselves to sleep. */
    if (xAlreadyYielded == pdFALSE)
    {
        portYIELD_WITHIN_API();
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}

#endif /* INCLUDE_vTaskDelayUntil */
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskDelay == 1)

void vTaskDelay(const TickType_t xTicksToDelay)
{
    BaseType_t xAlreadyYielded = pdFALSE;

    /* A delay time of zero just forces a reschedule. */
    if (xTicksToDelay > (TickType_t)0U)
    {
        configASSERT(uxSchedulerSuspended == 0);
        vTaskSuspendAll();
        {
            traceTASK_DELAY();

            /* A task that is removed from the event list while the
                scheduler is suspended will not get placed in the ready
                list or removed from the blocked list until the scheduler
                is resumed.

                This task cannot be in an event list as it is the currently
                executing task. */
            prvAddCurrentTaskToDelayedList(xTicksToDelay, pdFALSE);
        }
        xAlreadyYielded = xTaskResumeAll();
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    /* Force a reschedule if xTaskResumeAll has not already done so, we may
        have put ourselves to sleep. */
    if (xAlreadyYielded == pdFALSE)
    {
        portYIELD_WITHIN_API();
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}

#endif /* INCLUDE_vTaskDelay */
/*-----------------------------------------------------------*/

#if (INCLUDE_uxTaskPriorityGet == 1)

UBaseType_t uxTaskPriorityGet(const TaskHandle_t xTask)
{
    TCB_t const *pxTCB;
    UBaseType_t uxReturn;

    taskENTER_CRITICAL();
    {
        /* If null is passed in here then it is the priority of the task
            that called uxTaskPriorityGet() that is being queried. */
        pxTCB = prvGetTCBFromHandle(xTask);
        uxReturn = pxTCB->uxPriority;
    }
    taskEXIT_CRITICAL();

    return uxReturn;
}

#endif /* INCLUDE_uxTaskPriorityGet */
/*-----------------------------------------------------------*/

#if (INCLUDE_uxTaskPriorityGet == 1)

UBaseType_t uxTaskPriorityGetFromISR(const TaskHandle_t xTask)
{
    TCB_t const *pxTCB;
    UBaseType_t uxReturn, uxSavedInterruptState;

    /* RTOS ports that support interrupt nesting have the concept of a
        maximum    system call (or maximum API call) interrupt priority.
        Interrupts that are    above the maximum system call priority are keep
        permanently enabled, even when the RTOS kernel is in a critical section,
        but cannot make any calls to FreeRTOS API functions.  If configASSERT()
        is defined in FreeRTOSConfig.h then
        portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
        failure if a FreeRTOS API function is called from an interrupt that has
        been assigned a priority above the configured maximum system call
        priority.  Only FreeRTOS functions that end in FromISR can be called
        from interrupts    that have been assigned a priority at or (logically)
        below the maximum system call interrupt priority.  FreeRTOS maintains a
        separate interrupt safe API to ensure interrupt entry is as fast and as
        simple as possible.  More information (albeit Cortex-M specific) is
        provided on the following link:
        https://www.freertos.org/RTOS-Cortex-M3-M4.html */
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

    uxSavedInterruptState = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        /* If null is passed in here then it is the priority of the calling
            task that is being queried. */
        pxTCB = prvGetTCBFromHandle(xTask);
        uxReturn = pxTCB->uxPriority;
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptState);

    return uxReturn;
}

#endif /* INCLUDE_uxTaskPriorityGet */
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskPrioritySet == 1)

void vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority)
{
    TCB_t *pxTCB;
    UBaseType_t uxCurrentBasePriority, uxPriorityUsedOnEntry;
    BaseType_t xYieldRequired = pdFALSE;

    configASSERT((uxNewPriority < configMAX_PRIORITIES));

    /* Ensure the new priority is valid. */
    if (uxNewPriority >= (UBaseType_t)configMAX_PRIORITIES)
    {
        uxNewPriority = (UBaseType_t)configMAX_PRIORITIES - (UBaseType_t)1U;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    taskENTER_CRITICAL();
    {
        /* If null is passed in here then it is the priority of the calling
            task that is being changed. */
        pxTCB = prvGetTCBFromHandle(xTask);

        traceTASK_PRIORITY_SET(pxTCB, uxNewPriority);

#if (configUSE_MUTEXES == 1)
        {
            uxCurrentBasePriority = pxTCB->uxBasePriority;
        }
#else
        {
            uxCurrentBasePriority = pxTCB->uxPriority;
        }
#endif

        if (uxCurrentBasePriority != uxNewPriority)
        {
            /* The priority change may have readied a task of higher
                priority than the calling task. */
            if (uxNewPriority > uxCurrentBasePriority)
            {
                if (pxTCB != pxCurrentTCB)
                {
                    /* The priority of a task other than the currently
                        running task is being raised.  Is the priority being
                        raised above that of the running task? */
                    if (uxNewPriority >= pxCurrentTCB->uxPriority)
                    {
                        xYieldRequired = pdTRUE;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                else
                {
                    /* The priority of the running task is being raised,
                        but the running task must already be the highest
                        priority task able to run so no yield is required. */
                }
            }
            else if (pxTCB == pxCurrentTCB)
            {
                /* Setting the priority of the running task down means
                    there may now be another task of higher priority that
                    is ready to execute. */
                xYieldRequired = pdTRUE;
            }
            else
            {
                /* Setting the priority of any other task down does not
                    require a yield as the running task must be above the
                    new priority of the task being modified. */
            }

            /* Remember the ready list the task might be referenced from
                before its uxPriority member is changed so the
                taskRESET_READY_PRIORITY() macro can function correctly. */
            uxPriorityUsedOnEntry = pxTCB->uxPriority;

#if (configUSE_MUTEXES == 1)
            {
                /* Only change the priority being used if the task is not
                    currently using an inherited priority. */
                if (pxTCB->uxBasePriority == pxTCB->uxPriority)
                {
                    pxTCB->uxPriority = uxNewPriority;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                /* The base priority gets set whatever. */
                pxTCB->uxBasePriority = uxNewPriority;
            }
#else
            {
                pxTCB->uxPriority = uxNewPriority;
            }
#endif

            /* Only reset the event list item value if the value is not
                being used for anything else. */
            if ((listGET_LIST_ITEM_VALUE(&(pxTCB->xEventListItem)) & taskEVENT_LIST_ITEM_VALUE_IN_USE) == 0UL)
            {
                listSET_LIST_ITEM_VALUE(&(pxTCB->xEventListItem), ((TickType_t)configMAX_PRIORITIES - (TickType_t)uxNewPriority)); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            /* If the task is in the blocked or suspended list we need do
                nothing more than change its priority variable. However, if
                the task is in a ready list it needs to be removed and placed
                in the list appropriate to its new priority. */
            if (listIS_CONTAINED_WITHIN(&(pxReadyTasksLists[uxPriorityUsedOnEntry]), &(pxTCB->xStateListItem)) != pdFALSE)
            {
                /* The task is currently in its ready list - remove before
                    adding it to it's new ready list.  As we are in a critical
                    section we can do this even if the scheduler is suspended. */
                if (uxListRemove(&(pxTCB->xStateListItem)) == (UBaseType_t)0)
                {
                    /* It is known that the task is in its ready list so
                        there is no need to check again and the port level
                        reset macro can be called directly. */
                    portRESET_READY_PRIORITY(uxPriorityUsedOnEntry, uxTopReadyPriority);
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
                prvAddTaskToReadyList(pxTCB);
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            if (xYieldRequired != pdFALSE)
            {
                taskYIELD_IF_USING_PREEMPTION();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            /* Remove compiler warning about unused variables when the port
                optimised task selection is not being used. */
            (void)uxPriorityUsedOnEntry;
        }
    }
    taskEXIT_CRITICAL();
}

#endif /* INCLUDE_vTaskPrioritySet */
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskSuspend == 1)

void vTaskSuspend(TaskHandle_t xTaskToSuspend)
{
    TCB_t *pxTCB;

    taskENTER_CRITICAL();
    {
        /* If null is passed in here then it is the running task that is
            being suspended. */
        pxTCB = prvGetTCBFromHandle(xTaskToSuspend);

        traceTASK_SUSPEND(pxTCB);

        /* Remove task from the ready/delayed list and place in the
            suspended list. */
        if (uxListRemove(&(pxTCB->xStateListItem)) == (UBaseType_t)0)
        {
            taskRESET_READY_PRIORITY(pxTCB->uxPriority);
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Is the task waiting on an event also? */
        if (listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) != NULL)
        {
            (void)uxListRemove(&(pxTCB->xEventListItem));
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        vListInsertEnd(&xSuspendedTaskList, &(pxTCB->xStateListItem));

#if (configUSE_TASK_NOTIFICATIONS == 1)
        {
            if (pxTCB->ucNotifyState == taskWAITING_NOTIFICATION)
            {
                /* The task was blocked to wait for a notification, but is
                    now suspended, so no notification was received. */
                pxTCB->ucNotifyState = taskNOT_WAITING_NOTIFICATION;
            }
        }
#endif
    }
    taskEXIT_CRITICAL();

    if (xSchedulerRunning != pdFALSE)
    {
        /* Reset the next expected unblock time in case it referred to the
            task that is now in the Suspended state. */
        taskENTER_CRITICAL();
        {
            prvResetNextTaskUnblockTime();
        }
        taskEXIT_CRITICAL();
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    if (pxTCB == pxCurrentTCB)
    {
        if (xSchedulerRunning != pdFALSE)
        {
            /* The current task has just been suspended. */
            configASSERT(uxSchedulerSuspended == 0);
            portYIELD_WITHIN_API();
        }
        else
        {
            /* The scheduler is not running, but the task that was pointed
                to by pxCurrentTCB has just been suspended and pxCurrentTCB
                must be adjusted to point to a different task. */
            if (listCURRENT_LIST_LENGTH(&xSuspendedTaskList) == uxCurrentNumberOfTasks) /*lint !e931 Right has no side effect, just volatile. */
            {
                /* No other tasks are ready, so set pxCurrentTCB back to
                    NULL so when the next task is created pxCurrentTCB will
                    be set to point to it no matter what its relative priority
                    is. */
                pxCurrentTCB = NULL;
            }
            else
            {
                vTaskSwitchContext();
            }
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}

#endif /* INCLUDE_vTaskSuspend */
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskSuspend == 1)

static BaseType_t prvTaskIsTaskSuspended(const TaskHandle_t xTask)
{
    BaseType_t xReturn = pdFALSE;
    const TCB_t *const pxTCB = xTask;

    /* Accesses xPendingReadyList so must be called from a critical
        section. */

    /* It does not make sense to check if the calling task is suspended. */
    configASSERT(xTask);

    /* Is the task being resumed actually in the suspended list? */
    if (listIS_CONTAINED_WITHIN(&xSuspendedTaskList, &(pxTCB->xStateListItem)) != pdFALSE)
    {
        /* Has the task already been resumed from within an ISR? */
        if (listIS_CONTAINED_WITHIN(&xPendingReadyList, &(pxTCB->xEventListItem)) == pdFALSE)
        {
            /* Is it in the suspended list because it is in the    Suspended
                state, or because is is blocked with no timeout? */
            if (listIS_CONTAINED_WITHIN(NULL, &(pxTCB->xEventListItem)) != pdFALSE) /*lint !e961.  The cast is only redundant when NULL is used. */
            {
                xReturn = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    return xReturn;
} /*lint !e818 xTask cannot be a pointer to const because it is a typedef. */

#endif /* INCLUDE_vTaskSuspend */
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskSuspend == 1)

void vTaskResume(TaskHandle_t xTaskToResume)
{
    TCB_t *const pxTCB = xTaskToResume;

    /* It does not make sense to resume the calling task. */
    configASSERT(xTaskToResume);

    /* The parameter cannot be NULL as it is impossible to resume the
        currently executing task. */
    if ((pxTCB != pxCurrentTCB) && (pxTCB != NULL))
    {
        taskENTER_CRITICAL();
        {
            if (prvTaskIsTaskSuspended(pxTCB) != pdFALSE)
            {
                traceTASK_RESUME(pxTCB);

                /* The ready list can be accessed even if the scheduler is
                    suspended because this is inside a critical section. */
                (void)uxListRemove(&(pxTCB->xStateListItem));
                prvAddTaskToReadyList(pxTCB);

                /* A higher priority task may have just been resumed. */
                if (pxTCB->uxPriority >= pxCurrentTCB->uxPriority)
                {
                    /* This yield may not cause the task just resumed to run,
                        but will leave the lists in the correct state for the
                        next yield. */
                    taskYIELD_IF_USING_PREEMPTION();
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        taskEXIT_CRITICAL();
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}

#endif /* INCLUDE_vTaskSuspend */

/*-----------------------------------------------------------*/

#if ((INCLUDE_xTaskResumeFromISR == 1) && (INCLUDE_vTaskSuspend == 1))

BaseType_t xTaskResumeFromISR(TaskHandle_t xTaskToResume)
{
    BaseType_t xYieldRequired = pdFALSE;
    TCB_t *const pxTCB = xTaskToResume;
    UBaseType_t uxSavedInterruptStatus;

    configASSERT(xTaskToResume);

    /* RTOS ports that support interrupt nesting have the concept of a
        maximum    system call (or maximum API call) interrupt priority.
        Interrupts that are    above the maximum system call priority are keep
        permanently enabled, even when the RTOS kernel is in a critical section,
        but cannot make any calls to FreeRTOS API functions.  If configASSERT()
        is defined in FreeRTOSConfig.h then
        portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
        failure if a FreeRTOS API function is called from an interrupt that has
        been assigned a priority above the configured maximum system call
        priority.  Only FreeRTOS functions that end in FromISR can be called
        from interrupts    that have been assigned a priority at or (logically)
        below the maximum system call interrupt priority.  FreeRTOS maintains a
        separate interrupt safe API to ensure interrupt entry is as fast and as
        simple as possible.  More information (albeit Cortex-M specific) is
        provided on the following link:
        https://www.freertos.org/RTOS-Cortex-M3-M4.html */
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if (prvTaskIsTaskSuspended(pxTCB) != pdFALSE)
        {
            traceTASK_RESUME_FROM_ISR(pxTCB);

            /* Check the ready lists can be accessed. */
            if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
            {
                /* Ready lists can be accessed so move the task from the
                    suspended list to the ready list directly. */
                if (pxTCB->uxPriority >= pxCurrentTCB->uxPriority)
                {
                    xYieldRequired = pdTRUE;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                (void)uxListRemove(&(pxTCB->xStateListItem));
                prvAddTaskToReadyList(pxTCB);
            }
            else
            {
                /* The delayed or ready lists cannot be accessed so the task
                    is held in the pending ready list until the scheduler is
                    unsuspended. */
                vListInsertEnd(&(xPendingReadyList), &(pxTCB->xEventListItem));
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

    return xYieldRequired;
}

#endif /* ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) ) */
/*-----------------------------------------------------------*/

void vTaskStartScheduler(void)
{
    BaseType_t xReturn;

/* Add the idle task at the lowest priority. */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    {
        StaticTask_t *pxIdleTaskTCBBuffer = NULL;
        StackType_t *pxIdleTaskStackBuffer = NULL;
        configSTACK_DEPTH_TYPE ulIdleTaskStackSize;

        /* The Idle task is created using user provided RAM - obtain the
        address of the RAM then create the idle task. */
        vApplicationGetIdleTaskMemory(&pxIdleTaskTCBBuffer, &pxIdleTaskStackBuffer, &ulIdleTaskStackSize);
        xIdleTaskHandle = xTaskCreateStatic(prvIdleTask,
                                            configIDLE_TASK_NAME,
                                            ulIdleTaskStackSize,
                                            (void *)NULL,      /*lint !e961.  The cast is not redundant for all compilers. */
                                            portPRIVILEGE_BIT, /* In effect ( tskIDLE_PRIORITY | portPRIVILEGE_BIT ), but tskIDLE_PRIORITY is zero. */
                                            pxIdleTaskStackBuffer,
                                            pxIdleTaskTCBBuffer); /*lint !e961 MISRA exception, justified as it is not a redundant explicit cast to all supported compilers. */

        if (xIdleTaskHandle != NULL)
        {
            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }
    }
#else
    {
        /* The Idle task is being created using dynamically allocated RAM. */
        xReturn = xTaskCreate(prvIdleTask,
                              configIDLE_TASK_NAME,
                              configMINIMAL_STACK_SIZE,
                              (void *)NULL,
                              portPRIVILEGE_BIT, /* In effect ( tskIDLE_PRIORITY | portPRIVILEGE_BIT ), but tskIDLE_PRIORITY is zero. */
                              &xIdleTaskHandle); /*lint !e961 MISRA exception, justified as it is not a redundant explicit cast to all supported compilers. */
    }
#endif /* configSUPPORT_STATIC_ALLOCATION */

#if (configUSE_TIMERS == 1)
    {
        if (xReturn == pdPASS)
        {
            xReturn = xTimerCreateTimerTask();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
#endif /* configUSE_TIMERS */

    if (xReturn == pdPASS)
    {
/* freertos_tasks_c_additions_init() should only be called if the user
        definable macro FREERTOS_TASKS_C_ADDITIONS_INIT() is defined, as that is
        the only macro called by the function. */
#ifdef FREERTOS_TASKS_C_ADDITIONS_INIT
        {
            freertos_tasks_c_additions_init();
        }
#endif

        /* Interrupts are turned off here, to ensure a tick does not occur
        before or during the call to xPortStartScheduler().  The stacks of
        the created tasks contain a status word with interrupts switched on
        so interrupts will automatically get re-enabled when the first task
        starts to run. */
        portDISABLE_INTERRUPTS();

        xNextTaskUnblockTime = portMAX_DELAY;
        xSchedulerRunning = pdTRUE;
        xTickCount = (TickType_t)configINITIAL_TICK_COUNT;

        /* If configGENERATE_RUN_TIME_STATS is defined then the following
        macro must be defined to configure the timer/counter used to generate
        the run time counter time base.   NOTE:  If configGENERATE_RUN_TIME_STATS
        is set to 0 and the following line fails to build then ensure you do not
        have portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() defined in your
        FreeRTOSConfig.h file. */
        portCONFIGURE_TIMER_FOR_RUN_TIME_STATS();

        traceTASK_SWITCHED_IN();

        /* Setting up the timer tick is hardware specific and thus in the
        portable interface. */
        if (xPortStartScheduler() != pdFALSE)
        {
            /* Should not reach here as if the scheduler is running the
            function will not return. */
        }
        else
        {
            /* Should only reach here if a task calls xTaskEndScheduler(). */
        }
    }
    else
    {
        /* This line will only be reached if the kernel could not be started,
        because there was not enough FreeRTOS heap to create the idle task
        or the timer task. */
        configASSERT(xReturn != errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY);
    }

    /* Prevent compiler warnings if INCLUDE_xTaskGetIdleTaskHandle is set to 0,
    meaning xIdleTaskHandle is not used anywhere else. */
    (void)xIdleTaskHandle;
}
/*-----------------------------------------------------------*/

void vTaskEndScheduler(void)
{
    /* Stop the scheduler interrupts and call the portable scheduler end
    routine so the original ISRs can be restored if necessary.  The port
    layer must ensure interrupts enable    bit is left in the correct state. */
    portDISABLE_INTERRUPTS();
    xSchedulerRunning = pdFALSE;
    vPortEndScheduler();
    portENABLE_INTERRUPTS(); /* As per comment, enable interrupts. */
}
/*----------------------------------------------------------*/

void vTaskSuspendAll(void)
{
    /* A critical section is not required as the variable is of type
    BaseType_t.  Please read Richard Barry's reply in the following link to a
    post in the FreeRTOS support forum before reporting this as a bug! -
    http://goo.gl/wu4acr */
    ++uxSchedulerSuspended;
}

BaseType_t xTaskResumeAll(void)
{
    TCB_t *pxTCB = NULL;
    BaseType_t xAlreadyYielded = pdFALSE;

    /* If uxSchedulerSuspended is zero then this function does not match a
    previous call to vTaskSuspendAll(). */
    configASSERT(uxSchedulerSuspended);

    /* It is possible that an ISR caused a task to be removed from an event
    list while the scheduler was suspended.  If this was the case then the
    removed task will have been added to the xPendingReadyList.  Once the
    scheduler has been resumed it is safe to move all the pending ready
    tasks from this list into their appropriate ready list. */
    taskENTER_CRITICAL();
    {
        --uxSchedulerSuspended;

        if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
        {
            if (uxCurrentNumberOfTasks > (UBaseType_t)0U)
            {
                /* Move any readied tasks from the pending list into the
                appropriate ready list. */
                while (listLIST_IS_EMPTY(&xPendingReadyList) == pdFALSE)
                {
                    pxTCB = listGET_OWNER_OF_HEAD_ENTRY((&xPendingReadyList)); /*lint !e9079 void * is used as this macro is used with timers and co-routines too.  Alignment is known to be fine as the type of the pointer stored and retrieved is the same. */
                    (void)uxListRemove(&(pxTCB->xEventListItem));
                    (void)uxListRemove(&(pxTCB->xStateListItem));
                    prvAddTaskToReadyList(pxTCB);

                    /* If the moved task has a priority higher than the current
                    task then a yield must be performed. */
                    if (pxTCB->uxPriority >= pxCurrentTCB->uxPriority)
                    {
                        xYieldPending = pdTRUE;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }

                if (pxTCB != NULL)
                {
                    /* A task was unblocked while the scheduler was suspended,
                    which may have prevented the next unblock time from being
                    re-calculated, in which case re-calculate it now.  Mainly
                    important for low power tickless implementations, where
                    this can prevent an unnecessary exit from low power
                    state. */
                    prvResetNextTaskUnblockTime();
                }

                /* If any ticks occurred while the scheduler was suspended then
                they should be processed now.  This ensures the tick count does
                not    slip, and that any delayed tasks are resumed at the correct
                time. */
                {
                    UBaseType_t uxPendedCounts = uxPendedTicks; /* Non-volatile copy. */

                    if (uxPendedCounts > (UBaseType_t)0U)
                    {
                        do
                        {
                            if (xTaskIncrementTick() != pdFALSE)
                            {
                                xYieldPending = pdTRUE;
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                            --uxPendedCounts;
                        } while (uxPendedCounts > (UBaseType_t)0U);

                        uxPendedTicks = 0;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }

                if (xYieldPending != pdFALSE)
                {
#if (configUSE_PREEMPTION != 0)
                    {
                        xAlreadyYielded = pdTRUE;
                    }
#endif
                    taskYIELD_IF_USING_PREEMPTION();
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    taskEXIT_CRITICAL();

    return xAlreadyYielded;
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCount(void)
{
    TickType_t xTicks;

    /* Critical section required if running on a 16 bit processor. */
    portTICK_TYPE_ENTER_CRITICAL();
    {
        xTicks = xTickCount;
    }
    portTICK_TYPE_EXIT_CRITICAL();

    return xTicks;
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCountFromISR(void)
{
    TickType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;

    /* RTOS ports that support interrupt nesting have the concept of a maximum
    system call (or maximum API call) interrupt priority.  Interrupts that are
    above the maximum system call priority are kept permanently enabled, even
    when the RTOS kernel is in a critical section, but cannot make any calls to
    FreeRTOS API functions.  If configASSERT() is defined in FreeRTOSConfig.h
    then portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
    failure if a FreeRTOS API function is called from an interrupt that has been
    assigned a priority above the configured maximum system call priority.
    Only FreeRTOS functions that end in FromISR can be called from interrupts
    that have been assigned a priority at or (logically) below the maximum
    system call    interrupt priority.  FreeRTOS maintains a separate interrupt
    safe API to ensure interrupt entry is as fast and as simple as possible.
    More information (albeit Cortex-M specific) is provided on the following
    link: https://www.freertos.org/RTOS-Cortex-M3-M4.html */
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

    uxSavedInterruptStatus = portTICK_TYPE_SET_INTERRUPT_MASK_FROM_ISR();
    {
        xReturn = xTickCount;
    }
    portTICK_TYPE_CLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

    return xReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t uxTaskGetNumberOfTasks(void)
{
    /* A critical section is not required because the variables are of type
    BaseType_t. */
    return uxCurrentNumberOfTasks;
}
/*-----------------------------------------------------------*/

char *pcTaskGetName(TaskHandle_t xTaskToQuery) /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
{
    TCB_t *pxTCB;

    /* If null is passed in here then the name of the calling task is being
    queried. */
    pxTCB = prvGetTCBFromHandle(xTaskToQuery);
    configASSERT(pxTCB);
    return &(pxTCB->pcTaskName[0]);
}
/*-----------------------------------------------------------*/

BaseType_t xTaskIncrementTick(void)
{
    TCB_t *pxTCB;
    TickType_t xItemValue;
    BaseType_t xSwitchRequired = pdFALSE;

    /* Called by the portable layer each time a tick interrupt occurs.
    Increments the tick then checks to see if the new tick value will cause any
    tasks to be unblocked. */
    traceTASK_INCREMENT_TICK(xTickCount);
    if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
    {
        /* Minor optimisation.  The tick count cannot change in this
        block. */
        const TickType_t xConstTickCount = xTickCount + (TickType_t)1;

        /* Increment the RTOS tick, switching the delayed and overflowed
        delayed lists if it wraps to 0. */
        xTickCount = xConstTickCount;

        if (xConstTickCount == (TickType_t)0U) /*lint !e774 'if' does not always evaluate to false as it is looking for an overflow. */
        {
            taskSWITCH_DELAYED_LISTS();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* See if this tick has made a timeout expire.  Tasks are stored in
        the    queue in the order of their wake time - meaning once one task
        has been found whose block time has not expired there is no need to
        look any further down the list. */
        if (xConstTickCount >= xNextTaskUnblockTime)
        {
            for (;;)
            {
                if (listLIST_IS_EMPTY(pxDelayedTaskList) != pdFALSE)
                {
                    /* The delayed list is empty.  Set xNextTaskUnblockTime
                    to the maximum possible value so it is extremely
                    unlikely that the
                    if( xTickCount >= xNextTaskUnblockTime ) test will pass
                    next time through. */
                    xNextTaskUnblockTime = portMAX_DELAY; /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
                    break;
                }
                else
                {
                    /* The delayed list is not empty, get the value of the
                    item at the head of the delayed list.  This is the time
                    at which the task at the head of the delayed list must
                    be removed from the Blocked state. */
                    pxTCB = listGET_OWNER_OF_HEAD_ENTRY(pxDelayedTaskList); /*lint !e9079 void * is used as this macro is used with timers and co-routines too.  Alignment is known to be fine as the type of the pointer stored and retrieved is the same. */
                    xItemValue = listGET_LIST_ITEM_VALUE(&(pxTCB->xStateListItem));

                    if (xConstTickCount < xItemValue)
                    {
                        /* It is not time to unblock this item yet, but the
                        item value is the time at which the task at the head
                        of the blocked list must be removed from the Blocked
                        state -    so record the item value in
                        xNextTaskUnblockTime. */
                        xNextTaskUnblockTime = xItemValue;
                        break; /*lint !e9011 Code structure here is deemed easier to understand with multiple breaks. */
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* It is time to remove the item from the Blocked state. */
                    (void)uxListRemove(&(pxTCB->xStateListItem));

                    /* Is the task waiting on an event also?  If so remove
                    it from the event list. */
                    if (listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) != NULL)
                    {
                        (void)uxListRemove(&(pxTCB->xEventListItem));
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* Place the unblocked task into the appropriate ready
                    list. */
                    prvAddTaskToReadyList(pxTCB);

/* A task being unblocked cannot cause an immediate
                    context switch if preemption is turned off. */
#if (configUSE_PREEMPTION == 1)
                    {
                        /* Preemption is on, but a context switch should
                        only be performed if the unblocked task has a
                        priority that is equal to or higher than the
                        currently executing task. */
                        if (pxTCB->uxPriority >= pxCurrentTCB->uxPriority)
                        {
                            xSwitchRequired = pdTRUE;
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
#endif /* configUSE_PREEMPTION */
                }
            }
        }

/* Tasks of equal priority to the currently running task will share
        processing time (time slice) if preemption is on, and the application
        writer has not explicitly turned time slicing off. */
#if ((configUSE_PREEMPTION == 1) && (configUSE_TIME_SLICING == 1))
        {
            if (listCURRENT_LIST_LENGTH(&(pxReadyTasksLists[pxCurrentTCB->uxPriority])) > (UBaseType_t)1)
            {
                xSwitchRequired = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
#endif /* ( ( configUSE_PREEMPTION == 1 ) && ( configUSE_TIME_SLICING == 1 ) ) */
    }
    else
    {
        ++uxPendedTicks;
    }

#if (configUSE_PREEMPTION == 1)
    {
        if (xYieldPending != pdFALSE)
        {
            xSwitchRequired = pdTRUE;
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
#endif /* configUSE_PREEMPTION */

    //REFILL FUNC
    unsigned char i;

    for (i = 0; i < MAX_REFILLS; i++)
    {
        if (refills[i].refillTick == xTaskGetTickCount())
        {
            serverCapacity += refills[i].refillAmount;
            print_number(refills[i].refillAmount);
            print_string(" refilled at : ");
            print_number(xTaskGetTickCount());
            print_string("\n");
            refills[i].refillAmount = 0;
        }
    }

    for (i = 0; i < MAX_TASKS_INPUT; i++)
    {
        if (taskParameters[i].taskType == APERIODIC_TASK_PRIORITY && taskParameters[i].arrival <= xTaskGetTickCount() && taskParameters[i].duration > 0 && taskParameters[i].create == 1)
        {
            print_number(xTaskGetTickCount());
            print_string(" - create ");
            print_string(taskParameters[i].taskName);
            print_string("\n");
            taskParameters[i].create = 0;
            taskParameters[i].duration = 0;
            // xTaskCreatePeriodic(taskAperiodic, taskParameters[i].taskName, 100, "aper", APERIODIC_TASK_PRIORITY, NULL, taskParameters[i].arrival, 0, taskParameters[i].duration);
        }
    }

    return pdTRUE;
}

int strCmp(char *s1, char *s2)
{
    int i = 0;
    while (s1[i])
    {
        if (s1[i] != s2[i])
            return 0;
        i++;
    }
    return 1;
}

void deleteTask(char *taskName)
{
    List_t *periodic_list = &pxReadyTasksLists[PERIODIC_TASK_PRIORITY];

    struct xLIST_ITEM *tmp = listGET_HEAD_ENTRY(periodic_list);

    while (1)
    {
        if (tmp == listGET_END_MARKER(periodic_list))
            break;

        TCB_t *task = listGET_LIST_ITEM_OWNER(tmp);

        if (strcmp(task->pcName, taskName) == 0)
        {
            vPortFree(task->pcName);
            vTaskDelete(task);
            print_string(task->pcName);
            print_string(" - Deleted\n");
            break;
        }
        tmp = listGET_NEXT(tmp);
    }
}
unsigned char kara = 0;

void vTaskSwitchContext(void)
{
    if (uxSchedulerSuspended != (UBaseType_t)pdFALSE)
    {
        /* The scheduler is currently suspended - do not allow a context
        switch. */
        xYieldPending = pdTRUE;
    }
    else
    {
        xYieldPending = pdFALSE;
        traceTASK_SWITCHED_OUT();

        taskCHECK_FOR_STACK_OVERFLOW();

        List_t *periodic_list = &pxReadyTasksLists[PERIODIC_TASK_PRIORITY];

        struct xLIST_ITEM *tmp = listGET_HEAD_ENTRY(periodic_list);

        TickType_t min_period = INT16_MAX;

        TCB_t *min_task = xIdleTaskHandle;

        while (1)
        {
            if (tmp == listGET_END_MARKER(periodic_list))
                break;

            TCB_t *task = listGET_LIST_ITEM_OWNER(tmp);

            if (task->period <= min_period && task->arrival + task->cycle * task->period <= xTaskGetTickCount())
            {
                min_task = task;
                min_period = task->period;
            }
            tmp = listGET_NEXT(tmp);
        }

        List_t *aperiodic_list = &pxReadyTasksLists[APERIODIC_TASK_PRIORITY];

        struct xLIST_ITEM *aperiodic = listGET_HEAD_ENTRY(aperiodic_list);

        TCB_t *aperiodicTask = listGET_LIST_ITEM_OWNER(aperiodic);

        if (aperiodicTask != NULL && serverPeriod < min_period && aperiodicTask->arrival <= xTaskGetTickCount() && serverCapacity > 0 && aperiodic_list->uxNumberOfItems > 0)
        {
            if (aperiodicTask->cycle == 0)
            {
                aperiodicTask->cycle = 1;
                setRefill(aperiodicTask->duration);
            }
            min_task = aperiodicTask;
        }

        pxCurrentTCB = min_task;

        if (restartTask != NULL)
        {
            restartTask->pxTopOfStack = pxPortInitialiseStack(&(restartTask->pxStack[restartTask->stackDepth - (configSTACK_DEPTH_TYPE)1]), restartTask->taskCode, restartTask->pvParameters);

            restartTask = NULL;
        }

        traceTASK_SWITCHED_IN();
    }
}

void vTaskPlaceOnEventList(List_t *const pxEventList, const TickType_t xTicksToWait)
{
    configASSERT(pxEventList);

    vListInsert(pxEventList, &(pxCurrentTCB->xEventListItem));

    prvAddCurrentTaskToDelayedList(xTicksToWait, pdTRUE);
}

void vTaskPlaceOnUnorderedEventList(List_t *pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait)
{
    configASSERT(pxEventList);

    /* THIS FUNCTION MUST BE CALLED WITH THE SCHEDULER SUSPENDED.  It is used by
    the event groups implementation. */
    configASSERT(uxSchedulerSuspended != 0);

    /* Store the item value in the event list item.  It is safe to access the
    event list item here as interrupts won't access the event list item of a
    task that is not in the Blocked state. */
    listSET_LIST_ITEM_VALUE(&(pxCurrentTCB->xEventListItem), xItemValue | taskEVENT_LIST_ITEM_VALUE_IN_USE);

    /* Place the event list item of the TCB at the end of the appropriate event
    list.  It is safe to access the event list here because it is part of an
    event group implementation - and interrupts don't access event groups
    directly (instead they access them indirectly by pending function calls to
    the task level). */
    vListInsertEnd(pxEventList, &(pxCurrentTCB->xEventListItem));

    prvAddCurrentTaskToDelayedList(xTicksToWait, pdTRUE);
}
/*-----------------------------------------------------------*/

#if (configUSE_TIMERS == 1)

void vTaskPlaceOnEventListRestricted(List_t *const pxEventList, TickType_t xTicksToWait, const BaseType_t xWaitIndefinitely)
{
    configASSERT(pxEventList);

    /* This function should not be called by application code hence the
        'Restricted' in its name.  It is not part of the public API.  It is
        designed for use by kernel code, and has special calling requirements -
        it should be called with the scheduler suspended. */

    /* Place the event list item of the TCB in the appropriate event list.
        In this case it is assume that this is the only task that is going to
        be waiting on this event list, so the faster vListInsertEnd() function
        can be used in place of vListInsert. */
    vListInsertEnd(pxEventList, &(pxCurrentTCB->xEventListItem));

    /* If the task should block indefinitely then set the block time to a
        value that will be recognised as an indefinite delay inside the
        prvAddCurrentTaskToDelayedList() function. */
    if (xWaitIndefinitely != pdFALSE)
    {
        xTicksToWait = portMAX_DELAY;
    }

    traceTASK_DELAY_UNTIL((xTickCount + xTicksToWait));
    prvAddCurrentTaskToDelayedList(xTicksToWait, xWaitIndefinitely);
}

#endif /* configUSE_TIMERS */

BaseType_t xTaskRemoveFromEventList(const List_t *const pxEventList)
{
    TCB_t *pxUnblockedTCB;
    BaseType_t xReturn;

    /* THIS FUNCTION MUST BE CALLED FROM A CRITICAL SECTION.  It can also be
    called from a critical section within an ISR. */

    /* The event list is sorted in priority order, so the first in the list can
    be removed as it is known to be the highest priority.  Remove the TCB from
    the delayed list, and add it to the ready list.

    If an event is for a queue that is locked then this function will never
    get called - the lock count on the queue will get modified instead.  This
    means exclusive access to the event list is guaranteed here.

    This function assumes that a check has already been made to ensure that
    pxEventList is not empty. */
    pxUnblockedTCB = listGET_OWNER_OF_HEAD_ENTRY(pxEventList); /*lint !e9079 void * is used as this macro is used with timers and co-routines too.  Alignment is known to be fine as the type of the pointer stored and retrieved is the same. */
    configASSERT(pxUnblockedTCB);
    (void)uxListRemove(&(pxUnblockedTCB->xEventListItem));

    if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
    {
        (void)uxListRemove(&(pxUnblockedTCB->xStateListItem));
        prvAddTaskToReadyList(pxUnblockedTCB);
    }
    else
    {
        /* The delayed and ready lists cannot be accessed, so hold this task
        pending until the scheduler is resumed. */
        vListInsertEnd(&(xPendingReadyList), &(pxUnblockedTCB->xEventListItem));
    }

    if (pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority)
    {
        /* Return true if the task removed from the event list has a higher
        priority than the calling task.  This allows the calling task to know if
        it should force a context switch now. */
        xReturn = pdTRUE;

        /* Mark that a yield is pending in case the user is not using the
        "xHigherPriorityTaskWoken" parameter to an ISR safe FreeRTOS function. */
        xYieldPending = pdTRUE;
    }
    else
    {
        xReturn = pdFALSE;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

void vTaskRemoveFromUnorderedEventList(ListItem_t *pxEventListItem, const TickType_t xItemValue)
{
    TCB_t *pxUnblockedTCB;

    /* THIS FUNCTION MUST BE CALLED WITH THE SCHEDULER SUSPENDED.  It is used by
    the event flags implementation. */
    configASSERT(uxSchedulerSuspended != pdFALSE);

    /* Store the new item value in the event list. */
    listSET_LIST_ITEM_VALUE(pxEventListItem, xItemValue | taskEVENT_LIST_ITEM_VALUE_IN_USE);

    /* Remove the event list form the event flag.  Interrupts do not access
    event flags. */
    pxUnblockedTCB = listGET_LIST_ITEM_OWNER(pxEventListItem); /*lint !e9079 void * is used as this macro is used with timers and co-routines too.  Alignment is known to be fine as the type of the pointer stored and retrieved is the same. */
    configASSERT(pxUnblockedTCB);
    (void)uxListRemove(pxEventListItem);

    /* Remove the task from the delayed list and add it to the ready list.  The
    scheduler is suspended so interrupts will not be accessing the ready
    lists. */
    (void)uxListRemove(&(pxUnblockedTCB->xStateListItem));
    prvAddTaskToReadyList(pxUnblockedTCB);

    if (pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority)
    {
        /* The unblocked task has a priority above that of the calling task, so
        a context switch is required.  This function is called with the
        scheduler suspended so xYieldPending is set so the context switch
        occurs immediately that the scheduler is resumed (unsuspended). */
        xYieldPending = pdTRUE;
    }
}
/*-----------------------------------------------------------*/

void vTaskSetTimeOutState(TimeOut_t *const pxTimeOut)
{
    configASSERT(pxTimeOut);
    taskENTER_CRITICAL();
    {
        pxTimeOut->xOverflowCount = xNumOfOverflows;
        pxTimeOut->xTimeOnEntering = xTickCount;
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vTaskInternalSetTimeOutState(TimeOut_t *const pxTimeOut)
{
    /* For internal use only as it does not use a critical section. */
    pxTimeOut->xOverflowCount = xNumOfOverflows;
    pxTimeOut->xTimeOnEntering = xTickCount;
}
/*-----------------------------------------------------------*/

BaseType_t xTaskCheckForTimeOut(TimeOut_t *const pxTimeOut, TickType_t *const pxTicksToWait)
{
    BaseType_t xReturn;

    configASSERT(pxTimeOut);
    configASSERT(pxTicksToWait);

    taskENTER_CRITICAL();
    {
        /* Minor optimisation.  The tick count cannot change in this block. */
        const TickType_t xConstTickCount = xTickCount;
        const TickType_t xElapsedTime = xConstTickCount - pxTimeOut->xTimeOnEntering;

#if (INCLUDE_vTaskSuspend == 1)
        if (*pxTicksToWait == portMAX_DELAY)
        {
            /* If INCLUDE_vTaskSuspend is set to 1 and the block time
                specified is the maximum block time then the task should block
                indefinitely, and therefore never time out. */
            xReturn = pdFALSE;
        }
        else
#endif

            if ((xNumOfOverflows != pxTimeOut->xOverflowCount) && (xConstTickCount >= pxTimeOut->xTimeOnEntering)) /*lint !e525 Indentation preferred as is to make code within pre-processor directives clearer. */
        {
            /* The tick count is greater than the time at which
            vTaskSetTimeout() was called, but has also overflowed since
            vTaskSetTimeOut() was called.  It must have wrapped all the way
            around and gone past again. This passed since vTaskSetTimeout()
            was called. */
            xReturn = pdTRUE;
        }
        else if (xElapsedTime < *pxTicksToWait) /*lint !e961 Explicit casting is only redundant with some compilers, whereas others require it to prevent integer conversion errors. */
        {
            /* Not a genuine timeout. Adjust parameters for time remaining. */
            *pxTicksToWait -= xElapsedTime;
            vTaskInternalSetTimeOutState(pxTimeOut);
            xReturn = pdFALSE;
        }
        else
        {
            *pxTicksToWait = 0;
            xReturn = pdTRUE;
        }
    }
    taskEXIT_CRITICAL();

    return xReturn;
}
/*-----------------------------------------------------------*/

void vTaskMissedYield(void)
{
    xYieldPending = pdTRUE;
}

/*
 * -----------------------------------------------------------
 * The Idle task.
 * ----------------------------------------------------------
 *
 * The portTASK_FUNCTION() macro is used to allow port/compiler specific
 * language extensions.  The equivalent prototype for this function is:
 *
 * void prvIdleTask( void *pvParameters );
 *
 */
static portTASK_FUNCTION(prvIdleTask, pvParameters)
{
    /* Stop warnings. */
    (void)pvParameters;

    /** THIS IS THE RTOS IDLE TASK - WHICH IS CREATED AUTOMATICALLY WHEN THE
    SCHEDULER IS STARTED. **/

    /* In case a task that has a secure context deletes itself, in which case
    the idle task is responsible for deleting the task's secure context, if
    any. */
    portALLOCATE_SECURE_CONTEXT(configMINIMAL_SECURE_STACK_SIZE);

    for (;;)
    {
        /* See if any tasks have deleted themselves - if so then the idle task
        is responsible for freeing the deleted task's TCB and stack. */
        prvCheckTasksWaitingTermination();

#if ((configUSE_PREEMPTION == 1) && (configIDLE_SHOULD_YIELD == 1))
        {
            /* When using preemption tasks of equal priority will be
            timesliced.  If a task that is sharing the idle priority is ready
            to run then the idle task should yield before the end of the
            timeslice.

            A critical region is not required here as we are just reading from
            the list, and an occasional incorrect value will not matter.  If
            the ready list at the idle priority contains more than one task
            then a task other than the idle task is ready to execute. */
            if (listCURRENT_LIST_LENGTH(&(pxReadyTasksLists[tskIDLE_PRIORITY])) > (UBaseType_t)1)
            {
                taskYIELD();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
#endif /* ( ( configUSE_PREEMPTION == 1 ) && ( configIDLE_SHOULD_YIELD == 1 ) ) */

#if (configUSE_IDLE_HOOK == 1)
        {
            extern void vApplicationIdleHook(void);

            /* Call the user defined function from within the idle task.  This
            allows the application designer to add background functionality
            without the overhead of a separate task.
            NOTE: vApplicationIdleHook() MUST NOT, UNDER ANY CIRCUMSTANCES,
            CALL A FUNCTION THAT MIGHT BLOCK. */
            vApplicationIdleHook();
        }
#endif /* configUSE_IDLE_HOOK */
    }
}

static void prvInitialiseTaskLists(void)
{
    UBaseType_t uxPriority;

    for (uxPriority = (UBaseType_t)0U; uxPriority < (UBaseType_t)configMAX_PRIORITIES; uxPriority++)
    {
        vListInitialise(&(pxReadyTasksLists[uxPriority]));
    }

    vListInitialise(&xDelayedTaskList1);
    vListInitialise(&xDelayedTaskList2);
    vListInitialise(&xPendingReadyList);

#if (INCLUDE_vTaskDelete == 1)
    {
        vListInitialise(&xTasksWaitingTermination);
    }
#endif /* INCLUDE_vTaskDelete */

#if (INCLUDE_vTaskSuspend == 1)
    {
        vListInitialise(&xSuspendedTaskList);
    }
#endif /* INCLUDE_vTaskSuspend */

    /* Start with pxDelayedTaskList using list1 and the pxOverflowDelayedTaskList
    using list2. */
    pxDelayedTaskList = &xDelayedTaskList1;
    pxOverflowDelayedTaskList = &xDelayedTaskList2;
}
/*-----------------------------------------------------------*/

static void prvCheckTasksWaitingTermination(void)
{

    /** THIS FUNCTION IS CALLED FROM THE RTOS IDLE TASK **/

#if (INCLUDE_vTaskDelete == 1)
    {
        TCB_t *pxTCB;

        /* uxDeletedTasksWaitingCleanUp is used to prevent taskENTER_CRITICAL()
        being called too often in the idle task. */
        while (uxDeletedTasksWaitingCleanUp > (UBaseType_t)0U)
        {
            taskENTER_CRITICAL();
            {
                pxTCB = listGET_OWNER_OF_HEAD_ENTRY((&xTasksWaitingTermination)); /*lint !e9079 void * is used as this macro is used with timers and co-routines too.  Alignment is known to be fine as the type of the pointer stored and retrieved is the same. */
                (void)uxListRemove(&(pxTCB->xStateListItem));
                --uxCurrentNumberOfTasks;
                --uxDeletedTasksWaitingCleanUp;
            }
            taskEXIT_CRITICAL();

            prvDeleteTCB(pxTCB);
        }
    }
#endif /* INCLUDE_vTaskDelete */
}

#if ((configUSE_TRACE_FACILITY == 1) || (INCLUDE_uxTaskGetStackHighWaterMark == 1))

static configSTACK_DEPTH_TYPE prvTaskCheckFreeStackSpace(const uint8_t *pucStackByte)
{
    uint32_t ulCount = 0U;

    while (*pucStackByte == (uint8_t)tskSTACK_FILL_BYTE)
    {
        pucStackByte -= portSTACK_GROWTH;
        ulCount++;
    }

    ulCount /= (uint32_t)sizeof(StackType_t); /*lint !e961 Casting is not redundant on smaller architectures. */

    return (configSTACK_DEPTH_TYPE)ulCount;
}

#endif /* ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) ) */
/*-----------------------------------------------------------*/

#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)

/* uxTaskGetStackHighWaterMark() under configENABLE_BACKWARD_COMPATIBILITY
     * differs only in its return type.  Using configSTACK_DEPTH_TYPE allows the
     * user to determine the return type.  It gets around the problem of the value
     * overflowing on 8-bit types without breaking backward compatibility for
     * applications that expect an 8-bit return type. */

#if configENABLE_BACKWARD_COMPATIBILITY == 1

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask)
{
    TCB_t *pxTCB;
    uint8_t *pucEndOfStack;
    UBaseType_t uxReturn;

    pxTCB = prvGetTCBFromHandle(xTask);

#if portSTACK_GROWTH < 0
    {
        pucEndOfStack = (uint8_t *)pxTCB->pxStack;
    }
#else
    {
        pucEndOfStack = (uint8_t *)pxTCB->pxEndOfStack;
    }
#endif

    uxReturn = (UBaseType_t)prvTaskCheckFreeStackSpace(pucEndOfStack);

    return uxReturn;
}

#else

configSTACK_DEPTH_TYPE uxTaskGetStackHighWaterMark(TaskHandle_t xTask)
{
    TCB_t *pxTCB;
    uint8_t *pucEndOfStack;
    configSTACK_DEPTH_TYPE uxReturn;

    pxTCB = prvGetTCBFromHandle(xTask);

#if portSTACK_GROWTH < 0
    {
        pucEndOfStack = (uint8_t *)pxTCB->pxStack;
    }
#else
    {
        pucEndOfStack = (uint8_t *)pxTCB->pxEndOfStack;
    }
#endif

    uxReturn = prvTaskCheckFreeStackSpace(pucEndOfStack);

    return uxReturn;
}

#endif /* configENABLE_BACKWARD_COMPATIBILITY */

#endif /* INCLUDE_uxTaskGetStackHighWaterMark */
/*-----------------------------------------------------------*/

#if (INCLUDE_vTaskDelete == 1)

static void prvDeleteTCB(TCB_t *pxTCB)
{
    /* This call is required specifically for the TriCore port.  It must be
        above the vPortFree() calls.  The call is also used by ports/demos that
        want to allocate and clean RAM statically. */
    portCLEAN_UP_TCB(pxTCB);

/* Free up the memory allocated by the scheduler for the task.  It is up
        to the task to free any memory allocated at the application level. */
#if (configUSE_NEWLIB_REENTRANT == 1)
    {
        _reclaim_reent(&(pxTCB->xNewLib_reent));
    }
#endif /* configUSE_NEWLIB_REENTRANT */

#if ((configSUPPORT_DYNAMIC_ALLOCATION == 1) && (configSUPPORT_STATIC_ALLOCATION == 0) && (portUSING_MPU_WRAPPERS == 0))
    {
        /* The task can only have been allocated dynamically - free both
            the stack and TCB. */
        vPortFree(pxTCB->pxStack);
        vPortFree(pxTCB);
    }
#elif (tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0) /*lint !e731 !e9029 Macro has been consolidated for readability reasons. */
    {
        /* The task could have been allocated statically or dynamically, so
            check what was statically allocated before trying to free the
            memory. */
        if (pxTCB->ucStaticallyAllocated == tskDYNAMICALLY_ALLOCATED_STACK_AND_TCB)
        {
            /* Both the stack and TCB were allocated dynamically, so both
                must be freed. */
            vPortFree(pxTCB->pxStack);
            vPortFree(pxTCB);
        }
        else if (pxTCB->ucStaticallyAllocated == tskSTATICALLY_ALLOCATED_STACK_ONLY)
        {
            /* Only the stack was statically allocated, so the TCB is the
                only memory that must be freed. */
            vPortFree(pxTCB);
        }
        else
        {
            /* Neither the stack nor the TCB were allocated dynamically, so
                nothing needs to be freed. */
            configASSERT(pxTCB->ucStaticallyAllocated == tskSTATICALLY_ALLOCATED_STACK_AND_TCB);
            mtCOVERAGE_TEST_MARKER();
        }
    }
#endif                                                 /* configSUPPORT_DYNAMIC_ALLOCATION */
}

#endif /* INCLUDE_vTaskDelete */
/*-----------------------------------------------------------*/

static void prvResetNextTaskUnblockTime(void)
{
    TCB_t *pxTCB;

    if (listLIST_IS_EMPTY(pxDelayedTaskList) != pdFALSE)
    {
        /* The new current delayed list is empty.  Set xNextTaskUnblockTime to
        the maximum possible value so it is    extremely unlikely that the
        if( xTickCount >= xNextTaskUnblockTime ) test will pass until
        there is an item in the delayed list. */
        xNextTaskUnblockTime = portMAX_DELAY;
    }
    else
    {
        /* The new current delayed list is not empty, get the value of
        the item at the head of the delayed list.  This is the time at
        which the task at the head of the delayed list should be removed
        from the Blocked state. */
        (pxTCB) = listGET_OWNER_OF_HEAD_ENTRY(pxDelayedTaskList); /*lint !e9079 void * is used as this macro is used with timers and co-routines too.  Alignment is known to be fine as the type of the pointer stored and retrieved is the same. */
        xNextTaskUnblockTime = listGET_LIST_ITEM_VALUE(&((pxTCB)->xStateListItem));
    }
}
/*-----------------------------------------------------------*/

#if ((INCLUDE_xTaskGetCurrentTaskHandle == 1) || (configUSE_MUTEXES == 1))

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    TaskHandle_t xReturn;

    /* A critical section is not required as this is not called from
        an interrupt and the current TCB will always be the same for any
        individual execution thread. */
    xReturn = pxCurrentTCB;

    return xReturn;
}

#endif /* ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) ) */
/*-----------------------------------------------------------*/

#if ((INCLUDE_xTaskGetSchedulerState == 1) || (configUSE_TIMERS == 1))

BaseType_t xTaskGetSchedulerState(void)
{
    BaseType_t xReturn;

    if (xSchedulerRunning == pdFALSE)
    {
        xReturn = taskSCHEDULER_NOT_STARTED;
    }
    else
    {
        if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
        {
            xReturn = taskSCHEDULER_RUNNING;
        }
        else
        {
            xReturn = taskSCHEDULER_SUSPENDED;
        }
    }

    return xReturn;
}

#endif /* ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) ) */
/*-----------------------------------------------------------*/

#if (configUSE_MUTEXES == 1)

BaseType_t xTaskPriorityInherit(TaskHandle_t const pxMutexHolder)
{
    TCB_t *const pxMutexHolderTCB = pxMutexHolder;
    BaseType_t xReturn = pdFALSE;

    /* If the mutex was given back by an interrupt while the queue was
        locked then the mutex holder might now be NULL.  _RB_ Is this still
        needed as interrupts can no longer use mutexes? */
    if (pxMutexHolder != NULL)
    {
        /* If the holder of the mutex has a priority below the priority of
            the task attempting to obtain the mutex then it will temporarily
            inherit the priority of the task attempting to obtain the mutex. */
        if (pxMutexHolderTCB->uxPriority < pxCurrentTCB->uxPriority)
        {
            /* Adjust the mutex holder state to account for its new
                priority.  Only reset the event list item value if the value is
                not being used for anything else. */
            if ((listGET_LIST_ITEM_VALUE(&(pxMutexHolderTCB->xEventListItem)) & taskEVENT_LIST_ITEM_VALUE_IN_USE) == 0UL)
            {
                listSET_LIST_ITEM_VALUE(&(pxMutexHolderTCB->xEventListItem), (TickType_t)configMAX_PRIORITIES - (TickType_t)pxCurrentTCB->uxPriority); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            /* If the task being modified is in the ready state it will need
                to be moved into a new list. */
            if (listIS_CONTAINED_WITHIN(&(pxReadyTasksLists[pxMutexHolderTCB->uxPriority]), &(pxMutexHolderTCB->xStateListItem)) != pdFALSE)
            {
                if (uxListRemove(&(pxMutexHolderTCB->xStateListItem)) == (UBaseType_t)0)
                {
                    taskRESET_READY_PRIORITY(pxMutexHolderTCB->uxPriority);
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                /* Inherit the priority before being moved into the new list. */
                pxMutexHolderTCB->uxPriority = pxCurrentTCB->uxPriority;
                prvAddTaskToReadyList(pxMutexHolderTCB);
            }
            else
            {
                /* Just inherit the priority. */
                pxMutexHolderTCB->uxPriority = pxCurrentTCB->uxPriority;
            }

            traceTASK_PRIORITY_INHERIT(pxMutexHolderTCB, pxCurrentTCB->uxPriority);

            /* Inheritance occurred. */
            xReturn = pdTRUE;
        }
        else
        {
            if (pxMutexHolderTCB->uxBasePriority < pxCurrentTCB->uxPriority)
            {
                /* The base priority of the mutex holder is lower than the
                    priority of the task attempting to take the mutex, but the
                    current priority of the mutex holder is not lower than the
                    priority of the task attempting to take the mutex.
                    Therefore the mutex holder must have already inherited a
                    priority, but inheritance would have occurred if that had
                    not been the case. */
                xReturn = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    return xReturn;
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if (configUSE_MUTEXES == 1)

BaseType_t xTaskPriorityDisinherit(TaskHandle_t const pxMutexHolder)
{
    TCB_t *const pxTCB = pxMutexHolder;
    BaseType_t xReturn = pdFALSE;

    if (pxMutexHolder != NULL)
    {
        /* A task can only have an inherited priority if it holds the mutex.
            If the mutex is held by a task then it cannot be given from an
            interrupt, and if a mutex is given by the holding task then it must
            be the running state task. */
        configASSERT(pxTCB == pxCurrentTCB);
        configASSERT(pxTCB->uxMutexesHeld);
        (pxTCB->uxMutexesHeld)--;

        /* Has the holder of the mutex inherited the priority of another
            task? */
        if (pxTCB->uxPriority != pxTCB->uxBasePriority)
        {
            /* Only disinherit if no other mutexes are held. */
            if (pxTCB->uxMutexesHeld == (UBaseType_t)0)
            {
                /* A task can only have an inherited priority if it holds
                    the mutex.  If the mutex is held by a task then it cannot be
                    given from an interrupt, and if a mutex is given by the
                    holding task then it must be the running state task.  Remove
                    the holding task from the ready list. */
                if (uxListRemove(&(pxTCB->xStateListItem)) == (UBaseType_t)0)
                {
                    taskRESET_READY_PRIORITY(pxTCB->uxPriority);
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                /* Disinherit the priority before adding the task into the
                    new    ready list. */
                traceTASK_PRIORITY_DISINHERIT(pxTCB, pxTCB->uxBasePriority);
                pxTCB->uxPriority = pxTCB->uxBasePriority;

                /* Reset the event list item value.  It cannot be in use for
                    any other purpose if this task is running, and it must be
                    running to give back the mutex. */
                listSET_LIST_ITEM_VALUE(&(pxTCB->xEventListItem), (TickType_t)configMAX_PRIORITIES - (TickType_t)pxTCB->uxPriority); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
                prvAddTaskToReadyList(pxTCB);

                /* Return true to indicate that a context switch is required.
                    This is only actually required in the corner case whereby
                    multiple mutexes were held and the mutexes were given back
                    in an order different to that in which they were taken.
                    If a context switch did not occur when the first mutex was
                    returned, even if a task was waiting on it, then a context
                    switch should occur when the last mutex is returned whether
                    a task is waiting on it or not. */
                xReturn = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    return xReturn;
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if (configUSE_MUTEXES == 1)

void vTaskPriorityDisinheritAfterTimeout(TaskHandle_t const pxMutexHolder, UBaseType_t uxHighestPriorityWaitingTask)
{
    TCB_t *const pxTCB = pxMutexHolder;
    UBaseType_t uxPriorityUsedOnEntry, uxPriorityToUse;
    const UBaseType_t uxOnlyOneMutexHeld = (UBaseType_t)1;

    if (pxMutexHolder != NULL)
    {
        /* If pxMutexHolder is not NULL then the holder must hold at least
            one mutex. */
        configASSERT(pxTCB->uxMutexesHeld);

        /* Determine the priority to which the priority of the task that
            holds the mutex should be set.  This will be the greater of the
            holding task's base priority and the priority of the highest
            priority task that is waiting to obtain the mutex. */
        if (pxTCB->uxBasePriority < uxHighestPriorityWaitingTask)
        {
            uxPriorityToUse = uxHighestPriorityWaitingTask;
        }
        else
        {
            uxPriorityToUse = pxTCB->uxBasePriority;
        }

        /* Does the priority need to change? */
        if (pxTCB->uxPriority != uxPriorityToUse)
        {
            /* Only disinherit if no other mutexes are held.  This is a
                simplification in the priority inheritance implementation.  If
                the task that holds the mutex is also holding other mutexes then
                the other mutexes may have caused the priority inheritance. */
            if (pxTCB->uxMutexesHeld == uxOnlyOneMutexHeld)
            {
                /* If a task has timed out because it already holds the
                    mutex it was trying to obtain then it cannot of inherited
                    its own priority. */
                configASSERT(pxTCB != pxCurrentTCB);

                /* Disinherit the priority, remembering the previous
                    priority to facilitate determining the subject task's
                    state. */
                traceTASK_PRIORITY_DISINHERIT(pxTCB, pxTCB->uxBasePriority);
                uxPriorityUsedOnEntry = pxTCB->uxPriority;
                pxTCB->uxPriority = uxPriorityToUse;

                /* Only reset the event list item value if the value is not
                    being used for anything else. */
                if ((listGET_LIST_ITEM_VALUE(&(pxTCB->xEventListItem)) & taskEVENT_LIST_ITEM_VALUE_IN_USE) == 0UL)
                {
                    listSET_LIST_ITEM_VALUE(&(pxTCB->xEventListItem), (TickType_t)configMAX_PRIORITIES - (TickType_t)uxPriorityToUse); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                /* If the running task is not the task that holds the mutex
                    then the task that holds the mutex could be in either the
                    Ready, Blocked or Suspended states.  Only remove the task
                    from its current state list if it is in the Ready state as
                    the task's priority is going to change and there is one
                    Ready list per priority. */
                if (listIS_CONTAINED_WITHIN(&(pxReadyTasksLists[uxPriorityUsedOnEntry]), &(pxTCB->xStateListItem)) != pdFALSE)
                {
                    if (uxListRemove(&(pxTCB->xStateListItem)) == (UBaseType_t)0)
                    {
                        taskRESET_READY_PRIORITY(pxTCB->uxPriority);
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    prvAddTaskToReadyList(pxTCB);
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*----------------------------------------------------------*/
TickType_t uxTaskResetEventItemValue(void)
{
    TickType_t uxReturn;

    uxReturn = listGET_LIST_ITEM_VALUE(&(pxCurrentTCB->xEventListItem));

    /* Reset the event list item to its normal value - so it can be used with
    queues and semaphores. */
    listSET_LIST_ITEM_VALUE(&(pxCurrentTCB->xEventListItem), ((TickType_t)configMAX_PRIORITIES - (TickType_t)pxCurrentTCB->uxPriority)); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */

    return uxReturn;
}
/*-----------------------------------------------------------*/

#if (configUSE_MUTEXES == 1)

TaskHandle_t pvTaskIncrementMutexHeldCount(void)
{
    /* If xSemaphoreCreateMutex() is called before any tasks have been created
        then pxCurrentTCB will be NULL. */
    if (pxCurrentTCB != NULL)
    {
        (pxCurrentTCB->uxMutexesHeld)++;
    }

    return pxCurrentTCB;
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if (configUSE_TASK_NOTIFICATIONS == 1)

uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait)
{
    uint32_t ulReturn;

    taskENTER_CRITICAL();
    {
        /* Only block if the notification count is not already non-zero. */
        if (pxCurrentTCB->ulNotifiedValue == 0UL)
        {
            /* Mark this task as waiting for a notification. */
            pxCurrentTCB->ucNotifyState = taskWAITING_NOTIFICATION;

            if (xTicksToWait > (TickType_t)0)
            {
                prvAddCurrentTaskToDelayedList(xTicksToWait, pdTRUE);
                traceTASK_NOTIFY_TAKE_BLOCK();

                /* All ports are written to allow a yield in a critical
                    section (some will yield immediately, others wait until the
                    critical section exits) - but it is not something that
                    application code should ever do. */
                portYIELD_WITHIN_API();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    taskEXIT_CRITICAL();

    taskENTER_CRITICAL();
    {
        traceTASK_NOTIFY_TAKE();
        ulReturn = pxCurrentTCB->ulNotifiedValue;

        if (ulReturn != 0UL)
        {
            if (xClearCountOnExit != pdFALSE)
            {
                pxCurrentTCB->ulNotifiedValue = 0UL;
            }
            else
            {
                pxCurrentTCB->ulNotifiedValue = ulReturn - (uint32_t)1;
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        pxCurrentTCB->ucNotifyState = taskNOT_WAITING_NOTIFICATION;
    }
    taskEXIT_CRITICAL();

    return ulReturn;
}

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if (configUSE_TASK_NOTIFICATIONS == 1)

BaseType_t xTaskNotifyWait(uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait)
{
    BaseType_t xReturn;

    taskENTER_CRITICAL();
    {
        /* Only block if a notification is not already pending. */
        if (pxCurrentTCB->ucNotifyState != taskNOTIFICATION_RECEIVED)
        {
            /* Clear bits in the task's notification value as bits may get
                set    by the notifying task or interrupt.  This can be used to
                clear the value to zero. */
            pxCurrentTCB->ulNotifiedValue &= ~ulBitsToClearOnEntry;

            /* Mark this task as waiting for a notification. */
            pxCurrentTCB->ucNotifyState = taskWAITING_NOTIFICATION;

            if (xTicksToWait > (TickType_t)0)
            {
                prvAddCurrentTaskToDelayedList(xTicksToWait, pdTRUE);
                traceTASK_NOTIFY_WAIT_BLOCK();

                /* All ports are written to allow a yield in a critical
                    section (some will yield immediately, others wait until the
                    critical section exits) - but it is not something that
                    application code should ever do. */
                portYIELD_WITHIN_API();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    taskEXIT_CRITICAL();

    taskENTER_CRITICAL();
    {
        traceTASK_NOTIFY_WAIT();

        if (pulNotificationValue != NULL)
        {
            /* Output the current notification value, which may or may not
                have changed. */
            *pulNotificationValue = pxCurrentTCB->ulNotifiedValue;
        }

        /* If ucNotifyValue is set then either the task never entered the
            blocked state (because a notification was already pending) or the
            task unblocked because of a notification.  Otherwise the task
            unblocked because of a timeout. */
        if (pxCurrentTCB->ucNotifyState != taskNOTIFICATION_RECEIVED)
        {
            /* A notification was not received. */
            xReturn = pdFALSE;
        }
        else
        {
            /* A notification was already pending or a notification was
                received while the task was waiting. */
            pxCurrentTCB->ulNotifiedValue &= ~ulBitsToClearOnExit;
            xReturn = pdTRUE;
        }

        pxCurrentTCB->ucNotifyState = taskNOT_WAITING_NOTIFICATION;
    }
    taskEXIT_CRITICAL();

    return xReturn;
}

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if (configUSE_TASK_NOTIFICATIONS == 1)

BaseType_t xTaskGenericNotify(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue)
{
    TCB_t *pxTCB;
    BaseType_t xReturn = pdPASS;
    uint8_t ucOriginalNotifyState;

    configASSERT(xTaskToNotify);
    pxTCB = xTaskToNotify;

    taskENTER_CRITICAL();
    {
        if (pulPreviousNotificationValue != NULL)
        {
            *pulPreviousNotificationValue = pxTCB->ulNotifiedValue;
        }

        ucOriginalNotifyState = pxTCB->ucNotifyState;

        pxTCB->ucNotifyState = taskNOTIFICATION_RECEIVED;

        switch (eAction)
        {
        case eSetBits:
            pxTCB->ulNotifiedValue |= ulValue;
            break;

        case eIncrement:
            (pxTCB->ulNotifiedValue)++;
            break;

        case eSetValueWithOverwrite:
            pxTCB->ulNotifiedValue = ulValue;
            break;

        case eSetValueWithoutOverwrite:
            if (ucOriginalNotifyState != taskNOTIFICATION_RECEIVED)
            {
                pxTCB->ulNotifiedValue = ulValue;
            }
            else
            {
                /* The value could not be written to the task. */
                xReturn = pdFAIL;
            }
            break;

        case eNoAction:
            /* The task is being notified without its notify value being
                    updated. */
            break;

        default:
            /* Should not get here if all enums are handled.
                    Artificially force an assert by testing a value the
                    compiler can't assume is const. */
            configASSERT(pxTCB->ulNotifiedValue == ~0UL);

            break;
        }

        traceTASK_NOTIFY();

        /* If the task is in the blocked state specifically to wait for a
            notification then unblock it now. */
        if (ucOriginalNotifyState == taskWAITING_NOTIFICATION)
        {
            (void)uxListRemove(&(pxTCB->xStateListItem));
            prvAddTaskToReadyList(pxTCB);

            /* The task should not have been on an event list. */
            configASSERT(listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) == NULL);

            if (pxTCB->uxPriority > pxCurrentTCB->uxPriority)
            {
                /* The notified task has a priority above the currently
                    executing task so a yield is required. */
                taskYIELD_IF_USING_PREEMPTION();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    taskEXIT_CRITICAL();

    return xReturn;
}

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if (configUSE_TASK_NOTIFICATIONS == 1)

BaseType_t xTaskGenericNotifyFromISR(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue, BaseType_t *pxHigherPriorityTaskWoken)
{
    TCB_t *pxTCB;
    uint8_t ucOriginalNotifyState;
    BaseType_t xReturn = pdPASS;
    UBaseType_t uxSavedInterruptStatus;

    configASSERT(xTaskToNotify);

    /* RTOS ports that support interrupt nesting have the concept of a
        maximum    system call (or maximum API call) interrupt priority.
        Interrupts that are    above the maximum system call priority are keep
        permanently enabled, even when the RTOS kernel is in a critical section,
        but cannot make any calls to FreeRTOS API functions.  If configASSERT()
        is defined in FreeRTOSConfig.h then
        portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
        failure if a FreeRTOS API function is called from an interrupt that has
        been assigned a priority above the configured maximum system call
        priority.  Only FreeRTOS functions that end in FromISR can be called
        from interrupts    that have been assigned a priority at or (logically)
        below the maximum system call interrupt priority.  FreeRTOS maintains a
        separate interrupt safe API to ensure interrupt entry is as fast and as
        simple as possible.  More information (albeit Cortex-M specific) is
        provided on the following link:
        http://www.freertos.org/RTOS-Cortex-M3-M4.html */
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

    pxTCB = xTaskToNotify;

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if (pulPreviousNotificationValue != NULL)
        {
            *pulPreviousNotificationValue = pxTCB->ulNotifiedValue;
        }

        ucOriginalNotifyState = pxTCB->ucNotifyState;
        pxTCB->ucNotifyState = taskNOTIFICATION_RECEIVED;

        switch (eAction)
        {
        case eSetBits:
            pxTCB->ulNotifiedValue |= ulValue;
            break;

        case eIncrement:
            (pxTCB->ulNotifiedValue)++;
            break;

        case eSetValueWithOverwrite:
            pxTCB->ulNotifiedValue = ulValue;
            break;

        case eSetValueWithoutOverwrite:
            if (ucOriginalNotifyState != taskNOTIFICATION_RECEIVED)
            {
                pxTCB->ulNotifiedValue = ulValue;
            }
            else
            {
                /* The value could not be written to the task. */
                xReturn = pdFAIL;
            }
            break;

        case eNoAction:
            /* The task is being notified without its notify value being
                    updated. */
            break;

        default:
            /* Should not get here if all enums are handled.
                    Artificially force an assert by testing a value the
                    compiler can't assume is const. */
            configASSERT(pxTCB->ulNotifiedValue == ~0UL);
            break;
        }

        traceTASK_NOTIFY_FROM_ISR();

        /* If the task is in the blocked state specifically to wait for a
            notification then unblock it now. */
        if (ucOriginalNotifyState == taskWAITING_NOTIFICATION)
        {
            /* The task should not have been on an event list. */
            configASSERT(listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) == NULL);

            if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
            {
                (void)uxListRemove(&(pxTCB->xStateListItem));
                prvAddTaskToReadyList(pxTCB);
            }
            else
            {
                /* The delayed and ready lists cannot be accessed, so hold
                    this task pending until the scheduler is resumed. */
                vListInsertEnd(&(xPendingReadyList), &(pxTCB->xEventListItem));
            }

            if (pxTCB->uxPriority > pxCurrentTCB->uxPriority)
            {
                /* The notified task has a priority above the currently
                    executing task so a yield is required. */
                if (pxHigherPriorityTaskWoken != NULL)
                {
                    *pxHigherPriorityTaskWoken = pdTRUE;
                }

                /* Mark that a yield is pending in case the user is not
                        using the "xHigherPriorityTaskWoken" parameter to an ISR
                        safe FreeRTOS function. */
                xYieldPending = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

    return xReturn;
}

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if (configUSE_TASK_NOTIFICATIONS == 1)

void vTaskNotifyGiveFromISR(TaskHandle_t xTaskToNotify, BaseType_t *pxHigherPriorityTaskWoken)
{
    TCB_t *pxTCB;
    uint8_t ucOriginalNotifyState;
    UBaseType_t uxSavedInterruptStatus;

    configASSERT(xTaskToNotify);

    /* RTOS ports that support interrupt nesting have the concept of a
        maximum    system call (or maximum API call) interrupt priority.
        Interrupts that are    above the maximum system call priority are keep
        permanently enabled, even when the RTOS kernel is in a critical section,
        but cannot make any calls to FreeRTOS API functions.  If configASSERT()
        is defined in FreeRTOSConfig.h then
        portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
        failure if a FreeRTOS API function is called from an interrupt that has
        been assigned a priority above the configured maximum system call
        priority.  Only FreeRTOS functions that end in FromISR can be called
        from interrupts    that have been assigned a priority at or (logically)
        below the maximum system call interrupt priority.  FreeRTOS maintains a
        separate interrupt safe API to ensure interrupt entry is as fast and as
        simple as possible.  More information (albeit Cortex-M specific) is
        provided on the following link:
        http://www.freertos.org/RTOS-Cortex-M3-M4.html */
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

    pxTCB = xTaskToNotify;

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        ucOriginalNotifyState = pxTCB->ucNotifyState;
        pxTCB->ucNotifyState = taskNOTIFICATION_RECEIVED;

        /* 'Giving' is equivalent to incrementing a count in a counting
            semaphore. */
        (pxTCB->ulNotifiedValue)++;

        traceTASK_NOTIFY_GIVE_FROM_ISR();

        /* If the task is in the blocked state specifically to wait for a
            notification then unblock it now. */
        if (ucOriginalNotifyState == taskWAITING_NOTIFICATION)
        {
            /* The task should not have been on an event list. */
            configASSERT(listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) == NULL);

            if (uxSchedulerSuspended == (UBaseType_t)pdFALSE)
            {
                (void)uxListRemove(&(pxTCB->xStateListItem));
                prvAddTaskToReadyList(pxTCB);
            }
            else
            {
                /* The delayed and ready lists cannot be accessed, so hold
                    this task pending until the scheduler is resumed. */
                vListInsertEnd(&(xPendingReadyList), &(pxTCB->xEventListItem));
            }

            if (pxTCB->uxPriority > pxCurrentTCB->uxPriority)
            {
                /* The notified task has a priority above the currently
                    executing task so a yield is required. */
                if (pxHigherPriorityTaskWoken != NULL)
                {
                    *pxHigherPriorityTaskWoken = pdTRUE;
                }

                /* Mark that a yield is pending in case the user is not
                        using the "xHigherPriorityTaskWoken" parameter in an ISR
                        safe FreeRTOS function. */
                xYieldPending = pdTRUE;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
}

#endif /* configUSE_TASK_NOTIFICATIONS */

/*-----------------------------------------------------------*/

#if (configUSE_TASK_NOTIFICATIONS == 1)

BaseType_t xTaskNotifyStateClear(TaskHandle_t xTask)
{
    TCB_t *pxTCB;
    BaseType_t xReturn;

    /* If null is passed in here then it is the calling task that is having
        its notification state cleared. */
    pxTCB = prvGetTCBFromHandle(xTask);

    taskENTER_CRITICAL();
    {
        if (pxTCB->ucNotifyState == taskNOTIFICATION_RECEIVED)
        {
            pxTCB->ucNotifyState = taskNOT_WAITING_NOTIFICATION;
            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }
    }
    taskEXIT_CRITICAL();

    return xReturn;
}

#endif /* configUSE_TASK_NOTIFICATIONS */

static void prvAddCurrentTaskToDelayedList(TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely)
{
    TickType_t xTimeToWake;
    const TickType_t xConstTickCount = xTickCount;

    /* Remove the task from the ready list before adding it to the blocked list
    as the same list item is used for both lists. */
    if (uxListRemove(&(pxCurrentTCB->xStateListItem)) == (UBaseType_t)0)
    {
        /* The current task must be in a ready list, so there is no need to
        check, and the port reset macro can be called directly. */
        portRESET_READY_PRIORITY(pxCurrentTCB->uxPriority, uxTopReadyPriority); /*lint !e931 pxCurrentTCB cannot change as it is the calling task.  pxCurrentTCB->uxPriority and uxTopReadyPriority cannot change as called with scheduler suspended or in a critical section. */
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

#if (INCLUDE_vTaskSuspend == 1)
    {
        if ((xTicksToWait == portMAX_DELAY) && (xCanBlockIndefinitely != pdFALSE))
        {
            /* Add the task to the suspended task list instead of a delayed task
            list to ensure it is not woken by a timing event.  It will block
            indefinitely. */
            vListInsertEnd(&xSuspendedTaskList, &(pxCurrentTCB->xStateListItem));
        }
        else
        {
            /* Calculate the time at which the task should be woken if the event
            does not occur.  This may overflow but this doesn't matter, the
            kernel will manage it correctly. */
            xTimeToWake = xConstTickCount + xTicksToWait;

            /* The list item will be inserted in wake time order. */
            listSET_LIST_ITEM_VALUE(&(pxCurrentTCB->xStateListItem), xTimeToWake);

            if (xTimeToWake < xConstTickCount)
            {
                /* Wake time has overflowed.  Place this item in the overflow
                list. */
                vListInsert(pxOverflowDelayedTaskList, &(pxCurrentTCB->xStateListItem));
            }
            else
            {
                /* The wake time has not overflowed, so the current block list
                is used. */
                vListInsert(pxDelayedTaskList, &(pxCurrentTCB->xStateListItem));

                /* If the task entering the blocked state was placed at the
                head of the list of blocked tasks then xNextTaskUnblockTime
                needs to be updated too. */
                if (xTimeToWake < xNextTaskUnblockTime)
                {
                    xNextTaskUnblockTime = xTimeToWake;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
        }
    }

#endif /* INCLUDE_vTaskSuspend */
}

/* Code below here allows additional code to be inserted into this source file,
especially where access to file scope functions and data is needed (for example
when performing module tests). */

#ifdef FREERTOS_MODULE_TEST
#include "tasks_test_access_functions.h"
#endif
