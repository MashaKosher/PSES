/*
 * Minimal RTX5 configuration for STM32F103RBT6 (20KB SRAM).
 * Tuned to fit small RAM targets for Lab5.
 *
 * This file is normally generated/managed by Keil RTE. Keeping it in the repo
 * makes the project reproducible after git pull.
 */

#ifndef RTX_CONFIG_H
#define RTX_CONFIG_H

/* ----------------------- Thread Configuration ---------------------------- */

/* Number of concurrent active user threads (not counting idle/timer threads) */
#ifndef OS_THREAD_NUM
#define OS_THREAD_NUM                 2U
#endif

/* Default thread stack size (bytes) */
#ifndef OS_THREAD_DEF_STACK_SIZE
#define OS_THREAD_DEF_STACK_SIZE      256U
#endif

/* Idle thread stack size (bytes) */
#ifndef OS_THREAD_IDLE_STACK_SIZE
#define OS_THREAD_IDLE_STACK_SIZE     256U
#endif

/* Timer thread stack size (bytes) */
#ifndef OS_THREAD_TIMER_STACK_SIZE
#define OS_THREAD_TIMER_STACK_SIZE    384U
#endif

/* Disable per-thread C library "libspace" to save RAM (we avoid printf in app) */
#ifndef OS_THREAD_LIBSPACE_NUM
#define OS_THREAD_LIBSPACE_NUM        0U
#endif

/* ----------------------- Object Configuration ---------------------------- */

/* Keep objects at minimum when not used by the application. */
#ifndef OS_TIMER_NUM
#define OS_TIMER_NUM                  0U
#endif

#ifndef OS_EVENTFLAGS_NUM
#define OS_EVENTFLAGS_NUM             0U
#endif

#ifndef OS_MUTEX_NUM
#define OS_MUTEX_NUM                  0U
#endif

#ifndef OS_SEMAPHORE_NUM
#define OS_SEMAPHORE_NUM              0U
#endif

#ifndef OS_MSGQUEUE_NUM
#define OS_MSGQUEUE_NUM               0U
#endif

#ifndef OS_MEMPOOL_NUM
#define OS_MEMPOOL_NUM                0U
#endif

/* ----------------------- Memory Configuration ---------------------------- */

/* RTX dynamic memory pool (bytes). Lower this if you still hit L6406E. */
#ifndef OS_DYNAMIC_MEM_SIZE
#define OS_DYNAMIC_MEM_SIZE           0x0800U   /* 2KB */
#endif

#endif /* RTX_CONFIG_H */


