/*
 * RTX5 configuration + weak hooks for Lab5 (STM32F103RBT6).
 *
 * Note: Keil RTE usually generates this file. We keep it in repo to ensure
 * consistent RAM usage across machines.
 */

#include "cmsis_compiler.h"
#include "RTX_Config.h"
#include "rtx_os.h"

/* OS Idle Thread */
__WEAK __NO_RETURN void osRtxIdleThread(void *argument) {
  (void)argument;
  for (;;) {
    __WFE();
  }
}

/* OS Error Callback function */
__WEAK uint32_t osRtxErrorNotify(uint32_t code, void *object_id) {
  (void)object_id;
  (void)code;
  for (;;) {
  }
}


