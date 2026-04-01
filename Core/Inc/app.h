#ifndef CORE_SRC_APP_H
#define CORE_SRC_APP_H

#if __cplusplus
extern "C" {
#endif

void Setup();  // Called before FreeRTOS starts.
void Loop();   // The default FreeRTOS task.

#if __cplusplus
}
#endif

#endif
