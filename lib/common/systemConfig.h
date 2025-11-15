#pragma once

#ifndef SYSTEMCONFIG_H
#define SYSTEMCONFIG_H

// MAX30102 read task configuration (only sensor-reading-related defs here)
#define MAX30102_TASK_STACK_SIZE 8192
#define MAX30102_TASK_PRIORITY   1
#define MAX30102_TASK_CORE       1

// Task watchdog timeout (seconds). If the registered task fails to reset within this time, ESP32 will reboot.
// This is used to protect system from stuck sensor-reading task.
#define WDT_TIMEOUT_S 10

#endif // SYSTEMCONFIG_H
