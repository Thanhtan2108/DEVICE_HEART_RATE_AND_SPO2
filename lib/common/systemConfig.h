#pragma once

#ifndef SYSTEMCONFIG_H
#define SYSTEMCONFIG_H

// MAX30102 task config
#define MAX30102_TASK_STACK_SIZE 8192
#define MAX30102_TASK_PRIORITY   1
#define MAX30102_TASK_CORE       1

// Example other app task
#define APP_TASK_STACK_SIZE 4096
#define APP_TASK_PRIORITY   1
#define APP_TASK_CORE       0

// Task watchdog timeout (seconds). If a registered task fails to reset within this time, ESP32 will reboot.
#define WDT_TIMEOUT_S 10

#endif // SYSTEMCONFIG_H
