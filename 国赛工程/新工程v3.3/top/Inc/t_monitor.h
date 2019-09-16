#ifndef __monitor_H
#define __monitor_H

/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "control.h"

typedef enum{
	monitor_err = 0,
	monitor_regular = 1,
}monitor_status_e;

typedef struct __monitor{
	uint32_t circle_number;
	monitor_status_e status;
	void (*toggle)(void);
	void (*monitor_process)(struct __monitor *Monitor);
	void (*MonitorParaInit)(struct __monitor *Monitor);
}Monitor_t;
	
extern void MonitorParaInit(Monitor_t *Monitor);
extern Monitor_t monitor_remote;
extern Monitor_t monitor_chassis_moto;
extern Monitor_t monitor_raising_moto;
extern Monitor_t monitor_hcan;

#endif



