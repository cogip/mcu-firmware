#ifndef PLATFORM_TASK_H_
#define PLATFORM_TASK_H_

void mach_tasks_init(void);

#if defined(MODULE_CALIBRATION)
extern void *task_calibration_entry(void *arg);
#endif  /* MODULE_CALIBRATION */

#endif  /* PLATFORM_TASK_H_ */
