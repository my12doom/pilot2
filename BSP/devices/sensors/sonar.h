#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int sonar_init(void);
int sonar_update(void);			// 0 : new data, -1 : ultrasonic flying
int sonar_result(void);			// return last result, negative value for invalid value, unit: mm

#ifdef __cplusplus
}
#endif
