#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint16_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

#endif // MOTOR_H