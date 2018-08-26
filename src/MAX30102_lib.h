

#include "stdint.h"

#include "nrf_drv_twi.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


void twi_init (void);

void MAX30102_reset (void);

void MAX30102_init (void);

void MAX30102_read_ID (void);

void MAX30102_read_fifo (uint32_t *pun_red_led, uint32_t *pun_ir_led);

void MAX30102_write_register (uint8_t reg_address, uint8_t data);

void MAX30102_read_register (uint8_t reg_address, uint8_t *data);