#pragma once

/* PIN definitions. */
#define I2C_SDA_PIN	0
#define I2C_SCL_PIN	1
#define PIO_PHI		2
#define PIO_NRD		3
#define PIO_NCS		4
#define PIO_A0		5
#define PIO_A15		20
#define PIO_D0		21
#define PIO_D7		28
#define PIO_DIR		29

#define NUM_ADDRESS_PINS  16
#define NUM_DATA_PINS     8
#define NUM_MISC_PINS     3
#define NUM_TOTAL_PINS   (NUM_ADDRESS_PINS + NUM_DATA_PINS + NUM_MISC_PINS)

#define PIO_SM_A15	0
#define PIO_SM_CS	1
#define PIO_SM_DI	2
#define PIO_SM_DO	3
