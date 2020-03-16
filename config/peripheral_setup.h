/*
 * peripheral_setup.h
 *
 *  Created on: Mar 10, 2020
 *      Author: pollop
 */

#ifndef CONFIG_PERIPHERAL_SETUP_H_
#define CONFIG_PERIPHERAL_SETUP_H_

/* I2C adapter fine-tuning */
//# define CONFIG_I2C_ENABLE_CRITICAL_SECTION     (1)

/**
 * I2C 1 configuration
 */
#define BMP180_SCL_PORT       ( HW_GPIO_PORT_0 )
#define BMP180_SCL_PIN        ( HW_GPIO_PIN_30 )

#define BMP180_SDA_PORT       ( HW_GPIO_PORT_0 )
#define BMP180_SDA_PIN        ( HW_GPIO_PIN_31 )

#endif /* CONFIG_PERIPHERAL_SETUP_H_ */
