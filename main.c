/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief FreeRTOS template application with retarget
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "osal.h"
#include "resmgmt.h"
#include "hw_cpm.h"
#include "hw_gpio.h"
#include "hw_watchdog.h"
#include "sys_clock_mgr.h"
#include "sys_power_mgr.h"

#include "hw_pdc.h"
#include "ad_i2c.h"
#include "hw_i2c.h"
#include "hw_wkup.h"
#include "hw_sys.h"
#include "peripheral_setup.h"
#include "bmp180.h"

/* Task priorities */
#define mainTEMPLATE_TASK_PRIORITY              ( OS_TASK_PRIORITY_NORMAL )
#define  mainI2C_TASK_PRIORITY              ( OS_TASK_PRIORITY_NORMAL )

/* The rate at which data is template task counter is incremented. */
#define mainCOUNTER_FREQUENCY_MS                OS_MS_2_TICKS(200)

/*
 * Error code returned after an I2C operation. It can be used
 * to identify the reason of a failure.
 */
__RETAINED static HW_I2C_ABORT_SOURCE I2C_error_code;

/* Task handle */
__RETAINED static OS_TASK prvI2CTask_h;

uint32_t pdc_wkup_combo_id  __attribute__((unused));

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware( void );
/*
 * Task functions .
 */
static void prvI2CTask_BMP(void *pvParameters);

static OS_TASK xHandle;

static void system_init( void *pvParameters )
{
        OS_BASE_TYPE status;

        REG_SETF(GPREG, DEBUG_REG, SYS_CPU_FREEZE_EN, 0);

#if defined CONFIG_RETARGET
        extern void retarget_init(void);
#endif /* CONFIG_RETARGET */

        /*
         * Prepare clocks. Note: cm_cpu_clk_set() and cm_sys_clk_set() can be called only
         * from a task since they will suspend the task until the XTAL32M has settled and,
         * maybe, the PLL is locked.
         */
        cm_sys_clk_init(sysclk_XTAL32M);
        cm_apb_set_clock_divider(apb_div1);
        cm_ahb_set_clock_divider(ahb_div1);
        cm_lp_clk_init();

        /* Prepare the hardware to run this demo */
        prvSetupHardware();

#if defined CONFIG_RETARGET
        retarget_init();
#endif /* CONFIG_RETARGET */


        /*
         * Upon a wakeup cycle, wait for the XTAL32M crystal to settle.
         * BLE, USB and UART blocks require the XTAL32M to be up and
         * running to work properly.
         */
        pm_set_wakeup_mode(true);


        /* Set the desired sleep mode. */
        pm_sleep_mode_set(pm_mode_extended_sleep);

        /*
         * Set the desired wakeup mode.
         *
         * \warning When set is Ultra-Fast wakeup mode, sleep voltage should be 0.9V
         *          and not less than that.
         *
         **/
        pm_set_sys_wakeup_mode(pm_sys_wakeup_mode_fast);

        /* I2C task  */
        status = OS_TASK_CREATE("I2C",    /* The text name assigned to the task, for
                                             debug only; not used by the kernel. */
                        prvI2CTask_BMP, /* The function that implements the task. */
                        NULL,              /* The parameter passed to the task. */
                        1024 * OS_STACK_WORD_SIZE, /* Stack size allocated for the task
                                                      in bytes. */
                        mainI2C_TASK_PRIORITY,   /* The priority assigned to the task. */
                        prvI2CTask_h );             /* The task handle. */
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);

        /* The work of the SysInit task is done */
        OS_TASK_DELETE(xHandle);
}

void os_delay (u32 millisec)
{
        OS_DELAY_MS(millisec);
}

static void set_target_address(uint16_t address)
{
        /*
         * Before address of the target will be changed I2C controller must be disabled. After
         * setting the proper address I2C can be reenabled.
         */
        hw_i2c_disable(HW_I2C1);

        hw_i2c_set_target_address(HW_I2C1, address);

        hw_i2c_enable(HW_I2C1);

        /*
         * Setting address in our case means we'll start some transfer with new device. For this
         * reason it's good to reset abort source in order to start in clean state (to avoid having
         * abort source set by previous transfer to other device), otherwise we need to remember
         * about this before every transfer separately.
         */
        hw_i2c_reset_abort_source(HW_I2C1);
}

static int8_t bmp_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t *val, uint8_t len)
{
        size_t wr_status = 0;
        I2C_error_code = HW_I2C_ABORT_NONE;
        /*
         * The first writing byte informs to which register rest data will be written.
         */
        hw_i2c_write_byte(HW_I2C1, reg);
        wr_status = hw_i2c_write_buffer_sync(HW_I2C1, val, len, &I2C_error_code, HW_I2C_F_WAIT_FOR_STOP);
        if ((wr_status < (ssize_t)len) || (I2C_error_code != HW_I2C_ABORT_NONE)) {
                printf("bmp180 write failure: %u\n", I2C_error_code);
                return 1;
        }
        return 0;
}

static int8_t bmp_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *val, uint8_t len)
{
        size_t rd_status = 0;
        I2C_error_code = HW_I2C_ABORT_NONE;
        /*
         * Before reading values from sensor registers we need to send one byte information to it
         * to inform which sensor register will be read now.
         */
        hw_i2c_write_byte(HW_I2C1, reg);
        rd_status = hw_i2c_read_buffer_sync(HW_I2C1, val, len, &I2C_error_code, HW_I2C_F_ADD_STOP);
        if ((rd_status < (size_t)len) || (I2C_error_code != HW_I2C_ABORT_NONE)) {
                printf("bmp180 read failure: %u", I2C_error_code);
                return 1;
        }
        return 0;
}

/**
 * @brief Template main creates a SysInit task, which creates a Template task
 */
int main( void )
{
        OS_BASE_TYPE status;


        /* Start the two tasks as described in the comments at the top of this
        file. */
        status = OS_TASK_CREATE("SysInit",              /* The text name assigned to the task, for
                                                           debug only; not used by the kernel. */
                        system_init,                    /* The System Initialization task. */
                        ( void * ) 0,                   /* The parameter passed to the task. */
                        configMINIMAL_STACK_SIZE * OS_STACK_WORD_SIZE,
                                                        /* The number of bytes to allocate to the
                                                           stack of the task. */
                        OS_TASK_PRIORITY_HIGHEST,       /* The priority assigned to the task. */
                        xHandle );                      /* The task handle */
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);



        /* Start the tasks and timer running. */
        vTaskStartScheduler();

        /* If all is well, the scheduler will now be running, and the following
        line will never be reached.  If the following line does execute, then
        there was insufficient FreeRTOS heap memory available for the idle and/or
        wkup_init();
        timer tasks to be created.  See the memory management section on the
        FreeRTOS web site for more details. */
        for ( ;; );

}

int read_bmp_sensor()
{
        struct bmp180_t bmp180;
        int32_t com_rslt = E_BMP_COMM_RES;
        uint16_t v_uncomp_temp_u16 = BMP180_INIT_VALUE;
        uint32_t v_uncomp_press_u32 = BMP180_INIT_VALUE;

        /*
        * Require I2C for exclusively reading temperature from FM75 and release it after operation.
        */
        resource_acquire(RES_MASK(RES_ID_I2C1), RES_WAIT_FOREVER);

        set_target_address(BMP180_I2C_ADDR);

        // Set up function pointers
        bmp180.bus_write = bmp_write_reg;
        bmp180.bus_read = bmp_read_reg;
        bmp180.dev_addr = BMP180_I2C_ADDR;
        bmp180.delay_msec = os_delay;

        com_rslt = bmp180_init(&bmp180);
        com_rslt += bmp180_get_calib_param();

        v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();
        v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

        com_rslt += bmp180_get_temperature(v_uncomp_temp_u16);

        com_rslt += bmp180_get_pressure(v_uncomp_press_u32);
        u16 temp = bmp180_get_temperature(v_uncomp_temp_u16);
        u32 pres = bmp180_get_pressure(v_uncomp_press_u32);

        resource_release(RES_MASK(RES_ID_I2C1));

        printf("Temp: %u (0.1)°C, Pressure: %u (1.0)Pa\r\n", temp, pres);

        return 0;
}

/**
 * @brief Template task increases a counter every mainCOUNTER_FREQUENCY_MS ms
 */
static void prvI2CTask_BMP( void *pvParameters )
{
//        ad_i2c_init();
        for ( ;; ) {
                read_bmp_sensor();
                 OS_DELAY_MS(1000);
        }
}

/**
 * @brief Initialize the peripherals domain after power-up.
 *
 */
static void periph_init(void)
{
}

/**
 * @brief Hardware Initialization
 */
static void prvSetupHardware( void )
{
        /* Init hardware */
        pm_system_init(periph_init);

        /* Enable the COM power domain before handling any GPIO pin */
        hw_sys_pd_com_enable();

        /* Configure the I2C GPIOs. */
        hw_gpio_configure_pin(BMP180_SCL_PORT, BMP180_SCL_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_I2C_SCL, true);
        hw_gpio_configure_pin(BMP180_SDA_PORT, BMP180_SDA_PIN, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_I2C_SDA, true);
        hw_gpio_pad_latch_enable(BMP180_SCL_PORT, BMP180_SCL_PIN);
        hw_gpio_pad_latch_enable(BMP180_SDA_PORT, BMP180_SDA_PIN);
        hw_gpio_configure_pin_power(BMP180_SCL_PORT, BMP180_SCL_PIN, HW_GPIO_POWER_V33);
        hw_gpio_configure_pin_power(BMP180_SDA_PORT, BMP180_SDA_PIN, HW_GPIO_POWER_V33);

        /*
         * Initialize I2C controller in master mode with standard communication speed (100 kb/s) and
         * transfer in 7-bit addressing mode.
         */
        static const i2c_config cfg = {
                .speed = HW_I2C_SPEED_HIGH,
                .mode = HW_I2C_MODE_MASTER,
                .addr_mode = HW_I2C_ADDRESSING_7B
        };

        hw_i2c_init(HW_I2C1, &cfg);
        /* Initialize the I2C interface. */
        hw_i2c_enable(HW_I2C1);


        /* Disable the COM power domain after handling the GPIO pins */
        hw_sys_pd_com_disable();

        srand(OS_GET_TICK_COUNT());
}

/**
 * @brief Malloc fail hook
 */
void vApplicationMallocFailedHook( void )
{
        /* vApplicationMallocFailedHook() will only be called if
        configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
        function that will get called if a call to OS_MALLOC() fails.
        OS_MALLOC() is called internally by the kernel whenever a task, queue,
        timer or semaphore is created.  It is also called by various parts of the
        demo application.  If heap_1.c or heap_2.c are used, then the size of the
        heap available to OS_MALLOC() is defined by configTOTAL_HEAP_SIZE in
        FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
        to query the size of free heap space that remains (although it does not
        provide information on how the remaining heap might be fragmented). */
        ASSERT_ERROR(0);
}

/**
 * @brief Application idle task hook
 */
void vApplicationIdleHook( void )
{
        /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
        to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
        task.  It is essential that code added to this hook function never attempts
        to block in any way (for example, call OS_QUEUE_GET() with a block time
        specified, or call OS_DELAY()).  If the application makes use of the
        OS_TASK_DELETE() API function (as this demo application does) then it is also
        important that vApplicationIdleHook() is permitted to return to its calling
        function, because it is the responsibility of the idle task to clean up
        memory allocated by the kernel to any task that has since been deleted. */
}

/**
 * @brief Application stack overflow hook
 */
void vApplicationStackOverflowHook( OS_TASK pxTask, char *pcTaskName )
{
        ( void ) pcTaskName;
        ( void ) pxTask;

        /* Run time stack overflow checking is performed if
        configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
        function is called if a stack overflow is detected. */
        ASSERT_ERROR(0);
}

/**
 * @brief Application tick hook
 */
void vApplicationTickHook( void )
{
}


