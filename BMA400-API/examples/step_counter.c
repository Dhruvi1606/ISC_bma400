#include <stdio.h>
#include <string.h>

#include "bma400.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define SPI_SCK_PIN 26
#define SPI_MISO_PIN 30
#define SPI_MOSI_PIN 29
#define SPI_SS_PIN 31

void set_interface(enum bma400_intf intf, struct bma400_dev *dev);
void delay_ms(uint32_t period);
uint32_t i2c_reg_write( uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint32_t i2c_reg_read( uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint32_t spi_reg_write(void *intf_ptr ,uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint32_t spi_reg_read(void *intf_ptr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);
struct bma400_dev bma;
struct bma400_int_enable step_int;
int8_t rslt;


void i2c_init(void)
{
		twi_master_init();
}
void acc_init(void)
{
		i2c_init();
    set_interface(BMA400_I2C_INTF, &bma);

    rslt = bma400_init(&bma);
    print_rslt(rslt);

    rslt = bma400_soft_reset(&bma);
    print_rslt(rslt);

    step_int.type = BMA400_STEP_COUNTER_INT_EN;
    step_int.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&step_int, 1, &bma);
    print_rslt(rslt);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma);
    print_rslt(rslt);

    printf("Steps counted, Activity classifier\r\n");
}
uint32_t step_count;
uint8_t activity;
void process_accelerometer(void)
{
		rslt = bma400_get_steps_counted(&step_count, &activity, &bma);
		NRF_LOG_INFO("%d", step_count);

		switch (activity)
		{
			case BMA400_STILL_ACT:
					NRF_LOG_INFO(", Still\r\n");
					break;
			case BMA400_WALK_ACT:
					NRF_LOG_INFO(", Walking\r\n");
					break;
			case BMA400_RUN_ACT:
					NRF_LOG_INFO(", Running\r\n");
					break;
			default:
					NRF_LOG_INFO(", undefined\r\n");
					break;
		}
}


void set_interface(enum bma400_intf intf, struct bma400_dev *dev)
{
    switch (intf)
    {
        case BMA400_I2C_INTF:
            dev->intf_ptr = NULL; /* To attach your interface device reference */
            dev->delay_ms = delay_ms;
            dev->dev_id = BMA400_I2C_ADDRESS_SDO_LOW;
            dev->read = i2c_reg_read;
            dev->write = i2c_reg_write;
            dev->intf = BMA400_I2C_INTF;
            break;
        case BMA400_SPI_INTF:
            dev->intf_ptr = NULL; /* To attach your interface device reference */
						dev->delay_ms = delay_ms;
            dev->dev_id = 0; /* Could be used to identify the chip select line. */
            dev->read = spi_reg_read;
            dev->write = spi_reg_write;
            dev->intf = BMA400_SPI_INTF;
            break;
        default:
            printf("Interface not supported.\r\n");
    }
}

void delay_ms(uint32_t period)
{
    /* Wait for a period amount of ms*/
	nrf_delay_ms(period);
}

uint32_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
		uint32_t err_code;
		err_code = twi_master_write_i2c_reg(BMA400_I2C_ADDRESS_SDO_LOW,reg_addr,&reg_data[0],length);
    /* Write to registers using I2C. Return 0 for a successful execution. */
    return err_code;
}

uint32_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
		uint32_t err_code;
		err_code = twi_master_read_i2c_reg(BMA400_I2C_ADDRESS_SDO_LOW,reg_addr,&reg_data[0],length);
    /* Read from registers using I2C. Return 0 for a successful execution. */
    return err_code;
}

uint32_t spi_reg_write(void *intf_ptr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
		uint32_t err_code;
	
    return err_code;
}

uint32_t spi_reg_read(void *intf_ptr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
		uint32_t err_code;
		
    return err_code;
}

void print_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:
						NRF_LOG_INFO("Accelerometer found\r\n", rslt);
            /* Do nothing */
            break;
        case BMA400_E_NULL_PTR:
            NRF_LOG_INFO("Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMA400_E_COM_FAIL:
            NRF_LOG_INFO("Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMA400_E_DEV_NOT_FOUND:
            NRF_LOG_INFO("Error [%d] : Device not found\r\n", rslt);
            break;
        case BMA400_E_INVALID_CONFIG:
            NRF_LOG_INFO("Error [%d] : Invalid configuration\r\n", rslt);
            break;
        case BMA400_W_SELF_TEST_FAIL:
            NRF_LOG_INFO("Warning [%d] : Self test failed\r\n", rslt);
            break;
        default:
            NRF_LOG_INFO("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}
