#include <stdio.h>
#include "bma400.h"
/* STEP 4.4 - Include header for usb */
#include <zephyr/usb/usb_device.h>

void set_interface(enum bma400_intf intf, struct bma400_dev *dev);
void delay_ms(uint32_t period);
int8_t i2c_reg_write(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

int main(int argc, char const *argv[])
{
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		int ret = usb_enable(NULL);
		if (ret) {
			return 0;
		}
	}
    struct bma400_dev bma;
    int8_t rslt;

    set_interface(BMA400_I2C_INTF, &bma);

    rslt = bma400_init(&bma);
    print_rslt(rslt);

    if (rslt == BMA400_OK)
    {
        printk("BMA400 found with chip ID 0x%X\r\n", bma.chip_id);
    }

    return 0;
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
     k_msleep(period);  // Zephyr API for millisecond delay
    /* Wait for a period amount of ms*/
}

int8_t i2c_reg_write(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    printk("I2C start");
    /* Write to registers using I2C. Return 0 for a successful execution. */
    const struct device *i2c_dev = (const struct device *)intf_ptr;

    // Prepare the buffer to hold the register address followed by the data
    uint8_t buf[length + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, length);

    // Perform the I2C write operation
    int ret = i2c_write(i2c_dev, buf, sizeof(buf), i2c_addr);
    if (ret != 0) {
        printk("I2C write failed %d\n", ret);
        return -1; // Return error code if write fails
    }
    return 0;
}

int8_t i2c_reg_read(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */
    const struct device *i2c_dev = (const struct device *)intf_ptr;
    
    // Write the register address to the device
    int ret = i2c_write(i2c_dev, &reg_addr, 1, i2c_addr);
    if (ret != 0) {
        printk("I2C write failed with error %d\n", ret);
        return -1; // Return error code if write fails
    }

    // Read the data from the device
    ret = i2c_read(i2c_dev, reg_data, length, i2c_addr);
    if (ret != 0) {
        printk("I2C read failed with error %d\n", ret);
        return -1; // Return error code if read fails
    }
    return 0;
}

int8_t spi_reg_write(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Write to registers using SPI. Return 0 for a successful execution. */
    
    return -1;
}

int8_t spi_reg_read(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using SPI. Return 0 for a successful execution. */
    return -1;
}

void print_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:

            /* Do nothing */
            break;
        case BMA400_E_NULL_PTR:
            printf("Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMA400_E_COM_FAIL:
            printf("Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMA400_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found\r\n", rslt);
            break;
        case BMA400_E_INVALID_CONFIG:
            printf("Error [%d] : Invalid configuration\r\n", rslt);
            break;
        case BMA400_W_SELF_TEST_FAIL:
            printf("Warning [%d] : Self test failed\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}