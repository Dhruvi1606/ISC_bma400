#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "bma400.h"
#include <math.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <dk_buttons_and_leds.h>
#include "my_lbs.h"

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY),
	800,
	801,
	NULL); 


LOG_MODULE_REGISTER(bma400_accel, LOG_LEVEL_INF);

//bluetooth 
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_LED DK_LED3
#define USER_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 7

#define RUN_LED_BLINK_INTERVAL 1000
#define NOTIFY_INTERVAL 500
static bool app_button_state;

#define I2C1_NODE DT_NODELABEL(bma400)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

#define GRAVITY_EARTH (9.80665f) /* Earth's gravity in m/s^2 */

void set_interface(enum bma400_intf intf, struct bma400_dev *dev);
void delay_ms(uint32_t period);
unsigned int i2c_reg_write(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned short length);
unsigned int i2c_reg_read(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned short length);
unsigned int spi_reg_write(unsigned char cs, unsigned char reg_addr, unsigned char *reg_data, unsigned short length);
unsigned int spi_reg_read(unsigned char cs, unsigned char reg_addr, unsigned char *reg_data, unsigned short length);
void print_rslt(int8_t rslt);
float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);
float sensor_ticks_to_s(uint32_t sensor_time);
struct bma400_int_enable step_int;
int8_t rslt;
uint32_t step_count;
uint8_t activity;
struct bma400_dev bma;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static void app_led_cb(bool led_state)
{
	dk_set_led(USER_LED, led_state);
}

static bool app_button_cb(void)
{
	return app_button_state;
}

void send_data_thread(int16_t x, int16_t y, int16_t z)
{
		send_bma_sensor_notify(x, y, z);
		k_sleep(K_MSEC(NOTIFY_INTERVAL));
}

void send_data_thread_2(uint32_t steps)
{
    steps_count_notify(steps);
    k_sleep(K_MSEC(NOTIFY_INTERVAL));
}

static struct my_lbs_cb app_callbacks = {
	.led_cb = app_led_cb,
	.button_cb = app_button_cb,
};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & USER_BUTTON) {
		uint32_t user_button_state = button_state & USER_BUTTON;
		/* STEP 6 - Send indication on a button press */
		my_lbs_send_button_state_indicate(user_button_state);
		app_button_state = user_button_state ? true : false;
	}
}
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");

	// dk_set_led_on(CON_STATUS_LED);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	// dk_set_led_off(CON_STATUS_LED);
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

static int init_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}

int main(int argc, char const *argv[])
{
    struct bma400_int_enable tap_int[2];
    struct bma400_sensor_conf conf[2];
    struct bma400_sensor_data data;
    uint8_t n_samples = 10;
    float t, x, y, z;
    uint32_t poll_period = 5, test_dur_ms = 20000;
    uint16_t int_status;
    int err;

    err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
	bt_conn_cb_register(&connection_callbacks);


	LOG_INF("Bluetooth initialized\n");
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return -1;
	}

    // Check if the I2C bus is ready
    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("I2C bus is not ready");
        return -1;
    }

    LOG_INF("\nBMA400 device is ready");

    struct bma400_dev bma;
    bma.intf_ptr = (void *)dev_i2c.bus;
    bma.delay_ms = delay_ms;
    bma.dev_id = BMA400_I2C_ADDRESS_SDO_LOW; // I2C address of BMA400
    bma.read = i2c_reg_read;
    bma.write = i2c_reg_write;
    bma.intf = BMA400_I2C_INTF;

    
    set_interface(BMA400_I2C_INTF, &bma);
    printk("Interface setup successful.\r\n");

    rslt = bma400_init(&bma);
    print_rslt(rslt);

    if (rslt == BMA400_OK)
    {
        printk("BMA400 found with chip ID 0x%X\r\n", bma.chip_id);
    }
    rslt = bma400_soft_reset(&bma);
    print_rslt(rslt);

    /* Select the type of configuration to be modified */
    conf[0].type = BMA400_ACCEL;
    conf[1].type = BMA400_TAP_INT;

    /* Get the accelerometer configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(conf, 2, &bma);
    print_rslt(rslt);

    /* Modify the desired configurations as per macros
     * available in bma400_defs.h file */
    conf[0].param.accel.odr = BMA400_ODR_200HZ;
    conf[0].param.accel.range = BMA400_4G_RANGE;
    conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;
    conf[0].param.accel.filt1_bw = BMA400_ACCEL_FILT1_BW_1;

    conf[1].param.tap.int_chan = BMA400_UNMAP_INT_PIN;
    conf[1].param.tap.axes_sel = BMA400_Z_AXIS_EN_TAP|BMA400_Y_AXIS_EN_TAP|BMA400_X_AXIS_EN_TAP;
    conf[1].param.tap.sensitivity = BMA400_TAP_SENSITIVITY_0;
    conf[1].param.tap.tics_th = BMA400_TICS_TH_18_DATA_SAMPLES;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(conf, 2, &bma);
    print_rslt(rslt);

    bma.delay_ms(100);

    tap_int[0].type = BMA400_SINGLE_TAP_INT_EN;
    tap_int[0].conf = BMA400_ENABLE;

    tap_int[1].type = BMA400_DOUBLE_TAP_INT_EN;
    tap_int[1].conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(tap_int, 2, &bma);
    print_rslt(rslt);

    bma.delay_ms(100);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma);
    LOG_INF("Power mode set result: %d", rslt);
    print_rslt(rslt);

    bma.delay_ms(100);

    printk("t[s], Ax[m/s2], Ay[m/s2], Az[m/s2]\r\n");

    while (n_samples && (rslt == BMA400_OK))
    {
        bma.delay_ms(10); /* Wait for 10ms as ODR is set to 100Hz */

        LOG_INF("Reading accelerometer data...");
        rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &data, &bma);

        LOG_INF("Data read result: %d", rslt);
        LOG_INF("Accel Data - X: %d, Y: %d, Z: %d, Sensor Time: %u", data.x, data.y, data.z, data.sensortime);
        send_data_thread(data.x, data.y, data.z);

        /* 12-bit accelerometer at range 2G */
        x = lsb_to_ms2(data.x, 2, 12);
        y = lsb_to_ms2(data.y, 2, 12);
        z = lsb_to_ms2(data.z, 2, 12);
        t = sensor_ticks_to_s(data.sensortime);
        
        printf("Before conversion - X: %.2f, Y: %.2f, Z: %.2f\r\n", x, y, z);

        n_samples--;
        k_msleep(1000);
    }
    
    
    if (rslt == BMA400_OK)
    {
        printk("Tap configured.\r\n");

        while (test_dur_ms)
        {
            bma.delay_ms(poll_period);

            rslt = bma400_get_interrupt_status(&int_status, &bma);
            print_rslt(rslt);

            if (int_status & BMA400_S_TAP_INT_ASSERTED)
            {
                printk("Single tap detected!\r\n");
            }

            if (int_status & BMA400_D_TAP_INT_ASSERTED)
            {
                printk("Double tap detected!\r\n");
            }

            test_dur_ms -= poll_period;
            
        }
        printk("tap configured over\r\n");
    }

    step_int.type = BMA400_STEP_COUNTER_INT_EN;
    step_int.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&step_int, 2, &bma);
    print_rslt(rslt);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma);
    print_rslt(rslt);

     while (1) {
        // Poll step count and activity
        printk("Steps counted, Activity classifier\r\n");
        rslt = bma400_get_steps_counted(&step_count, &activity, &bma);
        if (rslt == BMA400_OK) {
            LOG_INF("Steps counted: %d", step_count);
            send_data_thread_2(step_count);
            switch (activity) {
                case BMA400_STILL_ACT:
                    printf("Step counter interrupt received\n");
                    LOG_INF("Activity: Still");
                    break;
                case BMA400_WALK_ACT:
                    printf("Step counter interrupt received--1\n");
                    LOG_INF("Activity: Walking");
                    break;
                case BMA400_RUN_ACT:
                    LOG_INF("Activity: Running");
                    break;
                default:
                    LOG_INF("Activity: Undefined");
                    break;
            }
        } else {
            print_rslt(rslt);
        }

        k_msleep(1000); // Poll every second
    }
    
    rslt = bma400_perform_self_test(&bma);
    print_rslt(rslt);

    if (rslt == BMA400_OK)
    {
        printk("Self test passed.\r\n");
    }

    return 0;
}

void set_interface(enum bma400_intf intf, struct bma400_dev *dev)
{
    switch (intf)
    {
        case BMA400_I2C_INTF:
            dev->intf_ptr = (void *)dev_i2c.bus; /* To attach your interface device reference */
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
    k_msleep(period);
    /* Wait for a period amount of ms*/
}

unsigned int i2c_reg_write(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned short length) {
    const struct device *i2c_dev = (const struct device *)dev_i2c.bus; // Retrieve the I2C device instance
    uint8_t buf[length + 1]; // Buffer to hold register address + data
    
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, length);

    struct i2c_msg msgs[] = {
        {
            .buf = buf,
            .len = sizeof(buf),
            .flags = I2C_MSG_WRITE,
        }
    };

    int ret = i2c_transfer(i2c_dev, msgs, 1, i2c_addr);
    
    if (ret < 0) {
        return (unsigned int)ret; // Return the error code as unsigned int
    }

    return 0; // Success
}



unsigned int i2c_reg_read(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned short length) {
    const struct device *i2c_dev = (const struct device *)dev_i2c.bus; // Retrieve the I2C device instance
    struct i2c_msg msgs[] = {
        {
            .buf = &reg_addr,
            .len = 1,
            .flags = I2C_MSG_WRITE,
        },
        {
            .buf = reg_data,
            .len = length,
            .flags = I2C_MSG_READ,
        }
    };

    int ret = i2c_transfer(i2c_dev, msgs, 2, i2c_addr);
    
    if (ret < 0) {
        return (unsigned int)ret; // Return the error code as unsigned int
    }

    return 0; // Success
}


unsigned int spi_reg_write(unsigned char cs, unsigned char reg_addr, unsigned char *reg_data, unsigned short length) {
    // Implement SPI register write
    return -1; // Placeholder: replace with actual implementation
}

unsigned int spi_reg_read(unsigned char cs, unsigned char reg_addr, unsigned char *reg_data, unsigned short length) {
    // Implement SPI register read
    return -1; // Placeholder: replace with actual implementation
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

float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;

    return GRAVITY_EARTH * val * g_range / half_scale;
}

float sensor_ticks_to_s(uint32_t sensor_time)
{
    return (float)sensor_time * 0.0000390625f;
}

K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread_2, NULL, NULL, NULL, PRIORITY, 0, 0);