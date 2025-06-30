#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

 #define I2C_NODE DT_NODELABEL(mysensor)
#define SHT45_ADDR 0x44
#define SHT45_CMD  0xFD

int main(void)
{
    static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
    k_msleep(1000);
    if (!device_is_ready(dev_i2c.bus)) {
        printk("I2C device not ready!\n");
        return 1;
   } else{
        printk("I2c is ready\n");
   }

    while (1) {
        uint8_t cmd = SHT45_CMD;
        uint8_t rx_buf[6];
        int ret;

        ret = i2c_write_dt(&dev_i2c, &cmd, 1);
        if (ret) {
            printk("Write failed (%d)\n", ret);
            return 1;
        }

        k_msleep(10);

        ret = i2c_read_dt(&dev_i2c, rx_buf, 6);
        if (ret) {
            printk("Read failed (%d)\n", ret);
            return 1;
        }

        uint16_t temp_raw = (rx_buf[0] << 8) | rx_buf[1];
        uint16_t hum_raw  = (rx_buf[3] << 8) | rx_buf[4];

        float temperature = -45 + (175.0f * temp_raw) / 65535.0f;
        float humidity = -6 + (125.0f * hum_raw) / 65535.0f;

        printk("Temp: %.2f C, Humidity: %.2f %%RH\n", (double)temperature, (double)humidity);

        k_msleep(1000);
    }

    return 0;
}
