#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include <stdlib.h>

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 16;
const int I2C_SCL_GPIO = 17;

QueueHandle_t xQueuePos;

typedef struct adc {
    int axis;
    int val;
} adc_t;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    adc_t envio_mpu;

    while(1) {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
          .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
          .axis.y = gyro[1] / 131.0f,
          .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
          .axis.x = acceleration[0] / 16384.0f, // Conversão para g
          .axis.y = acceleration[1] / 16384.0f,
          .axis.z = acceleration[2] / 16384.0f,
        }; 

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        // const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
       
        // adc_t from_mpu;

        // printf("Giro no sentido y (Roll): %f \n", gyroscope.axis.y);

        // printf("Giro no sentido x (Pitch): %f \n", gyroscope.axis.x);
        // printf("Giro no sentido z (Yall): %f \n", gyroscope.axis.z);

        // printf("Aceleração no sentido x : %f \n", accelerometer.axis.x);

        // Pitch: 0 (x)
        envio_mpu.axis = 0;
        envio_mpu.val = gyroscope.axis.x;
        xQueueSend(xQueuePos, &envio_mpu, 0);

        // Yall: 1 (z)
        envio_mpu.axis = 1;
        envio_mpu.val = gyroscope.axis.z;
        xQueueSend(xQueuePos, &envio_mpu, 0);

        // Aceleração translacional para simular meu clique:
        float ax = accelerometer.axis.x;
        float ay = accelerometer.axis.y;
        float az = accelerometer.axis.z;
        // printf("Aceleração em x: %f \n", ax);
        // printf("Aceleração em y: %f \n", ay);
        // printf("Aceleração em z: %f \n", az);

        
        // envio_mpu.val = ay;
        // envio_mpu.val = ax;
        // envio_mpu.val = az;

        // Verifico se aconteceu o clique
        if (abs(ax) > 1 || abs(ay) > 1 || abs(az) > 1 ){
            envio_mpu.axis = 2;
            xQueueSend(xQueuePos, &envio_mpu, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p){
    adc_t from_mpu;

    while(1){
       
        if (xQueueReceive(xQueuePos, &from_mpu, portMAX_DELAY)){
            const uint8_t EOP = 0xFF;

            uint8_t axis_id = from_mpu.axis; 
            int16_t val = from_mpu.val;

            uint8_t LSB = (uint8_t)(val);
            uint8_t MSB = (uint8_t)(val >> 8);

            // Mandando o pacote 
            uart_putc_raw(uart0, (int)axis_id);
            uart_putc_raw(uart0, (int)LSB);
            uart_putc_raw(uart0, (int)MSB);
            uart_putc_raw(uart0, (int)EOP);
        }
    }
}

int main() {
    stdio_init_all();


    /* Create the queue before starting tasks so they can use it immediately */
    xQueuePos = xQueueCreate(32, sizeof(adc_t));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_Task", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
