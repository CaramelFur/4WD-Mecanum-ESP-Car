#include <stdio.h>
#include <algorithm>

#include "stupid_ass_workaround.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "nvs_flash.h"

// Static

#define PULSES_PER_ROTATION (10 * 90)

#define WATCHDOG_TIMEOUT 5000

// Pindefs

#define MOTORCOUNT 4

#define MOTORPINS 0
#define SENSEPINS 1

#define SDL_PIN 22
#define SDA_PIN 21

// Per device config

#define SEBAS 1

#ifdef SEBAS

#define NO_HALF_TANK_TURN
#define MAX_AXIS 128
#define AXIS_ZERO 5

uint8_t Motors[MOTORCOUNT][2][2] = {
    {
        // Left Front
        {32, 12}, // Motor pins
        {23, 5},  // Sensor pins
    },
    {
        // Right Front
        {16, 4},  // Motor pins
        {18, 19}, // Sensor pins
    },
    {
        // Left back
        {2, 15},  // Motor pins
        {36, 26}, // Sensor pins
    },
    {
        // Right Back
        {25, 27}, // Motor pins
        {13, 33}, // Sensor pins
    },
};
#endif

#ifdef RENZO

#define MAX_AXIS 100
#define AXIS_ZERO 15

uint8_t Motors[MOTORCOUNT][2][2] = {
    {
        // Left Front
        {2, 15},  // Motor pins
        {36, 26}, // Sensor pins
    },
    {
        // Right Front
        {12, 32}, // Motor pins
        {23, 5},  // Sensor pins
    },
    {
        // Left Back
        {4, 16},  // Motor pins
        {18, 19}, // Sensor pins
    },
    {
        // Right Back
        {25, 27}, // Motor pins
        {13, 33}, // Sensor pins
    },
};
#endif

uint8_t mymacaddr[] = {0x7c, 0x9e, 0xbd, 0xe2, 0xdf, 0x32};
int battery = 0;

int64_t lastMessageMS = 0;

int msToTicks(int ms);
void doAllMotor(float speed);
void doMotor(int motor, float speed);
int64_t getTimeSinceBoot();

void controller_event_cb(ps3_t data, ps3_event_t event)
{
    lastMessageMS = getTimeSinceBoot();

    if ((abs(event.analog_changed.stick.lx) + abs(event.analog_changed.stick.ly) > 0) || (abs(event.analog_changed.stick.rx) + abs(event.analog_changed.stick.ry) > 0))
    {
        //printf("%d %d - %d %d\n", data.analog.stick.lx, data.analog.stick.ly, data.analog.stick.rx, data.analog.stick.ry);

        double LY = 0;
        double LX = 0;
        if (abs(data.analog.stick.ly) > AXIS_ZERO)
            LY = (-(double)data.analog.stick.ly) / MAX_AXIS;
        else
            LY = (-(double)data.analog.stick.ry) / MAX_AXIS;
        if (abs(data.analog.stick.lx) > AXIS_ZERO)
            LX = ((double)data.analog.stick.lx) / MAX_AXIS;

        double RX = 0;
        if (abs(data.analog.stick.rx) > AXIS_ZERO)
            RX = ((double)data.analog.stick.rx) / MAX_AXIS;

        double leftF = 0;
        double rightF = 0;
        double leftB = 0;
        double rightB = 0;

        leftF = LY;
        rightF = LY;
        leftB = LY;
        rightB = LY;

        leftF += LX;
        rightF += -LX;
        leftB += -LX;
        rightB += LX;

#ifdef NO_HALF_TANK_TURN
        leftF += RX * 2;
        rightF += -RX * 2;
        leftB += RX * 2;
        rightB += -RX * 2;
#else
        leftF += RX;
        rightF += -RX;
        leftB += RX;
        rightB += -RX;
#endif

        double maxv = std::max(std::max(abs(leftF), abs(rightF)), std::max(abs(leftB), abs(rightB)));

        if (maxv > 1)
        {
            leftF = leftF / maxv;
            rightF = rightF / maxv;
            leftB = leftB / maxv;
            rightB = rightB / maxv;
        }

        leftF = leftF * 100;
        rightF = rightF * 100;
        leftB = leftB * 100;
        rightB = rightB * 100;

        printf("%d %d %d %d\n", (int8_t)leftF, (int8_t)rightF, (int8_t)leftB, (int8_t)rightB);
        doMotor(0, leftF);
        doMotor(1, rightF);
        doMotor(2, leftB);
        doMotor(3, rightB);
    }

    if (battery != data.status.battery)
    {
        battery = data.status.battery;
        printf("The controller battery is ");
        if (battery == ps3_status_battery_charging)
            printf("charging\n");
        else if (battery == ps3_status_battery_full)
            printf("FULL\n");
        else if (battery == ps3_status_battery_high)
            printf("HIGH\n");
        else if (battery == ps3_status_battery_low)
            printf("LOW\n");
        else if (battery == ps3_status_battery_dying)
            printf("DYING\n");
        else if (battery == ps3_status_battery_shutdown)
            printf("SHUTDOWN\n");
        else
            printf("UNDEFINED\n");
    }
}

void app_main()
{
    printf("Hello world!\n");

    // Init motors

    for (int i = 0; i < MOTORCOUNT; i++)
    {
        mcpwm_gpio_init((mcpwm_unit_t)(i / 2), (mcpwm_io_signals_t)(((i % 2) * 2) + 0), Motors[i][MOTORPINS][0]);
        mcpwm_gpio_init((mcpwm_unit_t)(i / 2), (mcpwm_io_signals_t)(((i % 2) * 2) + 1), Motors[i][MOTORPINS][1]);

        mcpwm_config_t c = {
            .frequency = 50000,
            .cmpr_a = 0,
            .cmpr_b = 0,
            .duty_mode = MCPWM_DUTY_MODE_0,
            .counter_mode = MCPWM_UP_COUNTER,
        };
        mcpwm_init((mcpwm_unit_t)(i / 2), (mcpwm_timer_t)(i % 2), &c);

        mcpwm_start((mcpwm_unit_t)(i / 2), (mcpwm_timer_t)(i % 2));
    }

    // Ps3 connect stuff
    nvs_flash_init();

    ps3SetBluetoothMacAddress(mymacaddr);

    ps3SetEventCallback(controller_event_cb);
    ps3Init();

    while (!ps3IsConnected())
    {
        // Prevent the Task Watchdog from triggering
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    printf("Core: %d\n", xPortGetCoreID());

    ps3SetLed(1);

    printf("We connected!\n");

    fflush(stdout);

    lastMessageMS = getTimeSinceBoot();

    while (true)
    {
        int64_t currentTime = getTimeSinceBoot();
        if (currentTime - lastMessageMS > msToTicks(WATCHDOG_TIMEOUT))
        {
            printf("Bye bye\n");
            doAllMotor(100);
            vTaskDelay(msToTicks(200));
            doAllMotor(0);

            esp_restart();
        }
        vTaskDelay(msToTicks(500));
    }
}

void doAllMotor(float speed)
{
    doMotor(0, speed);
    doMotor(1, speed);
    doMotor(2, speed);
    doMotor(3, speed);
}

void doMotor(int motor, float speed)
{
    mcpwm_set_duty((mcpwm_unit_t)(motor / 2), (mcpwm_timer_t)(motor % 2), (mcpwm_generator_t)0, speed < 0 ? abs(speed) : 0);
    mcpwm_set_duty((mcpwm_unit_t)(motor / 2), (mcpwm_timer_t)(motor % 2), (mcpwm_generator_t)1, speed > 0 ? abs(speed) : 0);
}

int64_t getTimeSinceBoot()
{
    return esp_timer_get_time() / 1000;
}

int msToTicks(int ms)
{
    return ms / portTICK_RATE_MS;
}