#include <Arduino.h>
#include <esp_task_wdt.h>

//1 = 25 - 27 || 13 - 33
//2 = 12 - 32 || 23 - 5

//3 = 4 - 16 || 18 - 19
//4 = 2 - 15 || 36 - 26

#define MOTORCOUNT 4

#define MOTORPINS 0
#define SENSEPINS 1

#define PULSES_PER_ROTATION (10 * 90)

uint8_t Motors[MOTORCOUNT][2][2] = {
    {
        {25, 27}, // Motor pins
        {13, 33}, // Sensor pins
    },
    {
        {12, 32},
        {23, 5},
    },
    {
        {4, 16},
        {18, 19},
    },
    {
        {2, 15},
        {36, 26},
    },
};

uint8_t counterState[MOTORCOUNT] = {0};
int counters[MOTORCOUNT] = {0};

TaskHandle_t Task1;

int * test = 0;

void Task1code(void *parameter)
{
  while (true)
  {

  }
}

void setup()
{
  btStop();

  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting");

  for (uint8_t i = 0; i < MOTORCOUNT; i++)
  {
    pinMode(Motors[i][MOTORPINS][0], OUTPUT);
    pinMode(Motors[i][MOTORPINS][1], OUTPUT);
  }

  for (uint8_t i = 0; i < MOTORCOUNT; i++)
  {
    pinMode(Motors[i][SENSEPINS][0], INPUT);
    pinMode(Motors[i][SENSEPINS][1], INPUT);
  }

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      1,         /* Priority of the task */
      &Task1,    /* Task handle. */
      0);        /* Core where the task should run */

  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  Serial.println("do");

  delay(10);
  int start = *test;
  delay(1000);
  int end = *test;

  Serial.print("Thing: ");
  Serial.println(*test);
  Serial.println(end - start);

  while (true)
  {
    continue;
    long mil = millis();

    //digitalWrite(Motors[0][MOTORPINS][0], HIGH); // 2
    //digitalWrite(Motors[1][MOTORPINS][0], HIGH);
    //digitalWrite(Motors[2][MOTORPINS][0], HIGH);
    //digitalWrite(Motors[3][MOTORPINS][0], HIGH);

    for (int i = 0; i < 4; i++)
    {
      Serial.print(i);
      Serial.print(" = ");
      Serial.println(counters[i]);
    }
    delay(1000);
    return;

    int j = 0;
    while (true)
    {
      for (int i = 0; i < 4; i++)
      {

        if (counters[i] >= PULSES_PER_ROTATION)
        {
          j++;
          digitalWrite(Motors[i][MOTORPINS][0], LOW);
          Serial.print(i);
          Serial.print(" = ");
          Serial.println(millis() - mil);
          Serial.println(counters[i]);
          counters[i] = 0;
        }
      }
      delay(1);
      if (j == 4)
        break;
    }

    delay(10000000);
  }
}

void loop() {}