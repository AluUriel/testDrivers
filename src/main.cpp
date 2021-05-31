#include <Arduino.h>

#include <TMCStepper.h>
#include <HardwareSerial.h>
// #include <SoftwareSerial.h>

#define STALL_VALUE 100 // [0..255]

// Pines de los drivers
#define EN_PIN 17 // Enable
#define DIR_PIN_1 32 // Direction
#define STEP_PIN_1 33 // Step

#define DIR_PIN_2 14 // Direction
#define STEP_PIN_2 25 // Step

#define DIR_PIN_3 18 // Direction
#define STEP_PIN_3 13 // Step

#define DIR_PIN_4 21 // Direction
#define STEP_PIN_4 19 // Step


#define SERIAL_PORT Serial1 //HardwareSerial port
#define DRIVER_ADDRESS 0b00
#define DRIVER_ADDRESS_2 0b01
#define DRIVER_ADDRESS_3 0b10
#define DRIVER_ADDRESS_4 0b11

#define R_SENSE 0.11f

// SoftwareSerial SERIAL_PORT(SW_RX, SW_TX); // RX, TX

//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_3);
TMC2209Stepper driver4(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_4);

void move(int pasos, int motor_d, int Speed, bool dir = true);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
    }
    Serial.println("\nStart...");

    SERIAL_PORT.begin(57600);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    // configuracion de pines de los drivers
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_1, OUTPUT);

    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(DIR_PIN_2, OUTPUT);

    pinMode(STEP_PIN_3, OUTPUT);
    pinMode(DIR_PIN_3, OUTPUT);

    pinMode(STEP_PIN_4, OUTPUT);
    pinMode(DIR_PIN_4, OUTPUT);

    // configuracion de los drivers por uart
    driver.begin();
    driver.pdn_disable(true);
    driver.toff(4);
    driver.blank_time(24);
    driver.rms_current(600);
    driver.microsteps(16);
    driver.TCOOLTHRS(0xFFFFF);
    driver.semin(0);
    driver.sedn(0b01);
    driver.SGTHRS(STALL_VALUE);

    driver2.begin();
    driver2.pdn_disable(true);
    driver2.toff(4);
    driver2.blank_time(24);
    driver2.rms_current(600);
    driver2.microsteps(16);
    driver2.TCOOLTHRS(0xFFFFF);
    driver2.semin(0);
    driver2.sedn(0b01);
    driver2.SGTHRS(STALL_VALUE);

    driver3.begin();
    driver3.pdn_disable(true);
    driver3.toff(4);
    driver3.blank_time(24);
    driver3.rms_current(600);
    driver3.microsteps(16);
    driver3.TCOOLTHRS(0xFFFFF);
    driver3.semin(0);
    driver3.sedn(0b01);
    driver3.SGTHRS(STALL_VALUE);

    driver4.begin();
    driver4.pdn_disable(true);
    driver4.toff(4);
    driver4.blank_time(24);
    driver4.rms_current(600);
    driver4.microsteps(16);
    driver4.TCOOLTHRS(0xFFFFF);
    driver4.semin(0);
    driver4.sedn(0b01);
    driver4.SGTHRS(STALL_VALUE);

    // digitalWrite(EN_PIN_3, LOW);
    // digitalWrite(DIR_PIN_3, HIGH);
    // move(2400, 1, 5000);

    // Serial.println("Test Motor 1");
    
}

void loop()
{
    uint8_t result;

    // Serial.println("===========Prueba del sensor Hall 1============");
    // Serial.print("A continuacion, se leerea la salida del sensor hall 1");
    // int sensorHallValue1;
    // for (int reading = 0; reading < 50; reading ++){

    // }


    Serial.println("===========Prueba del motor 1=============");
    result = driver.test_connection();
    if (result == 0) {
        Serial.println(F("test_connection del Motor 1: SUCCESSFUL"));
    } else {

        Serial.println(F("test_connection del Motor 1: FAIL!!!"));
    }
    Serial.println("verificar que el valor seteado y actual coincida");
    Serial.println("corriente   setteada: 600");
    Serial.println("microstteps setteado:  8");

    driver.rms_current(600);
    driver.microsteps(8);

    Serial.print("Corriente actual del driver 1: ");
    Serial.println(driver.rms_current());
    Serial.print("Microsteps actual del driver 1: ");
    Serial.println(driver.microsteps());

    Serial.println("corriente   setteada: 700");
    Serial.println("microstteps setteado:  16");

    driver.rms_current(700);
    driver.microsteps(16);

    Serial.print("Corriente actual del driver 1: ");
    Serial.println(driver.rms_current());
    Serial.print("Microsteps actual del driver 1: ");
    Serial.println(driver.microsteps());
    delay(500);

    Serial.println("El motor 1 girara 3200 pasos en un sentido...");
    delay(1000);
    move(3200, 1, 1000, true);
    Serial.println("El motor 1 girara 3200 pasos en el otro sentido...");
    delay(1000);
    move(3200, 1, 1000, false);

    Serial.println("===========Prueba del motor 2=============");
    result = driver2.test_connection();
    if (result == 0) {
        Serial.println(F("test_connection del Motor 2: SUCCESSFUL"));
    } else {

        Serial.println(F("test_connection del Motor 2: FAIL!!!"));
    }
    Serial.println("verificar que el valor seteado y actual coincida");
    Serial.println("corriente   setteada: 600");
    Serial.println("microstteps setteado:  8");

    driver2.rms_current(600);
    driver2.microsteps(8);

    Serial.print("Corriente actual del driver 2: ");
    Serial.println(driver2.rms_current());
    Serial.print("Microsteps actual del driver 2: ");
    Serial.println(driver2.microsteps());

    Serial.println("corriente   setteada: 700");
    Serial.println("microstteps setteado:  16");

    driver2.rms_current(700);
    driver2.microsteps(16);

    Serial.print("Corriente actual del driver 2: ");
    Serial.println(driver2.rms_current());
    Serial.print("Microsteps actual del driver 2: ");
    Serial.println(driver2.microsteps());
    delay(500);

    Serial.println("El motor 2 girara 3200 pasos en un sentido...");
    delay(1000);
    move(3200, 2, 1000, true);
    Serial.println("El motor 2 girara 3200 pasos en el otro sentido...");
    delay(1000);
    move(3200, 2, 1000, false);

    Serial.println("===========Prueba del motor 3=============");
    result = driver3.test_connection();
    if (result == 0) {
        Serial.println(F("test_connection del Motor 3: SUCCESSFUL"));
    } else {

        Serial.println(F("test_connection del Motor 3: FAIL!!!"));
    }
    Serial.println("verificar que el valor seteado y actual coincida");
    Serial.println("corriente   setteada: 600");
    Serial.println("microstteps setteado:  8");

    driver3.rms_current(600);
    driver3.microsteps(8);

    Serial.print("Corriente actual del driver 3: ");
    Serial.println(driver3.rms_current());
    Serial.print("Microsteps actual del driver 3: ");
    Serial.println(driver3.microsteps());

    Serial.println("corriente   setteada: 700");
    Serial.println("microstteps setteado:  16");

    driver3.rms_current(700);
    driver3.microsteps(16);

    Serial.print("Corriente actual del driver 3: ");
    Serial.println(driver3.rms_current());
    Serial.print("Microsteps actual del driver 3: ");
    Serial.println(driver3.microsteps());
    delay(500);

    Serial.println("El motor 3 girara 3200 pasos en un sentido...");
    delay(1000);
    move(3200, 3, 1000, true);
    Serial.println("El motor 3 girara 3200 pasos en el otro sentido...");
    delay(1000);
    move(3200, 3, 1000, false);

    Serial.println("===========Prueba del motor 4=============");
    result = driver4.test_connection();
    if (result == 0) {
        Serial.println(F("test_connection del Motor 4: SUCCESSFUL"));
    } else {

        Serial.println(F("test_connection del Motor 4: FAIL!!!"));
    }
    Serial.println("verificar que el valor seteado y actual coincida");
    Serial.println("corriente   setteada: 600");
    Serial.println("microstteps setteado:  8");

    driver4.rms_current(600);
    driver4.microsteps(8);

    Serial.print("Corriente actual del driver 4: ");
    Serial.println(driver4.rms_current());
    Serial.print("Microsteps actual del driver 4: ");
    Serial.println(driver4.microsteps());

    Serial.println("corriente   setteada: 700");
    Serial.println("microstteps setteado:  16");

    driver4.rms_current(700);
    driver4.microsteps(16);

    Serial.print("Corriente actual del driver 4: ");
    Serial.println(driver4.rms_current());
    Serial.print("Microsteps actual del driver 4: ");
    Serial.println(driver4.microsteps());
    delay(500);

    Serial.println("El motor 4 girara 3200 pasos en un sentido...");
    delay(1000);
    move(3200, 4, 1000, true);
    Serial.println("El motor 4 girara 3200 pasos en el otro sentido...");
    delay(1000);
    move(3200, 4, 1000, false);

    delay(1000);

    
}

void move(int pasos, int motor_d, int Speed, bool dir)
{
    if (motor_d == 1) {
        digitalWrite(DIR_PIN_1, dir);
        for (int i = 0; i < pasos; i++) {
            digitalWrite(STEP_PIN_1, HIGH);
            delayMicroseconds(Speed);
            digitalWrite(STEP_PIN_1, LOW);
            delayMicroseconds(Speed);
        }
    }

    if (motor_d == 2) {
        digitalWrite(DIR_PIN_2, dir);
        for (int i = 0; i < pasos; i++) {
            digitalWrite(STEP_PIN_2, HIGH);
            delayMicroseconds(Speed);
            digitalWrite(STEP_PIN_2, LOW);
            delayMicroseconds(Speed);
        }
    }

    if (motor_d == 3) {
        digitalWrite(DIR_PIN_3, dir);
        for (int i = 0; i < pasos; i++) {
            digitalWrite(STEP_PIN_3, HIGH);
            delayMicroseconds(Speed);
            digitalWrite(STEP_PIN_3, LOW);
            delayMicroseconds(Speed);
        }
    }

    if (motor_d == 4) {
        digitalWrite(DIR_PIN_4, dir);
        for (int i = 0; i < pasos; i++) {
            digitalWrite(STEP_PIN_4, HIGH);
            delayMicroseconds(Speed);
            digitalWrite(STEP_PIN_4, LOW);
            delayMicroseconds(Speed);
        }
    }
}