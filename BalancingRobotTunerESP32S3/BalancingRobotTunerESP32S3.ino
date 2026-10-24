// BalancingRobotTunerESP32S3 - BalancingRobotTuner ported from the Arduino Nano to an
// ESP32-S3-DevKitC-1 (N16R8: 16 MB flash, 8 MB octal PSRAM). Same wire protocol, parameters,
// telemetry, text debug mode and Remote/Avoid/Follow modes as the Nano sketch, so the phone
// app, tuner_gui.py, autotune.py and serial_monitor.py work with either board unchanged.
//
// What differs from the Nano sketch:
//   - Pins: see the table below and PINOUT.md. The ESP32-S3 runs at 3.3 V and its pins are NOT
//     5 V tolerant: the HC-SR04 echo (and any 5 V encoder or I2C pull-up) needs a divider or a
//     level shifter.
//   - The 5 ms balance loop runs in a FreeRTOS task (BalanceTask) instead of a Timer2 interrupt:
//     the ESP32 cannot do I2C from a timer interrupt.
//   - The ultrasonic sensor is read in its own task with a 25 ms pulseIn timeout, so a missing
//     echo can no longer stall the balance loop.
//   - cli()/sei() become a spinlock critical section (LOCK/UNLOCK); encoder interrupts use
//     attachInterrupt on any pin instead of INT0 and the D4 pin-change interrupt.
//   - EEPROM is the ESP32's flash emulation: EEPROM.begin() first, EEPROM.commit() to write.
//   - The MPU6050 is read directly over Wire (Mpu_Init/Mpu_Read do what the MPU6050 library's
//     initialize() and getMotion6() did): the old I2Cdev library does not build for the ESP32.
//
// Serial: build with "USB CDC On Boot: Enabled" (FQBN option CDCOnBoot=cdc) and plug the phone
// or PC into the board's "USB" port (native USB, shows up as CDC-ACM, no reset on open). With
// CDC On Boot disabled, Serial is UART0 on the "UART" port (CP2102N or CH343 bridge) instead;
// the protocol is the same on both. The baud rate (115200) only matters on the UART port.
//
// Wire protocol (all multi-byte values little-endian, chk = low byte of the sum of the bytes
// between the header and chk):
//   phone -> robot
//     A5 x y horn ccw cw mode1 mode2 mode3 chk 5A   control (11 bytes, chk not verified,
//                                                   as in the kit's sketch)
//     A6 id float32 chk 5A                          set parameter id (8 bytes)
//     A7 cmd arg chk 5A                             command (5 bytes), see CMD_*
//   robot -> phone
//     A5 dist dist 5A                               ultrasonic cm, kit format, 10 Hz
//     B6 id float32 chk 5A                          parameter value (reply to set/get)
//     B7 len payload[len] chk 5A                    telemetry, see Send_Telemetry()
//     B8 cmd result chk 5A                          command ack (result 1 = ok)
//
// Text debug mode, for any serial terminal (Arduino Serial Monitor/Plotter, screen,
// serial_monitor.py --text): the default at power-on. The robot prints one readable line per
// telemetry tick instead of the binary frames ("angle:1.23 gyro:-4.5 ..."), plus a parameter
// line; 'b', or any valid binary frame (so the app just works), switches to binary, 'd' back.

#include <BalanceCar.h>
#include <KalmanFilter.h>
#include <EEPROM.h>
#include <Wire.h>

/**********************Instantiate an object**********************/
BalanceCar      balancecar;
KalmanFilter    kalmanfilter;
/*************************END**************************/

/***********************Pin definition***********************/
// ESP32-S3-DevKitC-1 GPIOs. Avoided: 0, 3, 45, 46 (strapping), 19/20 (native USB), 26-37
// (flash and the N16R8's octal PSRAM), 38/48 (the on-board RGB LED), 43/44 (UART0).
// Nano pin each signal came from, in brackets.
#define I2C_SDA         8       // MPU6050 SDA (A4)
#define I2C_SCL         9       // MPU6050 SCL (A5)
// The ultrasonic module controls the pins
#define TRIG            10      // (D3)
#define ECHO            11      // (D5) 5 V output: divide it down to 3.3 V
// RGB Color lamp control pin, active low
#define RLED            13      // (A0)
#define GLED            14      // (A1)
#define BLED            21      // (A2)
// The buzzer controls the pin
#define BUZZER          12      // (D11)
// TB6612 Chip control pins; channel A = left motor, channel B = right motor
#define TB6612_STBY     17      // (D8)
#define TB6612_PWMA     4       // (D10)
#define TB6612_PWMB     7       // (D9)
#define TB6612_AIN1     5       // (D12)
#define TB6612_AIN2     6       // (D13)
#define TB6612_BIN1     15      // (D7)
#define TB6612_BIN2     16      // (D6)
// Motor encoder controls pins
#define MOTOR1          1       // left encoder (D2)
#define MOTOR2          2       // right encoder (D4)
/*************************END**************************/

/***********************Enumeration variable***********************/
enum COLOR
{
    RED = 0,        // red
    GREEN,          // green
    BLUE,           // blue
    YELLOW,         // yellow
    PURPLE,         // purple
    CYAN,           // cyan
    WHITE,          // white
    ALL_OFF         // off(black)
};
/*************************END**************************/

/***********************Protocol***********************/
#define HDR_CONTROL     0xA5
#define HDR_SET_PARAM   0xA6
#define HDR_COMMAND     0xA7
#define HDR_PARAM       0xB6
#define HDR_TELEMETRY   0xB7
#define HDR_ACK         0xB8
#define TAIL            0x5A

#define CMD_GET_PARAMS  1                                   // reply: every parameter, then ack
#define CMD_SAVE        2                                   // write parameters to EEPROM
#define CMD_DEFAULTS    3                                   // restore defaults (not saved)
#define CMD_MOTORS      4                                   // arg 0 = motors off (TB6612 standby), 1 = on
#define CMD_TELEM_RATE  5                                   // arg = telemetry rate in Hz, 0 = off, max 25
#define CMD_LOAD        6                                   // reload parameters from EEPROM
#define KEY_TEXT_DEBUG  'd'                                 // terminal keys, see the protocol above
#define KEY_BINARY      'b'

#define CONTROL_TIMEOUT_MS 1000                             // joystick/horn reset when no control frame arrives
#define SONAR_TIMEOUT_US   25000                            // no echo within this: about 4 m, reported as 0
/*************************END**************************/

/***********************Tunable parameters***********************/
// Index = parameter id on the wire. Order must match the app.
enum PARAM_ID
{
    P_KP = 0,           // angle loop P
    P_KD,               // angle loop D (gyro)
    P_KP_SPEED,         // speed loop P
    P_KI_SPEED,         // speed loop I (position)
    P_KP_TURN,          // turn loop P
    P_KD_TURN,          // turn loop D (gyro z)
    P_ANGLE0,           // mechanical balance angle offset, degrees
    P_DRIVE_SPEED,      // joystick forward/back command at full stick (kit used 30)
    P_COUNT
};
const float PARAM_DEFAULT[P_COUNT] = {40, 1.23, 6.7, 0.199, 10, 0.09, 6.31, 30};
const float PARAM_MIN[P_COUNT]     = { 0, 0,    0,    0,      0,  0,   -15,   0};
const float PARAM_MAX[P_COUNT]     = {100, 10,  50,   2,      50, 5,    15,   100};

#define EEPROM_MAGIC    0x5442                              // "TB"
struct StoredParams
{
    uint16_t magic;
    uint8_t  count;
    float    value[P_COUNT];
    uint8_t  chk;
};

struct Telemetry                                            // one telemetry tick, see Take_Telemetry()
{
    float angle, gyro, angleTerm, speedTerm, turnTerm, pwm1, pwm2;
    int countL, countR, dist;
    bool fallen;
};
/*************************END**************************/

/***********************Concurrency***********************/
// BalanceTask and loop() run on core 1, BalanceTask at the higher priority, so loop() never
// runs in the middle of a control step. LOCK keeps the balance task (and the encoder
// interrupts) out while loop() reads or writes several of the variables it uses, as cli()/sei()
// did on the Nano.
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
#define LOCK()          portENTER_CRITICAL(&lock)
#define UNLOCK()        portEXIT_CRITICAL(&lock)
#define BALANCE_CORE    1
#define BALANCE_PRIO    5                                   // above loop() (1)
#define SONAR_CORE      0
#define SONAR_PRIO      1
/*************************END**************************/

/***********************Variable definitions***********************/
byte TX_package[4] = {0xA5, 0, 0, 0x5A};                    // Packet header(0xA5) + original data (n*byte) + inspection(1byte) + Package the tail(0x5A)
byte RX_package[11] = {0};
byte RX_len = 0;                                            // bytes of the frame received so far
byte RX_want = 0;                                           // expected length of the current frame
unsigned long lastControlMs = 0;
unsigned long lastDistanceTxMs = 0;
unsigned long lastTelemetryMs = 0;
byte telemetryHz = 20;
bool motorsOn = true;
bool textDebug = true;                                      // readable lines instead of binary frames
int8_t x_axis = 0;                                          // Store variables along the X-axis (signed on every core)
int8_t y_axis = 0;                                          // Store variables on the Y axis
byte klaxon = 0;                                            // The default storage rate is1 ~ 255
byte S_Button = 0;                                          // Store a clockwise rotation variable
byte N_Button = 0;                                          // Store counterclockwise rotation variables
volatile byte mode = 1;                                     // 1 remote control, 2 obstacle avoidance, 3 follow

// Ultrasonic distance, cm; written by SonarTask
volatile int UT_distance = 0;

// Pulse calculation
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
volatile int tel_countl = 0;                                // wheel pulses since the last telemetry frame
volatile int tel_countr = 0;

// Kalman_Filte
float Q_angle = 0.001, Q_gyro = 0.005;                      // Angular data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5;                                       // Filter sampling interval in milliseconds
float dt = timeChange * 0.001;                              // Note: Dt is the filter sampling time

// Angle data
float K1 = 0.05;                                            // The weight of the accelerometer
float angle0 = -1;                                          // Angle of mechanical balance

// PID parameters, set from EEPROM or PARAM_DEFAULT in setup()
double kp, ki = 0.0, kd;
double kp_speed, ki_speed, kd_speed = 0.0;                  // kd_speed is ignored by BalanceCar::speedpiout
double kp_turn, ki_turn = 0, kd_turn;
float drive_speed;

int16_t ax, ay, az, gx, gy, gz;

int front = 0;                                              // Forward variables
int back = 0;                                               // Back variables
int turnl = 0;                                              // Turn left sign
int turnr = 0;                                              // Turn right
int spinl = 0;                                              // Rotate the left flag
int spinr = 0;                                              // Rotate the flag right

double setp0 = 0;                                           // Speed loop set point

// Turn and rotate parameters
int turncount = 0;                                          // Calculate the steering intervention time
float turnoutput = 0;

double Outputs = 0;                                         // speed loop output

int speedcc = 0;
volatile long count_right = 0;                              // encoder edges since the last control step
volatile long count_left = 0;
/*************************END**************************/

/*********************************************************
MPU6050 over Wire: the same registers and raw values as the MPU6050 library
*********************************************************/
#define MPU_ADDR            0x68                            // AD0 low
#define MPU_PWR_MGMT_1      0x6B
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_ACCEL_XOUT_H    0x3B

void Mpu_Write(byte reg, byte value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// MPU6050::initialize(): clock from the X gyro PLL, gyro +-250 deg/s, accel +-2 g, awake.
void Mpu_Init()
{
    Mpu_Write(MPU_PWR_MGMT_1, 0x01);
    Mpu_Write(MPU_GYRO_CONFIG, 0x00);
    Mpu_Write(MPU_ACCEL_CONFIG, 0x00);
}

// MPU6050::getMotion6(): the values are left as they were when the read fails.
void Mpu_Read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom(MPU_ADDR, 14) != 14) return;
    byte b[14];
    for (byte i = 0; i < 14; i++) b[i] = Wire.read();
    *ax = (int16_t)(b[0] << 8 | b[1]);
    *ay = (int16_t)(b[2] << 8 | b[3]);
    *az = (int16_t)(b[4] << 8 | b[5]);
    *gx = (int16_t)(b[8] << 8 | b[9]);                      // b[6..7] is the temperature
    *gy = (int16_t)(b[10] << 8 | b[11]);
    *gz = (int16_t)(b[12] << 8 | b[13]);
}

// One control step, every 5 ms: what Timer2Isr() did on the Nano, minus the ultrasonic read.
void Balance_Step()
{
    countpluse();                                           // Pulse superposition subfunction
    Mpu_Read(&ax, &ay, &az, &gx, &gy, &gz);                 // IIC obtains MPU6050 six-axis data ax ay az gx gy gz
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);                                   //Get Angle and Kaman filter
    angleout();                                             // Angle loop PD control
    speedcc++;
    if (speedcc >= 8)                                       // 40 ms into the speed loop control
    {
        Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
        speedcc = 0;
    }
    turncount++;
    if (turncount > 4)                                      // 25 ms into the rotation control
    {
        turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);                              //Rotate the subfunction
        turncount = 0;
    }
    balancecar.posture++;
    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB);                            //Total PWM output
}

void BalanceTask(void *)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
        Balance_Step();
    }
}

// HC-SR04 about every 20 ms, as the Nano did from its timer; 0 = no echo. pulseIn busy-waits,
// so the pause comes after it: with a timed-out echo every time, a fixed period would never
// let core 0's idle task run and the task watchdog would reset the chip.
void SonarTask(void *)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);
        unsigned long us = pulseIn(ECHO, HIGH, SONAR_TIMEOUT_US);
        UT_distance = us / 29 / 2;                          // as Adeept_Distance::getDistanceCentimeter()
    }
}

void IRAM_ATTR Code_right()
{
    portENTER_CRITICAL_ISR(&lock);
    count_right ++;
    portEXIT_CRITICAL_ISR(&lock);
}   //Right speed dial count
void IRAM_ATTR Code_left()
{
    portENTER_CRITICAL_ISR(&lock);
    count_left ++;
    portEXIT_CRITICAL_ISR(&lock);
}   //Left speed gauge count

void setup()
{
    Pin_Config();                                           // Module pin configuration
    Pin_Init();                                             // Module pin initialization
    Wire.begin(I2C_SDA, I2C_SCL);                           // Join the I2C bus sequence
    Wire.setClock(400000);
    Serial.begin(115200);                                   // USB serial (the app's SerialLink.BAUD)
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);                               // no host reading: drop output, do not block
#endif
    EEPROM.begin(sizeof(StoredParams));
    if (!Load_Params())
    {
        Default_Params();
    }
    Mpu_Init();                                             // Initialize the MPU6050
    delay(1500);                                            // Wait for the system to stabilize
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    attachInterrupt(digitalPinToInterrupt(MOTOR1), Code_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR2), Code_right, CHANGE);
    xTaskCreatePinnedToCore(BalanceTask, "balance", 4096, nullptr, BALANCE_PRIO, nullptr, BALANCE_CORE);
    xTaskCreatePinnedToCore(SonarTask, "sonar", 2048, nullptr, SONAR_PRIO, nullptr, SONAR_CORE);
    Print_Params();                                         // starts in text debug mode
}

void loop()
{
    RX_Information();                                       // Receive serial data
    TX_Information();                                       // Send distance and telemetry

    if (millis() - lastControlMs > CONTROL_TIMEOUT_MS)      // Link lost: stop driving and the horn
    {
        klaxon = 0;
        x_axis = 0;
        y_axis = 0;
        N_Button = 0;
        S_Button = 0;
    }

    if(mode == 1)
    {
        mode1();
    }
    else if(mode == 2)
    {
        mode2();
    }
    else if(mode == 3)
    {
        mode3();
    }
    delay(1);                                               // let the idle task feed the watchdog
}


void angleout()
{
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD Angle loop control
}

void countpluse()
{
    LOCK();                                                 // read and clear together: no edge lost in between
    lz = count_left;
    rz = count_right;
    count_left = 0;
    count_right = 0;
    UNLOCK();

    lpluse = lz;
    rpluse = rz;

    if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //When the moving direction of the trolley is judged to be backward (PWM motor voltage is negative), the pulse number is negative
    {
        rpluse = -rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                //Left wheel backward
    {
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))                //Right wheel backward
    {
        rpluse = -rpluse;
    }

    // Bring up the judgment
    balancecar.stopr += rpluse;
    balancecar.stopl += lpluse;

    // Every 5ms when the interrupt enters, the pulse number is superimposed
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;

    tel_countl += lpluse;
    tel_countr += rpluse;
}

/*********************************************************
Function name: mode1()
Function Function: Control the car direction and speed over the serial link.
Forward/back speed is proportional to the stick; turning can be combined with it.
*********************************************************/
void mode1()
{
    digitalWrite(BUZZER, klaxon > 0 ? HIGH : LOW);          // speaker switch

    int f = 0, b = 0, tl = 0, tr = 0, sl = 0, sr = 0;
    if(N_Button > 0)                                        // Rotate counterclockwise
    {
        sl = 1;
    }
    else if(S_Button > 0)                                   // Rotate clockwise
    {
        sr = 1;
    }
    else
    {
        int y = y_axis;
        int x = x_axis;
        int v = (int)(drive_speed * (abs(y) - 30) / 97);    // 0 at the dead band edge, drive_speed at full stick
        if (y > 30)                                         // Forward motion (the kit's sign convention)
        {
            b = -v;
        }
        else if (y < -30)                                   // Move backward
        {
            f = v;
        }
        if (x < -30)                                        // left
        {
            tl = 1;
        }
        else if (x > 30)                                    // right
        {
            tr = 1;
        }
    }

    LOCK();
    front = f; back = b; turnl = tl; turnr = tr; spinl = sl; spinr = sr;
    UNLOCK();

    RGB((f || b || tl || tr || sl || sr) ? GREEN : RED);
}

/*********************************************************
Mode2 ()
Function Function: Obstacle avoidance mode
Function parameters: None
The function returns: none
*********************************************************/
void mode2()
{
    if(UT_distance < 8)
    {
        LOCK();
        ResetCarState();
        front = 15;
        UNLOCK();
        RGB(RED);
    }
    else if(UT_distance < 15)
    {
        RGB(BLUE);
        turnr = 1;
        delay(200);
    }
    else
    {
        RGB(GREEN);
        LOCK();
        ResetCarState();
        back = -15;
        UNLOCK();
    }
}

/*********************************************************
Mode3 ()
Function Function: follow mode
Function parameters: None
The function returns: none
*********************************************************/
void mode3()
{
    if(UT_distance >= 20 && UT_distance < 35)
    {
        LOCK();
        ResetCarState();
        back = -15;
        UNLOCK();
        RGB(GREEN);
    }
    else if(UT_distance < 20)
    {
        LOCK();
        ResetCarState();
        front = 15;
        UNLOCK();
        RGB(BLUE);
    }
    else
    {
        RGB(RED);
        LOCK();
        ResetCarState();
        UNLOCK();
    }
}



void ResetCarState()
{
    turnl = 0;
    turnr = 0;
    front = 0;
    back = 0;
    spinl = 0;
    spinr = 0;
    turnoutput = 0;
}

/*********************************************************
Parameters: defaults, EEPROM, and the live variables the control loop reads
*********************************************************/
float Get_Param(byte id)
{
    switch (id)
    {
    case P_KP:          return kp;
    case P_KD:          return kd;
    case P_KP_SPEED:    return kp_speed;
    case P_KI_SPEED:    return ki_speed;
    case P_KP_TURN:     return kp_turn;
    case P_KD_TURN:     return kd_turn;
    case P_ANGLE0:      return angle0;
    case P_DRIVE_SPEED: return drive_speed;
    }
    return 0;
}

// Clamps to PARAM_MIN..MAX; the balance task reads these, so writes are atomic.
void Set_Param(byte id, float v)
{
    if (id >= P_COUNT || isnan(v)) return;
    v = constrain(v, PARAM_MIN[id], PARAM_MAX[id]);
    LOCK();
    switch (id)
    {
    case P_KP:          kp = v; break;
    case P_KD:          kd = v; break;
    case P_KP_SPEED:    kp_speed = v; break;
    case P_KI_SPEED:    ki_speed = v; break;
    case P_KP_TURN:     kp_turn = v; break;
    case P_KD_TURN:     kd_turn = v; break;
    case P_ANGLE0:      angle0 = v; break;
    case P_DRIVE_SPEED: drive_speed = v; break;
    }
    UNLOCK();
}

void Default_Params()
{
    for (byte i = 0; i < P_COUNT; i++) Set_Param(i, PARAM_DEFAULT[i]);
}

byte Stored_Chk(const StoredParams &s)
{
    const byte *p = (const byte *)&s;
    byte sum = 0;
    for (byte i = 0; i < offsetof(StoredParams, chk); i++) sum += p[i];
    return sum;
}

bool Load_Params()
{
    StoredParams s;
    EEPROM.get(0, s);
    if (s.magic != EEPROM_MAGIC || s.count != P_COUNT || s.chk != Stored_Chk(s)) return false;
    for (byte i = 0; i < P_COUNT; i++) Set_Param(i, s.value[i]);
    return true;
}

bool Save_Params()
{
    StoredParams s;
    memset(&s, 0, sizeof(s));                               // padding bytes count in the checksum
    s.magic = EEPROM_MAGIC;
    s.count = P_COUNT;
    for (byte i = 0; i < P_COUNT; i++) s.value[i] = Get_Param(i);
    s.chk = Stored_Chk(s);
    EEPROM.put(0, s);
    return EEPROM.commit();                                 // the flash write happens here
}

/*********************************************************
Function name: RX_Information()
Function Function: Collects frames byte by byte without blocking; see the protocol
at the top of the file.
*********************************************************/
void RX_Information(void)
{
    while (Serial.available() > 0)
    {
        byte c = Serial.read();
        if (RX_len == 0)
        {
            if (c == HDR_CONTROL)          RX_want = 11;
            else if (c == HDR_SET_PARAM)   RX_want = 8;
            else if (c == HDR_COMMAND)     RX_want = 5;
            else
            {
                if (c == KEY_TEXT_DEBUG && !textDebug) { textDebug = true; Print_Params(); }
                else if (c == KEY_BINARY)          textDebug = false;
                continue;                                   // not a header: keep hunting
            }
        }
        RX_package[RX_len++] = c;
        if (RX_len < RX_want) continue;
        RX_len = 0;
        if (RX_package[RX_want - 1] != TAIL) continue;      // bad frame: drop it and resync

        if (RX_package[0] == HDR_CONTROL)
        {
            textDebug = false;                              // a binary client (the app) is talking
            Handle_Control();
            continue;
        }
        byte sum = 0;
        for (byte i = 1; i < RX_want - 2; i++) sum += RX_package[i];
        if (sum != RX_package[RX_want - 2]) continue;
        textDebug = false;
        if (RX_package[0] == HDR_SET_PARAM)
        {
            float v;
            memcpy(&v, &RX_package[2], 4);
            Set_Param(RX_package[1], v);
            Send_Param(RX_package[1]);                      // echo the value actually applied
        }
        else
        {
            Handle_Command(RX_package[1], RX_package[2]);
        }
    }
}

void Handle_Control()
{
    lastControlMs = millis();
    x_axis = (int8_t)RX_package[1];                         // The X-axis value
    y_axis = (int8_t)RX_package[2];                         // The Y-axis value
    klaxon = RX_package[3];                                 // the horn
    N_Button = RX_package[4];                               // Rotate counterclockwise
    S_Button = RX_package[5];                               // Rotate clockwise
    byte want = RX_package[6] ? 1 : RX_package[7] ? 2 : RX_package[8] ? 3 : 0;
    if (want && want != mode)                               // only act on a mode change
    {
        LOCK();
        ResetCarState();
        UNLOCK();
        digitalWrite(BUZZER, LOW);
        RGB(want == 1 ? GREEN : want == 2 ? BLUE : PURPLE);
        mode = want;
    }
}

void Handle_Command(byte cmd, byte arg)
{
    byte ok = 1;
    switch (cmd)
    {
    case CMD_GET_PARAMS:
        for (byte i = 0; i < P_COUNT; i++) Send_Param(i);
        break;
    case CMD_SAVE:
        ok = Save_Params();
        break;
    case CMD_DEFAULTS:
        Default_Params();
        for (byte i = 0; i < P_COUNT; i++) Send_Param(i);
        break;
    case CMD_MOTORS:
        motorsOn = arg != 0;
        digitalWrite(TB6612_STBY, motorsOn ? HIGH : LOW);   // standby: outputs float, control keeps running
        break;
    case CMD_TELEM_RATE:
        telemetryHz = min((int)arg, 25);
        break;
    case CMD_LOAD:
        ok = Load_Params();
        for (byte i = 0; i < P_COUNT; i++) Send_Param(i);
        break;
    default:
        ok = 0;
    }
    byte ack[5] = {HDR_ACK, cmd, ok, (byte)(cmd + ok), TAIL};
    Serial.write(ack, 5);
}

void Send_Param(byte id)
{
    if (id >= P_COUNT) return;
    byte f[8] = {HDR_PARAM, id};
    float v = Get_Param(id);
    memcpy(&f[2], &v, 4);
    byte sum = 0;
    for (byte i = 1; i < 6; i++) sum += f[i];
    f[6] = sum;
    f[7] = TAIL;
    Serial.write(f, 8);
}

/*********************************************************
Function name: TX_Information()
Function Function: Sends the kit's distance packet at 10 Hz and the telemetry frame
at telemetryHz. At 115200 baud the link carries ~11500 bytes/s; 20 Hz telemetry uses ~560,
a text debug line ~100 bytes (2 kB/s at 20 Hz).
*********************************************************/
void TX_Information(void)
{
    unsigned long now = millis();
    if (!textDebug && now - lastDistanceTxMs >= 100)
    {
        lastDistanceTxMs = now;
        byte dat = UT_distance > 127 ? 127 : UT_distance;
        TX_package[1] = dat;
        TX_package[2] = dat;                                // Check the sum
        Serial.write(TX_package, 4);
    }
    if (telemetryHz && now - lastTelemetryMs >= 1000 / telemetryHz)
    {
        lastTelemetryMs = now;
        Telemetry t;
        Take_Telemetry(t);
        if (textDebug) Print_Telemetry(t);
        else           Send_Telemetry(now, t);
    }
}

void Put16(byte *p, int v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

int Clamp16(float v)
{
    return (int)constrain(v, -32000.0, 32000.0);
}

// Consistent snapshot of what the balance task computes; resets the pulse counters.
void Take_Telemetry(Telemetry &t)
{
    LOCK();
    t.angle = kalmanfilter.angle;
    t.gyro = kalmanfilter.Gyro_x;
    t.angleTerm = balancecar.angleoutput;
    t.speedTerm = Outputs;
    t.turnTerm = turnoutput;
    t.pwm1 = balancecar.pwm1;
    t.pwm2 = balancecar.pwm2;
    t.countL = tel_countl;
    t.countR = tel_countr;
    tel_countl = 0;
    tel_countr = 0;
    t.dist = UT_distance;
    UNLOCK();
    t.fallen = t.angle > 30 || t.angle < -30;
}

// Payload, int16 each: t_ms, angle*100, gyro_x*10, angle term, speed term, turn term*10,
// pwm left, pwm right, left pulses, right pulses (since last frame), distance cm, flags
// (bit0 motors on, bit1 fallen |angle|>30, bits2-3 mode).
void Send_Telemetry(unsigned long now, const Telemetry &t)
{
    const byte n = 24;
    byte f[n + 4];
    f[0] = HDR_TELEMETRY;
    f[1] = n;
    byte *p = &f[2];
    Put16(p + 0, (int)(now & 0xFFFF));
    Put16(p + 2, Clamp16(t.angle * 100));
    Put16(p + 4, Clamp16(t.gyro * 10));
    Put16(p + 6, Clamp16(t.angleTerm));
    Put16(p + 8, Clamp16(t.speedTerm));
    Put16(p + 10, Clamp16(t.turnTerm * 10));
    Put16(p + 12, Clamp16(t.pwm1));
    Put16(p + 14, Clamp16(t.pwm2));
    Put16(p + 16, t.countL);
    Put16(p + 18, t.countR);
    Put16(p + 20, t.dist);
    Put16(p + 22, (motorsOn ? 1 : 0) | (t.fallen ? 2 : 0) | (mode << 2));
    byte sum = 0;
    for (byte i = 0; i < n; i++) sum += p[i];
    f[n + 2] = sum;
    f[n + 3] = TAIL;
    Serial.write(f, n + 4);
}

void Print_Field(const __FlashStringHelper *name, float v, byte decimals)
{
    Serial.print(name);
    Serial.print(v, decimals);
    Serial.print(' ');
}

// One "name:value" line per tick; the Arduino Serial Plotter graphs every field.
// tilt = angle + angle0, the error the angle loop drives to zero.
void Print_Telemetry(const Telemetry &t)
{
    Print_Field(F("tilt:"), t.angle + angle0, 2);
    Print_Field(F("angle:"), t.angle, 2);
    Print_Field(F("gyro:"), t.gyro, 1);
    Print_Field(F("A:"), t.angleTerm, 0);
    Print_Field(F("S:"), t.speedTerm, 0);
    Print_Field(F("T:"), t.turnTerm, 1);
    Print_Field(F("pwmL:"), t.pwm1, 0);
    Print_Field(F("pwmR:"), t.pwm2, 0);
    Print_Field(F("pulL:"), t.countL, 0);
    Print_Field(F("pulR:"), t.countR, 0);
    Print_Field(F("dist:"), t.dist, 0);
    Print_Field(F("motors:"), motorsOn, 0);
    Print_Field(F("fallen:"), t.fallen, 0);
    Serial.print(F("mode:"));
    Serial.println(mode);
}

void Print_Params()
{
    Serial.println();
    Serial.print(F("# text debug ('b' = binary). kp:"));
    Serial.print(kp, 2);
    Serial.print(F(" kd:"));
    Serial.print(kd, 3);
    Serial.print(F(" kp_speed:"));
    Serial.print(kp_speed, 3);
    Serial.print(F(" ki_speed:"));
    Serial.print(ki_speed, 4);
    Serial.print(F(" kp_turn:"));
    Serial.print(kp_turn, 2);
    Serial.print(F(" kd_turn:"));
    Serial.print(kd_turn, 3);
    Serial.print(F(" angle0:"));
    Serial.print(angle0, 2);
    Serial.print(F(" drive_speed:"));
    Serial.print(drive_speed, 0);
    Serial.print(F(" telemetry_hz:"));
    Serial.println(telemetryHz);
}

/*********************************************************
Function name: Pin_Init()
Function Initialize pin high/low level
Function parameters: None
The function returns: none
*********************************************************/
void Pin_Init()
{
    digitalWrite(BUZZER, LOW);                              // The buzzer controls the pin output low level
    digitalWrite(TRIG, LOW);

    digitalWrite(TB6612_STBY, HIGH);                        // TB6612 enable control pin output high level
    digitalWrite(TB6612_PWMA, LOW);                         // TB6612 PWMA control pin output low level
    digitalWrite(TB6612_PWMB, LOW);                         //
    digitalWrite(TB6612_AIN1, LOW);                         //
    digitalWrite(TB6612_AIN2, HIGH);                        //
    digitalWrite(TB6612_BIN1, HIGH);                        //
    digitalWrite(TB6612_BIN2, LOW);                         //
    RGB(ALL_OFF);
}

/*********************************************************
Function name: Pin_Config()
The pin I/O function configures pin I/O mode
Function parameters: None
The function returns: none
*********************************************************/
void Pin_Config()
{
    pinMode(TRIG, OUTPUT);                                  // Ultrasonic Trig control pin configuration output
    pinMode(ECHO, INPUT);                                   // Ultrasonic Echo control pin configuration input

    pinMode(RLED, OUTPUT);                                  // RGB color lights red control pin configuration output
    pinMode(GLED, OUTPUT);                                  // RGB color light green control pin configuration output
    pinMode(BLED, OUTPUT);                                  // RGB color light blue control pin configuration output

    pinMode(BUZZER, OUTPUT);                                // Buzzer control pin configuration output

    pinMode(TB6612_STBY, OUTPUT);                           // TB6612 Enable control pin configuration output
    pinMode(TB6612_PWMA, OUTPUT);                           // TB6612 PWMA control pin configuration output
    pinMode(TB6612_PWMB, OUTPUT);                           // TB6612 PWMB controls pin configuration output
    pinMode(TB6612_AIN1, OUTPUT);                           //
    pinMode(TB6612_AIN2, OUTPUT);                           //
    pinMode(TB6612_BIN1, OUTPUT);                           //
    pinMode(TB6612_BIN2, OUTPUT);                           //

    pinMode(MOTOR1, INPUT_PULLUP);                          // left encoder (the Nano left D2 floating; pull-up is harmless)
    pinMode(MOTOR2, INPUT_PULLUP);                          // right encoder, pull-up as on the Nano
}

/*********************************************************
Function name: RGB()
Function function: control RGB output various colors
Function parameters: color, various color reference enumeration@color
The function returns: none
*********************************************************/
void RGB(enum COLOR color)
{
    switch (color)
    {
    case RED:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, HIGH);
        break;
    case GREEN:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, HIGH);
        break;
    case BLUE:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, LOW);
        break;
    case YELLOW:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, HIGH);
        break;
    case PURPLE:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, LOW);
        break;
    case CYAN:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, LOW);
        break;
    case WHITE:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, LOW);
        break;
    default:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, HIGH);
        break;
    }
}
