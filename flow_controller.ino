// 06 Nov 2024

#include <Arduino.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "hardware/gpio.h"
#include <Wire.h>
#include <EEPROM.h>
#include "pico/multicore.h"

// Define specific Modbus holding register addresses
#define ValveType 14
#define Proportional 15
#define Integral 16
#define Derivative 17
#define Speed 18        // Speed/Step
#define Media 19        // Valve 1 Media Size
#define Freq 20         // Valve 1 Frequency
#define Gain_Select 21  // Valve 1 Gain Select
#define Proximity 23    // Proximity Homing Sensor
#define P_SlaveID 25    // Programmable Slave ID
#define Start 30        // Control Signal
#define Auto_Manual 31  // Auto/Manual Mode
#define Save 32         // Flash Save
#define Save3 34        // Flash Save 3 (uWave Equation)
#define FB 40           // Flow Sensor Feedback
#define Duty_FB 41      // Duty Cycle Feedback
#define Flow 50         // Flowrate setting
#define Duty 58         // Duty Cycle (Manual Setting)
#define E1 60           // Equation 1 Media Size
#define E2 65           // Equation 2 Media Size
#define E3 70           // Equation 3 Media Size
#define E4 75           // Equation 4 Media Size

// Pin definitions
#define PROXIMITY_PIN 15
#define VALVE_PIN 13  // Yellow LED
#define Red_LED 24
#define Green_LED 25

// Constants and variables
#define I2C_ADDRESS 0x55
#define UART_BAUD 115200
#define pid_period 55500  // PID period
// Timer Interval Constants
#define I2C_INTERVAL_MS 500
#define MODBUS_INTERVAL_MS 50
#define DEBUG_INTERVAL_MS 5000
#define MAX_HOLDING_REG 100
#define MEDIA_SECTION_SIZE 20
#define RS485 Serial1
#define MOTOR Serial2
// I2C pin definitions for GPIO27 (SCL) and GPIO26 (SDA)
#define I2C_SCL_PIN 27
#define I2C_SDA_PIN 26
#define AVG_FLOW_RATE_SIZE 16  // Size of the buffer for averaging

int holdingReg[MAX_HOLDING_REG] = { 0 };
int MediaConstant[MAX_HOLDING_REG] = { 0 };

uint8_t SlaveAddress = 1;  //Slave Address
//Media
uint8_t last_Media = 0;
uint8_t last_E1 = 0, last_E2 = 0, last_E3 = 0, last_E4 = 0;

// Define the points (x1, y1) and (x2, y2)
// Calculate the line equation and constants
int X1 = 321;
int Y1 = 35;
int X2 = 2981;
int Y2 = 60;
float Const_M, Const_C;
float Offset, Gradient;

uint16_t PID_Check = 0;  // PID check
uint16_t delay_start = 0;
float SMC_Target = 0;       // Stepper Motor Position (0 ~ 255)
uint16_t Avg_FlowRate = 0;  //
uint16_t Reported_FlowRate = 0;
uint8_t Stepper_Motor_position = 0;  // Stepper Motor Position (0 ~ 255)

int datacount = 0;
float Kp, Ki, Kd;
float pid_output, error, integral_error = 0, last_error = 0, derivative;
int FlowRate_when_Valve_Closed = 0;
int initial_opening = 0;
int Duty_Freq = 0;
bool green_led_state = false;

unsigned long last_blink_time = 0;
uint8_t modbusBuffer[100];
byte response[100];
int FB_values[100];
int i;

volatile bool i2cReadFlag = false;
volatile bool Valve_On = false;           // False
volatile bool modbusProcessFlag = false;  // Flag to trigger Modbus process in main loop
volatile bool DebugProcessFlag = false;
volatile bool modbusDataReady = false;   // Flag to signal Modbus data processing
volatile bool valveControlFlag = false;  // Flag to indicate the ISR has fired
RPI_PICO_TimerInterrupt ITimer0(0), ITimer2(2), RS485_Timer(3), ValveControlTimer(4);


int flow_rate_buffer[AVG_FLOW_RATE_SIZE] = { 0 };  // Circular buffer to store the last 16 Avg_FlowRate values
int flow_rate_index = 0;                           // Index to keep track of the current position in the buffer
int flow_rate_sum = 0;                             // Sum of all values in the buffer

// Function to update the buffer and calculate the average of the last 16 values
int update_and_calculate_average(int new_value) {
  // Subtract the oldest value from the sum
  flow_rate_sum -= flow_rate_buffer[flow_rate_index];

  // Add the new value to the buffer and sum
  flow_rate_buffer[flow_rate_index] = new_value;
  flow_rate_sum += new_value;

  // Increment the index, wrapping around if necessary
  flow_rate_index = (flow_rate_index + 1) % AVG_FLOW_RATE_SIZE;

  // Calculate and return the average
  return flow_rate_sum / AVG_FLOW_RATE_SIZE;
}

// CRC Calculation and Validation
uint16_t calculateCRC(uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

bool validateCRC(uint8_t *frame, uint16_t length) {
  uint16_t receivedCRC = (frame[length - 2] | (frame[length - 1] << 8));
  return calculateCRC(frame, length - 2) == receivedCRC;
}

void read_reg(byte *data) {
  int reg_address = (data[0] << 8) | data[1];
  int num_regs = (data[2] << 8) | data[3];

  // Check if requested registers are within the valid range
  if (reg_address < MAX_HOLDING_REG && (reg_address + num_regs) <= MAX_HOLDING_REG) {
    response[0] = SlaveAddress;  // Set slave address
    response[1] = 0x03;          // Function code for Read Holding Registers
    response[2] = num_regs * 2;  // Byte count for requested registers

    // Populate response with requested register values
    for (int i = 0, j = 3; i < num_regs; i++, j += 2) {
      response[j] = (holdingReg[reg_address + i] >> 8) & 0xFF;  // High byte
      response[j + 1] = holdingReg[reg_address + i] & 0xFF;     // Low byte
    }

    // Calculate and append CRC to the response
    uint16_t crc = calculateCRC(response, 3 + num_regs * 2);
    response[3 + num_regs * 2] = crc & 0xFF;  // CRC low byte
    response[4 + num_regs * 2] = crc >> 8;    // CRC high byte

    // Send response via RS485
    RS485.write(response, 5 + num_regs * 2);
    RS485.flush();  // Ensure all data is transmitted
  }
}

// Modbus Register Write
void write_reg(byte *data) {
  int reg_address = (data[0] << 8) | data[1];
  int value = (data[2] << 8) | data[3];

  if (reg_address < 100) {
    holdingReg[reg_address] = value;

    memcpy(response, data - 2, 6);
    uint16_t crc = calculateCRC(response, 6);
    response[6] = crc & 0xFF;
    response[7] = crc >> 8;

    RS485.write(response, 8);
    RS485.flush();  // Ensure all data is transmitted
  }
}

// Write Multiple Registers
void write_multiple_regs(byte *data) {
  // Extract start address and number of registers
  int start_address = (data[0] << 8) | data[1];
  int num_regs = (data[2] << 8) | data[3];
  byte byte_count = data[4];

  // Debug print statements to trace start address, number of registers, and byte count
  /*
  Serial.print("Start Address: ");
  Serial.println(start_address);
  Serial.print("Number of Registers: ");
  Serial.println(num_regs);
  Serial.print("Byte Count: ");
  Serial.println(byte_count);*/

  // Check if byte_count is valid
  if (byte_count != num_regs * 2) {
    Serial.println("Error: Byte count does not match the number of registers.");
    return;  // Exit if byte count is incorrect
  }

  // Ensure we don't write out of bounds
  if (start_address + num_regs > MAX_HOLDING_REG) {
    Serial.println("Error: Attempt to write beyond holding register limits.");
    return;  // Exit if the requested range exceeds the holding register limit
  }

  // Write data to holding registers
  int j = 5;
  for (int i = 0; i < num_regs; i++) {
    // Write data to holding registers: high byte first, then low byte
    holdingReg[start_address + i] = (data[j] << 8) | data[j + 1];

    // Debug print each register and value written
    /*
    Serial.print("Writing to holdingReg[");
    Serial.print(start_address + i);
    Serial.print("]: 0x");
    Serial.println(holdingReg[start_address + i]);*/

    j += 2;  // Move to the next pair of bytes
  }

  // Prepare Modbus response
  response[0] = SlaveAddress;
  response[1] = 0x10;             // Function code for Write Multiple Registers
  memcpy(&response[2], data, 4);  // Start address and number of registers

  // Calculate and append CRC
  uint16_t crc = calculateCRC(response, 6);
  response[6] = crc & 0xFF;  // Low byte of CRC
  response[7] = crc >> 8;    // High byte of CRC

  // Debug final response being sent
  /*
  Serial.print("Response: ");
  for (int k = 0; k < 8; k++) {
    Serial.print(response[k], HEX);
    Serial.print(" ");
  }
  Serial.println();*/

  // Send response via RS485
  RS485.write(response, 8);
  RS485.flush();  // Ensure all data is transmitted
}


void initializeHoldingRegisters() {
  EEPROM.get(0, holdingReg);  // Retrieve entire holdingReg array from EEPROM

  // Check if EEPROM is uninitialized or holdingReg[0] is invalid
  if (holdingReg[0] == -1) {
    // Set default values if EEPROM is uninitialized
    holdingReg[0] = 0x574D;
    holdingReg[1] = 0x312D;
    holdingReg[2] = 0x0035;
    holdingReg[3] = 0x0000;

    // Serial Number
    holdingReg[4] = 0x002D;
    holdingReg[5] = 0x0000;
    holdingReg[6] = 0x0000;
    holdingReg[7] = 0x0000;

    // Manufacturing Date
    holdingReg[8] = 0x002D;
    holdingReg[9] = 0x0000;
    holdingReg[10] = 0x0000;

    // Revision
    holdingReg[11] = 0x3252;  // 2R
    holdingReg[12] = 0x302E;  // 0.
    holdingReg[13] = 0x0036;  // 6 - Update this when revision is updated

    // Additional Parameters
    holdingReg[ValveType] = 0x0000;  // Indicate Microwave Valve
    holdingReg[Proportional] = 9;    // Used in µWave_FC
    holdingReg[Integral] = 2;        // Used in µWave_FC
    holdingReg[Derivative] = 2;      // Used in µWave_FC
    holdingReg[Speed] = 1000;        // Not used in µWave_FC
    holdingReg[Media] = 0;
    holdingReg[Freq] = 17;        // Used in µWave_FC
    holdingReg[Gain_Select] = 2;  // Not used in µWave_FC

    holdingReg[Flow] = 0;  // Flowrate Setting

    // Media Constant Flash Init
    holdingReg[E1] = 0;  // Media 1 Value
    holdingReg[E2] = 0;  // Media 2 Value
    holdingReg[E3] = 0;  // Media 3 Value
    holdingReg[E4] = 0;  // Media 4 Value

    holdingReg[E1 + 1] = 500;
    holdingReg[E1 + 2] = 2500;
    holdingReg[E1 + 3] = 30;
    holdingReg[E1 + 4] = 50;
    holdingReg[E2 + 1] = 500;
    holdingReg[E2 + 2] = 2500;
    holdingReg[E2 + 3] = 30;
    holdingReg[E2 + 4] = 50;
    holdingReg[E3 + 1] = 500;
    holdingReg[E3 + 2] = 2500;
    holdingReg[E3 + 3] = 30;
    holdingReg[E3 + 4] = 50;
    holdingReg[E4 + 1] = 500;
    holdingReg[E4 + 2] = 2500;
    holdingReg[E4 + 3] = 30;
    holdingReg[E4 + 4] = 50;

    // P_SlaveID
    holdingReg[P_SlaveID] = 0;  // Programmable Slave ID

    // Save initialized values to EEPROM
    saveHoldingRegisters();
  }
}

void saveHoldingRegisters() {
  EEPROM.put(0, holdingReg);  // Store entire holdingReg array to EEPROM
  EEPROM.commit();            // Commit changes to EEPROM
  Serial.println("Holding registers saved to EEPROM.");
}

// Function to check conditions and save as needed
void CheckAndSave() {
  // Check if Save flag is set and Freq is non-zero
  if (holdingReg[Save] == 1 && holdingReg[Freq] != 0) {
    holdingReg[Save] = 0;
    saveHoldingRegisters();
  }

  // Check if Save3 flag is set
  if (holdingReg[Save3] == 1) {
    holdingReg[Save3] = 0;
    saveHoldingRegisters();
    Media_Write();  // Save initialized values to EEPROM
  }
}

// Save MediaConstant to EEPROM
void saveMediaConstant() {
  EEPROM.put(400, MediaConstant);
  EEPROM.commit();
  Serial.println("Media constants saved to EEPROM.");
}



// I2C Read Timer ISR
bool TimerISR_I2C(struct repeating_timer *t) {
  i2cReadFlag = true;
  return true;
}

// Modbus Processing Timer ISR
bool TimerISR_Debug(struct repeating_timer *t) {
  DebugProcessFlag = true;
  return true;
}

// ISR for RS485 to process Modbus request
bool RS485_ISR(struct repeating_timer *t) {
  modbusDataReady = RS485.available() > 0;  // Check if data is available
  return true;                              // Continue the timer
}

void setup() {
  Serial.begin(115200);
  RS485.setRX(1);
  RS485.setTX(0);
  RS485.begin(115200);
  MOTOR.setRX(5);
  MOTOR.setTX(4);
  MOTOR.begin(115200);

  pinMode(Red_LED, OUTPUT);
  pinMode(Green_LED, OUTPUT);
  pinMode(PROXIMITY_PIN, INPUT);
  pinMode(VALVE_PIN, OUTPUT);
  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin();
  EEPROM.begin(1000);

  ITimer0.attachInterruptInterval(I2C_INTERVAL_MS * 1000, TimerISR_I2C);
  ITimer2.attachInterruptInterval(DEBUG_INTERVAL_MS * 1000, TimerISR_Debug);

  initializeHoldingRegisters();

  holdingReg[Start] = 1;  // 0x01 = STOP 停止
  holdingReg[Duty] = 0;   // Duty Cycle (Manual Setting) (0 ~ 769)
  holdingReg[Flow] = 0;   // Flowrate Setting 流量设定 (0 ~ 65535)
  holdingReg[FB] = 0;
  holdingReg[Auto_Manual] = 2;

  last_Media = 0;
  last_E1 = holdingReg[E1];
  last_E2 = holdingReg[E2];
  last_E3 = holdingReg[E3];
  last_E4 = holdingReg[E4];
  Media_Read();

  if (holdingReg[P_SlaveID] != 0 && SlaveAddress != holdingReg[P_SlaveID]) SlaveAddress = holdingReg[P_SlaveID];
}


void setup1() {
  RS485_Timer.attachInterruptInterval(MODBUS_INTERVAL_MS * 1000, RS485_ISR);  // 30ms interval
}

void loop1() {
  if (modbusDataReady) {
    modbusDataReady = false;  // Clear the flag
    process_modbus_request();
  }
}

/*
    Address: 1 byte
    Function Code: 1 byte
    Start Address: 2 bytes
    Quantity of Registers: 2 bytes
    Byte Count: 1 byte
    Register Data: (2 * Quantity of Registers) bytes
    CRC: 2 bytes
*/

void process_modbus_request() {
  static uint8_t bufferIndex = 0;
  const int minimumFrameLength = 8;
  int expectedFrameLength = minimumFrameLength;
  unsigned long startTime = millis();  // Record the start time for timeout handling
  unsigned long timeoutDuration = 6;   // Set timeout to 6 ms (slightly longer than 4ms needed for 49 bytes)

  // Step 1: Read available bytes from RS485 within the timeout period
  while (millis() - startTime < timeoutDuration) {
    // Check if bytes are available in RS485 buffer
    if (RS485.available()) {
      uint8_t byteReceived = RS485.read();
      modbusBuffer[bufferIndex++] = byteReceived;

      // Adjust expected length based on function and register count (after receiving first 6 bytes)
      if (bufferIndex == 6 && modbusBuffer[1] == 0x10) {
        int numberOfRegisters = (modbusBuffer[4] << 8) | modbusBuffer[5];
        expectedFrameLength = 7 + (numberOfRegisters * 2) + 2;  // Full frame length for Write Multiple Registers
      }

      // Safety check: prevent buffer overflow
      if (bufferIndex >= sizeof(modbusBuffer)) {
        Serial.println("Buffer overflow, resetting.");
        bufferIndex = 0;
        memset(modbusBuffer, 0, sizeof(modbusBuffer));  // Clear the buffer
        return;
      }
    }

    // Exit if the full frame is received
    if (bufferIndex >= expectedFrameLength) {
      break;
    }
  }

  // Step 2: Check if the full frame is received or if timeout occurred
  if (bufferIndex >= expectedFrameLength) {
    // Validate Slave Address
    if (modbusBuffer[0] == SlaveAddress) {
      uint16_t extractedCRC = modbusBuffer[expectedFrameLength - 2] | (modbusBuffer[expectedFrameLength - 1] << 8);
      uint16_t calculatedCRC = calculateCRC(modbusBuffer, expectedFrameLength - 2);

      if (extractedCRC == calculatedCRC) {
        validate_and_process_frame(modbusBuffer, expectedFrameLength);  // Process valid frame
      } else {
        Serial.println("CRC check failed. Discarding frame.");
        Serial.print("Frame Data: ");
        for (int i = 0; i < bufferIndex; i++) {
          Serial.print(modbusBuffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  } else {
    // If timeout occurred before receiving the full frame
    //Serial.println("Timeout occurred, incomplete frame received.");
  }

  // Reset the buffer after processing (either valid or invalid)
  bufferIndex = 0;
  memset(modbusBuffer, 0, sizeof(modbusBuffer));
}

void validate_and_process_frame(uint8_t *buffer, uint16_t length) {
  byte address = buffer[0];
  byte function = buffer[1];

  if (address == SlaveAddress) {  // Accept broadcast address (0) or matching slave address
    //Serial.print("Processing function code: ");
    //Serial.println(function, HEX);

    switch (function) {
      case 0x03:  // Read Holding Registers
        read_reg(&buffer[2]);
        break;
      case 0x06:  // Write Single Register
        write_reg(&buffer[2]);
        break;
      case 0x10:  // Write Multiple Registers
        write_multiple_regs(&buffer[2]);
        break;
      default:
        Serial.println("Unsupported function code.");
    }
  }
}

void readI2CData() {
  Wire1.requestFrom(I2C_ADDRESS, 2);
  if (Wire1.available() == 2) {
    int lowByte = Wire1.read();
    int highByte = Wire1.read();
    Avg_FlowRate = (highByte << 8) | lowByte;
    holdingReg[FB] = Avg_FlowRate;
    Reported_FlowRate = update_and_calculate_average(Avg_FlowRate);
  }
}

void processStepperMotor() {
  if (MOTOR.available()) {
    String recv_buf = MOTOR.readStringUntil('\n');
    recv_buf.trim();
    if (recv_buf.length() > 0) {
      int pos = recv_buf.toInt();
      Stepper_Motor_position = constrain(pos, 0, 255);
    }
  }
}

void processValveControl() {
  if (holdingReg[Start] == 0x02 && !Valve_On && holdingReg[Proximity] == 1) {
    delay_start++;
    if (delay_start > 2) {
      delay_start = 0;
      Valve_On = true;
      Duty_Freq = int(10000 / float(holdingReg[Freq]));

      if (holdingReg[Auto_Manual] == 0x02) {
        Serial.println("AUTO Valve operation started.");

        Kp = float(holdingReg[Proportional]) / float(holdingReg[Flow] + 1);
        Ki = float(holdingReg[Integral]) / 1000.0f;
        Kd = float(holdingReg[Derivative]) / float(holdingReg[Flow] + 1);
        integral_error = 0;
        PID_Check = 0;

        FlowRate_when_Valve_Closed = Avg_FlowRate;
        initial_opening = int((Offset + holdingReg[Flow] / Gradient) * Duty_Freq / 769) + 10;
        SMC_Target = max(10, min(255, initial_opening));

        MOTOR.write(uint8_t(SMC_Target));
        holdingReg[Duty_FB] = int(SMC_Target * 769 / 255);

      } else if (holdingReg[Auto_Manual] == 0x01) {
        Serial.println("MANUAL Valve operation started.");
        FlowRate_when_Valve_Closed = Avg_FlowRate;
      }
    }
  }

  else if (holdingReg[Start] == 0x01 && Valve_On) {
    Serial.println("Valve operation stopped.");
    Valve_On = false;
    delay_start = 0;
    SMC_Target = 0;
    FlowRate_when_Valve_Closed = 0;

    holdingReg[Duty_FB] = 0;
    holdingReg[FB] = 0;

    MOTOR.write(uint8_t(SMC_Target));
  }

  else if (holdingReg[Proximity] == 0 && holdingReg[Start] == 0x01 && SMC_Target == 0) {
    MOTOR.write(uint8_t(SMC_Target));
  }

  if (holdingReg[Start] == 0x02 && Valve_On) {
    unsigned long current_time = millis();
    float blink_interval = (0.3f - (float(SMC_Target) / 255)) * 0.27f + 0.03f;
    if (current_time - last_blink_time > blink_interval) {
      green_led_state = !green_led_state;
      digitalWrite(Green_LED, green_led_state);
      last_blink_time = current_time;
    }

    if (holdingReg[Auto_Manual] == 0x01) {
      SMC_Target = int(holdingReg[Duty] * 255 / 769);
      holdingReg[Duty_FB] = int(SMC_Target * 769 / 255);
      MOTOR.write(uint8_t(SMC_Target));
      holdingReg[FB] = Reported_FlowRate;
    }

    else if (holdingReg[Auto_Manual] == 0x02) {
      holdingReg[FB] = Reported_FlowRate;

      if (PID_Check >= pid_period) {
        PID_Check = 0;
        error = float(holdingReg[Flow] - Avg_FlowRate);
        integral_error += error;
        integral_error = constrain(integral_error, -100, 100);
        derivative = error - last_error;
        last_error = error;

        Kp = float(holdingReg[Proportional]) / float(holdingReg[Flow] + 1);
        Ki = float(holdingReg[Integral]) / 1000.0f;
        Kd = float(holdingReg[Derivative]) / float(holdingReg[Flow] + 1);

        Serial.printf("Error: %.1f, Kp: %.6f, Ki: %.6f, Kd: %.6f SMC_Target %f\n", error, Kp, Ki, Kd, SMC_Target);

        pid_output = Kp * error + Ki * integral_error + Kd * derivative;
        pid_output = constrain(pid_output, -20, 20);

        SMC_Target += pid_output;
        SMC_Target = constrain(SMC_Target, 0, 255);
        MOTOR.write(uint8_t(SMC_Target));
        holdingReg[Duty_FB] = int(SMC_Target * 769 / 255);

      } else {
        if (abs(Stepper_Motor_position - SMC_Target) < 5) {
          PID_Check++;
        }
      }
    }
  }
}



void checkProximity() {
  if (digitalRead(PROXIMITY_PIN) == HIGH) {
    holdingReg[Proximity] = 1;
  } else {
    holdingReg[Proximity] = 0;
  }
}
void handleLEDs() {
  if (millis() - last_blink_time > 500) {
    digitalWrite(Green_LED, !digitalRead(Green_LED));
    last_blink_time = millis();
  }
}

// Function to find the line equation (gradient and offset) based on two points
void find_line_equation(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, float *gradient, float *offset) {
  // Calculate gradient and offset
  *gradient = ((float)Y2 - (float)Y1) / ((float)X2 - (float)X1);
  *offset = Y1 - ((*gradient) * X1);
}

void Media_Read(void) {
  EEPROM.get(400, MediaConstant);  // Retrieve MediaConstant array from EEPROM

  // Check if Media is uninitialized or has changed
  if (holdingReg[Media] == 0 || (holdingReg[Media] != last_E1 && holdingReg[Media] != last_E2 && holdingReg[Media] != last_E3 && holdingReg[Media] != last_E4)) {
    last_Media = holdingReg[Media];

    // Default values for media
    uint16_t defaultValues[20] = { 321, 630, 960, 1280, 1600, 1920, 2240, 2560, 2880, 3200,
                                   28, 55, 88, 120, 150, 180, 210, 250, 280, 310 };

    // Copy default values to holding registers
    for (int i = 0; i < 20; i++) {
      holdingReg[80 + i] = defaultValues[i];
    }

  } else {
    last_Media = holdingReg[Media];
    int offset = 0;

    // Determine the offset for media
    if (holdingReg[Media] == holdingReg[E1]) {
      offset = 0;
      X1 = int(holdingReg[E1 + 1]);
      X2 = int(holdingReg[E1 + 2]);
      Y1 = int(holdingReg[E1 + 3]);
      Y2 = int(holdingReg[E1 + 4]);
    } else if (holdingReg[Media] == holdingReg[E2]) {
      offset = 20;
      X1 = int(holdingReg[E2 + 1]);
      X2 = int(holdingReg[E2 + 2]);
      Y1 = int(holdingReg[E2 + 3]);
      Y2 = int(holdingReg[E2 + 4]);
    } else if (holdingReg[Media] == holdingReg[E3]) {
      offset = 40;
      X1 = int(holdingReg[E3 + 1]);
      X2 = int(holdingReg[E3 + 2]);
      Y1 = int(holdingReg[E3 + 3]);
      Y2 = int(holdingReg[E3 + 4]);
    } else if (holdingReg[Media] == holdingReg[E4]) {
      offset = 60;
      X1 = int(holdingReg[E4 + 1]);
      X2 = int(holdingReg[E4 + 2]);
      Y1 = int(holdingReg[E4 + 3]);
      Y2 = int(holdingReg[E4 + 4]);
    }

    // Copy media constants from EEPROM to holding registers
    for (int i = 0; i < 20; i++) {
      holdingReg[80 + i] = MediaConstant[i + offset];
    }

    find_line_equation(X1, Y1, X2, Y2, &Const_M, &Const_C);
    Offset = (int)(Const_C * 255 / 100);
    Gradient = (int)((1 / Const_M) * 100 / 255);
    // Equivalent C printf statement:
    printf("X1=%d, X2=%d, Y1=%d, Y2=%d. Line equation: y = %.2fx + %.2f, Gradient = %.2f, Offset = %.2f\n",
           X1, X2, Y1, Y2, Const_M, Const_C, Gradient, Offset);
  }
}

void Media_Write(void) {
  int offset = 0;

  if (holdingReg[Media] == holdingReg[E1] && holdingReg[E1] != 0u) {
    last_E1 = holdingReg[E1];
    offset = 0;
  } else if (holdingReg[Media] == holdingReg[E2] && holdingReg[E2] != 0u) {
    last_E2 = holdingReg[E2];
    offset = 20;
  } else if (holdingReg[Media] == holdingReg[E3] && holdingReg[E3] != 0u) {
    last_E3 = holdingReg[E3];
    offset = 40;
  } else if (holdingReg[Media] == holdingReg[E4] && holdingReg[E4] != 0u) {
    last_E4 = holdingReg[E4];
    offset = 60;
  }

  // Copy values from holdingReg to MediaConstant and save to EEPROM
  for (int i = 0; i < 20; i++) {
    MediaConstant[i + offset] = holdingReg[i + 80];
  }

  saveMediaConstant();  // Save all changes once after updating the constants
}

void loop() {
  if (i2cReadFlag) {
    i2cReadFlag = false;
    readI2CData();
  }

  if (DebugProcessFlag) {
    DebugProcessFlag = false;
    printMediaADCandKGS();
  }

  // Check the flag and call processValveControl() if set
  if (valveControlFlag) {
    valveControlFlag = false;  // Reset the flag
  }

  CheckAndSave();
  checkProximity();
  handleLEDs();
  processStepperMotor();
  processValveControl();

  if ((holdingReg[Media] != last_Media) || (last_Media == 0)) {
    Media_Read();
  }

  // Check if SlaveAddress needs to be updated based on holding register P_SlaveID
  if (SlaveAddress != holdingReg[P_SlaveID] && holdingReg[P_SlaveID] != 0) {
    // Debugging output to show change
    Serial.printf("P_SlaveID=%d, SlaveAddress= %d\n", P_SlaveID, SlaveAddress);
    // Update SlaveAddress with the new value from P_SlaveID
    SlaveAddress = holdingReg[P_SlaveID];
    // Indicate that the slave address has changed
    Serial.println("Slave_Add_Changed");
  }
}

void printMediaADCandKGS() {
  for (int i = 0; i < 25; i++) {
    printf("holdingReg[%d]=0x%x\tholdingReg[%d]=%x\tholdingReg[%d]=%x\n", i, holdingReg[i], i + 25, holdingReg[i + 25], i + 50, holdingReg[i + 50]);
  }
  for (int i = 0; i < 10; i++) {
    printf("holdingReg[%d]=%d,holdingReg[%d]=%d,MediaConstant[%d]=%d,MediaConstant[%d]=%d\n", i + 80, holdingReg[i + 80], i + 90, holdingReg[i + 90], i, MediaConstant[i], i + 10, MediaConstant[i + 10]);
  }
  printf("X1=%d, X2=%d, Y1=%d, Y2=%d. Line equation: y = %.2fx + %.2f, Gradient = %.2f, Offset = %.2f\n",
         X1, X2, Y1, Y2, Const_M, Const_C, Gradient, Offset);
  printf("Kp=%.6f, Ki=%.6f, kd=%.6f, err=%.2f, Target=%f, Stepper=%d, Auto=%d initial=%d\n", Kp, Ki, Kd, error, SMC_Target, Stepper_Motor_position, holdingReg[Auto_Manual], initial_opening);
  printf("[Proportional]=%d, [Integral]=%d, [Derivative]=%d FlowRate=%d, [Flow]=%d, Duty_Freq=%d\n", holdingReg[Proportional], holdingReg[Integral], holdingReg[Derivative], Avg_FlowRate, holdingReg[Flow], Duty_Freq);
}