#include <stdio.h>
#include <math.h>
#include "../../../src/dynamixel2pico/Dynamixel2Pico.h"

#include "pico/bootrom.h"
#define UART_ID     uart1
#define DXL_SERIAL  UART_ID
#define UART_RX_PIN   5
#define UART_TX_PIN   4
#define DXL_DIR_PIN   6

using namespace ControlTableItem;
//Please see eManual Control Table section of your DYNAMIXEL.
//This example is written for DYNAMIXEL X series(excluding XL-320)
#define OPERATING_MODE_ADDR         11
#define OPERATING_MODE_ADDR_LEN     1
#define TORQUE_ENABLE_ADDR          64
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    65
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          116
#define GOAL_POSITION_ADDR_LEN      4
#define PRESENT_POSITION_ADDR       132
#define PRESENT_POSITION_ADDR_LEN   4
#define POSITION_CONTROL_MODE       3
#define TIMEOUT 10    //default communication timeout 10ms


uint8_t turn_on = 1;
uint8_t turn_off = 0;
const uint8_t DXL_ID = 1;
const uint8_t DXL_1_ID = 1;
const uint8_t DXL_2_ID = 2;
const uint8_t DXL_3_ID = 3;
const uint8_t numOfMotors = 3;
const float DXL_PROTOCOL_VERSION = 2.0;
volatile int16_t goal_position;
uint8_t operatingMode = POSITION_CONTROL_MODE;
Dynamixel2Pico dxl(DXL_SERIAL, DXL_DIR_PIN, UART_RX_PIN, UART_TX_PIN);

uint32_t userInput = 0;
int command = 0;
bool ret = false;
#define SECRET_CODE     64209
#define DEFAULT_VELOCITY 50
// ----------------------------------------------------------------
// User defined functions definitions -----------------------------
void moveTo(uint32_t position);

void moveToAngle(uint32_t angle);

void moveMotorToAngle(const uint8_t ID, uint32_t angle);

float getMotorPosition(const uint8_t ID, uint8_t unit);

float getPosition(uint8_t id, uint8_t unit = (uint8_t)0U);

float getCurrentAngle(uint8_t id, uint8_t unit = (uint8_t)0U);

uint32_t parse_numeric_value();

void dynamixelSetup(const uint8_t totalMotors);

float getCurrentVelocity(uint8_t id, uint8_t unit);

void setVelocity(uint8_t id, uint32_t velocity);

// ----------------------------------------------------------------
int main() {
    
    stdio_init_all();
    dxl.begin(57600);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    dynamixelSetup(numOfMotors);
    while (true) {   
        uint8_t command = getchar_timeout_us(50);                
        if(command == 'p' || command == 'p') {            
          uint8_t sub_command = getchar_timeout_us(0);
          if(sub_command == '?'){
            float current_position = getPosition(DXL_ID);
            printf("Present position: %f\n", current_position);
          }
          else if(sub_command == '='){
            userInput = parse_numeric_value();
            if(userInput >= 0){ 
                goal_position = userInput; // Set Goal Position
                printf("Goal Position : %" PRIu32 "\n", goal_position);
                moveTo(goal_position);
            }
          }
        }
        else if(command == 'a'){
          uint8_t sub_command = getchar_timeout_us(0);
          
          if(sub_command == '?'){
            float current_angle = getCurrentAngle(DXL_ID);
            printf("Present angle: %f\n", current_angle);
          }
          else if(sub_command == '='){
            userInput = parse_numeric_value();
            if(userInput >= 0 && userInput <= 360){
                printf("Goal angle : %" PRIu32 "\n", userInput);
                moveToAngle(userInput); // Set Goal Angle
            }
          } 
        }
        else if(command == '?'){
          int16_t current_position = getPosition(DXL_ID);
          printf("%d, %d\n", current_position, goal_position);
        }
        else if (command == 'v' || command == 'V'){
          uint8_t sub_command = getchar_timeout_us(0);
          printf("Command: %c sub-command: %c\n",command, sub_command);
          if(sub_command == '?'){
            float current_velocity = getCurrentVelocity(DXL_ID, UNIT_RAW);
            printf("Current Velocity: %f", current_velocity);
          }
          else if (sub_command == '='){
            userInput = parse_numeric_value();
            setVelocity(DXL_ID, userInput);
          }
        }
        else if(command == 'l'){
          printf("Blinking Dynamixel LED\n");
          dxl.ledOn(DXL_ID);  // Turn on the LED on DYNAMIXEL
          sleep_ms(500);
          dxl.ledOff(DXL_ID); // Turn off the LED on DYNAMIXEL
          sleep_ms(500);
        }
        else if(command == 'k'){
          uint8_t subcommand = getchar_timeout_us(0);
          if(subcommand == '?'){
            uint16_t p = dxl.readControlTableItem(POSITION_P_GAIN, DXL_ID);
            uint16_t i = dxl.readControlTableItem(POSITION_I_GAIN, DXL_ID);
            uint16_t d = dxl.readControlTableItem(POSITION_D_GAIN, DXL_ID);
            printf("pid values: %d %d %d\n", p, i, d);
          }
          else if(subcommand == 'p'){
            uint16_t userInput = parse_numeric_value();
            dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, userInput);
          }
          else if(subcommand == 'i'){
            uint16_t userInput = parse_numeric_value();
            dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, userInput); 
          }
          else if(subcommand == 'd'){
            uint16_t userInput = parse_numeric_value();
            dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, userInput);
          }

        }
        else if (command == '#'){   
            command = getchar_timeout_us(0);
            if(command == 'r' || command == 'R'){
                command = getchar_timeout_us(0);
                if(command == 'p' || command == 'P'){
                    int dummy_data = parse_numeric_value();
                    if(dummy_data == SECRET_CODE){
                        reset_usb_boot(0, 0);
                    }
                }
            }
        }
        else if(command == 'r'){
            printf("Command: %c\n",command);
            ret = dxl.factoryReset(DXL_ID, 0x02, TIMEOUT);

            if(ret) {
                printf("Reset successfull !!!\n");
                sleep_ms(1000);
                // dynamixelSetup();
            }
            else {
                printf("Reset failed!!!\n");
            }
        }

     }
    return 0;
}

//----------------------------------------------------------------
void moveTo(uint32_t position) {
  dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&position, GOAL_POSITION_ADDR_LEN, TIMEOUT);
}

void moveToAngle(uint32_t angle)
{
  dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
}
void moveMotorToAngle(const uint8_t ID, uint32_t angle)
{
  dxl.setGoalPosition(ID, angle, UNIT_DEGREE);
  printf("Motor ID: %d \t Position(angle): %d\n", ID, angle);
}

float getCurrentAngle(uint8_t id, uint8_t unit)
{
  float position = -1;
  position = dxl.getPresentPosition(id,unit);
  sleep_ms(100);
  printf("Present position(angle): %f\n",position);
  return position;
}

float getPosition(uint8_t id, uint8_t unit){
  float position = -1;
  position = dxl.getPresentPosition(id, unit);
  sleep_ms(100);
  // printf("Present position: %f\n", position);

  return position;
}

float getMotorPosition(const uint8_t ID, uint8_t unit)
{
  float position = -1;
  position = dxl.getPresentPosition(ID, unit);
  sleep_ms(100);
  printf("Present position: %f\n", position);

  return position;
}

float getCurrentVelocity(uint8_t id, uint8_t unit){
  int velocity = -1;
  velocity = dxl.getPresentVelocity(id, unit);
  sleep_ms(100);
  printf("Present velocity(RPM): %d\n", velocity);
  return velocity;
}

void setVelocity(uint8_t id, uint32_t velocity){
  bool ret = dxl.writeControlTableItem(PROFILE_VELOCITY, id, velocity);
  sleep_ms(100);
  if(ret){
    printf("Successfully set goal velocity(RPM): %d\n", velocity);
  }
  else{
    printf("Error: Set goal velocity failed\n");
  }
}

uint32_t parse_numeric_value(){
    int buffer = 0, decimal_power = 0;
    float var = 0;
    bool neg_data = false, first_byte = true, decimal_data = false;
    while (true)
    {   
        buffer = getchar_timeout_us(100);
        if(first_byte && buffer == '-'){
            neg_data = true;
            first_byte = false;
        }
        if(buffer>='0' && buffer<='9'){    
            var = var*10 + (buffer - 48);
            if (decimal_data){
                decimal_power++;
            }
        }
        else if(buffer == '.'){
            decimal_data = true;
        }
        else if(buffer == '\n' || buffer == ' '){
            break;
        }    
        else if (buffer <= 0 || buffer == '-'){
            return PICO_ERROR_TIMEOUT;

        }
    }
    if (neg_data == true) var = -var;
    if(decimal_data)    var = var/(pow(10, decimal_power));
    return var;
}
void dynamixelSetup(const uint8_t totalMotors){
  for(int i = 1; i < totalMotors + 1; i++){
    // Get DYNAMIXEL information
    // Turn off torque when configuring items in EEPROM area
    if(dxl.write(i, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
      printf("DYNAMIXEL Torque off\n");
    else
      printf("Error: Torque off failed\n");

    // Set Operating Mode
    if(dxl.write(i, OPERATING_MODE_ADDR, (uint8_t*)&operatingMode, OPERATING_MODE_ADDR_LEN, TIMEOUT))
      printf("Set operating mode\n");
    else
      printf("Error: Set operating mode failed\n");

    // Turn on torque
    if(dxl.write(i, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
      printf("Torque on\n");
    else
      printf("Error: Torque on failed\n");
    setVelocity(i, DEFAULT_VELOCITY);
  }
} 
