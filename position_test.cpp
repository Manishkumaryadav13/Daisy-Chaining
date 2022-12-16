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
const float DXL_PROTOCOL_VERSION = 2.0;

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

float getPosition(uint8_t id, uint8_t unit = (uint8_t)0U);

float getCurrentAngle(uint8_t id, uint8_t unit = (uint8_t)0U);

uint32_t parse_numeric_value();

void dynamixelSetup();

float getCurrentVelocity(uint8_t id, uint8_t unit);

void setVelocity(uint8_t id, uint32_t velocity);

// ----------------------------------------------------------------
int main() {
    
    stdio_init_all();
    dxl.begin(57600);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    dynamixelSetup();
    while (true) {   
        command = getchar_timeout_us(100);                
        if (command == '#'){   
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
        if(command == 'a' || command == 'A') {
            sleep_ms(100);
            userInput = parse_numeric_value();
            if(userInput >=0U){
                // Set Goal Position
                printf("Command: %c\n",command);
                printf("Goal Position(angle) : %" PRIu32 "\n", userInput);
                moveToAngle(userInput);
            }
        }
        if(command == 'p' || command == 'P'){
            printf("Command: %c\n",command);
            getCurrentAngle(DXL_ID, UNIT_DEGREE);
        }
        if(command == 'r' || command == 'R'){
            printf("Command: %c\n",command);
            ret = dxl.factoryReset(DXL_ID, 0x02, TIMEOUT);

            if(ret) {
                printf("Reset successfull !!!\n");
                sleep_ms(1000);
                dynamixelSetup();
            }
            else {
                printf("Reset failed!!!\n");
            }
        }
        if (command == 's' || command == 'S'){
            sleep_ms(100);
            command = getchar_timeout_us(0);
            if(command == '1') {
                dynamixelSetup();
            }
            if(command == 'v' || command == 'V'){
                userInput = parse_numeric_value();
                setVelocity(DXL_ID, userInput);
            }
            command = '0';
        }
        if(command == 'l' || command == 'L'){
        // Turn on the LED on DYNAMIXEL
        dxl.ledOn(DXL_ID);
        sleep_ms(500);
        // Turn off the LED on DYNAMIXEL
        dxl.ledOff(DXL_ID);
        sleep_ms(500);
        }

        if(command == 'u'|| command == 'U')
        {
          printf("DynamixelController\n");
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
void dynamixelSetup(){

    // Get DYNAMIXEL information
  // Turn off torque when configuring items in EEPROM area
  if(dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    printf("DYNAMIXEL Torque off\n");
  else
    printf("Error: Torque off failed\n");

  // Set Operating Mode
  if(dxl.write(DXL_ID, OPERATING_MODE_ADDR, (uint8_t*)&operatingMode, OPERATING_MODE_ADDR_LEN, TIMEOUT))
    printf("Set operating mode\n");
  else
    printf("Error: Set operating mode failed\n");

  // Turn on torque
  if(dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    printf("Torque on\n");
  else
    printf("Error: Torque on failed\n");
  setVelocity(DXL_ID, DEFAULT_VELOCITY);
} 
