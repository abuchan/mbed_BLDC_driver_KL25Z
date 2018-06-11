#include "mbed.h"
#include "rtos.h"
#include "encoder.h"
#include "protocol.h"
#include "packet_parser.h"

// Logical to physical pin definitions (change these when changing boards)
#define LED_1_PIN     PTC2
#define LED_2_PIN     PTC3
#define LED_3_PIN     PTA18

#define UART_TX_PIN   PTD7
#define UART_RX_PIN   PTD6

#define SPI_MOSI_PIN  PTC6
#define SPI_MISO_PIN  PTC7
#define SPI_SCK_PIN   PTC5
#define SPI_CS_PIN    PTC4

#define ENABLE_PIN    PTA4
#define DIRECTION_PIN PTA2
#define BRAKE_PIN     PTA1

#define CURRENT_PIN     PTB0
#define TEMPERATURE_PIN PTE30
#define VOLTAGE_PIN     PTD5

// Global object/variable instantiations (these should not change)
DigitalOut led_1(LED_1_PIN);
Encoder encoder(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN, SPI_CS_PIN);

PwmOut enable(ENABLE_PIN);
DigitalOut direction(DIRECTION_PIN);
DigitalOut brake(BRAKE_PIN);

AnalogIn current_sense(CURRENT_PIN);
AnalogIn temperature_sense(TEMPERATURE_PIN);
AnalogIn voltage_sense(VOLTAGE_PIN);

Timer system_timer;
sensor_data_t last_sensor_data;
command_data_t last_command_data;

// This was the best compromise speed since mbed is 41.94MHz and ImageProc is
// 40MHz
#define SERIAL_BAUDRATE 873813

// ticks to radians in 15.16 fixed point: multiply by 2^16 for scaled value
// 2*pi/(2^14)*2^16 = 25.1327
// this approximation accumulates 0.5% error
#define TICK_TO_RAD_FIXP 25 //6434
int32_t revolutions = 0;
int32_t integrator = 0;
int32_t velocity_filtered = 0;
int32_t current_position = 0;

uint8_t control_mode = 0;

int32_t position_absolute;
int32_t velocity_unfiltered;
int32_t dt;

#define COMMAND_LIMIT 1.0 // maximum duty cycle: [0,1] (1 is full on)
#define V_TAU 3000 // velocity LP time constant in us

// 15.16 fixed point: divide by 2^16 for scaled value
#define LOW_STOP -2*2*314*(1<<16)/100 // motor position limit
#define HIGH_STOP 17*2*314*(1<<16)/100 // motor high position limit
#define INTEGRATOR_MAX 1*(1<<16) // integrator saturation in rad*s

int32_t kp = (5<<16)/10; // fraction of command per rad
int32_t ki = (0<<16)/100; // fraction of command per rad*s
int32_t kd = (1<<16)/1000; // fraction of command per rad/s

// MOTOR PARAMETERS
// Custom wound motor
//*
#define MOT_KV 86 // (rad/s)/V from 825 RPM/V
#define MOT_R 2.4  // Ohms
#define I_LIM 15.0 // Amps
#define V_SUP 12.6 // Volts
//*/
// Reterminated stock motor
/*
#define MOT_KV 86 // (rad/s)/V from 825 RPM/V
#define MOT_R 2.5  // Ohms
#define I_LIM 15.0 // Amps
#define V_SUP 12.6 // Volts
*/
// Stock motor
/*
#define MOT_KV 173 // (rad/s)/V from 1650 RPM/V
#define MOT_R 0.6  // Ohms
#define I_LIM 15.0 // Amps
#define V_SUP 12.6 // Volts
*/

// Fills a packet with the most recent acquired sensor data
void get_last_sensor_data(packet_t* pkt, uint8_t flags) {
  pkt->header.type = PKT_TYPE_SENSOR;
  pkt->header.length = sizeof(header_t) + sizeof(sensor_data_t) + 1;
  pkt->header.flags = flags;
  sensor_data_t* sensor_data = (sensor_data_t*)pkt->data_crc;
  sensor_data->time = last_sensor_data.time;
  sensor_data->position = last_sensor_data.position;
  sensor_data->velocity = last_sensor_data.velocity;
  sensor_data->current = last_sensor_data.current;
  sensor_data->voltage = last_sensor_data.voltage;
  sensor_data->temperature = last_sensor_data.temperature;
  sensor_data->uc_temp = last_sensor_data.uc_temp;
}

// Sense and control function to be run at 1kHz
void sense_control_thread(void const *arg) {
  uint32_t last_time = last_sensor_data.time;
  int32_t last_position = current_position; // 3us
  last_sensor_data.time = system_timer.read_us(); // 19.6us
  encoder.update_state();

  dt = last_sensor_data.time - last_time; // timing
  current_position = encoder.get_cal_state();
  
  // Position: rad in 15.16 fixed point
  if (current_position > 3<<12 && last_position < 1<<12) {
    revolutions--;
  } else if (current_position < 1<<12 && last_position > 3<<12) {
    revolutions++;
  }
  int32_t last_position_absolute = position_absolute;
  position_absolute = TICK_TO_RAD_FIXP * 
        (revolutions*(1<<14) + current_position);

  // Velocity: rad/s in 15.16 fixed point
  velocity_unfiltered = 1000000*(((int64_t)position_absolute - 
        (int64_t)last_position_absolute)/dt);
  velocity_filtered = 
        (int64_t)dt*(int64_t)velocity_unfiltered/(V_TAU + (int64_t)dt) + 
        V_TAU*(int64_t)velocity_filtered/(V_TAU + (int64_t)dt);

  // Add data to struct
  last_sensor_data.position = position_absolute;
  last_sensor_data.velocity = velocity_filtered;
  //last_sensor_data.current = current_sense.read_u16(); //TODO: add real current sensing
  last_sensor_data.temperature = temperature_sense.read_u16();
  last_sensor_data.uc_temp = 0x1234;

  // Calculate and apply control
  int32_t command = 0; // command in 15.16 fixed point
  int32_t target_position = 0*(1<<16); // rad in 15.16 fixed point
  int32_t target_velocity = 0*(1<<16); // rad/s in 15.16 fixed point
  int32_t position_error = 0; // rad in 15.16 fixed point

  switch (control_mode) {
    case 0: // Disabled
      command = 0;
      integrator = 0;
      break;

    case 1: // Current control
      // TODO: Current control
      command = 0;
      integrator = 0;
      break;

    case 2: // Position control
      /*
      // debugging toggle
      if (last_sensor_data.time % 10000000 < 5000000) {
        target_position = 30*(1<<16);//1.5*(1<<16);
      }
      */
      /*
      // debugging sping slowly
      target_position = 6*(int64_t)last_sensor_data.time*(1<<16)/1000000;
      target_velocity = 6*(1<<16);
      */
      target_position = last_command_data.position_setpoint;

      position_error = position_absolute - target_position;

      // Integrator: rad*s in 15.16 fixed point
      integrator += (int32_t)((int64_t)position_error*(uint64_t)dt/1000000);
      if (integrator > INTEGRATOR_MAX) { // integrator saturation to prevent windup
        integrator = INTEGRATOR_MAX;
      } else if (integrator < -INTEGRATOR_MAX) {
        integrator = -INTEGRATOR_MAX;
      }
			
      kp = (int32_t)((uint16_t)(last_command_data.current_setpoint >> 16));
      kd = (int32_t)((uint16_t)last_command_data.current_setpoint);

      // Command
      command = (-kp*(int64_t)position_error)/(1<<16) + 
            (-kd*((int64_t)velocity_filtered-(int64_t)target_velocity))/(1<<16) + 
            (-ki*(int64_t)integrator)/(1<<16);
      break;

    case 3: // Voltage control
      command = last_command_data.position_setpoint;
      integrator = 0;
      break;

    case 16: // Zero position calibration
      encoder.set_offset(last_command_data.current_setpoint);
      control_mode = 0;
      break;

    default: // Disabled
      command = 0;
      integrator = 0;

  }

  // End stops
  if (control_mode != 3) {
    if (position_absolute > HIGH_STOP) { // high limit stop
        command = command > 0.0 ? 0.0 : command;
    } else if (position_absolute < LOW_STOP) { // low limit stop
        command = command < 0.0 ? 0.0 : command;
    }
  }

  // Current limit (hack)
  float v1 = I_LIM*MOT_R;
  float bemf = (((float)velocity_filtered)/(1<<16))/MOT_KV;
  float vMax = (v1 + bemf)/V_SUP;
  float vMin = (-v1 + bemf)/V_SUP;
  float command_float = static_cast<float>(command)/(1<<16);
  command_float = command_float > vMax
      ? vMax : (command_float < vMin
      ? vMin : command_float);

  // Command saturation
  float command_magnitude = fabs(command_float);
  command_magnitude = command_magnitude > COMMAND_LIMIT 
      ? COMMAND_LIMIT : command_magnitude; // limit maximum duty cycle
  enable.write(1-command_magnitude);
  direction.write(command > 0);
  // end Calculate and apply control (JY)

  last_sensor_data.current = (int16_t)(command_magnitude*(1<<15)*(2*(command>0)-1)); 
	  //TODO: add real current sensing

  // This puts the packet in the outgoing queue if0 there is space
  PacketParser* parser = (PacketParser*)(arg);
  packet_union_t* sensor_pkt = parser->get_send_packet();
  if (sensor_pkt != NULL) {
    get_last_sensor_data(&(sensor_pkt->packet),0);
    parser->send_packet(sensor_pkt);
  }
}

int main() {  
  enable.period_us(50);
  enable.write(1.0f);
  direction.write(0);
  brake.write(1);

  control_mode = 0; // by default, disable the motor

  led_1 = 1;

  // Start the microsecond time
  system_timer.start();

  // Instantiate packet parser
  PacketParser parser(
      SERIAL_BAUDRATE, UART_TX_PIN, UART_RX_PIN, LED_2_PIN, LED_3_PIN);

  // Run the sense/control thread at 1kHz
  RtosTimer sense_control_ticker(
      &sense_control_thread, osTimerPeriodic, (void*)(&parser));

  sense_control_ticker.start(1);
  
  // Pointer to received packet
  packet_union_t* recv_pkt = NULL;

  while(1) {

    // See if there is a received packet
    recv_pkt = parser.get_received_packet();

    if (recv_pkt != NULL) {

      led_1 = !(led_1.read());

      switch (recv_pkt->packet.header.type) {

        // If got a command packet, store command value
        case PKT_TYPE_COMMAND:
          memcpy(&last_command_data,recv_pkt->packet.data_crc,
              sizeof(command_data_t));
          // TODO: state machine for control modes
          control_mode = recv_pkt->packet.header.flags;
          // end control modes (JY)
          break;

        default:
          // TODO: do something if we got an unrecognized packet type?
          break;
      }
      parser.free_received_packet(recv_pkt);
    }

    // This will process any outgoing packets
    parser.send_worker();
  }
}
