#include "mbed.h"
#include "rtos.h"
#include "enc.h"
#include "protocol.h"
#include "packet_parser.h"

#define LED1 PTC2
#define LED2 PTC3
#define LED3 PTA18

//#define SERIAL_BAUDRATE 230400
#define SERIAL_BAUDRATE 921600
#define UART_TX PTD7
#define UART_RX PTD6

//Ticker RTI;
//Serial imProc(UART_TX,UART_RX);
DigitalOut led_1(LED1);
//DigitalOut led_2(LED2);
//DigitalOut led_3(LED3);
enc motPos(PTC6,PTC7,PTC5,PTC4);
PwmOut EN(PTA4);
DigitalOut DR(PTA2);
DigitalOut BRAKE(PTA1);
AnalogIn motCurrent(PTB0);
AnalogIn temp(PTE30);
AnalogIn voltage(PTD5);

void get_state(void){
  motPos.update_pos();
}

void fill_sensor_packet(packet_t* pkt, uint8_t flags, uint32_t time, int32_t position,
  int32_t velocity, uint16_t current, uint16_t voltage, uint16_t temperature,
  uint16_t uc_temp) {
  pkt->header.type = PKT_TYPE_SENSOR;
  pkt->header.length = sizeof(header_t) + sizeof(sensor_data_t) + 1;
  pkt->header.flags = flags;
  sensor_data_t* sensor_data = (sensor_data_t*)pkt->data_crc;
  sensor_data->time = time;
  sensor_data->position = position;
  sensor_data->velocity = velocity;
  sensor_data->current = current;
  sensor_data->voltage = voltage;
  sensor_data->temperature = temperature;
  sensor_data->uc_temp = uc_temp;
}

Timer system_timer;

sensor_data_t last_sensor_data;
command_data_t last_command_data;

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


// Converts ticks/us to rad/s in 16.16 fixed point. Divide resulting number by
// 2^16 for final result
#define TICK_US_TO_RAD_S_FIXP 78640346

// Sense and control function to be run periodcially
void sense_control(void const *n) {
  led_1 = !(led_1.read());

  uint32_t last_time = last_sensor_data.time;  // 3us
  int32_t last_position = last_sensor_data.position;
  last_sensor_data.time = system_timer.read_us(); // 19.6us
  last_sensor_data.position = motPos.ams_read(); // 393us
  last_sensor_data.velocity = (
        TICK_US_TO_RAD_S_FIXP*(last_sensor_data.position - last_position)) /
            (last_sensor_data.time-last_time); // 8.1us
  last_sensor_data.current = motCurrent.read_u16(); //85.6us
  last_sensor_data.temperature = temp.read_u16(); // 85.6us
  last_sensor_data.uc_temp = 0x1234;

  led_1 = !(led_1.read());
}

RtosTimer sense_control_ticker(&sense_control,osTimerPeriodic);

int main() {  
  EN.period_us(50);
  EN.write(0.9f);
  DR.write(0);
  BRAKE.write(1);
  motPos.set_offset(2160);
  led_1 = 1;
//  RTI.attach(&get_state, 0.01f);

  system_timer.start();
  sense_control_ticker.start(1);
  uint32_t last_time = system_timer.read_us();
  uint32_t current_time = last_time;
  
  PacketParser parser(SERIAL_BAUDRATE, UART_TX, UART_RX, LED2, LED3);
  packet_union_t* recv_pkt = NULL;
  packet_union_t* sensor_pkt = parser.get_send_packet();

  while(1) {
    recv_pkt = parser.get_received_packet();

    if (recv_pkt != NULL) {
      led_1 = !(led_1.read());
      switch (recv_pkt->packet.header.type) {
        // If got a command packet, store command and respond with sensor packet.
        case PKT_TYPE_COMMAND:
          memcpy(&last_command_data,recv_pkt->packet.data_crc,sizeof(command_data_t));
          /*if (sensor_pkt != NULL) {
            get_last_sensor_data(&(sensor_pkt->packet),0);
            parser.send_blocking(sensor_pkt);
            sensor_pkt = parser.get_send_packet();
          }*/
          break;
      }
      parser.free_received_packet(recv_pkt);
    }

    // Send sensor packet periodically
    current_time = system_timer.read_us();
    if (current_time - last_time > 1000) {
      last_time = current_time;
      if (sensor_pkt != NULL) {
        get_last_sensor_data(&(sensor_pkt->packet),0);
        parser.send_blocking(sensor_pkt);
        sensor_pkt = parser.get_send_packet();
      }
    }
  }
}
