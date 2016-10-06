#include "mbed.h"

#include "enc.h"
#include "protocol.h"
#include "packet_parser.h"

#define LED1 PTC2
#define LED2 PTC3
#define LED3 PTA18

#define SERIAL_BAUDRATE 230400
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

void get_state(){
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

int main() {  
  EN.period_us(50);
  EN.write(0.9f);
  DR.write(0);
  BRAKE.write(1);
  motPos.set_offset(2160);
  led_1 = 1;
//  RTI.attach(&get_state, 0.01f);


  Timer system_timer;
  system_timer.start();
  uint32_t last_time = system_timer.read_ms();
  uint32_t current_time = last_time;
  

  PacketParser parser(SERIAL_BAUDRATE, UART_TX, UART_RX, LED2, LED3);
  packet_union_t* recv_pkt = NULL;
  command_data_t* command;
  
  packet_union_t* sensor_pkt = parser.get_send_packet();

  while(1) {
    //wait(0.5);
    //led_1 = !led_1;
    //led_2 = !led_2;
    //led_3 = !led_3;

    /*recv_pkt = parser.get_received_packet();

    if (recv_pkt != NULL) {
      
      switch (recv_pkt->packet.header.type) {
        
        case PKT_TYPE_COMMAND:
          command = (command_data_t*)recv_pkt->packet.data_crc;
          if (sensor_pkt != NULL) {
            fill_sensor_packet(&(sensor_pkt->packet),
              motPos.ams_read(),
              0,
              motCurrent.read_u16(),
              voltage.read_u16(),
              temp.read_u16()
            );
            parser.send_packet(sensor_pkt);
            sensor_pkt = parser.get_send_packet();
          }
          break;
      }
      
      parser.free_received_packet(recv_pkt);
    }*/
    

    current_time = system_timer.read_ms();
    if (current_time - last_time > 500) {
      last_time = current_time;
      led_1 = !led_1;

      sensor_pkt = parser.get_send_packet();
      if (sensor_pkt != NULL) {
    	  fill_sensor_packet(&(sensor_pkt->packet),
          0, //flags
          current_time,
          motPos.ams_read(),
          1234, // velocity
          motCurrent.read_u16(),
          voltage.read_u16(),
          temp.read_u16(),
          0x1234 // uC temp
        );
        parser.send_blocking(sensor_pkt);
      }
    }
    /*
    Thread::yield();
    */
  }
}
