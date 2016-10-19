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

// Converts ticks/us to rad/s in 16.16 fixed point. Divide resulting number by
// 2^16 for final result
#define TICK_US_TO_RAD_S_FIXP 78640346

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
  int32_t last_position = last_sensor_data.position; // 3us
  last_sensor_data.time = system_timer.read_us(); // 19.6us
  encoder.update_state();
  last_sensor_data.position = encoder.get_cal_state(); // 393us
  last_sensor_data.velocity = (
        TICK_US_TO_RAD_S_FIXP*(last_sensor_data.position - last_position)) /
            (last_sensor_data.time-last_time); // 8.1us
  last_sensor_data.current = current_sense.read_u16(); //85.6us
  last_sensor_data.temperature = temperature_sense.read_u16(); // 85.6us
  last_sensor_data.uc_temp = 0x1234;

  // TODO: Calculate and apply control

  // This puts the packet in the outgoing queue if there is space
  PacketParser* parser = (PacketParser*)(arg);
  packet_union_t* sensor_pkt = parser->get_send_packet();
  if (sensor_pkt != NULL) {
    get_last_sensor_data(&(sensor_pkt->packet),0);
    parser->send_packet(sensor_pkt);
  }
}

int main() {  
  enable.period_us(50);
  enable.write(0.9f);
  direction.write(0);
  brake.write(1);

  encoder.set_offset(2160);

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
