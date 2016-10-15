#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PKT_START_CHAR 0xAA

/**
 * Packet type characters.
 */
#define PKT_TYPE_COMMAND  'C'
#define PKT_TYPE_SENSOR   'S'

/**
 * Defines the total maximum size of a packet, including header
 */
#define MAX_PACKET_LENGTH 256

/**
 * Packet structure definitions
 */
typedef struct header_t {
  uint8_t start;
  uint8_t length;
  char type;
  uint8_t flags;
  uint32_t sequence;
} header_t;

typedef struct packet_t {
  header_t header;
  uint8_t data_crc[MAX_PACKET_LENGTH-sizeof(header_t)];
} packet_t;

typedef union packet_union_t {
  packet_t packet;
  char raw[MAX_PACKET_LENGTH];
} packet_union_t;

typedef struct command_data_t {
  int32_t position_setpoint;
  uint32_t current_setpoint;
} command_data_t;

typedef struct sensor_data_t {
  uint32_t time;
  int32_t position;
  int32_t velocity;
  uint16_t current;
  uint16_t voltage;
  uint16_t temperature;
  uint16_t uc_temp;
} sensor_data_t;

#endif
