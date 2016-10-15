#include "packet_parser.h"

PacketParser::PacketParser(
  uint32_t baudrate, PinName tx_pin, PinName rx_pin, PinName tx_led, PinName rx_led) :
  pc_(tx_pin, rx_pin),
  tx_led_(tx_led),
  rx_led_(rx_led) {
  tx_led_ = 1;
  rx_led_ = 1;
  pc_.baud(baudrate);

  out_pkt_ = NULL;
  tx_sequence_ = 0;

  pc_.attach(this, &PacketParser::receive_callback, MODSERIAL::RxIrq);
  in_pkt_ = (packet_union_t*)in_box_.alloc();
  in_pkt_idx_ = 0;
  in_pkt_len_ = MAX_PACKET_LENGTH;
  in_pkt_crc_ = 0;
}

packet_union_t* PacketParser::get_received_packet(void) {
  osEvent evt = in_box_.get(0);
  if (evt.status == osEventMail) {
    return (packet_union_t*)evt.value.p;
  } else {
    return NULL;
  }
}

void PacketParser::free_received_packet(packet_union_t* packet) {
  in_box_.free(packet);
}

packet_union_t* PacketParser::get_send_packet(void) {
  return (packet_union_t*)out_box_.alloc();
}

void PacketParser::send_packet(packet_union_t* packet) {
  out_box_.put(packet);
}

void PacketParser::send_worker(void) {
  while(true) {
    osEvent evt = out_box_.get();
    if (evt.status == osEventMail) {
      tx_led_ = 0;
      out_pkt_ = (packet_union_t*)evt.value.p;
      out_pkt_->packet.header.start = PKT_START_CHAR;
      out_pkt_->packet.header.sequence = tx_sequence_++;
      uint8_t crc_value = calculate_crc8(out_pkt_->raw, out_pkt_->packet.header.length-1);
      out_pkt_->raw[out_pkt_->packet.header.length-1] = crc_value;
      for (int i = 0; i < out_pkt_->packet.header.length; i++) {
        pc_.putc(out_pkt_->raw[i]);
      }
      tx_led_ = 1;
      tx_led_ = 0;
      out_box_.free(out_pkt_);
      out_pkt_ = NULL;
      tx_led_ = 1;
    }
    Thread::yield();
  }
}

void PacketParser::send_blocking(packet_union_t* out_pkt) {
  tx_led_ = 0;
  out_pkt->packet.header.start = PKT_START_CHAR;
  out_pkt->packet.header.sequence = tx_sequence_++;
  uint8_t crc_value = calculate_crc8(out_pkt->raw, out_pkt->packet.header.length-1);
  out_pkt->raw[out_pkt->packet.header.length-1] = crc_value;
  for (int i = 0; i < out_pkt->packet.header.length; i++) {
    pc_.putc(out_pkt->raw[i]);
  }
  out_box_.free(out_pkt);
  tx_led_ = 1;
}

void PacketParser::send_complete(MODSERIAL_IRQ_INFO *q) {
  tx_led_ = 0;
  if (out_pkt_ != NULL) {
    out_box_.free(out_pkt_);
    out_pkt_ = NULL;
  }
  tx_led_ = 1;
}

void PacketParser::receive_callback(MODSERIAL_IRQ_INFO *q) {
  rx_led_ = 0;
  MODSERIAL* serial = q->serial;

  if (in_pkt_ != NULL) {
    while(serial->readable()) {
      char c = serial->getc();

      // If we just received the second character, set packet length
      if (in_pkt_idx_ == 1) {
        in_pkt_len_ = c;
      }

      // If there has been a parse error, reset packet buffer
      if ((in_pkt_idx_ == 0 && c != PKT_START_CHAR) || in_pkt_len_ < sizeof(header_t)+1 ) {
        in_pkt_idx_ = 0;
        in_pkt_len_ = MAX_PACKET_LENGTH;
        in_pkt_crc_ = 0;

      // Store byte in packet buffer and update CRC
      } else {
        in_pkt_->raw[in_pkt_idx_++] = c;
        in_pkt_crc_ = update_crc8(in_pkt_crc_, c);
      }

      // If we just received the last character, put valid packets in mailbox
      // and reset packet buffer
      if (in_pkt_idx_ == in_pkt_len_) {
        if (in_pkt_crc_ == 0) {
          in_box_.put(in_pkt_);
          in_pkt_ = (packet_union_t*)in_box_.alloc();
        }
        in_pkt_idx_ = 0;
        in_pkt_len_ = MAX_PACKET_LENGTH;
        in_pkt_crc_ = 0;
      }
    }
  } else {
    in_pkt_ = (packet_union_t*)in_box_.alloc();
  }

  rx_led_ = 1;
}
