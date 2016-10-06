#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include "mbed.h"

//#include "MODDMA.h"
#include "MODSERIAL.h"
#include "rtos.h"
#include "lib_crc.h"

#include "protocol.h"

/**
 * Defines the number of packets in the incoming and outgoing buffers.
 */
#define PACKET_BUFFER_LENGTH 4

/**
 * Thread flag to start work.
 */
#define START_THREAD_FLAG (1<<0)
#define DMA_COMPLETE_FLAG (1<<1)

class PacketParser {
  
  public:

    /**
     * Constructor.
     *
     * Creates a packet parsing protocol on the USB serial connection.
     *
     */
    PacketParser(uint32_t baudrate, PinName tx_pin, PinName rx_pin, PinName tx_led, PinName rx_led);

    /**
     * Get a pointer to the next received packet, or NULL if there is no packet.
     */
    packet_union_t* get_received_packet(void);

    /**
     * Return a received packet to the packet pool. Must be called after using
     * a packet from get_received_packet.
     *
     * @param packet - pointer to packet to be freed.
     */
    void free_received_packet(packet_union_t* packet);

    /**
     * Get a pointer to a packet to be sent. Will return NULL if there are no
     * available outgoing packets.
     */
    packet_union_t* get_send_packet(void);

    /**
     * Send the packet returned by get_send_packet.
     *
     * @param packet - pointer to packet to be sent.
     */
    void send_packet(packet_union_t* packet);

    void send_blocking(packet_union_t* out_pkt);

  private:

    MODSERIAL pc_;
    //MODDMA dma_;
    
    DigitalOut tx_led_;
    uint32_t tx_sequence_;

    Mail<packet_union_t, PACKET_BUFFER_LENGTH> out_box_;
    packet_union_t* out_pkt_;
    
    static void thread_starter(void const *p);
    Thread send_thread_;
    
    void send_worker(void);
    void send_complete(MODSERIAL_IRQ_INFO *q);

    DigitalOut rx_led_;
    Mail<packet_union_t, PACKET_BUFFER_LENGTH> in_box_;
    
    packet_union_t* in_pkt_;
    uint32_t in_pkt_idx_;
    uint32_t in_pkt_len_;
    uint8_t in_pkt_crc_;

    void receive_callback(MODSERIAL_IRQ_INFO *q);
};

#endif
