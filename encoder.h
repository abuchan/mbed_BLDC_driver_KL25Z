#include "mbed.h"

class Encoder {
  public:
      //Connect over i2c
      Encoder(PinName mosi, PinName miso, PinName sck, PinName cs);

      //Destroy instance
      ~Encoder();

      void update_state(void);

      void set_offset(uint16_t zpos);

      //Calibrated state
      unsigned int get_cal_state(void);

  private:
      unsigned int pos_; // raw reading from sensor 14 bits
      long oticks_;  // revolution counter
      unsigned int calibPos_;  // 0 to 2pi, converted to 16 bits
      unsigned int offset_; // initial reading on setup - relative zero position

      SPI m_spi_;
      DigitalOut csl_;

      unsigned int ams_read(void);

      /*
      *****************************************************************************
      * Calculate even parity of a 16 bit unsigned integer
      *
      * This function is used by the SPI interface to calculate the even parity
      * of the data which will be sent via SPI to the encoder.
      *
      * \param[in] value : 16 bit unsigned integer whose parity shall be calculated
      *
      * \return : Even parity
      *
      *****************************************************************************
      */
      static uint16_t bit_parity(uint16_t value);

      uint16_t write_spi(uint16_t reg, uint8_t rw);

      uint16_t read_spi(uint16_t reg);
};





