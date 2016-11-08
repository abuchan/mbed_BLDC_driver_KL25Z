#include "encoder.h"

Encoder::Encoder(PinName mosi, PinName miso, PinName sck, PinName cs) :
  m_spi_(mosi, miso, sck), csl_(cs) {

  csl_=1;
  m_spi_.format(8, 1);
  m_spi_.frequency(1000000);
  wait(0.01);

  set_offset(2134); // est. 9/13/2016
  csl_=1;

  write_spi(0x0019, 0); //SETTINGS2
  write_spi(0x001D, 0);
  write_spi(0x0018, 0); //SETTINGS1
  write_spi(0x0018, 0);
    
  pos_ = 0;
  oticks_ = 0;
  offset_ = 0;
  calibPos_ = 0;
}

Encoder::~Encoder() {}

unsigned int Encoder::get_cal_state(){
  return pos_;
}

void Encoder::set_offset(uint16_t zpos){
  uint16_t zhold;
  zhold = zpos >> 6; //8MSB of zero position
  write_spi(0x0016, 0);
  write_spi(zhold, 0);

  zhold = zpos & 0x003F; //6 LSB of zero position
  write_spi(0x0017, 0);
  write_spi(zhold, 0);
}

uint16_t Encoder::write_spi(uint16_t reg, uint8_t rw){
  uint16_t data;
  reg |= rw << 14;
  reg |= bit_parity(reg) << 15;
  csl_=0;
  data = m_spi_.write(reg>>8) << 8;
  data |= m_spi_.write(reg);
  csl_=1;
  return data;
}

uint16_t Encoder::read_spi(uint16_t reg) {
  uint16_t data;
  write_spi(reg, 1);
  data = write_spi(0x0000, 1);
  return data;
}

uint16_t Encoder::bit_parity(uint16_t value) {
  value ^= value>>8;
  value ^= value>>4;
  value ^= value>>2;
  value ^= value>>1;
  value &= 1;
  return value;
}

void Encoder::update_state(){
    pos_ = ams_read();
    // TODO: calibrate state
}

unsigned int Encoder::ams_read() {
  unsigned int enc_data;

  write_spi(0x3FFE, 1);
  enc_data = write_spi(0x0000, 1);

  return enc_data & 0x3FFF;
}
