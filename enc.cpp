#include "enc.h"

enc::enc(PinName mosi, PinName miso, PinName sck, PinName cs) : m_spi(mosi, miso, sck), csl(cs){
    set_offset(2134); // est. 9/13/2016 
    csl=1;
    m_spi.format(8, 1);
    m_spi.frequency(1000000);
    wait(0.05);

    write_spi(0x0019, 0); //SETTINGS2
    write_spi(0x001D, 0);
    write_spi(0x0018, 0); //SETTINGS1
    write_spi(0x0018, 0);
      
    enc_pos.pos = 0;
    enc_pos.oticks = 0;
    enc_pos.offset = 0;
    enc_pos.calibPos = 0;
    }
    
enc::~enc(){
 
}

unsigned int enc::cal_state(){
    return enc_pos.pos;
}

void enc::set_offset(uint16_t zpos){
    uint16_t zhold;
    zhold = zpos >> 6; //8MSB of zero position
    write_spi(0x0016, 0);
    write_spi(zhold, 0);
   
    zhold = zpos & 0x003F; //6 LSB of zero position
    write_spi(0x0017, 0);
    write_spi(zhold, 0);   
}

uint16_t enc::write_spi(uint16_t reg, uint8_t rw){
    uint16_t data;
    reg |= rw << 14;
    reg |= par(reg);
    csl=0;
    data = m_spi.write(reg>>8) << 8;
    data |= m_spi.write(reg);
    csl=1;
    return data;
}

uint16_t enc::read_spi(uint16_t reg) {
    uint16_t data;
    write_spi(reg, 1);
    data = write_spi(0x0000, 1);
    return data;  
}

/*!
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



uint16_t enc::par(uint16_t value) {
    value ^= value>>8;
    value ^= value>>4;
    value ^= value>>2;
    value ^= value>>1;
    value &= 1;
    return value << 15;
}

void enc::update_pos(){
    enc_pos.pos = ams_read();
}

unsigned int enc::ams_read() {
    unsigned int enc_data;

    write_spi(0x3FFE, 1);
    enc_data = write_spi(0x0000, 1);

    //enc_data = write_spi(0x0001, 1);
    //write_spi(0x0000, 1);
        
    return enc_data & 0x3FFF;
}
