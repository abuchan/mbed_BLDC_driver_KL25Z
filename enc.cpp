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

void enc::set_offset(ushort zpos){
    ushort zhold;
    zhold = zpos >> 6; //8MSB of zero position
    write_spi(0x0016, 0);
    write_spi(zhold, 0);
   
    zhold = zpos & 0x003F; //6 LSB of zero position
    write_spi(0x0017, 0);
    write_spi(zhold, 0);   
}

ushort enc::write_spi(ushort reg, u8 rw){
    ushort data;
    reg = reg | rw << 14;
    reg = reg | par(reg);
    char d1 = reg >> 8;
    char d2 = reg & 0x00FF;
    csl=0;
    d1 = m_spi.write(d1);
    d2 = m_spi.write(d2);
    
    csl=1;
    wait(0.0001);
    data = (d1 << 8) | (d2 & 0xFF);
    return data;
    
}

ushort enc::read_spi(ushort reg){
    ushort data;
    reg = reg | 1 << 14;
    reg = reg | par(reg);
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
ushort enc::par(ushort value)
{
    u8 cnt = 0;
    u8 i;
 
    for (i = 0; i < 16; i++)
    {
        if (value & 0x1)
        {
            cnt++;
        }
        value >>= 1;
    }
    cnt = cnt & 0x1;
    return cnt << 15;
}

void enc::update_pos(){
    enc_pos.pos = ams_read();
}

unsigned int enc::ams_read() {
    unsigned int enc_data;

    enc_data = read_spi(0x3FFE);
    read_spi(0x0001);
        
    return enc_data & 0x3FFF;
}
