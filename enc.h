#include "mbed.h"

typedef struct {
    unsigned int pos; // raw reading from sensor 14 bits
    long oticks;  // revolution counter
    unsigned int calibPos;  // 0 to 2pi, converted to 16 bits
    unsigned int offset; // initial reading on setup - relative zero position
} EncObj; 

class enc
{
public:
    //Connect over i2c
    enc(PinName mosi, PinName miso, PinName sck, PinName cs);
    //Destroy instance
    ~enc();
    unsigned int  ams_read();
    void update_pos();
    void set_offset(uint16_t zpos);
    unsigned int cal_state(); //Calibrated stat
    static uint16_t par(uint16_t value);
    uint16_t write_spi(uint16_t reg, uint8_t rw);
    uint16_t read_spi(uint16_t reg);
    EncObj enc_pos;
    
private:
    SPI m_spi;
    DigitalOut csl;

};





