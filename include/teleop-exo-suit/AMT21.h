#ifndef _AMT21
#define _AMT21

#define ENC_SCALE 0.00038349519

#include "teleop-exo-suit/Serial.h"

struct{

    uint8_t READ_POSITION = 0x00;
    uint8_t SET_ZERO      = 0x02;
    uint8_t RESET         = 0x03;

} AMT21_CONST;

class AMT21 : public Serial{
    public:
        AMT21(const char* _port, 
              int baudrate,
              uint8_t _address)
              : 
                address(_address),
                Serial(_port, baudrate){};

        uint8_t address;

        float read_position(uint16_t timeout=1000);
        uint8_t* read_raw_position(uint16_t timeout=1000);
        void set_address(uint8_t _address);
        void set_zero();
        void reset();
        bool is_checksum(uint8_t* payload);
};

void AMT21::set_address(uint8_t _address){
    this->address = _address;
}

uint8_t* AMT21::read_raw_position(uint16_t timeout){
    size_t num_payload = 1;
    uint8_t* payload   = (uint8_t*)calloc(num_payload, sizeof(*payload));
    if (payload == nullptr) {
        return nullptr;
    }

    payload[0] = address + AMT21_CONST.READ_POSITION;  

    this->write( payload, num_payload );
    free(payload);

    DataRecv data_recv = this->recvWithTimeout( timeout );
    if (data_recv.bytes_read == 0) {
        return nullptr;
    }
    
    if(!this->is_checksum(data_recv.data_recv)) 
        return nullptr; 

    uint8_t* position  = (uint8_t*)calloc(data_recv.bytes_read, sizeof(*position));
    if (position == nullptr) {
        return nullptr;
    }

    position[0] = data_recv.data_recv[1] & 0x3F;
    position[1] = data_recv.data_recv[0];

    return position;
}

float AMT21::read_position(uint16_t timeout){
    uint8_t* raw_position = this->read_raw_position(timeout);
    float position = (*(uint16_t*)(raw_position)) * ENC_SCALE;
    
    free(raw_position);

    return position; 
}

void AMT21::set_zero(){
    size_t num_payload = 1;
    uint8_t* payload   = (uint8_t*)calloc(num_payload, sizeof(*payload));

    payload[0] = address + AMT21_CONST.SET_ZERO;
    
    this->write( payload, num_payload );

    free(payload);
}

void AMT21::reset(){
    size_t num_payload = 1;
    uint8_t* payload   = (uint8_t*)calloc(num_payload, sizeof(*payload));

    payload[0] = address + AMT21_CONST.RESET;

    this->write( payload, num_payload );

    free(payload);
}

bool AMT21::is_checksum(uint8_t* payload){
    bool k0 = payload[14];
    bool k1 = payload[15];

    bool _k0 = 0;
    bool _k1 = 0;

    for(int i=0; i<7; ++i) _k1 += payload[2*i+1];
    for(int j=0; j<7; ++j) _k0 += payload[2*j]; 

    return !(k0 == _k0 && k1 == _k1);  
}

#endif