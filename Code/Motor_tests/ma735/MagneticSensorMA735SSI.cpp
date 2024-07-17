#include "./MagneticSensorMA735SSI.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

MagneticSensorMA735SSI::MagneticSensorMA735SSI(SPISettings settings) : settings(settings) {

}


MagneticSensorMA735SSI::~MagneticSensorMA735SSI() {

}

void MagneticSensorMA735SSI::init(SPIClass* _spi) {
    this->spi=_spi;
    this->Sensor::init();
}

// check 40us delay between each read?
float MagneticSensorMA735SSI::getSensorAngle() {
    float angle_data = readRawAngleSSI();
    angle_data = ( angle_data / (float)MA735_CPR ) * _2PI;
    // return the shaft angle
    return angle_data;
}


uint16_t MagneticSensorMA735SSI::readRawAngleSSI() {
    spi->beginTransaction(settings);
    uint16_t value = spi->transfer16(0x0000);
    //uint16_t parity = spi->transfer(0x00);
    spi->endTransaction();
    return (value<<1); //>>1)&0x3FFF;
}; // 14bit angle value
