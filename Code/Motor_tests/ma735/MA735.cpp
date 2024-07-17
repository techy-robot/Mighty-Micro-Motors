#include "MA735.h"

MA735::MA735(SPISettings settings, int nCS) : settings(settings), nCS(nCS) {

};
MA735::~MA735() {

};

void MA735::init(SPIClass* _spi) {
    spi = _spi;
    if (nCS >= 0) {
        pinMode(nCS, OUTPUT);
        digitalWrite(nCS, HIGH);
    }
};

float MA735::getCurrentAngle() {
    return (readRawAngle() * _2PI)/MA735_CPR;
}; // angle in radians, return current value

uint16_t MA735::readRawAngle() {
    uint16_t angle = transfer16(0x0000);
    return angle;
}; // 9-13bit angle value

uint16_t MA735::getZero() {
    uint16_t result = readRegister(MA735_REG_ZERO_POSITION_MSB)<<8;
    result |= readRegister(MA735_REG_ZERO_POSITION_LSB);
    return result;
};
uint8_t MA735::getBiasCurrentTrimming() {
    return readRegister(MA735_REG_BCT);
};
bool MA735::isBiasCurrrentTrimmingX() {
    return (readRegister(MA735_REG_ET) & 0x01)==0x01;
};
bool MA735::isBiasCurrrentTrimmingY() {
    return (readRegister(MA735_REG_ET) & 0x02)==0x02;
};
uint16_t MA735::getPulsesPerTurn() {
    uint16_t result = readRegister(MA735_REG_ILIP_PPT_LSB)>>6;
    result |= ((uint16_t)readRegister(MA735_REG_PPT_MSB))<<2;
    return result+1;
};
uint8_t MA735::getIndexLength() {
    return (readRegister(MA735_REG_ILIP_PPT_LSB)>>2)&0x0F;
};
uint8_t MA735::getRotationDirection() {
    return (readRegister(MA735_REG_RD)>>7);
};
uint8_t MA735::getFieldStrengthHighThreshold() {
    return (readRegister(MA735_REG_MGLT_MGHT)>>2)&0x07;
};
uint8_t MA735::getFieldStrengthLowThreshold() {
    return (readRegister(MA735_REG_MGLT_MGHT)>>5)&0x07;
};
FieldStrength MA735::getFieldStrength() {
    return (FieldStrength)(readRegister(MA735_REG_MGH_MGL)>>6);
};
uint8_t MA735::getFilterWindow() {
    return readRegister(MA735_REG_FW);
};
uint8_t MA735::getHysteresis() {
    return readRegister(MA735_REG_HYS);
};



void MA735::setZero(uint16_t value) {
    writeRegister(MA735_REG_ZERO_POSITION_MSB, value>>8);
    writeRegister(MA735_REG_ZERO_POSITION_LSB, value&0x00FF);
};
void MA735::setBiasCurrentTrimming(uint8_t value) {
    writeRegister(MA735_REG_BCT, value);
};
void MA735::setBiasCurrrentTrimmingEnabled(bool Xenabled, bool Yenabled) {
    uint8_t val = Xenabled ? 0x01 : 0x00;
    val |= (Yenabled ? 0x02 : 0x00);
    writeRegister(MA735_REG_ET, val);
};
void MA735::setPulsesPerTurn(uint16_t value) {
    uint16_t pptVal = value - 1;
    writeRegister(MA735_REG_PPT_MSB, pptVal>>2);
    uint8_t val = readRegister(MA735_REG_ILIP_PPT_LSB);
    val &= 0x3F;
    val |= (pptVal&0x03)<<6;
    writeRegister(MA735_REG_ILIP_PPT_LSB, val);
};
void MA735::setIndexLength(uint8_t value) {
    uint8_t val = readRegister(MA735_REG_ILIP_PPT_LSB);
    val &= 0xC0;
    val |= ((value<<2)&0x3F);
    writeRegister(MA735_REG_ILIP_PPT_LSB, val);
};
void MA735::setRotationDirection(uint8_t value) {
    if (value==0)
        writeRegister(MA735_REG_RD, 0x00);
    else
        writeRegister(MA735_REG_RD, 0x80);
};
void MA735::setFieldStrengthThresholds(uint8_t high, uint8_t low) {
    uint8_t val = (low<<5) | (high<<2);
    writeRegister(MA735_REG_MGLT_MGHT, val);
};
void MA735::setFilterWindow(uint8_t value) {
    writeRegister(MA735_REG_FW, value);
};
void MA735::setHysteresis(uint8_t value) {
    writeRegister(MA735_REG_HYS, value);
};


uint16_t MA735::transfer16(uint16_t outValue) {
    spi->beginTransaction(settings);
    if (nCS >= 0)
        digitalWrite(nCS, LOW);
    uint16_t value = spi->transfer16(outValue);
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);
    spi->endTransaction();
    return value;
};
uint8_t MA735::readRegister(uint8_t reg) {
    uint16_t cmd = 0x4000 | ((reg&0x001F)<<8);
    uint16_t value = transfer16(cmd);
    delayMicroseconds(1);
    value = transfer16(0x0000);
    return value>>8;
};
uint8_t MA735::writeRegister(uint8_t reg, uint8_t value) {
    uint16_t cmd = 0x8000 | ((reg&0x1F)<<8) | value;
    uint16_t result = transfer16(cmd);
    delay(20); // 20ms delay required
    result = transfer16(0x0000);
    return result>>8;
};
