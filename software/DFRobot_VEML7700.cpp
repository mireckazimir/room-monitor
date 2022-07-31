/*!
 * @file  DFRobot_VEML7700.cpp
 * @brief  DFRobot's Light Sensor
 * @n      High Accuracy Ambient Light Sensor
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [yangyang](971326313@qq.com)
 * @version  V1.0
 * @date  2016-12-08
 * @url  https://github.com/DFRobot/DFRobot_VEML7700
 */

#include "DFRobot_VEML7700.h"

DFRobot_VEML7700::DFRobot_VEML7700()
{
}

void DFRobot_VEML7700::init(int i2c_fd){
  fd_i2c = i2c_fd;
}

void DFRobot_VEML7700::begin()
{
  // write initial state to DFRobot_VEML7700
  register_cache[0] = ( (uint32_t(ALS_GAIN_x2) << ALS_SM_SHIFT) |
                        (uint32_t(ALS_INTEGRATION_100ms) << ALS_IT_SHIFT) |
                        (uint32_t(ALS_PERSISTENCE_1) << ALS_PERS_SHIFT) |
                        (uint32_t(0) << ALS_INT_EN_SHIFT) |
                        (uint32_t(0) << ALS_SD_SHIFT) );
  register_cache[1] = 0x0000;
  register_cache[2] = 0xffff;
  register_cache[3] = ( (uint32_t(ALS_POWER_MODE_3) << PSM_SHIFT) |
                        (uint32_t(0) << PSM_EN_SHIFT) );
  for (uint8_t i=0; i<4; i++){
    sendData(i, register_cache[i]);
  }

  // wait at least 2.5ms as per datasheet
  //delay(3);
  usleep(3000);
}

void DFRobot_VEML7700::begin(uint8_t als_gain)
{  
  // write initial state to DFRobot_VEML7700
  register_cache[0] = ( (uint32_t(als_gain) << ALS_SM_SHIFT) |
                        (uint32_t(ALS_INTEGRATION_100ms) << ALS_IT_SHIFT) |
                        (uint32_t(ALS_PERSISTENCE_1) << ALS_PERS_SHIFT) |
                        (uint32_t(0) << ALS_INT_EN_SHIFT) |
                        (uint32_t(0) << ALS_SD_SHIFT) );
  register_cache[1] = 0x0000;
  register_cache[2] = 0xffff;
  register_cache[3] = ( (uint32_t(ALS_POWER_MODE_3) << PSM_SHIFT) |
                        (uint32_t(0) << PSM_EN_SHIFT) );
  for (uint8_t i=0; i<4; i++){
    sendData(i, register_cache[i]);
  }

  // wait at least 2.5ms as per datasheet
  //delay(3);
  usleep(3000);
}

uint8_t DFRobot_VEML7700::sendData(uint8_t command, uint32_t data)
{
  uint8_t buffer[3] = {0};

  buffer[0] = command;
  buffer[1] = data & 0xFF;
  buffer[2] = (data >> 8) & 0xFF;

  if(ioctl(fd_i2c, I2C_SLAVE, I2C_ADDRESS) < 0){
    return STATUS_ERROR;
  }

  if(write(fd_i2c, buffer, 3) != 3){
    return STATUS_ERROR;
  }
  return STATUS_OK;
}

uint8_t DFRobot_VEML7700::receiveData(uint8_t command, uint32_t& data)
{
  uint8_t buffer[2] = {0};

  buffer[0] = command;
  
  if(ioctl(fd_i2c, I2C_SLAVE, I2C_ADDRESS) < 0){
    return STATUS_ERROR;
  }

  if(write(fd_i2c, buffer, 1) != 1){
    return STATUS_ERROR;
  }

  buffer[0] = 0x00;
  buffer[1] = 0x00;

  //if ((data = wiringPiI2CReadReg16(fd, command)) == -1){
  if(read(fd_i2c, buffer, 2) != 2){
    return STATUS_ERROR;
  }

  data = 0 | (uint32_t) buffer[0] | ((uint32_t) buffer[1]<<8);

  return STATUS_OK;
}

uint8_t DFRobot_VEML7700::setGain(eAlsGain_t gain)
{
  uint32_t reg = ( (register_cache[COMMAND_ALS_SM] & ~ALS_SM_MASK) | 
                   ((uint32_t(gain) << ALS_SM_SHIFT) & ALS_SM_MASK) );
  register_cache[COMMAND_ALS_SM] = reg;
  return sendData(COMMAND_ALS_SM, reg);
}

uint8_t DFRobot_VEML7700::getGain(eAlsGain_t& gain)
{
  gain = eAlsGain_t(
    (register_cache[COMMAND_ALS_SM] & ALS_SM_MASK) >> ALS_SM_SHIFT );
  return STATUS_OK;
}

uint8_t DFRobot_VEML7700::setIntegrationTime(eAlsItime_t itime)
{
  uint32_t reg = ( (register_cache[COMMAND_ALS_IT] & ~ALS_IT_MASK) | 
                   ((uint32_t(itime) << ALS_IT_SHIFT) & ALS_IT_MASK) );
  register_cache[COMMAND_ALS_IT] = reg;
  return sendData(COMMAND_ALS_IT, reg);
}

uint8_t DFRobot_VEML7700::getIntegrationTime(eAlsItime_t& itime)
{
  itime = eAlsItime_t(
    (register_cache[COMMAND_ALS_IT] & ALS_IT_MASK) >> ALS_IT_SHIFT );
  return STATUS_OK;
}

uint8_t DFRobot_VEML7700::setPersistence(eAlsPersist_t persist)
{
  uint32_t reg = ( (register_cache[COMMAND_ALS_PERS] & ~ALS_PERS_MASK) | 
                   ((uint32_t(persist) << ALS_PERS_SHIFT) & ALS_PERS_MASK) );
  register_cache[COMMAND_ALS_PERS] = reg;
  return sendData(COMMAND_ALS_PERS, reg);
}

uint8_t DFRobot_VEML7700::setPowerSavingMode(eAlsPowerMode_t powerMode)
{
  uint32_t reg = ( (register_cache[COMMAND_PSM] & ~PSM_MASK) | 
                   ((uint32_t(powerMode) << PSM_SHIFT) & PSM_MASK) );
  register_cache[COMMAND_PSM] = reg;
  return sendData(COMMAND_PSM, reg);
}

uint8_t DFRobot_VEML7700::setPowerSaving(uint8_t enabled)
{
  uint32_t reg = ( (register_cache[COMMAND_PSM_EN] & ~PSM_EN_MASK) | 
                   ((uint32_t(enabled) << PSM_EN_SHIFT) & PSM_EN_MASK) );
  register_cache[COMMAND_PSM_EN] = reg;
  return sendData(COMMAND_PSM_EN, reg);
}

uint8_t DFRobot_VEML7700::setInterrupts(uint8_t enabled)
{
  uint32_t reg = ( (register_cache[COMMAND_ALS_INT_EN] & ~ALS_INT_EN_MASK) | 
                   ((uint32_t(enabled) << ALS_INT_EN_SHIFT) & 
                    ALS_INT_EN_MASK) );
  register_cache[COMMAND_ALS_INT_EN] = reg;
  return sendData(COMMAND_ALS_INT_EN, reg);
}

uint8_t DFRobot_VEML7700::setPower(uint8_t on)
{
  uint32_t reg = ( (register_cache[COMMAND_ALS_SD] & ~ALS_SD_MASK) | 
                   ((uint32_t(~on) << ALS_SD_SHIFT) & ALS_SD_MASK) );
  register_cache[COMMAND_ALS_SD] = reg;
  uint8_t status = sendData(COMMAND_ALS_SD, reg);
  if (on) {
    //delay(3); // minimu 2.5us delay per datasheet
    usleep(3000);
  }
  return status;
}

uint8_t DFRobot_VEML7700::setALSHighThreshold(uint32_t thresh)
{
  return sendData(COMMAND_ALS_WH, thresh);
}

uint8_t DFRobot_VEML7700::setALSLowThreshold(uint32_t thresh)
{
  return sendData(COMMAND_ALS_WL, thresh);
}

uint8_t DFRobot_VEML7700::getALS(uint32_t& als)
{
  return receiveData(COMMAND_ALS, als);
}

uint8_t DFRobot_VEML7700::getWhite(uint32_t& white)
{
  return receiveData(COMMAND_WHITE, white);
}

uint8_t DFRobot_VEML7700::getHighThresholdEvent(uint8_t& event)
{
  uint32_t reg;
  uint8_t status = receiveData(COMMAND_ALS_IF_H, reg);
  event = (reg & ALS_IF_H_MASK) >> ALS_IF_H_SHIFT;
  return status;
}

uint8_t DFRobot_VEML7700::getLowThresholdEvent(uint8_t& event)
{
  uint32_t reg;
  uint8_t status = receiveData(COMMAND_ALS_IF_L, reg);
  event = (reg & ALS_IF_L_MASK) >> ALS_IF_L_SHIFT;
  return status;
}

void DFRobot_VEML7700::scaleLux(uint32_t raw_counts, float& lux)
{
  eAlsGain_t gain;
  eAlsItime_t itime;
  getGain(gain);
  getIntegrationTime(itime);

  float factor1, factor2, result;
  static uint8_t x1=0, x2=1, d8=0;

  switch(gain & 0x3){
  case ALS_GAIN_x1:
    factor1 = 1.f;
    break;
  case ALS_GAIN_x2:
    factor1 = 0.5f;
    break;
  case ALS_GAIN_d8:
    factor1 = 8.f;
    break;
  case ALS_GAIN_d4:
    factor1 = 4.f;
    break;
  default:
    factor1 = 1.f;
    break;
  }

  switch(itime){
  case ALS_INTEGRATION_25ms:
    factor2 = 0.2304f;
    break;
  case ALS_INTEGRATION_50ms:
    factor2 = 0.1152f;
    break;
  case ALS_INTEGRATION_100ms:
    factor2 = 0.0576f;
    break;
  case ALS_INTEGRATION_200ms:
    factor2 = 0.0288f;
    break;
  case ALS_INTEGRATION_400ms:
    factor2 = 0.0144f;
    break;
  case ALS_INTEGRATION_800ms:
    factor2 = 0.0072f;
    break;
  default:
    factor2 = 0.2304f;
    break;
  }

  result = raw_counts * factor1 * factor2;
  if((result > 1880.00f) && (result < 3771.00f)){
	  if(x1 == 1){
		begin(ALS_GAIN_x1);
		x1 = 0; x2 = 1; d8 = 1;
	  }
  }else if(result>3770.00f){
	  if(d8 == 1){
		begin(ALS_GAIN_d8);
		x1 = 1; x2 = 1; d8 = 0;
	  }
  }else{
	  if(x2 == 1){
		begin();  
		x1 = 1; x2 = 0; d8 = 1;
	  }
  }
  lux = result;
  // apply correction from App. Note for all readings
  //   using Horner's method
  lux = lux * (1.0023f + lux * (8.1488e-5f + lux * (-9.3924e-9f + 
                                                    lux * 6.0135e-13f)));
}

uint8_t DFRobot_VEML7700::getALSLux(float& lux)
{
  uint32_t raw_counts = 0;
  uint8_t status = getALS(raw_counts);
  scaleLux(raw_counts, lux);
  return status;
}

uint8_t DFRobot_VEML7700::getWhiteLux(float& lux)
{
  uint32_t raw_counts = 0;
  uint8_t status = getWhite(raw_counts);
  scaleLux(raw_counts, lux);
  return status;
}

uint8_t DFRobot_VEML7700::getAutoXLux(float& lux,
                                      DFRobot_VEML7700::getCountsFunction counts_func,
                                      DFRobot_VEML7700::eAlsGain_t& auto_gain,
                                      DFRobot_VEML7700::eAlsItime_t& auto_itime,
                                      uint32_t& raw_counts)
{
  eAlsGain_t gains[4] = { ALS_GAIN_d8,
                          ALS_GAIN_d4,
                          ALS_GAIN_x1,
                          ALS_GAIN_x2 };
  eAlsItime_t itimes[6] = {ALS_INTEGRATION_25ms,
                           ALS_INTEGRATION_50ms,
                           ALS_INTEGRATION_100ms,
                           ALS_INTEGRATION_200ms,
                           ALS_INTEGRATION_400ms,
                           ALS_INTEGRATION_800ms };

  uint32_t counts_threshold = 200;

  int8_t itime_idx;
  uint8_t gain_idx;
  if (setPower(0)){
    return STATUS_ERROR;
  }
  for (itime_idx = 2; itime_idx < 6; itime_idx++){
    if (setIntegrationTime(itimes[itime_idx])){
      return STATUS_ERROR;
    }
    for (gain_idx = 0; gain_idx < 4; gain_idx++){
      if (setGain(gains[gain_idx])){
        return STATUS_ERROR;
      }
      if (setPower(1)){
        return STATUS_ERROR;
      }
      sampleDelay();
      if ((this->*counts_func)(raw_counts)){
        return STATUS_ERROR;
      }

      if (raw_counts > counts_threshold){
        do {
          if (raw_counts < 10000){
            scaleLux(raw_counts, lux);
            auto_gain = gains[gain_idx];
            auto_itime = itimes[itime_idx];
            return STATUS_OK;  
          }
          if(setPower(0)){
            return STATUS_ERROR;
          }
          itime_idx--;
          if (setIntegrationTime(itimes[itime_idx])){
            return STATUS_ERROR;
          }          
          if (setPower(1)){
            return STATUS_ERROR;
          }
          sampleDelay();
          if ((this->*counts_func)(raw_counts)){
            return STATUS_ERROR;
          }
        } while (itime_idx > 0);
        scaleLux(raw_counts, lux);
        auto_gain = gains[gain_idx];
        auto_itime = itimes[itime_idx];
        return STATUS_OK;  
      }
      if(setPower(0)){
        return STATUS_ERROR;
      }
    }
  }
  scaleLux(raw_counts, lux);
  auto_gain = gains[gain_idx];
  auto_itime = itimes[itime_idx];
  return STATUS_OK;
}

uint8_t DFRobot_VEML7700::getAutoALSLux(float& lux)
{
  DFRobot_VEML7700::eAlsGain_t auto_gain;
  DFRobot_VEML7700::eAlsItime_t auto_itime;
  uint32_t raw_counts;
  return getAutoXLux(lux,
                     &DFRobot_VEML7700::getALS,
                     auto_gain,
                     auto_itime,
                     raw_counts);
}

uint8_t DFRobot_VEML7700::getAutoWhiteLux(float& lux)
{
  DFRobot_VEML7700::eAlsGain_t auto_gain;
  DFRobot_VEML7700::eAlsItime_t auto_itime;
  uint32_t raw_counts;
  return getAutoXLux(lux,
                     &DFRobot_VEML7700::getWhite,
                     auto_gain,
                     auto_itime,
                     raw_counts);
}


uint8_t DFRobot_VEML7700::getAutoALSLux(float& lux,
                                        DFRobot_VEML7700::eAlsGain_t& auto_gain,
                                        DFRobot_VEML7700::eAlsItime_t& auto_itime,
                                        uint32_t& raw_counts)
{
  return getAutoXLux(lux,
                     &DFRobot_VEML7700::getALS,
                     auto_gain,
                     auto_itime,
                     raw_counts);
}

uint8_t DFRobot_VEML7700::getAutoWhiteLux(float& lux,
                                          DFRobot_VEML7700::eAlsGain_t& auto_gain,
                                          DFRobot_VEML7700::eAlsItime_t& auto_itime,
                                          uint32_t& raw_counts)
{
  return getAutoXLux(lux,
                     &DFRobot_VEML7700::getWhite,
                     auto_gain,
                     auto_itime,
                     raw_counts);
}

void DFRobot_VEML7700::sampleDelay()
{
  eAlsItime_t itime;
  getIntegrationTime(itime);

  // extend nominal delay to ensure new sample is generated
  #define extended_delay(ms) usleep(2000*(ms)) // delay(2*(ms))

  switch(itime){
  case ALS_INTEGRATION_25ms:
    extended_delay(25);
    break;
  case ALS_INTEGRATION_50ms:
    extended_delay(50);
    break;
  case ALS_INTEGRATION_100ms:
    extended_delay(100);
    break;
  case ALS_INTEGRATION_200ms:
    extended_delay(200);
    break;
  case ALS_INTEGRATION_400ms:
    extended_delay(400);
    break;
  case ALS_INTEGRATION_800ms:
    extended_delay(800);
    break;
  default:
    extended_delay(100);
    break;
  }
}