/*
 * PulsedLightI2C.h
 *
 *  Created on: May 22, 2017
 *      Author: Ivan
 */

#ifndef DRIVERS_PULSEDLIGHTI2C_H_
#define DRIVERS_PULSEDLIGHTI2C_H_

#include "stdint.h"

class PulsedLightI2C
{
public:
    void Init();
    void SoftInit();
    int16_t Update();
    int16_t SoftUpdate();

private:
    uint16_t I2CReceive(uint16_t, uint16_t, uint8_t);
    uint16_t I2CSend(uint16_t, uint8_t , uint8_t);

    uint16_t SoftI2CSend(uint16_t, uint8_t , uint8_t);

    // I2C Stuff
    void Wait();
    void Start();
    void Stop();
    void Idle();

    void PulseCLK();
    void Send(uint8_t data);
    bool GetAck();
};

#endif /* DRIVERS_PULSEDLIGHTI2C_H_ */
