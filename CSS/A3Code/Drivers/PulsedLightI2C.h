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
    void SoftInit();
    int16_t SoftUpdate();

private:
    uint8_t SoftI2CReceive8(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg);
    uint16_t SoftI2CSend(uint16_t, uint8_t , uint8_t);

    // I2C Stuff
    void Wait();
    void Start();
    void Stop();
    void Idle();

    void PulseCLK();
    void Send(uint8_t data);
    uint8_t Receive();
    bool GetAck();
    void SetAck();
    void SetNAck();
};

#endif /* DRIVERS_PULSEDLIGHTI2C_H_ */
