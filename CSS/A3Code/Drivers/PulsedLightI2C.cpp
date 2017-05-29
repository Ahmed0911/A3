/*
 * PulsedLightI2C.cpp
 *
 *  Created on: May 22, 2017
 *      Author: Ivan
 */

#include "PulsedLightI2C.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c1294ncpdt.h"

extern uint32_t g_ui32SysClock;

uint16_t LidarWriteAddr = 0xc4;
uint16_t LidarReadAddr = 0xc5;


void PulsedLightI2C::SoftInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 );
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 );
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_OD);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_OD);
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Reset
    SoftI2CSend(LidarWriteAddr,0x00,0x00);

    //wait for 20 ms
    SysCtlDelay(g_ui32SysClock / 50);

    Idle();
}

int16_t PulsedLightI2C::SoftUpdate()
{
    // Integer to store data
    uint16_t Dist = 0;

    // Prepare Lidar for reading
    SoftI2CSend(LidarWriteAddr,0x00,0x04);

    //wait for 20 ms
    SysCtlDelay(g_ui32SysClock / (50*3));

    // Read Data
    Dist =  SoftI2CReceive8(LidarReadAddr,LidarWriteAddr,0x0F);

    // Shift Data
    Dist = Dist << 8;

    // get low byte
    Dist+=SoftI2CReceive8(LidarReadAddr,LidarWriteAddr,0x10);


    return Dist;
}

uint8_t PulsedLightI2C::SoftI2CReceive8(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg)
{
    uint8_t Data = 0;

    // Set Read request
    Start();

    // send address
    Send(WriteAddr);

    // ACK pulse
    GetAck();

    // send address
    Send(reg);

    // ACK pulse
    GetAck();

    // stop pulse
    Stop();

    // Get Data
    // start
    Start();

    // send address
    Send(ReadAddr);

    // ACK pulse
    GetAck();

    // get first byte
    Data = Receive();

    // send ack
    SetNAck();

    // Stop
    Stop();

    SysCtlDelay(g_ui32SysClock / 50);

    return Data;
}

uint16_t PulsedLightI2C::SoftI2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value)
{
    // start
    Start();

    // send address
    Send(WriteAddr);

    // ACK pulse
    bool ack = GetAck();

    // send reg
    Send(reg);

    // ACK pulse
    GetAck();

    // send Value
    Send(value);

    // ACK pulse
    GetAck();

    // Stop
    Stop();
}

void PulsedLightI2C::Wait()
{
    SysCtlDelay(g_ui32SysClock / 300000);
}

void PulsedLightI2C::Idle()
{
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 );
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 );
    Wait();
}

void PulsedLightI2C::Start()
{
    // set idle
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 );
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 );
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 ); // SDA to low
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
    Wait();
}

void PulsedLightI2C::Stop()
{
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); // SCL high
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 );  // SDA low
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 ); // SDA to high
    Wait();
}

void PulsedLightI2C::PulseCLK()
{
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); // SCL high
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
    Wait();
}

void PulsedLightI2C::Send(uint8_t data)
{
    for(int i = 7; i>=0; i--)
    {
        uint8_t bit = data & (1<<i);
        if( bit )GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 );
        else GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 );
        PulseCLK();
    }
}

bool PulsedLightI2C::GetAck()
{
    // set as input
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_1);
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); // SCL high
    Wait();
    // check ACK
    int val = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_1); // set as output
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 ); // SDA low
    Wait();

    return (val == 0);
}

uint8_t PulsedLightI2C::Receive()
{
   uint8_t dataX = 0;
   GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_1);
   for(int i = 7; i>=0; i--)
   {
       Wait();
       GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); // SCL high
       int val = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1);
       dataX = (dataX << 1);
       if( val ) dataX+=1;
       Wait();
       GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
       Wait();
   }
   GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_1); // set as output
   GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 ); // SDA low

   return dataX;
}


void PulsedLightI2C::SetAck()
{
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 ); // SDA low
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); // SCL high
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
    Wait();
}

void PulsedLightI2C::SetNAck()
{
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 ); // SDA high
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); // SCL high
    Wait();
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 ); // SDA low
    Wait();
}
