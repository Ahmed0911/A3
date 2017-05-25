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
uint16_t Lidar2ByteRead = 0x8f;

void PulsedLightI2C::Init()
{
        //Enable i2c module 1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

        //Reset the i2c module 1
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

        //Enable the GPIO periph that contains i2c1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

        //Configure the pin muxing for i2c1, functions on port PG0, PG1.
        GPIOPinConfigure(GPIO_PG0_I2C1SCL);
        GPIOPinConfigure(GPIO_PG1_I2C1SDA);

        //Select i2c funciton on pins, SCL & SDA.
        GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);
        GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);

        //Initialize the i2c master block, I2C module
        I2CMasterInitExpClk(I2C1_BASE, g_ui32SysClock, false);

        //Clear i2c FIFOs
        HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}

void PulsedLightI2C::SoftInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 );
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1 );
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_OD);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_OD);
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    Idle();
}

int16_t PulsedLightI2C::Update()
{
    // Integer to store data
    uint16_t Dist = 0;

    // Prepare Lidar for reading
    I2CSend(LidarWriteAddr,0x00,0x04);

    //wait for 20 ms
    //SysCtlDelay(g_ui32SysClock / (20));

    // Read Data
    //Dist =  I2CReceive(LidarReadAddr,LidarWriteAddr,Lidar2ByteRead);

    return Dist;
}

int16_t PulsedLightI2C::SoftUpdate()
{
        // Integer to store data
        uint16_t Dist = 0;

        // Prepare Lidar for reading
        SoftI2CSend(LidarWriteAddr,0x00,0x04);

        //wait for 20 ms
        //SysCtlDelay(g_ui32SysClock / (20));

        // Read Data
        //Dist =  I2CReceive(LidarReadAddr,LidarWriteAddr,Lidar2ByteRead);

        return Dist;
}

// Read function for I2C1
uint16_t PulsedLightI2C::I2CReceive(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg)
{
    // Integer to store data.
    uint16_t Data = 0;

    //Initialize the i2c for writing to slave device
    I2CMasterSlaveAddrSet(I2C1_BASE, WriteAddr >> 1, false);

    //Register to be read
    I2CMasterDataPut(I2C1_BASE, reg);

    //Sends control - and register address -byte to slave device
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    //Wait until slave device finished it's transaction.
    while(I2CMasterBusy(I2C1_BASE));

    //Initialize the i2c to read from slave device
    I2CMasterSlaveAddrSet(I2C1_BASE, ReadAddr>>1, true);

    //Read first Byte
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    //Wait until slave device finished it's transaction
    while(I2CMasterBusy(I2C1_BASE));

    // Read 8 MSB
    Data = I2CMasterDataGet(I2C1_BASE);

    // Shift Data
    Data = Data << 8;

    // Read last 8 Byte
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C1_BASE));

    // Shift in LSB
    Data |= I2CMasterDataGet(I2C1_BASE);

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //Wait until slave device finished it's transaction
    while(I2CMasterBusy(I2C1_BASE));

    // If error occur store messg
    char test = I2CMasterErr(I2C1_BASE);

    // Return Data
    return Data;
}

//Write to I2C1
uint16_t PulsedLightI2C::I2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value)
{
    //Writing register adress to slave device
    I2CMasterSlaveAddrSet(I2C1_BASE, WriteAddr >> 1, false);

    //Register to be read
    I2CMasterDataPut(I2C1_BASE, reg);

    //Sends control - and register address -byte to slave device
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //Wait until slave device finished it's transaction
    while(I2CMasterBusy(I2C1_BASE));

    //Register address to slave device
    I2CMasterSlaveAddrSet(I2C1_BASE, WriteAddr >> 1, false);

    // Value to send.
    I2CMasterDataPut(I2C1_BASE, value);

    //Sends control - and register address -byte to slave device
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    while(I2CMasterBusy(I2C1_BASE));

        // If error occur return message.
    return I2CMasterErr(I2C1_BASE);
}

uint16_t PulsedLightI2C::SoftI2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value)
{
    // start
    Start();

    // send address
    Send(WriteAddr);

    // ACK pulse
    GetAck();

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
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_1); // set as output
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0 ); // SDA low
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0 ); // SCL low
    Wait();

    return (val == 0);
}
