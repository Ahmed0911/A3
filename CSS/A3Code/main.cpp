/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"

#include "Drivers/DBGLed.h"
#include "Drivers/Timer.h"
#include "Drivers/WpnOutputs.h"
#include "Drivers/PWMDrv.h"
#include "Drivers/SerialDriver.h"
#include "Drivers/SBUSComm.h"
#include "Drivers/ADCDrv.h"
#include "Drivers/BaroDrv.h"
#include "Drivers/MPU9250Drv.h"
#include "Drivers/LSM90DDrv.h"
#include "Drivers/UBloxGPS.h"
#include "Drivers/EtherDriver.h"
#include "Drivers/HopeRF.h"
#include "Drivers/IMU.h"
#include "Drivers/EEPROMDrv.h"
#include "Drivers/CANDrv.h"
#include "Drivers/PulsedLightI2C.h"

#include "LLConverter.h"
#include "CRC32.h"
#include "Comm433MHz.h"

uint32_t g_ui32SysClock;

// Drivers
DBGLed dbgLed;
Timer timerLoop;
WpnOutputs wpnOut;
PWMDrv pwmDrv;
SerialDriver serialU2;
SerialDriver serialU3;
SerialDriver serialU5;
SBUSComm sbusRecv;
ADCDrv adcDrv;
BaroDrv baroDrv;
MPU9250Drv mpu9250Drv;
LSM90DDrv lsm90Drv;
UBloxGPS gps;
//EtherDriver etherDrv;
//HopeRF    hopeRF;
IMU imu;
EEPROMDrv eeprom;
CRC32 crc;
Comm433MHz comm433MHz;
CANDrv canDrv;
PulsedLightI2C pulseI2C;

// System Objects
LLConverter llConv;

// Systick
#define SysTickFrequency 50
volatile bool SysTickIntHit = false;

// Buffers
#define COMMBUFFERSIZE 1024
BYTE CommBuffer[COMMBUFFERSIZE];
BYTE HopeRFbuffer[255];

// Global Functions


// Global Data
int MainLoopCounter;
float PerfLoopTimeMS;
float PerfCpuTimeMS;
float PerfCpuTimeMSMAX;
float Acc[3];
float Gyro[3];
float Mag[3];
float Pressure0 = 101300;
unsigned short PulsedLidarRange = 0;

// OFFSETS
float GyroOffX = 0;
float GyroOffY = 0;
float GyroOffZ = 0;
float MagOffX = 0;
float MagOffY = 0;
float MagOffZ = 0;
float AttOffRoll = 0;
float AttOffPitch = 0;

void main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    FPULazyStackingEnable();

    // Ensure that ext. osc is used!
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

    // set clock
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Init
    dbgLed.Init();
    timerLoop.Init();
    crc.Init();
    wpnOut.Init();
    pwmDrv.Init();
    pwmDrv.SetWidthUS(0, 1000); // Set zero PWMs
    serialU2.Init(UART2_BASE, 9600); // GPS
    serialU3.Init(UART3_BASE, 100000); // SBUS
    serialU5.Init(UART5_BASE, 115200); // Ext. Comm
    sbusRecv.Init();
    adcDrv.Init();
    baroDrv.Init();
    mpu9250Drv.Init();
    //lsm90Drv.Init();
    //InitGPS(); // init GPS
    //etherDrv.Init();
    //hopeRF.Init();
    imu.Init();
    canDrv.Init();
    pulseI2C.SoftInit();

    // Systick
    SysTickPeriodSet(g_ui32SysClock/SysTickFrequency);
    SysTickIntEnable();
    SysTickEnable();

    // Master INT Enable
    IntMasterEnable();

    // FAKE GPS
    //llConv.SetHome(45.80349, 15.88388);

    while(1)
    {
        timerLoop.Start(); // start timer
        MainLoopCounter++;

        /////////////////////////////////
        // INPUTS
        /////////////////////////////////
        // SBUS Data
        int rd = serialU3.Read(CommBuffer, COMMBUFFERSIZE); // read data from SBUS Recv [2500 bytes/second, read at least 3x per second for 1k buffer!!!]
        sbusRecv.NewRXPacket(CommBuffer, rd); // process data

        // ADC + Current Calc
        adcDrv.Update();

        // Baro
        baroDrv.Update(); // [??? us]

        // PulseLight
        PulsedLidarRange = pulseI2C.SoftUpdate();

        // IMU1
        mpu9250Drv.Update();
        Acc[0] = -mpu9250Drv.Accel[1];
        Acc[1] = -mpu9250Drv.Accel[0];
        Acc[2] = mpu9250Drv.Accel[2];
        Gyro[0] = mpu9250Drv.Gyro[1] - GyroOffX;
        Gyro[1] = mpu9250Drv.Gyro[0] - GyroOffY;
        Gyro[2] = -mpu9250Drv.Gyro[2] - GyroOffZ;
        Mag[0] = mpu9250Drv.Mag[0] - MagOffX;
        Mag[1] = mpu9250Drv.Mag[1] - MagOffY;
        Mag[2] = mpu9250Drv.Mag[2] - MagOffZ;
        imu.Update(Acc[0], Acc[1], Acc[2], Gyro[0], Gyro[1], Gyro[2], Mag[0], Mag[1], Mag[2]);
        imu.Roll -= AttOffRoll; // Correct IMU offsets
        imu.Pitch -= AttOffPitch;

        // IMU2
        //lsm90Drv.Update();

        // Set Pressure0 on 2 sec
        if(MainLoopCounter == (SysTickFrequency * 2)) Pressure0 = baroDrv.PressurePa;

        // process ethernet (RX)
        //etherDrv.Process(1000/SysTickFrequency); // 2.5ms tick

        // Read Lora Data
        int dataReceived = serialU5.Read(CommBuffer, COMMBUFFERSIZE);
        comm433MHz.NewRXPacket(CommBuffer, dataReceived); // calls ProcessCommand callback!!!




        /////////////////////////////////
        // OUTPUTS
        /////////////////////////////////
        //pwmDrv.SetWidthUS(0, ctrl.rtY.PWM1);

        // DBG LED
        //dbgLed.Set(ctrl.rtY.GreenLED);


        // Get CPU Time
        PerfCpuTimeMS = timerLoop.GetUS()/1000.0f;
        if( PerfCpuTimeMS > PerfCpuTimeMSMAX ) PerfCpuTimeMSMAX = PerfCpuTimeMS;
        // wait next
        while(!SysTickIntHit);
        SysTickIntHit = false;
        // Get total loop time
        PerfLoopTimeMS = timerLoop.GetUS()/1000.0f;
    }
}

///////////////
// INTERRUPTS
///////////////
extern "C" void UART2IntHandler(void)
{
    serialU2.IntHandler();
}

extern "C" void UART3IntHandler(void)
{
    serialU3.IntHandler();
}

extern "C" void UART5IntHandler(void)
{
    serialU5.IntHandler();
}

extern "C" void IntGPIOA(void)
{
    lsm90Drv.MotionINTG();
    lsm90Drv.MotionINTX();
}

extern "C" void IntGPIOH(void)
{
    lsm90Drv.MotionINTM();
}

extern "C" void IntGPIOK(void)
{
    mpu9250Drv.MotionINT();
}

extern "C" void IntGPION(void)
{
    //hopeRF.IntHandler();
}

extern "C" void SysTickIntHandler(void)
{
    SysTickIntHit = true;
}
