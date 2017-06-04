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
#include "CommData.h"

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
EtherDriver etherDrv;
//HopeRF    hopeRF;
IMU imu;
EEPROMDrv eeprom;
CRC32 crc;
Comm433MHz comm433MHz;
CANDrv canDrv;
PulsedLightI2C pulseI2C;

// System Objects
LLConverter llConv;

// Systick (50Hz, 20ms)
#define SysTickFrequency 50
volatile bool SysTickIntHit = false;

// Buffers
#define COMMBUFFERSIZE 1024
BYTE CommBuffer[COMMBUFFERSIZE];
BYTE HopeRFbuffer[255];

// Global Functions
void SendPeriodicDataEth(void);
void ProcessCommand(int cmd, unsigned char* data, int dataSize);

////////////////////
// Global Data
////////////////////
// System/Perf Stuff
int MainLoopCounter;
float PerfLoopTimeMS;
float PerfCpuTimeMS;
float PerfCpuTimeMSMAX;

// IMU
float Acc[3];
float Gyro[3];
float Mag[3];

// Pressure/Altitude Stuff
float PressurePa = 0;
float PressurePa0 = 101300;
float TemperatureC = 0;

// Rangefinders
unsigned short SharpDistanceSensorADC = 0;
unsigned short PulsedLidarRangeCM = 0;
int PulsedLidarErrors = 0;

// SBUS Inputs
unsigned int SBUSThrottle = 0;
unsigned int SBUSSwitchD = 0;

// Control Inputs
unsigned int PWMRefThrottle = 900;

// Outputs
unsigned int OutPWMThrottle;

// Control Stuff
unsigned int ActiveMode;
// 0 - Manual Override
// 1 - Matlab/External

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
    etherDrv.Init();
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
        SBUSThrottle = sbusRecv.Channels[0];
        SBUSSwitchD = sbusRecv.Channels[5];

        // ADC + Sharp Rangefinder
        adcDrv.Update();
        SharpDistanceSensorADC = adcDrv.GetValue(ADCBATTCURRENT);

        // PulseLight
        unsigned short newRange = pulseI2C.SoftUpdate();
        if( newRange != 0 ) PulsedLidarRangeCM = newRange;
        else PulsedLidarErrors++;

        // Baro/Pressure
        baroDrv.Update(); // [??? us]
        PressurePa = baroDrv.PressurePa;
        TemperatureC = baroDrv.TemperatureC;
        // Set Pressure0 on 2 sec
        if(MainLoopCounter == (SysTickFrequency * 2)) PressurePa0 = baroDrv.PressurePa;

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

        // process ethernet (RX)
        etherDrv.Process(1000/SysTickFrequency); // Tick

        // Read Lora Data
        //int dataReceived = serialU5.Read(CommBuffer, COMMBUFFERSIZE);
        //comm433MHz.NewRXPacket(CommBuffer, dataReceived); // calls ProcessCommand callback!!!


        /////////////////////////////////
        // Processing/Control
        /////////////////////////////////
        // 1. Check Override Mode
        if( SBUSSwitchD < 600 )
        {
            // manual override mode
            int PWMScaled = 900;
            if( SBUSThrottle < 400) PWMScaled = 900;
            else PWMScaled = ((SBUSThrottle - 400) * 1100) / 1500 + 900;
            OutPWMThrottle = PWMScaled;
        }
        else
        {
            // Matlab/External Mode
            OutPWMThrottle = PWMRefThrottle;
        }




        /////////////////////////////////
        // OUTPUTS
        /////////////////////////////////
        pwmDrv.SetWidthUS(0, PWMRefThrottle);

        // DBG LED
        //dbgLed.Set(ctrl.rtY.GreenLED);

        // send periodic data (ethernet)
        SendPeriodicDataEth();


        /////////////////////////////////
        // System Stuff
        /////////////////////////////////
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

void SendPeriodicDataEth(void)
{
    // Fill data
    SCommEthData data;
    memset(&data,0, sizeof(data) );
    data.LoopCounter = MainLoopCounter;
    data.ActualMode = 0;
    data.Roll = imu.Roll;
    data.Pitch = imu.Pitch;
    data.Yaw = imu.Yaw;
    data.dRoll = Gyro[0];
    data.dPitch = Gyro[1];
    data.dYaw = Gyro[2];
    data.AccX = Acc[0];
    data.AccY = Acc[1];
    data.AccZ = Acc[2];
    data.MagX = Mag[0];
    data.MagY = Mag[1];
    data.MagZ = Mag[2];

    data.Pressure = PressurePa;
    data.Temperature = TemperatureC;

    // RF Data + Perf
    data.EthReceivedCount = etherDrv.ReceivedFrames;
    data.EthSentCount = etherDrv.SentFrames;
    data.PerfCpuTimeMS = PerfCpuTimeMS;
    data.PerfCpuTimeMSMAX = PerfCpuTimeMSMAX;
    data.PerfLoopTimeMS = PerfLoopTimeMS;

    // send packet (type 0x20 - data)
    //etherDrv.SendPacket(0x20, (char*)&data, sizeof(data));

    // Matlab test stuff
    SCommEthMatlabData dataMat;
    memset(&dataMat,0, sizeof(dataMat) );
    dataMat.LoopCounter = MainLoopCounter;
    dataMat.SharpDistanceSensorADC = SharpDistanceSensorADC;
    dataMat.PulsedLidarRangeCM = PulsedLidarRangeCM;
    dataMat.PulsedLidarErrors = (unsigned short)PulsedLidarErrors; // clip
    dataMat.SBUSThrottle = SBUSThrottle;
    dataMat.SBUSSwitchD = SBUSSwitchD;
    dataMat.RefPWMThrottle = PWMRefThrottle;
    dataMat.OutPWMThrottle = OutPWMThrottle;

    etherDrv.SendPacket(0x20, (char*)&dataMat, sizeof(dataMat));
}

// Process commands received from Ethernet and HopeRF
void ProcessCommand(int cmd, unsigned char* data, int dataSize)
{
    switch( cmd )
    {
        case 0x20:
        {
            // Get PWM Ref
            unsigned int pwmRef;
            if( dataSize == sizeof(pwmRef))
            {
                memcpy(&pwmRef, data, sizeof(pwmRef));
                PWMRefThrottle = pwmRef;
            }
            break;
        }
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
