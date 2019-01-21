/* 
 * File:   main.c
 * Author: bmcgarvey
 *
 * Created on January 15, 2019, 10:48 AM
 */

#include <xc.h>

// PIC18F87J11 Configuration Bit Settings
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG1H
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

// CONFIG2L
#pragma config FOSC = HSPLL     // Oscillator Selection bits (HS oscillator, PLL enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler Select bits (1:32768)

// CONFIG3L
#pragma config EASHFT = ON      // External Address Bus Shift Enable bit (Address shifting enabled, address on external bus is offset to start at 000000h)
#pragma config MODE = MM        // External Memory Bus Configuration bits (Microcontroller mode - External bus disabled)
#pragma config BW = 16          // Data Bus Width Select bit (16-bit external bus mode)
#pragma config WAIT = OFF       // External Bus Wait Enable bit (Wait states on the external bus are disabled)

// CONFIG3H
#pragma config CCP2MX = DEFAULT // ECCP2 MUX bit (ECCP2/P2A is multiplexed with RC1)
#pragma config ECCPMX = DEFAULT // ECCPx MUX bit (ECCP1 outputs (P1B/P1C) are multiplexed with RE6 and RE5; ECCP3 outputs (P3B/P3C) are multiplexed with RE4 and RE3)
#pragma config PMPMX = DEFAULT  // PMP Pin Multiplex bit (PMP port pins connected to EMB (PORTD and PORTE))
#pragma config MSSPMSK = MSK7   // MSSP Address Masking Mode Select bit (7-Bit Address Masking mode enable)

//Project includes
#include <stdio.h>
#include "../src/MPU9250.h"
#include "../eMPL/inv_mpu.h"
#include "../eMPL/inv_mpu_dmp_motion_driver.h"
#include <math.h>

/*
Connections:
        Master RD5 <-> SDA
        Master RD6 <-> SCL
        Master RB1 <-> INT
 */

volatile char dataReady = 0;
short gyro[3];
short accel[3];
short compass[3];
long quat[4];
unsigned long timeStamp;
signed char gyroMatrix[9] = {0, 1, 0,
    1, 0, 0,
    0, 0, 1};

void computeEulerAngles(long *quat);

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    __delay_ms(10); //Wait for PLL to stabilize
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 1;
    //Configure the USART for 115200 baud asynchronous transmission
    SPBRG1 = 68; //115200 baud
    SPBRGH1 = 0;
    TXSTA1bits.BRGH = 1;
    BAUDCON1bits.BRG16 = 1;
    TXSTA1bits.SYNC = 0;
    RCSTA1bits.SPEN = 1;
    TXSTA1bits.TXEN = 1;
    printf("Starting\r\n");

    //setup INT1 for falling edge
    TRISB |= 0b00000010;
    INTCON2bits.INTEDG1 = 0;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;

    //setup Timer2 for 1ms ticks
    T2CONbits.T2CKPS = 0b10; //1:16 prescale
    T2CONbits.TOUTPS = 4; //1:5 postscale gives 100 kHz count rate
    PR2 = 100;
    TMR2 = 0;
    tickCount = 0;
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;

    pic18_i2c_enable();
    unsigned char reg = 0;
    mpuReset();
    pic18_i2c_read(0x68, 117, 1, &reg);
    printf("Who am I = %02x\r\n", reg);
    int error = 0;
    error = mpu_init();
    if (error) {
        printf("mpu_init failed\r\n");
    } else {
        printf("mpu initialized\r\n");
    }
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    long gyroBias[3];
    long accelBias[3];
    error = mpu_run_6500_self_test(gyroBias, accelBias, 0);
    printf("Self test:\r\n");
    printf("   Gyro:");
    if (error & 1) {
        printf("PASSED\r\n");
    } else {
        printf("FAILED\r\n");
    }
    printf("   Accelerometer:");
    if (error & 2) {
        printf("PASSED\r\n");
    } else {
        printf("FAILED\r\n");
    }
    printf("   Compass:");
    if (error & 4) {
        printf("PASSED\r\n");
    } else {
        printf("FAILED\r\n");
    }
    if (error != 0x07) {
        printf("Halting\r\n");
        while (1);
    }
    for (int i = 0; i < 3; i++) {
        gyroBias[i] = (long) (gyroBias[i] * 32.8f); //convert to +-1000dps
        accelBias[i] *= 2048.f; //convert to +-16G
        accelBias[i] = accelBias[i] >> 16;
        gyroBias[i] = (long) (gyroBias[i] >> 16);
    }
    mpu_set_gyro_bias_reg(gyroBias);
    mpu_set_accel_bias_6500_reg(accelBias);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_compass_sample_rate(4);
    mpu_set_sample_rate(4);
//    do {
//        error = dmp_load_motion_driver_firmware();
//        if (error) {
//            printf("Error loading motion driver\r\n");
//        } else {
//            printf("Motion driver loaded\r\n");
//        }
//    } while (error);
//    __delay_ms(1000);
//    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyroMatrix));
//    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
//            | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
//    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
//    dmp_set_fifo_rate(1);
//    mpu_set_dmp_state(1);
    //enable interrupts and turn on TMR2
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    T2CONbits.TMR2ON = 1;
    while (1) {
        if (dataReady) {
            dataReady = 0;
            LATDbits.LATD0 ^= 1;
            unsigned char sensors;
            unsigned char more;
            unsigned long timeStamp = 0;
            more = 0;
            do {
                mpu_read_fifo(gyro, accel, &timeStamp, &sensors, &more);
                mpu_get_compass_reg(compass, NULL);
                //error = dmp_read_fifo(gyro, accel, quat, &timeStamp, &sensors, &more);
                //printf("More = %u\r\n", more);
            } while (more > 0);
            //printf("Time = %lu Error %d\r\n", timeStamp, error);
            printf("Accel: %d %d %d\r\n", accel[0], accel[1], accel[2]);
            printf("Gyro: %d %d %d\r\n", gyro[0], gyro[1], gyro[2]);
            printf("Compass: %d %d %d\r\n\r\n", compass[0], compass[1], compass[2]);
//            //printf("Quat: %ld %ld %ld %ld\r\n\r\n", quat[0], quat[1], quat[2], quat[3]);
//            computeEulerAngles(quat);
        }
    }
}

void __interrupt(high_priority) HighIsr(void) {
    if (PIR1bits.TMR2IF == 1) {
        ++tickCount;
        PIR1bits.TMR2IF = 0;
    }
    if (INTCON3bits.INT1IF == 1) {
        dataReady = 1;
        INTCON3bits.INT1IF = 0;
    }
}

void putch(char c) {
    while (TX1IF == 0);
    TXREG1 = c;
}

void computeEulerAngles(long *quat) {
    char degrees = 1;
    float dqw = quat[0] / 1073741824.0;
    float dqx = quat[1] / 1073741824.0;
    float dqy = quat[2] / 1073741824.0;
    float dqz = quat[3] / 1073741824.0;

    //    float ysqr = dqy * dqy;
    //    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    //    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    //    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    //    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    //    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
    //  
    //	// Keep t2 within range of asin (-1, 1)
    //    t2 = t2 > 1.0f ? 1.0f : t2;
    //    t2 = t2 < -1.0f ? -1.0f : t2;
    //  
    //    float pitch = asin(t2) * 2;
    //    float roll = atan2(t3, t4);
    //    float yaw = atan2(t1, t0);

    //My method for pitch and roll
    //TODO Add yaw calculation?
    float roll = atan2(dqy * dqz + dqw * dqx, 0.5f - (dqx * dqx + dqy * dqy));
    float pitch = asin(-2.0f * (dqx * dqz - dqw * dqy));
    float yaw = 0.0;


#define PI 3.14159
    if (degrees) {
        pitch *= (180.0 / PI);
        roll *= (180.0 / PI);
        yaw *= (180.0 / PI);
        //if (pitch < 0) pitch = 360.0 + pitch;
        //if (roll < 0) roll = 360.0 + roll;
        //if (yaw < 0) yaw = 360.0 + yaw;	
    }
    printf("Pitch = %.1f Roll = %.1f Yaw = %.1f\r\n\r\n", pitch, roll, yaw);
}
