/* 
 * File:   MPU6050.h
 * Author: bmcgarvey
 *
 * Created on January 15, 2019, 10:53 AM
 */

#ifndef MPU9250_H
#define	MPU9250_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

    //i2c functions
    int pic18_i2c_enable(void);
    int pic18_i2c_disable(void);
    int pic18_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
    int pic18_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

    //timing functions
    #define _XTAL_FREQ   32000000L
    extern unsigned long tickCount;
    int pic18_delay_ms(unsigned long num_ms);
    int pic18_get_ms(unsigned long *count);
    
    //Utility fucntions
    void mpuReset(void);
    
    //DMP library defines
    #define MPU9250
    #define EMPL_NO_64BIT
    #define i2c_write   pic18_i2c_write
    #define i2c_read    pic18_i2c_read
    #define delay_ms    pic18_delay_ms
    #define get_ms      pic18_get_ms
    #define log_i       printf
    #define log_e       printf
    #define min(a,b) ((a<b)?a:b)

#ifdef	__cplusplus
}
#endif

#endif	/* MPU9250_H */

