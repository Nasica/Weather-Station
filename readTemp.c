#include <pigpio.h> // For gpioInitialise(), i2cOpen(), i2cWriteByte(), i2cReadDevice(),
#include <stdio.h> // For perror(),
#include <stdlib.h> // For exit(),
#include <unistd.h> // For sleep(),

#define CMD_RESET       0x1E
#define PROM_START      0xA0
#define PROM_STOP       0xAE
#define ADC_READ        0x00

#define CONVERT_D1_OSR8192      0x4A
#define CONVERT_D2_OSR8192      0x5A


int initGPIO(int i2cBus, int i2cAddr);
static int getCalibData(int fd, u_int16_t *proms);
static u_int32_t getRawTemp(int fd);
static int32_t calcTempDiff(u_int32_t rawTemp, u_int16_t refTemp);
static int32_t calcTemp(int32_t rawTemp, u_int16_t *proms);
static int secondOrderTempComp(int32_t temp, int32_t dTemp);
static int power(int base, int exp);

/*
 * Initialises a gives i2c address on a given i2c bus
 * parameters - i2cBus - Integer of the i2c bus the device is on
 *              i2cAddr - Integer of the address of the i2c device
 *
 * Returns -    fd(int) - File Descriptor of the i2c device
 */
int initGPIO(int i2cBus, int i2cAddr){
    int fd;
    //Initialise GPIO
    if(gpioInitialise() < 0) {
        perror("Initialisation failed\n");
        exit(EXIT_FAILURE);
    }
    
    //Open the I2C bus
    fd = i2cOpen(i2cBus, i2cAddr, 0);
    if(fd < 0) {
        perror("Device failed to  open\n");
        exit(EXIT_FAILURE);
    }

    //Send Reset
    if(i2cWriteByte(fd, CMD_RESET) != 0){
        perror("Error sending reset---\n");
    }
    return(fd);
}

/*
 * Reads the calibration data from the PROMs on the sensor
 * Parameters - fd - File Descriptor of the i2c device
 *              *proms - Pointer to an array of 8 unsigned 16-bit integers
 */
static int getCalibData(int fd, u_int16_t *proms) {
    const int bytesToRead = 2;
    char buf[2] = {0};
    // Populate for each prom (7)
    for(int i = PROM_START; i < PROM_STOP; i = i + 2){
        // Write PROM read commands
        if(i2cWriteByte(fd, i) != 0) {
            perror("Error writing PROM command!!\n");
            exit(EXIT_FAILURE);
        }

        // Read result from PROM
        if(i2cReadDevice(fd, buf, bytesToRead) <= 0) {
            perror("Error reading from PROM\n");
            exit(EXIT_FAILURE);
        }

        // Store result in array
        proms[(i - PROM_START) / 2] = (buf[0] << 8) | (buf[1]);
    }
    return(0);
}

int getTemp(int fd){
    u_int16_t proms[8];
    u_int32_t rawTemp = getRawTemp(fd);
    getCalibData(fd, proms);
    int temp = calcTemp(rawTemp, proms);
    return(temp) ;
}


/*
 * Gets the raw temperature data from the sensor
 * Parameters - fd - File Descriptor of the sensor
 * 
 * Returns -    rawTemp - Raw, uncompensated temp value (u_int32_t)
 */
static u_int32_t getRawTemp(int fd){
    const int bytesToRead = 3;
    char buf[3] = {0};
    int32_t rawTemp;

    // Request for most accurate temp data
    if(i2cWriteByte(fd, CONVERT_D2_OSR8192) != 0){
        perror("Error writing PROM command\n");
        exit(EXIT_FAILURE);
    }
    sleep(1); // Wait for on-chip processing

    // Signal ready to read the data
    if(i2cWriteByte(fd, ADC_READ) != 0){
        perror("Error writing PROM command\n");
        exit(EXIT_FAILURE);
    }

    // Read the temperature, storing it in buf
    if(i2cReadDevice(fd, buf, bytesToRead) <= 0){
        perror("Error reading from device\n");
        exit(EXIT_FAILURE);
    }
    sleep(1); // Wait for on-chip processing

    rawTemp = (buf[0] << 16) | (buf[1] << 8) | (buf[2]);
    return(rawTemp);
}

/*
 * Calculate the difference between the actual and reference temperature
 * Parameters - rawTemp - Raw Temperature data
 *              refTemp - Reference temperature (read from PROM 5)
 * 
 * Returns -    Temperature difference (int32_t)
 */
static int32_t calcTempDiff(u_int32_t rawTemp, u_int16_t refTemp){
    int dTemp;

    dTemp = rawTemp - refTemp * (1U << 8);
    return dTemp;
}

/*
 * Calculates the temperature from given raw temperature data
 * Parameters - rawTemp - Raw temperature data
 *              proms   - Array of u_int16_t PROM calibration data
 *              sizeOfProms - Number of elements in PROMs
 *
 * Returns -    temp - Actual temperature (int32_t)
 */
static int32_t calcTemp(int32_t rawTemp, u_int16_t *proms){
    int32_t temp = 0;
    int correction;
    int dTemp = calcTempDiff(rawTemp, proms[5]);

    temp = 2000 + dTemp * proms[6] / (1UL << 23);
    //correction = secondOrderTempComp(temp, dTemp);
    //temp = temp - correction;
    return temp;
}

/*
 * Conducts second order temperature compensation to improve accuracy at low temps
 * Parameters - temp - Calculated temperature 
*              dTemp - Difference between actual and reference temperature
 *
 * Returns - Offset to be applied to temperature
 */
static int secondOrderTempComp(int32_t temp, int32_t dTemp){
    int correction = 0;
    if(temp < 2000){
        correction = 3 * (power(dTemp, 2)) / (1ULL << 32);
    } else {
        correction = 5 * (power(dTemp, 2)) / (1ULL << 38);
    }
    return(correction);
}

/*
 * Gets the raw pressure data from the sensor
 */
static u_int32_t getRawPressure(int fd){
    char buf[3] = {0};
    const int bytesToRead = 3;
    u_int32_t rawPressure;
    
    // Set resolution to highest available
    if(i2cWriteByte(fd, CONVERT_D1_OSR8192) != 0) {
        perror("Unable to write to device\n");
        exit(EXIT_FAILURE);
    }
    sleep(1); // Wait for on-chip processing

    // Send ready for read command
    if(i2cWriteByte(fd, ADC_READ) != 0) {
        perror("Unable to write to device\n");
        exit(EXIT_FAILURE);
    }

    if(i2cReadDevice(fd, buf, 3) <= 3) {
        perror("Unable to read from device\n");
        exit(EXIT_FAILURE);
    }
    sleep(1);

    rawPressure = (buf[0] << 16) | (buf[1] << 8) | (buf[0]);
    return(rawPressure);
}

//!!!!!!!!!!!!!!!!!!!!!!!
// Correct OFFSET and SENS with second order 


/*
 * Calculates the pressure offset at actual temperature
 * Parameters - dTemp - Difference between actual and reference temperature
 *              pressOffset - proms[2]
 *              tempCoeffOfPressOffset - proms[4]
 *
 * Returns - offset (int64_t)
 */
static int64_t calcPressOffsetAtTemp(int32_t dTemp, u_int16_t pressOffset, u_int16_t tempCoeffOfPressOffset) {
    int64_t pressOffsetAtTemp;

    pressOffsetAtTemp = pressOffset * (1U << 17) + (tempCoeffOfPressOffset * dTemp) / (1U << 6);
    return(pressOffsetAtTemp);
}

/*
 * Calculates the sensitivity at actual temperature
 * Parameters - dTemp - Difference between actual and reference temperature
                pressSens - proms[1]
 *              tempCoeffOfPressSens - proms[3]
 * Returns - Sensitivity at temperature (int64_t)
 */
static int64_t calcSensAtTemp(int32_t dTemp, u_int16_t pressSens, u_int16_t tempCoeffOfPressSens){
    int64_t sensAtTemp;

    sensAtTemp = pressSens * (1UL << 16) + (tempCoeffOfPressSens * dTemp) / (1U << 7);
    return(sensAtTemp);
}

/*
 * 
 */
static int32_t calcPressure(u_int32_t rawPressure, int64_t sensAtTemp, int64_t pressOffsetAtTemp ) {
    int32_t pressure;
    
    pressure = (rawPressure * sensAtTemp / (1UL << 21) - pressOffsetAtTemp) / (1UL << 15);
    return(pressure);
}


/*
 * Calculates base to the power of exp
 */
static int power(int base, int exp){
    if (exp != 0){
        return (base*power(base, exp - 1));
    } else {
        return 1;
    }
}





