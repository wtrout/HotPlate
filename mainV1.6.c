/*

PORT B

PB0     8   DQ
PB1     9   Sense (check if MOSFET/heater is functioning)
PB2    10   Display DP
PB3    11   UI0 (rotary A)
PB4    12   UI1 (rotary B)
PB5    13   UI2 (rotary click)


PORT C

PC0    A0   Display A
PC1    A1   Display E
PC2    A2   Display F
PC3    A3   Display D
PC4    A4   Display C
PC5    A5   Display G


PORT D

PD0     0   RX
PD1     1   TX
PD2     2   Display Digit 4
PD3     3   Display Digit 2
PD4     4   Display Digit 3
PD5     5   PWM
PD6     6   Display B
PD7     7   Display Digit 1


// peak rise is something like 0.1 °C / sec -> 20 centi per loop

*/

/*

Program Flow

- Setup
    - configure ports, setup timers, hardware tests

- Loop
    - looks to see if user input was captured -> change set point
    - looks to see if it's time to run the PID routine -> update temp and pwm

- ** Interrupts **
    - every 1,600 µs (625 Hz) Timer2: cycle to next display digit; check UI button states
    - every 2 seconds Timer1: set the PID flag indicating it's time to run the PID routine; also resets the watchdog timer

*/


#include <Arduino.h>
#include <OneWire.h>
#include <avr/wdt.h>

OneWire ds(8);
uint8_t wireAddr[8]; // address of 1-wire temperature sensor
uint8_t data[9];
uint8_t CRC = 0;
// Using the full CRC table because we have plenty of memory and reducing the time is helpful
constexpr uint8_t CRC_TABLE[] = {0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 5};

uint8_t fbuf[4]; // digits in segment display; 0 is left most digit
int32_t setPoint = 3000; // starting temperature set point after power loss centi°C
int32_t temp = 2000; // current temperature centi°C
int32_t pwm = 0; // 0-255 heater power; int32 for PID calc convenience
volatile bool PID_Tick = true;
volatile int8_t dispHold = 0;

constexpr uint32_t MAX_SET_POINT             = 120 * 100;
constexpr uint32_t MIN_SET_POINT             =   0 * 100;
constexpr  int32_t HIGHEST_VALID_MEASUREMENT = 125 * 100; //replaced w/ CRC
constexpr  int32_t LOWEST_VALID_MEASUREMENT  =   0 * 100; //replaced w/ CRC
constexpr  int32_t roomTemp                  =  19 * 100;


ISR(TIMER1_COMPA_vect){
    PID_Tick = true; // tells PID() to update in the main loop
    if(dispHold > 0){ // we also use this to time the user input display hold because it's convenient
        dispHold--;
    }
}

ISR(TIMER2_COMPA_vect){

    //                                   0     1     2     3     4     5     6     7     8     9     [ ]   E   
    const uint8_t digitSegments_C[12] = {0x1F, 0x10, 0x2B, 0x39, 0x34, 0x3D, 0x3F, 0x11, 0x3F, 0x35, 0x00, 0x2F};
    const uint8_t digitSegments_D[12] = {0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x40, 0x40, 0x40, 0x00, 0x00};
    const uint8_t digitPlacements[4]  = {0x80, 0x08, 0x10, 0x04};
    static uint8_t index = 0;

    uint8_t value = fbuf[index];
    PORTC = digitSegments_C[value];
    PORTD = digitSegments_D[value] | digitPlacements[index];
    index == 3 ? index = 0 : index++;
}


// Accepts integer for display
void setInt(uint32_t value){
    value = constrain(value, 0, 150);
    uint8_t h = value / 100;

    fbuf[1] = h ? h : 10; // if hundreds is zero put a space (10)
    fbuf[3] = value % 10; // singles
    uint8_t t = (value / 10) % 10; // tens
    fbuf[2] = (t || h) ? t : 10; // handle < 10°C but also 10X °C
}

void error(int8_t errno){
    TCCR0A = bit(WGM01) | bit(WGM00);
    PORTD &= 0xDF;

   if((errno >= 5) || (errno < 1)){
        errno = 0;
    }

    fbuf[0] = 11; // "E"
    fbuf[1] = 10; // " "
    fbuf[2] = 10; // " "
    fbuf[3] = errno;

    while(true){
        if(errno == 1){
            for(uint8_t i=0; i<9; i++){
                //Serial.print(data[i]);
                //Serial.print(",");
            }
            //Serial.print(CRC);
        }
            
        wdt_reset();
    }
    // display error number:
    // 0 - out of range error number (not sure how)
    // 1 - multiple temperature sensor mis-reads
    // 2 - watchdog timer reset
    // 3 - heater drain low when MOSFET is switched off
    // 4 - heater drain high when MOSFET is switched on
}

bool updateTempCRC(){
    CRC = 0;
    // read data
    ds.reset();
    ds.select(wireAddr);
    ds.write(0xBE); // read scratchpad command

    for(uint8_t i=0; i<9; i++){
        data[i] = ds.read();
        CRC = CRC_TABLE[CRC^data[i]];
    }

    // CRC of data and transmitted CRC = 0 when there are no errors
    if(CRC == 0){
        uint16_t lower = data[0];
        uint16_t upper = data[1];
        uint32_t reading = (upper << 8) + lower;
        reading *= 100;
        reading /= 16;
        temp = reading;
    }

    // request temperature update for next time (needs >750 ms to aquire)
    ds.reset();
    ds.select(wireAddr);
    ds.write(0x44, 1); // aquire temperature command

    return (CRC == 0);
}

bool updateTemp(){
    bool valid = false;
    // read data
    ds.reset();
    ds.select(wireAddr);
    ds.write(0xBE); // read scratchpad command

    uint16_t lower = ds.read();
    uint16_t upper = ds.read();
    uint32_t reading = (upper << 8) + lower;
    reading *= 100;
    reading /= 16;
    //Serial.println(reading);

    if((reading < HIGHEST_VALID_MEASUREMENT) && (reading > LOWEST_VALID_MEASUREMENT)){
        temp = reading;
        valid = true;
    }

    // request temperature update for next time (needs >750 ms to aquire)
    ds.reset();
    ds.select(wireAddr);
    ds.write(0x44, 1); // aquire temperature command
    
    return valid;
}

void PID(){
    static int8_t tempErrorCount = 0;
    static int32_t lastTemp = 0;

    // Turn Off PWM - ensures reliable temperature data aquisition
    // The pin must remain in output mode so that we can drain the FET gate
    TCCR0A = bit(WGM01) | bit(WGM00);
    PORTD &= 0xDF;
    wdt_reset();

    bool validTemp = updateTemp();
    // if we have a valid measurement do the PID stuff; change this to CRC check and/or(?)
    if(validTemp){

        // These are essentially like the PID k values.
        constexpr int32_t PROPORTIONAL_MULTIPLIER = 58;
        constexpr int32_t LOAD_MULTIPLIER = 4;
        constexpr int32_t PL_DIVIDER = 256;
        constexpr int32_t INTEGRAL_DIVIDER = 256;

        // Simple PID parameters
        int32_t delta = setPoint - temp;        // standard proportional error value
        int32_t load = setPoint - roomTemp;     // atypical 'constant' value - accounts for cooling losses proportional to set point and rt
        int32_t velocity = temp - lastTemp;     // standard derivative value
        static int32_t accumulator = 0;         // standard error accumulator value
        pwm = (delta * PROPORTIONAL_MULTIPLIER) + (load * LOAD_MULTIPLIER); // int based scaler to...
        pwm /= PL_DIVIDER;                                                  //  map PID to 8bit PWM


        // Integrator Constraints

        // if power is 100% accumulator cannot rise
        // if power is 0% accumulator cannot fall

            // not saturated    ||   saturated but accumulator is moving in the opposite direction of saturation
        if( (pwm < 255 && pwm > 0) || (pwm >= 255 && delta < 0) || (pwm <= 0 && delta > 0) ){
            accumulator += delta;
            if(accumulator > 16384){
                accumulator = 16384;
            }
            if(accumulator < -16384){
                accumulator = -16384;
            }
        }

        // if temperature is rising rapidly and accumulator is positive, zero it
        if(velocity > 4 && accumulator > 0){
            accumulator = 0;
        }
        // if temperature is fally rapidly and accumulator is negative, zero it
        if(velocity < -3 && accumulator < 0){
            accumulator = 0;
        }

        // accumulator scaler to 8bit PWM
        pwm += (accumulator / INTEGRAL_DIVIDER);
        /*
        if(tempErrorCount > 0){
            // reduce the error counter since we got a valid signal
            tempErrorCount--;
        }*/
        tempErrorCount = 0;

    }
    else{ // temperature error
        tempErrorCount++;
        if(tempErrorCount > 3){
            error(1);
        }
    }


    pwm = constrain(pwm, 0, 255);

    uint8_t pwr = 0;
    if(pwm == 255){
        pwr = 9;
    }
    else if(pwm > 0){
        pwr = (pwm + 31) / 32;
    }
    fbuf[0] = pwr;


    if(pwm > 0){
        // then turn the power back on
        OCR0B = pwm;
        TCCR0A = bit(WGM01) | bit(WGM00) | bit(COM0B1);
        // If it's 0 then just leave it off (turned off for noise during measurement)
    }

}

int16_t rotary(){
    // Rotary Input Section

    static uint8_t state = 0;
    static bool dir = 0;
    uint8_t input = PINB & 0x18;
    int16_t output = 0;

    if(input != 0x18){ // some unstable state (not at bottom rest position)
        switch (input){
            case 0x00: //top
                state |= 0x04;
                break;
            case 0x08: //set case 2
                state |= 0x02;
                dir = HIGH;
                break;
            case 0x10: //set case 1
                state |= 0x01;
                dir = LOW;
                break;
        }
    }
    else{ // rest state
        if(state == 0x07){ // we have a tick
            if(dir){
                output += 100; // swap these to change rotary direction
            }
            else{
                output -= 100;
            }
        }
        state = 0; // clear the state regardless of a tick or not
    }
    return output;
}

void startupTest(){
    // check mosfet and temperature sensor and reboot reason

    // Check to see if there was a WTDR
    if(MCUSR & bit(WDRF)){
        error(2); // terminates
    }

    // ensure PWM pin is LOW
    TCCR0A = 0;
    PORTD &= 0xDF;

    // MOSFET drain should be HIGH, otherwise -> error 3
    if(digitalRead(9) == LOW){
        error(3); // terminates
    }
    digitalWrite(5, HIGH);
    delay(1);
    // MOSFET drain should be LOW now, otherwise -> error 4
    if(digitalRead(9) == HIGH){
        error(4); // terminates
    }
}



void setup(){
    Serial.begin(115200);
    // get address of 1-wire sensor and request temp update
    ds.search(wireAddr);
    ds.reset();
    ds.select(wireAddr);
    ds.write(0x44, 1);
    delay(750);

    //startupTest();

    DDRC = 0x3F; //all of PORTC to OUTPUT for LEDs
    DDRD = 0xFC; //all except Serial pins 1111 1100; LED cathode transistors and PWM
    pinMode(13, INPUT_PULLUP); // config the rotary encoder pins
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);

    // PWM Clock
    // ~1kHz fast PWM with output turned off for now
    TCCR0A = bit(WGM01) | bit(WGM00);// | bit(COM0B1);
    TCCR0B = bit(CS01) | bit(CS00);
    OCR0B = 0;

    // PID Clock 
    // ~0.5 Hz activates the temperature update and PID algorithm
    TCCR1A = 0;
    TCCR1B = bit(CS12) | bit(CS10) | bit(WGM12);
    OCR1A = 31250;
    TIMSK1 = bit(OCIE1A);

    // Display Update Clock
    // 625 Hz scans through the 4 digits so refresh rate is ~150 Hz
    TCCR2A = bit(WGM21); // CTC mode, OCRA is top for finer adjustment
    TCCR2B = bit(CS22) | bit(CS21);
    OCR2A = 255; // 16 MHz / 256 / 100 -> 625 Hz
    TIMSK2 = bit(OCIE2A);

    fbuf[0] = 10; // clear left-most digit; not in use currently
    setInt(setPoint / 100); // show the set point


    // config watchdog
    MCUSR = 0;
    WDTCSR = bit(WDCE) | bit(WDE); // prep register then immediately config
    WDTCSR = bit(WDE) | bit(WDP3); // 4s timeout to reset
    wdt_reset();

    interrupts(); // for both PID and Display
}

void loop(){

    if(PID_Tick){ // PID_Tick is set to true when the PID timer expires (~0.5 Hz)
        PID();
        PID_Tick = false;
        if(dispHold == 0){ // countdown ticker that prevents the current temperature from immediately overwritting user input set point
            setInt(temp / 100);
        }
    }

    int16_t userInput = rotary();
    if(userInput){
        setPoint += userInput;
        if(setPoint < MIN_SET_POINT){
            setPoint = MIN_SET_POINT;
        }
        if(setPoint > MAX_SET_POINT){
            setPoint = MAX_SET_POINT;
            }
        setInt(setPoint / 100);
        dispHold = 3; // holds the setpoint briefly before resuming current temp updates
    }
    

}

/*
!@#

- todo
+ done
x canceled


Software todo:
    + breakout character rendering so we can put heating indicators and custom stuff on the display
    x skip PID during dispHold timout
    + mosfet drain sensor
    - switch temp value range checker to CRC

Error codes:
    + heater current failure
        MOSFET off -> measure voltage at drain; MOSFET on -> measure no voltage at drain
    - heating response failure
        algorithm
    

Failsafes:
    - thermal cutout
    - max power accumulator (like if stuck at 100% for a while cut it)
    - beeper
    + WDT


Hardware to add:
    - beeper
    + V sense on MOSFET drain



*/