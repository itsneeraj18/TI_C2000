//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
//! \addtogroup driver_example_list
//! <h1>Empty Project Example</h1> 
//!
//! This example is an empty project setup for Driverlib development.
//!
//#############################################################################

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"

// LED States
#define LED_ON 0
#define LED_OFF 1
uint32_t  interruptCounter =0;
typedef struct {
    uint8_t LED_State;
    uint32_t LED_Pin;
} LED_Controls;

LED_Controls led_green = {LED_ON, LED_GREEN}; // Initialize green LED
LED_Controls led_red = {LED_OFF, LED_RED};       // Initialize red LED

void LED_Control(void);
__interrupt void INT_myEPWM0_ISR(void);

// LED Control Function
void LED_Control() {
    GPIO_writePin(led_red.LED_Pin, led_red.LED_State); // Turn on the red LED
}

// Main Function
void main(void) {
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pull-ups
    Device_initGPIO();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell ISR
    Interrupt_initVectorTable();

    // PinMux and Peripheral Initialization
    Board_init();

    // C2000Ware Library initialization
    C2000Ware_libraries_init();

    // Register ISR
    Interrupt_register(INT_EPWM1, INT_myEPWM0_ISR);
    Interrupt_enable(INT_EPWM1);

    // Enable Global Interrupt (INTM) and real-time interrupt (DBGM)
    EINT;
    ERTM;

    // Control LED states
    LED_Control();

    while (1) {
        // Main loop code
    }
}

// Define the ISR
__interrupt void INT_myEPWM0_ISR(void) {
    // Toggle green LED state

    //NEER_ADD


    interruptCounter++;
    if (interruptCounter <5)
    {
        GPIO_writePin(LED_GREEN,0);
    }

    else if ( interruptCounter <10)
       {
           GPIO_writePin(LED_GREEN,1);
           interruptCounter=0;
       }



    //THis is using timer counts directly
/*
    led_green.LED_State = !led_green.LED_State;
    GPIO_writePin(led_green.LED_Pin, led_green.LED_State); // Write new state
    led_red.LED_State = !led_green.LED_State;
        GPIO_writePin(led_red.LED_Pin, led_green.LED_State); // Write new state
*/

/*
    interruptCounter++;

        // Check if we've reached the count for 3 seconds (which is ~38 interrupts)
        if (interruptCounter >= 99) { // 3 seconds / 0.08 seconds = 37.5, round up to 38
            // Toggle the green LED state
            led_green.LED_State = !led_green.LED_State; // Toggle state
            GPIO_writePin(led_green.LED_Pin, led_green.LED_State); // Write new state

            // Reset the counter after toggling
            interruptCounter = 0; // Reset to count again
        }
*/
    // Clear INT flag for this timer
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    // Acknowledge this interrupt to receive more interrupts from group 3
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// End of File
