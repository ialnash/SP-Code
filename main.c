//#############################################################################
//
// FILE:   main.c
//
// TITLE:  Universal Empty Project
//
// Universal Empty Project Example
//
// This example is an empty project setup for Driverlib development.
//!
//!  \note This example project has support for migration across our C2000 
//!  device families. If you are wanting to build this project from launchpad
//!  or controlCARD, please specify in the .syscfg file the board you're using.
//!  At any time you can select another device to migrate this example.
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
//The peripherals are initialized by the auto-generated code from the .syscfg file, and can be accessed either from the .syscfg file or "board.c"
//The configurations can be reached from the header file "board.h" .

//
// Included Files
//
#include "autoinit.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
//#include "sfra_cfg.c"
//#include "sfra_gui_scicomms_driverlib.c"

//
// Globals
//
uint16_t cpuTimer0IntCount;
uint16_t fault_trigger;
float32_t duty_cycle;
volatile float32_t set_duty_cycle = 0;
uint16_t compare_value;
uint16_t ePWM0_IntCount;
uint16_t timer1state = 0;
uint16_t cpuTimer1IntCount;
uint16_t led_counter;
float32_t duty_stepmax = 0.01;
volatile uint16_t activeload_cont;
volatile uint16_t activeload_enabled;
float32_t scalefactor_i_l_max = 7.52;
float32_t scalefactor_v_in_max = 13.2;
float32_t scalefactor_v_out_max = 6.7;
float32_t tripvalue_i_l = 5.2;
float32_t adc_12bfactor = 4096;
float32_t v_inFBdisplay;
float32_t v_outFBdisplay;
float32_t i_lFBdisplay;
volatile float32_t set_trip_value = 0;
uint16_t invalid_adjustment;
volatile uint16_t clear_trip_flag;
float32_t compare_float;
float32_t period_float;
//
// Function Prototypes
//
void configure_duty(float32_t cycle_update);
void configure_activeresistor(void);
float32_t dutyadjust_slew(void);
void adc_readvalues(void);

//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
     Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();

    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();


    //
    // ePWM time based clock initialization
    //

    //NOTE: In the final build before the open loop, migrate initialization module to a function


    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);


    while(1)
    {
        
    }
}

__interrupt void cpuTimer0ISR(void)
{
    cpuTimer0IntCount ++;

    //Update duty_cycle value
    //NOTE: This should be replaced with code to control slew rate given input of duty cycle.

    compare_value = EPWM_getCounterCompareValue(EPWM1_BASE ,EPWM_COUNTER_COMPARE_A);

    compare_float = (float)(compare_value);

    period_float = (float)(myEPWM0_TBPRD);

    duty_cycle = ((compare_float)/(period_float));

    //Set duty cycle to the user input from debug variable "set_duty_cycle", checking for valid values and triggering fault management for invalid values
    if (set_duty_cycle == 0) {
        ;
    }
    else if(set_duty_cycle > 0 && set_duty_cycle < 0.5) {
        float32_t dutycycle_stepped = duty_cycle + dutyadjust_slew();

        configure_duty(dutycycle_stepped);
    }
    //For an out-of-bounds adjustment attempt, check the current duty cycle. If the current configuration is valid, flash a warning LED and reject the operation. If invalid, trigger fatal error trip on the PWM
    else {
        if(duty_cycle > 0 && duty_cycle < 0.9) {
            invalid_adjustment ++;
        }
        else {
            fault_trigger ++;
        }
    }
    //Fault management and error control module, kill operation if a fatal error is detected; otherwise acknowledge the interrupt and end the iteration

    if (invalid_adjustment > 0) {
        if ( (set_duty_cycle > 0 && set_duty_cycle < 0.5) && (set_trip_value > 0 && set_trip_value < scalefactor_i_l_max)) {
            invalid_adjustment = 0;
        }
        else
        {
            GPIO_writePin(faultLED_GPIO, 0);

            DEVICE_DELAY_US(500000);

            GPIO_writePin(faultLED_GPIO, 1);
        }

    }

    if (fault_trigger == 0) {

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    }
    else {
    //Kill operation, stay here and blink fault LED until fault_trigger is cleared
        //ePWM_TZCTL; EPWM1_BASE is the address
        EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);

        while (fault_trigger == 1) {
            GPIO_writePin(faultLED_GPIO, 0);

            DEVICE_DELAY_US(100000);

            GPIO_writePin(faultLED_GPIO, 1);

            DEVICE_DELAY_US(100000);
            if (clear_trip_flag == 1) {
                fault_trigger --;

                clear_trip_flag = 0;

                Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
            }

        }
    }
}
__interrupt void INT_myCPUTIMER1_ISR(void)
{ // 10 ms clock to service SFRA gui and control heartbeat LED. Primitive flow control implemented to represent state machine functionality and servicing of SFRA.



    cpuTimer1IntCount ++;
    //SFRA_GUI_runSerialHostComms(SFRA_F32);
    led_counter ++;
    if(led_counter == 10) {
        GPIO_togglePin(testLED_GPIO);
        led_counter = 0;
    }



}
__interrupt void INT_myCPUTIMER2_ISR(void)
{ // 100 ms clock to adjust trip values and other variables, add target value adjustments here. NOTE:Slew control in the 1hz module.

    uint16_t cmpsshigh_value;

    configure_activeresistor();

    if (set_trip_value == 0)
    {

    }

    else if (set_trip_value > 0 && set_trip_value < scalefactor_i_l_max)
    {
        tripvalue_i_l = set_trip_value;
    }
    else
    {
        invalid_adjustment ++;
    }

    cmpsshigh_value = ((tripvalue_i_l) / (scalefactor_i_l_max));

    CMPSS_setDACValueHigh(CMPSS1_BASE, cmpsshigh_value);


}

__interrupt void INT_adc_C_2_ISR(void)
{
    adc_readvalues();

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

}

__interrupt void INT_myEPWM0_TZ_ISR(void)
{
    ePWM0_IntCount ++;
    uint16_t dummy_var = 0;
    fault_trigger ++;
    dummy_var ++;

}

//Function to configure duty cycle of ePWM modules. DO NOT call directly, this function is used in the slew rate adjusted "dutyadjust_slew" function.

// NOTE: Generalize by adding an input variable to determine the ePWM module to configure, achieving control of the active load. Different periods are expected to be irrelevant for this case and can be left unchanged.

void configure_duty(float32_t cycle_update)

{
    //Define period and counter compare a values. Pass integerized value of counter compare A to the ePWM Counter Compare module.
    uint16_t period, integerized_cmpa;
    float32_t rawvalue_cmpa;

    period = myEPWM0_TBPRD;

    rawvalue_cmpa = (float) ((period)*(cycle_update));

    integerized_cmpa = (int)rawvalue_cmpa ;

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, integerized_cmpa);
}

// Function that generates slewed input steps to configure duty cycle of synchronous ePWM module.
float32_t dutyadjust_slew(void)
{
    float32_t duty_step;

    float32_t step_return;

    duty_step = (set_duty_cycle - duty_cycle);

    if(duty_step > duty_stepmax)
    {
        step_return = duty_stepmax;

    }
    else if (duty_step < (0 - duty_stepmax))
    {
        step_return = (0 - duty_stepmax);
    }

    else
    {
        step_return = (set_duty_cycle - duty_cycle);
    }

    return step_return;
}
//Function that configures the active resistor of the BOOSTXL-BUCK. The function takes no arguments and does not return a value since it's run entirely through the expressions window in debug mode.
//activeload_config is declared as a local variable with a default value of 0. Since the conditional expressions are always processed before the final instruction, this is not a problem.
//Time based counter period is 35000 and in up count mode, TBCLK = 0 sets pins to high and TBCLK = CMPA sets pins low. We use (TBPRD-50) and (TBPRD/2 - 50) because of a known ePWM glitch.
void configure_activeresistor(void)
{
    uint16_t activeload_config = 50;

    if(activeload_enabled == 1)
    {
        if(activeload_cont == 1)
        {
            activeload_config = 34950;

            EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, activeload_config);

        }
        else
        {
            activeload_config = 17550;

            EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, activeload_config);

        }
    }
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, activeload_config);
}
void adc_readvalues(void)
{
        v_inFBdisplay = (float) (((scalefactor_v_in_max)*(ADC_readResult(adc_B_RESULT_BASE,adc_B_SOC0))) / (adc_12bfactor));

        v_outFBdisplay = (float) (((scalefactor_v_out_max)*(ADC_readResult(adc_C_RESULT_BASE,adc_C_SOC0))) / (adc_12bfactor));

        i_lFBdisplay = (float) (((scalefactor_i_l_max)*(ADC_readResult(adc_C_RESULT_BASE,adc_C_SOC1))) / (adc_12bfactor));

}


    //uint16_t sample_count = 8;
    //float32_t sum_i_l = 0;
   // float32_t sum_v_out = 0;
    //float32_t sum_v_in = 0;


//
// End of File
//

