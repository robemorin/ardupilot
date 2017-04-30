#include "Copter.h"
#include "Raspberry.h"

// #include <AP_HAL/AP_HAL.h>
// const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Raspberry::Raspberry()
{

}
void Raspberry::helloWorld(void)
{
 hal.console->printf("Hola Ardupilot");
}
void Raspberry::OneLoop()
{
  uint16_t radio_in;
  radio_in = hal.rcin->read(5);
  uint8_t msj[]={0,2,2,1,5};
  if(radio_in>1500){
    msj[3]=2;
    hal.rcout->force_safety_off();
    hal.rcout->enable_ch(0);
    hal.rcout->write(0, 1500);
  }else{
    msj[3]=1;
  }
  msj[4]=4+msj[3];
  for (int i=0;i<5;++i) hal.uartD->write(msj[i]);
}

// AP_HAL_MAIN();