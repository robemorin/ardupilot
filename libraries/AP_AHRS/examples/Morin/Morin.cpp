//
// Simple test for the AP_AHRS interface
//

#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

void setup();
void loop();
void sendPose(double r,double p, double y);
void writeRpi(unsigned char cad[],int N);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// INS and Baro declaration
AP_InertialSensor ins;

Compass compass;

AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;

class DummyVehicle {
public:
    RangeFinder sonar {serial_manager, ROTATION_PITCH_270};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, sonar, EKF2, EKF3,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF2 EKF2{&ahrs, barometer, sonar};
    NavEKF3 EKF3{&ahrs, barometer, sonar};
};

static DummyVehicle vehicle;

// choose which AHRS system to use
// AP_AHRS_DCM  ahrs(ins, baro, gps);
AP_AHRS_NavEKF ahrs(vehicle.ahrs);

void setup(void)
{
    AP_BoardConfig{}.init();

    ins.init(100);
    ahrs.init();
    serial_manager.init();
    
    hal.uartD->begin(57600);
    
    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(nullptr, serial_manager);
}
void writeRpi(unsigned char cad[],int N){
  unsigned char S=0,n=0;
  
  hal.uartD->write(S);//	hal.console->printf("%d ",S);
  hal.uartD->write(N);//	hal.console->printf("%d ",N);
  S=N;
  while(n<N){
    S+=cad[n];
    /*hal.console->printf("%d ",cad[n]);*/ hal.uartD->write(cad[n++]);	
    
  }
  hal.uartD->write(S);//	hal.console->printf("%d\n",S);
    
}
  
void sendPose(double r,double p, double y){
  unsigned char cad[7], N=254;
  
  r+=(r<0)?360:0;
  p+=(p<0)?360:0;
  y+=(y<0)?360:0;
  
  cad[0]=1;
  cad[1]=(unsigned char) 1+floor((N+1)*r/360);
  cad[2]=(unsigned char) 1+round(N*(N+1)*r/360-N*(cad[1]-1));
  cad[3]=(unsigned char) 1+floor((N+1)*p/360);
  cad[4]=(unsigned char) 1+round(N*(N+1)*p/360-N*(cad[3]-1));
  cad[5]=(unsigned char) 1+floor((N+1)*y/360);
  cad[6]=(unsigned char) 1+round(N*(N+1)*y/360-N*(cad[5]-1));
  hal.console->printf("(%.1f, %.1f, %.1f)->0, 7,  %d, %d, %d, %d, %d, %d, %d, S\n",r,p,y,cad[0],cad[1],cad[2],cad[3],cad[4],cad[5],cad[6]);
  writeRpi(cad,7);
}


void loop(void)
{
  
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
    uint32_t now = AP_HAL::micros();
    uint16_t radio_in;
    radio_in = hal.rcin->read(5);
//     float heading = 0;
// 
    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;
// 
    if (now - last_compass > 100 * 1000UL &&
        compass.read()) {
//         heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned());
//         read compass at 10Hz
        last_compass = now;
    }

    ahrs.update();
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
//       //////////////////////
//       uint8_t cadena[]={2,3,4},i,cadena2[200];
//       for(i=0;i<3;++i) hal.uartD->write(cadena[i]);
//       
//       i=0;
//       
//       while(hal.uartD->available()) //checks if there is data available on the UART
//       {
// 	cadena2[i] = hal.uartD-> read();
// 	i++;
//       }
//       
//       for(i=0;i<3;++i) hal.console->write(cadena2[i]);
// 	
// 
//       //////////////////////
        //Vector3f drift  = ahrs.get_gyro_drift();
        hal.console->printf(
                "r:%4.1f  p:%4.1f y:%4.1f, CH:%u \n",
                (double)ToDeg(ahrs.roll),
                (double)ToDeg(ahrs.pitch),
                (double)ToDeg(ahrs.yaw),radio_in);
	
	sendPose((double)ToDeg(ahrs.roll), (double)ToDeg(ahrs.pitch), (double)ToDeg(ahrs.yaw));
        last_print = now;
        counter = 0;
    }
}

GCS _gcs;

AP_HAL_MAIN();
