  /*
  ******************************************************************************
  * @Revision : ver 1.0
  * @Date     : 2014/12/29 9:27
  ******************************************************************************
  */
#include<sys/time.h>
#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ControlCAN.h"
#include "math.h"
#include "dualmotor.h"
#include "device.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define	CAN_MODE_LOOP_BACK		1
#define	CAN_DEBUG			0


int main(void)
{

    boost::shared_ptr<pcan_device> pcan;
    pcan = boost::make_shared<pcan_device>(1);
    pcan->scan_device();
    pcan->open_device();
    pcan->init_device();
    pcan->set_filter();
    pcan->start_device();
   

    std::pair <int,int> vel;
    boost::shared_ptr<dualmotor> dual_motor;
    dual_motor = boost::make_shared<dualmotor>(0x63F, 0x67F);

    dual_motor->power_on();

    dual_motor->set_vel(1000,1000);
    dual_motor->Read_Callback_Data();
    sleep(5);
    vel = dual_motor->get_vel();
    printf("\n motor speed is (%d, %d) \n \n" , vel.first, vel.second);

    dual_motor->set_vel(100,100);
    dual_motor->Read_Callback_Data();
    sleep(2);
    vel = dual_motor->get_vel();
    printf("\n motor speed is (%d, %d) \n \n" , vel.first, vel.second);

    dual_motor->set_vel(-500,-500);
    dual_motor->Read_Callback_Data();
    sleep(2);
    vel = dual_motor->get_vel();
    printf("\n motor speed is (%d, %d) \n \n" , vel.first, vel.second);

    dual_motor->slow_down();
    vel = dual_motor->get_vel();
    printf("\n motor speed is (%d, %d) \n \n" , vel.first, vel.second);

    dual_motor->shut_down();
    sleep(1);
    printf("\n \n");

    pcan->reset_device();
    pcan->close_device();
    return 0;
}
