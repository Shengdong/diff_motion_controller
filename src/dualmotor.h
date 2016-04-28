#include<sys/time.h>
#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ControlCAN.h"
#include "math.h"
#include <utility>

class dualmotor
{
public:
    explicit dualmotor(int id_1, int id_2);

    void Read_Callback_Data(void);

    std::pair <int,int> get_vel(void);

    void Send_Data(uint16_t Order_Code, uint8_t Sub_Code, uint16_t data1, uint16_t data2);

    void Read_Request(uint16_t Order_Code, uint8_t Sub_Code);

    void set_vel(int vel_1, int vel_2);

    void power_on(void);

    void slow_down(void);

    void shut_down(void);
private:
    int m_id_1;
    int m_id_2;
    std::pair <int, int> m_vel;
    VCI_CAN_OBJ CAN_SendData[2];
    VCI_CAN_OBJ VEL_DATA[2];
    VCI_CAN_OBJ VEL_REC[2];

    
};
