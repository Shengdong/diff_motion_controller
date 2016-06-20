#include<sys/time.h>
#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ControlCAN.h"
#include "math.h"
#include <utility>

class singlemotor
{
public:
    explicit singlemotor(int id, int dev_n, int typeData);

    int get_vel(void);

    int get_pos(void);

    void set_vel(int vel);

    void set_pos(int pos);

    void reset_pos(void);

    void power_on(void);

    void slow_down(void);

    void shut_down(void);
private:
    int m_id;
    int dev_n;
    int m_typeData;
    int m_vel;
    int m_pos;
    int m_DataNum;
    int m_ReadDataNum;
    void Send_Data(uint16_t Order_Code, uint8_t Sub_Code, uint16_t data1, uint16_t data2);
    void Read_Callback_Data(void);
    VCI_CAN_OBJ	*CAN_SendData     = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
    VCI_CAN_OBJ	*pCAN_ReceiveData = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
};
