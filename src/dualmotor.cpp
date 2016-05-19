#include "dualmotor.h"

dualmotor::dualmotor(int id_1, int id_2, int typeData)
 : m_id_1(id_1)
 , m_id_2(id_2)
 , m_typeData(typeData)
{
    CAN_SendData[0].DataLen = 8;
    CAN_SendData[0].ExternFlag = 0;
    CAN_SendData[0].RemoteFlag = 0;

    CAN_SendData[1].DataLen = 8;
    CAN_SendData[1].ExternFlag = 0;
    CAN_SendData[1].RemoteFlag = 0;

    VEL_DATA[0].DataLen = 8;
    VEL_DATA[1].DataLen = 8;
    VEL_DATA[0].ExternFlag = 0;
    VEL_DATA[1].ExternFlag = 0;
    VEL_DATA[0].RemoteFlag = 0;
    VEL_DATA[1].RemoteFlag = 0;
    if (m_typeData)
    {
        CAN_SendData[0].SendType = 2;
        CAN_SendData[1].SendType = 2;
        VEL_DATA[0].SendType = 2;
        VEL_DATA[1].SendType = 2;
    }
    else
    {
        CAN_SendData[0].SendType = 0;
        CAN_SendData[1].SendType = 0;
        VEL_DATA[0].SendType = 0;
        VEL_DATA[1].SendType = 0;
    }

    sleep(0.5);
    Send_Data(0x3000, 0x00, 0x0001, 0x0000);
    Read_Callback_Data();

    Send_Data(0x3003, 0x00, 0x0003, 0x0000);
    Read_Callback_Data();

    Send_Data(0x3962, 0x00, 0x1000, 0x0000);
    Read_Callback_Data();

    Send_Data(0x39A0, 0x00, 0x0001, 0x0000);
    Read_Callback_Data();

    Send_Data(0x39A0, 0x18, 0x0004, 0x0000);
    Read_Callback_Data();

    Send_Data(0x39A0, 0x1A, 0x0004, 0x0000);
    Read_Callback_Data();

    Send_Data(0x3310, 0x00, 0x000C, 0x0000);
    Read_Callback_Data();

    Send_Data(0x3300, 0x00, 0x0000, 0x0000);
    Read_Callback_Data();
    sleep(0.5);
}

void 
dualmotor::Read_Callback_Data(void)
{
   int DataNum = 0;
   while(!DataNum)
   {
       DataNum = VCI_GetReceiveNum(VCI_USBCAN2, 0, 0);
       if((DataNum > 0)&&(VEL_REC != NULL))
       {
           VCI_Receive(VCI_USBCAN2, 0, 0, VEL_REC, DataNum);
           //int ReadDataNum = VCI_Receive(VCI_USBCAN2, 0, 0, VEL_REC, DataNum);
/*
           for(int i= 0; i<ReadDataNum; i++)
           {
               printf("--CAN_ReceiveData.ID = 0x%X\n",VEL_REC[i].ID);
               printf("--CAN_ReceiveData.Data:");
               for(int j=0;j<VEL_REC[i].DataLen;j++)
               {
                   printf("%02X ",VEL_REC[i].Data[j]);
               }
               printf("\n");
          }
*/
       }
   }
}
    
void 
dualmotor::Send_Data(uint16_t Order_Code, uint8_t Sub_Code, uint16_t data1, uint16_t data2)
{
   CAN_SendData[0].ID = m_id_1;
   CAN_SendData[0].Data[0] = 0x23;
   CAN_SendData[0].Data[1] = (Order_Code & 0x00FF);
   CAN_SendData[0].Data[2] = Order_Code >> 8;
   CAN_SendData[0].Data[3] = Sub_Code;
   CAN_SendData[0].Data[4] = (data1 & 0x00FF);
   CAN_SendData[0].Data[5] = data1 >> 8;
   CAN_SendData[0].Data[6] = (data2 & 0x00FF);
   CAN_SendData[0].Data[7] = data2 >> 8; 

   CAN_SendData[1].ID = m_id_2;
   CAN_SendData[1].Data[0] = 0x23;
   CAN_SendData[1].Data[1] = (Order_Code & 0x00FF);
   CAN_SendData[1].Data[2] = Order_Code >> 8;
   CAN_SendData[1].Data[3] = Sub_Code;
   CAN_SendData[1].Data[4] = (data1 & 0x00FF);
   CAN_SendData[1].Data[5] = data1 >> 8;
   CAN_SendData[1].Data[6] = (data2 & 0x00FF);
   CAN_SendData[1].Data[7] = data2 >> 8; 

   VCI_Transmit(VCI_USBCAN2,0,0,CAN_SendData,2);
}

void 
dualmotor::set_vel(int vel_1, int vel_2)
{
   VEL_DATA[0].ID = m_id_1;
   VEL_DATA[0].Data[0] = 0x23;
   VEL_DATA[0].Data[1] = 0x00;
   VEL_DATA[0].Data[2] = 0x33;
   VEL_DATA[0].Data[3] = 0x00;

   if(vel_1 >= 0)
   {
       VEL_DATA[0].Data[4] = (vel_1 & 0x00FF);
       VEL_DATA[0].Data[5] = (vel_1 & 0xFF00) >> 8;
       VEL_DATA[0].Data[6] = 0x00;
       VEL_DATA[0].Data[7] = 0x00;
   }
   else
   {
       vel_1 = 0xFFFF + (vel_1 + 1);
       VEL_DATA[0].Data[4] = (vel_1 & 0x00FF);
       VEL_DATA[0].Data[5] = (vel_1 & 0xFF00) >> 8;
       VEL_DATA[0].Data[6] = 0xFF;
       VEL_DATA[0].Data[7] = 0xFF;       
   }

   VEL_DATA[1].ID = m_id_2;
   VEL_DATA[1].Data[0] = 0x23;
   VEL_DATA[1].Data[1] = 0x00;
   VEL_DATA[1].Data[2] = 0x33;
   VEL_DATA[1].Data[3] = 0x00;

   if(vel_2 >= 0)
   {
       VEL_DATA[1].Data[4] = (vel_2 & 0x00FF);
       VEL_DATA[1].Data[5] = (vel_2 & 0xFF00) >> 8;
       VEL_DATA[1].Data[6] = 0x00;
       VEL_DATA[1].Data[7] = 0x00;
   }
   else
   {
       vel_2 = 0xFFFF + (vel_2 + 1);
       VEL_DATA[1].Data[4] = (vel_2 & 0x00FF);
       VEL_DATA[1].Data[5] = (vel_2 & 0xFF00) >> 8;
       VEL_DATA[1].Data[6] = 0xFF;
       VEL_DATA[1].Data[7] = 0xFF;       
   }

   VCI_Transmit(VCI_USBCAN2,0,0,VEL_DATA,2);

}

std::pair <int,int> 
dualmotor::get_vel(void)
{
   VEL_DATA[0].ID = m_id_1;
   VEL_DATA[0].Data[0] = 0x40;
   VEL_DATA[0].Data[1] = 0x04;
   VEL_DATA[0].Data[2] = 0x3A;
   VEL_DATA[0].Data[3] = 0x01;
   VEL_DATA[0].Data[4] = 0x00;
   VEL_DATA[0].Data[5] = 0x00;
   VEL_DATA[0].Data[6] = 0x00;
   VEL_DATA[0].Data[7] = 0x00;

   VEL_DATA[1].ID = m_id_2;
   VEL_DATA[1].Data[0] = 0x40;
   VEL_DATA[1].Data[1] = 0x04;
   VEL_DATA[1].Data[2] = 0x3A;
   VEL_DATA[1].Data[3] = 0x01;
   VEL_DATA[1].Data[4] = 0x00;
   VEL_DATA[1].Data[5] = 0x00;
   VEL_DATA[1].Data[6] = 0x00;
   VEL_DATA[1].Data[7] = 0x00;

   VCI_Transmit(VCI_USBCAN2,0,0,VEL_DATA,2);
   int DataNum = 0;

   while(!DataNum)
   {
       DataNum = VCI_GetReceiveNum(VCI_USBCAN2, 0, 0);
       if((DataNum > 0)&&(VEL_REC != NULL))
       {
           int ReadDataNum = VCI_Receive(VCI_USBCAN2, 0, 0, VEL_REC, DataNum); 
           for(int i=0; i< ReadDataNum; i++)
           {
/*
               printf("--CAN_ReceiveData.ID = 0x%X\n", VEL_REC[i].ID);
               printf("--CAN_ReceiveData.Data:");
               for(int j=0;j<VEL_REC[i].DataLen;j++)
               {
                   printf("%02X ",VEL_REC[i].Data[j]);
               }
               printf("\n");
*/
               int Vel;

               if (VEL_REC[i].Data[7] == 0x00)
               {
                   Vel = VEL_REC[i].Data[4] + VEL_REC[i].Data[5]*256;
               } 
               else if(VEL_REC[i].Data[7] == 0xFF)
               {   
                   Vel = (VEL_REC[i].Data[4]) | (VEL_REC[i].Data[5] << 8);
                   Vel = Vel - 0xFFFF - 1;
               }
               else
               {
                   Vel =0;
               }
               if (VEL_REC[i].ID == 0x5BF)
               {
                    m_vel.first = Vel;
               }
               else if (VEL_REC[i].ID == 0x5FF)
               {
                    m_vel.second = Vel;
               }
           }
       } 
   }
   return m_vel;
}

void dualmotor::power_on(void)
{
    Send_Data(0x3004, 0x00, 0x0001, 0x0000);
    Read_Callback_Data();
    sleep(1);
}

void dualmotor::slow_down(void)
{
    set_vel(0,0);
    Read_Callback_Data();
    sleep(3); 
}

void dualmotor::shut_down(void)
{
    Send_Data(0x3004, 0x00, 0, 0);
    Read_Callback_Data();
    sleep(1);
}
