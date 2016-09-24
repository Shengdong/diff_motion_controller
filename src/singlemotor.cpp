#include "singlemotor.h"

singlemotor::singlemotor(int id, int dev_n, int typeData)
 : m_id(id)
 , dev_n(dev_n)
 , m_typeData(typeData)
{
    CAN_SendData->DataLen = 8;
    CAN_SendData->ExternFlag = 0;
    CAN_SendData->RemoteFlag = 0;
    CAN_SendData->ID = m_id;

    pCAN_ReceiveData->DataLen = 8;
    pCAN_ReceiveData->ExternFlag = 0;
    pCAN_ReceiveData->RemoteFlag = 0;

    if (m_typeData)
    {
        CAN_SendData->SendType = 2;
    }
    else
    {
        CAN_SendData->SendType = 0;
    }

//  Clear error and quick Start
    Send_Data(0x3000, 0x00, 0x0001, 0x0000);
    Read_Callback_Data();
    sleep(0.5);

//  Set motor mode to position mode
    Send_Data(0x3003, 0x00, 0x0007, 0x0000);
    Read_Callback_Data();

//  Set encoder resolution to 4096
    Send_Data(0x3962, 0x00, 0x1000, 0x0000);
    Read_Callback_Data();

//  Brake management
    Send_Data(0x39A0, 0x00, 0x0001, 0x0000);
    Read_Callback_Data();

    Send_Data(0x39A0, 0x18, 0x0004, 0x0000);
    Read_Callback_Data();

    Send_Data(0x39A0, 0x1A, 0x0004, 0x0000);
    Read_Callback_Data();

//  Set Vel PID P gain
    Send_Data(0x3310, 0x00, 0x000C, 0x0000);
    Read_Callback_Data();

//  Set Positive Max Vel Limit to 4096rpm
    Send_Data(0x3321, 0x00, 0x1000, 0x0000);
    Read_Callback_Data();

//  Set Negative Max Vel Limit to 4096rpm
    Send_Data(0x3323, 0x00, 0x1000, 0x0000);
    Read_Callback_Data();

//  Set position mode Vel to be 0
    Send_Data(0x3300, 0x00, 0x0000, 0x0000);
    Read_Callback_Data();
    sleep(0.5);
}

void 
singlemotor::Read_Callback_Data(void)
{
   m_DataNum = 0;
   while(!m_DataNum)
   {
       m_DataNum = VCI_GetReceiveNum(VCI_USBCAN2,dev_n, 0);
       if((m_DataNum > 0)&&(pCAN_ReceiveData != NULL))
       {
           m_ReadDataNum = VCI_Receive(VCI_USBCAN2,dev_n, 0, pCAN_ReceiveData, m_DataNum);
/*
           for(int i= 0; i<m_ReadDataNum; i++)
           {
               printf("--CAN_ReceiveData.ID = 0x%X\n",pCAN_ReceiveData->ID);
               printf("--CAN_ReceiveData.Data:");
               for(int j=0;j<pCAN_ReceiveData->DataLen;j++)
               {
                   printf("%02X ",pCAN_ReceiveData->Data[j]);
               }
               printf("\n");
           }
*/
       }
   }
}
    
void 
singlemotor::Send_Data(uint16_t Order_Code, uint8_t Sub_Code, uint16_t data1, uint16_t data2)
{
   CAN_SendData->Data[0] = 0x23;
   CAN_SendData->Data[1] = (Order_Code & 0x00FF);
   CAN_SendData->Data[2] = Order_Code >> 8;
   CAN_SendData->Data[3] = Sub_Code;
   CAN_SendData->Data[4] = (data1 & 0x00FF);
   CAN_SendData->Data[5] = data1 >> 8;
   CAN_SendData->Data[6] = (data2 & 0x00FF);
   CAN_SendData->Data[7] = data2 >> 8; 

   VCI_Transmit(VCI_USBCAN2,dev_n,0,CAN_SendData,1);
}

void 
singlemotor::set_vel(int vel)
{
   CAN_SendData->Data[0] = 0x23;
   CAN_SendData->Data[1] = 0x00;
   CAN_SendData->Data[2] = 0x33;
   CAN_SendData->Data[3] = 0x00;

   if(vel >= 0)
   {
       CAN_SendData->Data[4] = (vel & 0x00FF);
       CAN_SendData->Data[5] = (vel & 0xFF00) >> 8;
       CAN_SendData->Data[6] = 0x00;
       CAN_SendData->Data[7] = 0x00;
   }
   else
   {
       vel = 0xFFFF + (vel + 1);
       CAN_SendData->Data[4] = (vel & 0x00FF);
       CAN_SendData->Data[5] = (vel & 0xFF00) >> 8;
       CAN_SendData->Data[6] = 0xFF;
       CAN_SendData->Data[7] = 0xFF;       
   }

   VCI_Transmit(VCI_USBCAN2,dev_n,0,CAN_SendData,1);
   Read_Callback_Data();
}


void 
singlemotor::set_pos(int pos)
{
   CAN_SendData->Data[0] = 0x23;
   CAN_SendData->Data[1] = 0x90;
   CAN_SendData->Data[2] = 0x37;
   CAN_SendData->Data[3] = 0x00;

   if(pos >= 0)
   {
       CAN_SendData->Data[4] = (pos & 0x00FF);
       CAN_SendData->Data[5] = (pos & 0xFF00) >> 8;
       CAN_SendData->Data[6] = (pos & 0xFF0000) >> 16;
       CAN_SendData->Data[7] = (pos & 0xFF000000) >> 24;
   }
   else
   {
       pos = 0xFFFFFFFF + (pos + 1);
       CAN_SendData->Data[4] = (pos & 0x00FF);
       CAN_SendData->Data[5] = (pos & 0xFF00) >> 8;
       CAN_SendData->Data[6] = (pos & 0xFF0000) >> 16;
       CAN_SendData->Data[7] = (pos & 0xFF000000) >> 24;      
   }

   VCI_Transmit(VCI_USBCAN2,dev_n,0,CAN_SendData,1);
   Read_Callback_Data();
}

int
singlemotor::get_vel(void)
{
   CAN_SendData->Data[0] = 0x40;
   CAN_SendData->Data[1] = 0x04;
   CAN_SendData->Data[2] = 0x3A;
   CAN_SendData->Data[3] = 0x01;
   CAN_SendData->Data[4] = 0x00;
   CAN_SendData->Data[5] = 0x00;
   CAN_SendData->Data[6] = 0x00;
   CAN_SendData->Data[7] = 0x00;

   VCI_Transmit(VCI_USBCAN2,dev_n,0,CAN_SendData,1);
   m_DataNum = 0;

   while(!m_DataNum)
   {
       m_DataNum = VCI_GetReceiveNum(VCI_USBCAN2,dev_n, 0);
       if((m_DataNum > 0)&&(pCAN_ReceiveData != NULL))
       {
           m_ReadDataNum = VCI_Receive(VCI_USBCAN2,dev_n, 0, pCAN_ReceiveData, m_DataNum); 
           for(int i=0; i< m_ReadDataNum; i++)
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

               if (pCAN_ReceiveData->Data[7] == 0x00)
               {
                   Vel = pCAN_ReceiveData->Data[4] + pCAN_ReceiveData->Data[5]*256;
               } 
               else if(pCAN_ReceiveData->Data[7] == 0xFF)
               {   
                   Vel = (pCAN_ReceiveData->Data[4]) | (pCAN_ReceiveData->Data[5] << 8);
                   Vel = Vel - 0xFFFF - 1;
               }
               else
               {
                   Vel =0;
               }
               m_vel = Vel;
           }
       } 
   }
   return m_vel;
}

int
singlemotor::get_pos(void)
{
   CAN_SendData->Data[0] = 0x40;
   CAN_SendData->Data[1] = 0x62;
   CAN_SendData->Data[2] = 0x37;
   CAN_SendData->Data[3] = 0x00;
   CAN_SendData->Data[4] = 0x00;
   CAN_SendData->Data[5] = 0x00;
   CAN_SendData->Data[6] = 0x00;
   CAN_SendData->Data[7] = 0x00;

   VCI_Transmit(VCI_USBCAN2,dev_n,0,CAN_SendData,1);
   m_DataNum = 0;

   while(!m_DataNum)
   {
       m_DataNum = VCI_GetReceiveNum(VCI_USBCAN2,dev_n, 0);
       if((m_DataNum > 0)&&(pCAN_ReceiveData != NULL))
       {
           m_ReadDataNum = VCI_Receive(VCI_USBCAN2,dev_n, 0, pCAN_ReceiveData, m_DataNum); 
           for(int i=0; i< m_ReadDataNum; i++)
           {
/*
               printf("--CAN_ReceiveData.ID = 0x%X\n", pCAN_ReceiveData->ID);
               printf("--CAN_ReceiveData.Data:");
               for(int j=0;j<pCAN_ReceiveData->DataLen;j++)
               {
                   printf("%02X ",pCAN_ReceiveData->Data[j]);
               }
               printf("\n");
*/
               int Pos;

               if ( (pCAN_ReceiveData->Data[7]&&0x01) == 0x00)
               {
                   Pos = pCAN_ReceiveData->Data[4] + pCAN_ReceiveData->Data[5] * 256 + pCAN_ReceiveData->Data[6] * 65536 + pCAN_ReceiveData->Data[7] * 16777216;
               }  
               else if( (pCAN_ReceiveData->Data[7]&&0x01) == 0x01)
               {   
                   Pos = (pCAN_ReceiveData->Data[4]) | (pCAN_ReceiveData->Data[5] << 8) | (pCAN_ReceiveData->Data[6] << 16) | (pCAN_ReceiveData->Data[7] << 24);
                   Pos = Pos - 0xFFFFFFFF - 1;
               }
               else
               {
                   Pos =0;
               }
               m_pos = Pos;
           }
       } 
   }
   return m_pos;
}



void singlemotor::power_on(void)
{
    Send_Data(0x3004, 0x00, 0x0001, 0x0000);
    Read_Callback_Data();
    sleep(1);
}

void singlemotor::slow_down(void)
{
    set_vel(0);
    sleep(3); 
}

void 
singlemotor::shut_down(void)
{
    Send_Data(0x3004, 0x00, 0, 0);
    Read_Callback_Data();
    sleep(1);
}

void
singlemotor::reset_pos(void)
{
    Send_Data(0x3762, 0x00, 0, 0);
    Read_Callback_Data();
    sleep(0.1);
}

