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

#define	CAN_MODE_LOOP_BACK		1
#define	CAN_DEBUG			0

VCI_CAN_OBJ	*pCAN_ReceiveData = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
VCI_CAN_OBJ	*CAN_SendData     = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
VCI_CAN_OBJ     VEL_DATA[2];
VCI_CAN_OBJ     VEL_REC[2];

int Vel;
int DataNum;
int ReadDataNum;
double timeuse;
double t1;
double timeuse1;
double t_start;
double t_test;
double timeuse2;
int v_r, v_l;


double tic() 
{
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

int sign(double k)
{
    if(k>=0)
        return 1;
    else
        return -1;
}

void Read_Callback_Data(void)
{
   t1 = tic();
   DataNum = 0;
   while(!DataNum)
   {
       DataNum = VCI_GetReceiveNum(VCI_USBCAN2, 0, 0);
       if((DataNum > 0)&&(VEL_REC != NULL))
       {
           ReadDataNum = VCI_Receive(VCI_USBCAN2, 0, 0, VEL_REC, DataNum);
           for(int i= 0; i<ReadDataNum; i++)
           {
               #if CAN_DEBUG
               printf("--CAN_ReceiveData.ID = 0x%X\n",VEL_REC[i].ID);
               printf("--CAN_ReceiveData.Data:");
               for(int j=0;j<VEL_REC[i].DataLen;j++)
               {
                   printf("%02X ",VEL_REC[i].Data[j]);
               }
               #endif
               printf("\n");
          }
       }
   }
   timeuse = tic() - t1;
   printf("Read Callback Data Use Time:%f\n",timeuse);
   printf("\n");
}

void Get_Vel(int Frame1, int Frame2)
{
   t1 = tic();   
   VEL_DATA[0].ID = Frame1;
   VEL_DATA[0].Data[0] = 0x40;
   VEL_DATA[0].Data[1] = 0x04;
   VEL_DATA[0].Data[2] = 0x3A;
   VEL_DATA[0].Data[3] = 0x01;
   VEL_DATA[0].Data[4] = 0x00;
   VEL_DATA[0].Data[5] = 0x00;
   VEL_DATA[0].Data[6] = 0x00;
   VEL_DATA[0].Data[7] = 0x00;

   VEL_DATA[1].ID = Frame2;
   VEL_DATA[1].Data[0] = 0x40;
   VEL_DATA[1].Data[1] = 0x04;
   VEL_DATA[1].Data[2] = 0x3A;
   VEL_DATA[1].Data[3] = 0x01;
   VEL_DATA[1].Data[4] = 0x00;
   VEL_DATA[1].Data[5] = 0x00;
   VEL_DATA[1].Data[6] = 0x00;
   VEL_DATA[1].Data[7] = 0x00;

   VCI_Transmit(VCI_USBCAN2,0,0,VEL_DATA,2);
   DataNum = 0;

   while(!DataNum)
   {
       DataNum = VCI_GetReceiveNum(VCI_USBCAN2, 0, 0);
       if((DataNum > 0)&&(VEL_REC != NULL))
       {
           ReadDataNum = VCI_Receive(VCI_USBCAN2, 0, 0, VEL_REC, DataNum); 
           for(int i=0; i< ReadDataNum; i++)
           {

               #if CAN_DEBUG
               printf("--CAN_ReceiveData.ID = 0x%X\n", VEL_REC[i].ID);
               printf("--CAN_ReceiveData.Data:");
               for(int j=0;j<VEL_REC[i].DataLen;j++)
               {
                   printf("%02X ",VEL_REC[i].Data[j]);
               }
               printf("\n");
               #endif

               if (VEL_REC[i].Data[7] == 0x00)
               {
                   Vel = VEL_REC[i].Data[4] + VEL_REC[i].Data[5]*256;
               } 
               else if(VEL_REC[i].Data[7] == 0xFF)
               {   
                   Vel = (VEL_REC[i].Data[4]) | (VEL_REC[i].Data[5] << 8);
                   Vel = Vel - 0xFFFF - 1;
               }
               if (VEL_REC[i].ID == 0x5BF)
               {
                   v_r = Vel;
               }
               else if (VEL_REC[i].ID == 0x5FF)
               {
                   v_l = Vel;
               }
           }
       } 
   }

   timeuse = tic()-t1;

   #if CAN_DEBUG
   printf("Get Vel Use Time:%f\n",timeuse);
   printf("\n");
   #endif

//   return Vel;
}

void Send_Data(uint16_t Order_Code, uint8_t Sub_Code, uint16_t data1, uint16_t data2, int Frame)
{
   t1 = tic(); 
   CAN_SendData->ID = Frame;
   CAN_SendData->Data[0] = 0x23;
   CAN_SendData->Data[1] = (Order_Code & 0x00FF);
   CAN_SendData->Data[2] = Order_Code >> 8;
   CAN_SendData->Data[3] = Sub_Code;
   CAN_SendData->Data[4] = (data1 & 0x00FF);
   CAN_SendData->Data[5] = data1 >> 8;
   CAN_SendData->Data[6] = (data2 & 0x00FF);
   CAN_SendData->Data[7] = data2 >> 8; 
   VCI_Transmit(VCI_USBCAN2,0,0,CAN_SendData,1);

   timeuse = tic() - t1;
   printf("Send Data Use Time:%f\n \n",timeuse);
}

void Read_Data(uint16_t Order_Code, uint8_t Sub_Code, uint16_t data1, uint16_t data2, int Frame)
{
   t1 = tic(); 
   CAN_SendData->ID = Frame;
   CAN_SendData->Data[0] = 0x40;
   CAN_SendData->Data[1] = (Order_Code & 0x00FF);
   CAN_SendData->Data[2] = Order_Code >> 8;
   CAN_SendData->Data[3] = Sub_Code;
   CAN_SendData->Data[4] = (data1 & 0x00FF);
   CAN_SendData->Data[5] = data1 >> 8;
   CAN_SendData->Data[6] = (data2 & 0x00FF);
   CAN_SendData->Data[7] = data2 >> 8; 
   VCI_Transmit(VCI_USBCAN2,0,0,CAN_SendData,1);
   timeuse = tic() - t1;
   printf("Send Data Use Time:%f\n \n",timeuse);
}


void Set_Vel(int data1, int data2, int Frame1, int Frame2)
{
   t1 = tic();

   VEL_DATA[0].ID = Frame1;
   VEL_DATA[0].Data[0] = 0x23;
   VEL_DATA[0].Data[1] = 0x00;
   VEL_DATA[0].Data[2] = 0x33;
   VEL_DATA[0].Data[3] = 0x00;

   if(data1 >= 0)
   {
       VEL_DATA[0].Data[4] = (data1 & 0x00FF);
       VEL_DATA[0].Data[5] = (data1 & 0xFF00) >> 8;
       VEL_DATA[0].Data[6] = 0x00;
       VEL_DATA[0].Data[7] = 0x00;
   }
   else
   {
       data1 = 0xFFFF + (data1 + 1);
       VEL_DATA[0].Data[4] = (data1 & 0x00FF);
       VEL_DATA[0].Data[5] = (data1 & 0xFF00) >> 8;
       VEL_DATA[0].Data[6] = 0xFF;
       VEL_DATA[0].Data[7] = 0xFF;       
   }

   VEL_DATA[1].ID = Frame2;
   VEL_DATA[1].Data[0] = 0x23;
   VEL_DATA[1].Data[1] = 0x00;
   VEL_DATA[1].Data[2] = 0x33;
   VEL_DATA[1].Data[3] = 0x00;

   if(data2 >= 0)
   {
       VEL_DATA[1].Data[4] = (data2 & 0x00FF);
       VEL_DATA[1].Data[5] = (data2 & 0xFF00) >> 8;
       VEL_DATA[1].Data[6] = 0x00;
       VEL_DATA[1].Data[7] = 0x00;
   }
   else
   {
       data2 = 0xFFFF + (data2 + 1);
       VEL_DATA[1].Data[4] = (data2 & 0x00FF);
       VEL_DATA[1].Data[5] = (data2 & 0xFF00) >> 8;
       VEL_DATA[1].Data[6] = 0xFF;
       VEL_DATA[1].Data[7] = 0xFF;       
   }


   VCI_Transmit(VCI_USBCAN2,0,0,VEL_DATA,2);
   timeuse = tic() - t1;
   printf("Set Vel Use Time:%f\n",timeuse);
}


int main(void)
{
    int DevNum,Status;  
    //Scan device
    DevNum = VCI_ScanDevice(1);
    if(DevNum > 0){
        printf("Have %d device connected!\n",DevNum);
    }else{
        printf("No device connected!\n");
        return 0;
    }
    //Open device
    Status = VCI_OpenDevice(VCI_USBCAN2,0,0);
    if(Status==STATUS_ERR){
        printf("Open device failed!\n");
        return 0;
    }else{	CAN_SendData->DataLen = 8;
	CAN_SendData->ExternFlag = 0;
	CAN_SendData->RemoteFlag = 0;
	CAN_SendData->ID = 0x63F;

        printf("Open device success!\n");
    }

    VCI_INIT_CONFIG_EX	CAN_InitEx;
    //Config device
    CAN_InitEx.CAN_ABOM = 0;
#if CAN_MODE_LOOP_BACK
    CAN_InitEx.CAN_Mode = 1;
#else
    CAN_InitEx.CAN_Mode = 0;

#endif
    //1Mbps
    CAN_InitEx.CAN_BRP = 36;//6;
    CAN_InitEx.CAN_BS1 = 6;//3;
    CAN_InitEx.CAN_BS2 = 1;//2;
    CAN_InitEx.CAN_SJW = 1;

    CAN_InitEx.CAN_NART = 0;
    CAN_InitEx.CAN_RFLM = 0;
    CAN_InitEx.CAN_TXFP = 1;
	CAN_InitEx.CAN_RELAY = 0;
    Status = VCI_InitCANEx(VCI_USBCAN2,0,0,&CAN_InitEx);
    if(Status==STATUS_ERR){
        printf("Init device failed!\n");
        return 0;
    }else{
        printf("Init device success!\n");
    }
    //Set filter
	VCI_FILTER_CONFIG CAN_FilterConfig;
    CAN_FilterConfig.FilterIndex = 0;
    CAN_FilterConfig.Enable = 1;		//Enable
    CAN_FilterConfig.ExtFrame = 0;
    CAN_FilterConfig.FilterMode = 0;
    CAN_FilterConfig.ID_IDE = 0;
    CAN_FilterConfig.ID_RTR = 0;
    CAN_FilterConfig.ID_Std_Ext = 0;
    CAN_FilterConfig.MASK_IDE = 0;
    CAN_FilterConfig.MASK_RTR = 0;
    CAN_FilterConfig.MASK_Std_Ext = 0;
    Status = VCI_SetFilter(VCI_USBCAN2,0,0,&CAN_FilterConfig);
    if(Status==STATUS_ERR){
        printf("Set filter failed!\n");
        return 0;
    }else{
        printf("Set filter success!\n");
    }

    //Start CAN
    Status = VCI_StartCAN(VCI_USBCAN2,0,0);
    if(Status==STATUS_ERR){
        printf("Start CAN failed!\n");
        return 0;
    }else{
        printf("Start CAN success!\n");
    }

    printf("\n \n");

    CAN_SendData->DataLen = 8;
    CAN_SendData->ExternFlag = 0;
    CAN_SendData->RemoteFlag = 0;

    VEL_DATA[0].DataLen = 8;
    VEL_DATA[1].DataLen = 8;
    VEL_DATA[0].ExternFlag = 0;
    VEL_DATA[1].ExternFlag = 0;
    VEL_DATA[0].RemoteFlag = 0;
    VEL_DATA[1].RemoteFlag = 0;
    

#if CAN_MODE_LOOP_BACK
        CAN_SendData->SendType = 2;
        VEL_DATA[0].SendType = 2;
        VEL_DATA[1].SendType = 2;

#else
	CAN_SendData->SendType = 0;
        VEL_DATA[0].SendType = 0;
        VEL_DATA[1].SendType = 0;

#endif//CAN_MODE_LOOP_BACK
/*--------------------------initialize RPi GPIO--------------------------*/
       
    int val1,val2,val3,val4,val5,val6,val1_old,val3_old,val4_old,val6_old,val2_old;
    val1 = 0;
    val2 = 0;
    val3 = 0;
    val4 = 0;
    val5 = 0;
    val6 = 0;
    val1_old = 0;
    val2_old = 0;
    val3_old = 0;
    val4_old = 0;
    val6_old = 0;
    double x, y, theta,beta,alpha;
    bool Forward, Backward;
    x = 0;
    y = 0.05;
    theta = 0;
    beta = 0;
    double p1;
    double p2;
    double p3;
    double p4;
    double p5;
    double gain;
    p1 = 0.6;
    p2 = 0.4;
    p3 = 0.6;
    p4 = 1.6;
    p5 = -0.3;
    int v;
    double dist;
/*--------------------------Initialize Motor---------------------------------*/

    printf("\n \n----------------------------Initialize------------------------\n \n");

    Send_Data(0x3000, 0x00, 0x0001, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x3000, 0x00, 0x0001, 0x0000,0x67F);
    Read_Callback_Data();


    Send_Data(0x3003, 0x00, 0x0003, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x3003, 0x00, 0x0003, 0x0000,0x67F);
    Read_Callback_Data();


    Send_Data(0x3962, 0x00, 0x1000, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x3962, 0x00, 0x1000, 0x0000,0x67F);
    Read_Callback_Data();


    Send_Data(0x39A0, 0x00, 0x0001, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x39A0, 0x00, 0x0001, 0x0000,0x67F);
    Read_Callback_Data();


    Send_Data(0x39A0, 0x18, 0x0004, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x39A0, 0x18, 0x0004, 0x0000,0x67F);
    Read_Callback_Data();


    Send_Data(0x39A0, 0x1A, 0x0004, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x39A0, 0x1A, 0x0004, 0x0000,0x67F);
    Read_Callback_Data();


    Send_Data(0x3310, 0x00, 0x000C, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x3310, 0x00, 0x000C, 0x0000,0x67F);
    Read_Callback_Data();

    Send_Data(0x3300, 0x00, 0x0000, 0x0000,0x63F);
    Read_Callback_Data();
    Send_Data(0x3300, 0x00, 0x0000, 0x0000,0x67F);
    Read_Callback_Data();

/*-------------------------Choose Mode---------------------------------*/

    while(1)
    {
        t_test = tic();

        val1 = 0;
        val2 = 0;
        val3 = 0;
        val4 = 0;
        val5 = 1;
        val6 = 0;
/*--------------------------Power On---------------------------------*/

        if(val1==1)
        {
            if(val1_old != 1)
            {
                printf("\n \n----------------------------Power On------------------------\n \n");

                Send_Data(0x3004, 0x00, 0x0001, 0x0000,0x63F);
                Read_Callback_Data();
                Send_Data(0x3004, 0x00, 0x0001, 0x0000,0x67F);
                Read_Callback_Data();
                sleep(0.1);
            }
        }
/*--------------------------Move Forward---------------------------------*/

        else if(val2==1)
        {
            if(val2_old != 1)
            {
                printf("\n \n----------------------------Move Forward------------------------\n \n");
           
                Set_Vel(1000, -1000, 0x63F, 0x67F);
                Read_Callback_Data();
                Forward = true;
                Backward = false;
            }
            else
            {
                if (Forward && y> 2)
                {
                   if (abs(v_r)>10 && abs(v_l)>10)
                   {
                       Set_Vel(0, 0, 0x63F, 0x67F);     
                       Read_Callback_Data();
                   }
                   else
                   {
                       Set_Vel(-500,500,0x63F,0x67F);
                       Read_Callback_Data();
                       Forward = false;
                       Backward = true;
                   }
                }

                if (Backward && y< 0)
                {
                   if (abs(v_r)>10 && abs(v_l)>10)
                   {
                       Set_Vel(0, 0, 0x63F, 0x67F);     
                       Read_Callback_Data();
                   }
                   else
                   {
                       Set_Vel(1000,-1000,0x63F,0x67F);
                       Read_Callback_Data();
                       Forward = true;
                       Backward = false;
                   }
                }
            }
            //Get_Vel(0x63F);
            //Get_Vel(0x67F);   
        }
/*--------------------------Move Backward---------------------------------*/

        else if(val3==1)
        {
            if(val3_old != 1)
            {
                printf("\n \n----------------------------Move BackWard------------------------\n \n");
              
                Set_Vel(-800,800, 0x63F, 0x67F);
                Read_Callback_Data();
                x = 0.0;
                theta = 0.00;
                Backward = true;
            }
            else
            {
                if(y < -4)
                {
                     Send_Data(0x3004, 0x00, 0, 0,0x63F);
                     Read_Callback_Data();
                     Send_Data(0x3004, 0x00, 0, 0,0x67F);
                     Read_Callback_Data();

                }
                else
                {
                    gain = p1*y+p2*theta;
                    gain = -gain*10000/3.1416;
                    if(abs(gain)>200)
                    {
                        gain = sign(gain) * 200;
                    }
                    //printf("gain is %f \n", floor(gain));
                    Set_Vel((-800+floor(gain)),(800+floor(gain)), 0x63F, 0x67F);
                    Read_Callback_Data();
                }
            }   
        }
/*--------------------------Spin Clockwise---------------------------------*/

        else if(val4==1)
        {
            if(val4_old != 1)
            {
                printf("\n \n----------------------------Spin Clockwise------------------------\n \n");            
                Set_Vel(500,-500,0x63F,0x67F);
                Read_Callback_Data();
                x = 0.1;
                theta = 0.00;
                Forward = true;
                Backward = false;
            }
            else
            {
                v = dist*p3*10000/3.14;
                if(v>500)
                {
                   v = 500;
                }
                gain = 0.3265*(p4*alpha + p5*beta);
                gain = gain * 10000/3.14;
                if (gain > 30)
                {
                   gain = 30;
                }
                Set_Vel(floor(v)+floor(gain),-floor(v)+floor(gain), 0x63F, 0x67F);
                printf("(v_r, v_l) is (%d, %d) \n", v_r, v_l);

                Read_Callback_Data();
            }
        }
/*--------------------------Power off---------------------------------*/

        else if(val5==1)
        {
            printf("\n \n----------------------------Shut Down------------------------\n \n");
            
            Send_Data(0x3004, 0x00, 0, 0,0x63F);
            Read_Callback_Data();
            Send_Data(0x3004, 0x00, 0, 0,0x67F);
            Read_Callback_Data();
            sleep(1);
            break;
        }
/*--------------------------Stop---------------------------------*/

        else if(val6==1)
        {
            if(val6_old != 1)
            {
                printf("\n \n----------------------------Stop------------------------\n \n");

                Set_Vel(0,0,0x63F,0x67F);
                Read_Callback_Data(); 
            }
        }
        val1_old = val1;
        val2_old = val2;
        val6_old = val6;
        val4_old = val4;
        val3_old = val3;

        Get_Vel(0x63F,0x67F);   

        timeuse2 = tic() - t_test;
//        printf("Full Loop Use Time:%f\n",timeuse2);

//        printf("(v_r, v_l) is (%d, %d) \n", v_r, v_l);
        if(Backward)
        {
            y = y - (-v_l+v_r)/2 * sin(theta) * 0.018849555 * timeuse2/60;
            x = x - (-v_l+v_r)/2 * cos(theta) * 0.018849555 * timeuse2/60;
        }
        else
        {
            y = y + (-v_l+v_r)/2 * sin(theta) * 0.018849555 * timeuse2/60;
            x = x + (-v_l+v_r)/2 * cos(theta) * 0.018849555 * timeuse2/60;
        }
        theta = theta + (v_r + v_l)/0.653 * 0.018849555 * timeuse2/60;

        beta = -atan2(-y, 3-x);
        alpha = -beta - theta;
        dist = sqrt((x-3)*(x-3)+y*y);
    }
    printf("(x, y, theta) : (%f, %f, %f) \n \n", x, y, theta);

    sleep(0.1);

    //Stop receive can data
    Status = VCI_ResetCAN(VCI_USBCAN2,0,0);
    printf("VCI_ResetCAN %d\n",Status);
    VCI_CloseDevice(VCI_USBCAN2,0);
    printf("VCI_CloseDevice\n");
    return 0;
}
