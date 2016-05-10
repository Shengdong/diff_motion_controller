#include "device.h"
#include <iostream>

pcan_device::pcan_device(int id, int type)
 : m_id(id)
 , m_type(type)
{
}

bool 
pcan_device::scan_device(void)
{
    //Scan device
    int DevNum; 
    DevNum = VCI_ScanDevice(m_id);
    if(DevNum > 0)
    {
        m_deviceIndex = DevNum - 1;

        printf("Have %d device connected!\n",DevNum);
        return true;
    }
    else
    {
        printf("No device connected!\n");
        return false;
    }
}

bool 
pcan_device::open_device(void)
{
    m_status = VCI_OpenDevice(VCI_USBCAN2,m_deviceIndex,0);
    if(m_status == STATUS_ERR){
        printf("Open device failed!\n");
        return false;
    }else{	
        printf("Open device success!\n");
        return true;
    }
}

bool 
pcan_device::init_device(void)
{
    VCI_INIT_CONFIG_EX CAN_InitEx;
    //Config device
    CAN_InitEx.CAN_ABOM = 0;
//#if CAN_MODE_LOOP_BACK
    if (m_type)
        CAN_InitEx.CAN_Mode = 1;
    else
        CAN_InitEx.CAN_Mode = 0;
//#else
//    CAN_InitEx.CAN_Mode = 0;
//#endif
    //1Mbps
    CAN_InitEx.CAN_BRP = 36;//6;
    CAN_InitEx.CAN_BS1 = 6;//3;
    CAN_InitEx.CAN_BS2 = 1;//2;
    CAN_InitEx.CAN_SJW = 1;

    CAN_InitEx.CAN_NART = 0;
    CAN_InitEx.CAN_RFLM = 0;
    CAN_InitEx.CAN_TXFP = 1;
    CAN_InitEx.CAN_RELAY = 0;
    m_status = VCI_InitCANEx(VCI_USBCAN2,m_deviceIndex,0,&CAN_InitEx);
    if(m_status==STATUS_ERR){
        printf("Init device failed!\n");
        return false;
    }else{
        printf("Init device success!\n");
        return true;
    }
}

bool 
pcan_device::set_filter(void)
{
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
    m_status = VCI_SetFilter(VCI_USBCAN2,m_deviceIndex,0,&CAN_FilterConfig);
    if(m_status==STATUS_ERR){
        printf("Set filter failed!\n");
        return false;
    }else{
        printf("Set filter success!\n");
        return true;
    }
}

bool 
pcan_device::start_device(void)
{
    //Start CAN
    m_status = VCI_StartCAN(VCI_USBCAN2,m_deviceIndex,0);
    if(m_status==STATUS_ERR){
        printf("Start CAN failed!\n");
        return false;
    }else{
        printf("Start CAN success!\n");
        return true;
    }
}

void
pcan_device::reset_device(void)
{
    m_status = VCI_ResetCAN(VCI_USBCAN2,m_deviceIndex,0);
    printf("VCI_ResetCAN %d\n",m_status);
}

void
pcan_device::close_device(void)
{
    //Stop receive can data
    VCI_CloseDevice(VCI_USBCAN2,m_deviceIndex);
    printf("VCI_CloseDevice\n");    
}
