#include "usb_gpio_device.h"
#include <iostream>
gpio_io::gpio_io(int id)
: m_id(id)
{
   m_deviceID = 1;
}

bool
gpio_io::scan_io(void)
{
    ret = VGI_ScanDevice(m_id);
    if (ret<=0)
    {
       printf("No GPIO device connected!\n");
       return false;
    }
    else
    {
       printf("%d GPIO device connected!\n", ret);
       return true;
    }
}

bool
gpio_io::open_io(void)
{
    ret = VGI_OpenDevice(VGI_USBGPIO,m_deviceID,0);
    if(ret !=ERR_SUCCESS)
    {
       printf("Error Open GPIO device!\n");
       return false;
    }
    else
    {
       printf("Opened GPIO device!\n");
       return true;
    }
}

void
gpio_io::set_input(int num)
{
    ret = VGI_SetInput(VGI_USBGPIO, m_deviceID, VGI_GPIO_PIN4 | VGI_GPIO_PIN5);
    if (ret != ERR_SUCCESS)
    {
        printf("Set pin input error!\n");
    }
}

void
gpio_io::set_opendrain(int num)
{
    ret = VGI_SetOpenDrain(VGI_USBGPIO, m_deviceID, 1<<num);
    if (ret != ERR_SUCCESS)
    {
        printf("Set pin open drain error!\n");
    }
}

void
gpio_io::set_pin(int num)
{
    ret = VGI_SetPins(VGI_USBGPIO, m_deviceID, 1<<num);
    if (ret != ERR_SUCCESS)
    {
        printf("Set pin high error!\n");
    }
}

void
gpio_io::reset_pin(int num)
{
    ret = VGI_ResetPins(VGI_USBGPIO, m_deviceID, 1<<num);
    if (ret != ERR_SUCCESS)
    {
        printf("Set pin low error!\n");
    }
}

int
gpio_io::read_data(int num)
{
    uint16_t pin_value = 0;

    ret = VGI_ReadDatas(VGI_USBGPIO, m_deviceID, 1<<num, &pin_value);
    if (ret != ERR_SUCCESS)
    {
        printf("Get pin data error!\n");
        return -1;
    }
    else
    {
        if ((pin_value & 1<<num) != 0)
           return 1;
        else
           return 0;
    }

}

bool
gpio_io::close(void)
{
    ret = VGI_CloseDevice(VGI_USBGPIO, m_deviceID);
    if (ret == ERR_SUCCESS)
    {
        printf("Close GPIO Device!\n");
        return true;
    }
    return false;
}
