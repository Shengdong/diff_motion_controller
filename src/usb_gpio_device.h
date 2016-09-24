#include "ControlGPIO.h"
#include "ErrorType.h"

class gpio_io
{
  public:
    explicit gpio_io(int id);
    bool scan_io(void);
    bool open_io(void);
    void close_io(void);
    void set_input(int num);
    int read_data(int num);
    void set_opendrain(int num);
    void set_pin(int num);
    void reset_pin(int num);
    bool close(void);
    

  private:
    int m_id;
    int ret;
    int m_deviceID;
};
