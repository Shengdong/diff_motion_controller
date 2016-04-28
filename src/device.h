#include "ControlCAN.h"

class pcan_device
{
public:
    explicit pcan_device(int id);

    bool scan_device(void);
 
    bool open_device(void);

    bool init_device(void);

    bool set_filter(void);

    bool start_device(void);

    void reset_device(void);
 
    void close_device(void);

private:
    int m_status;
    int m_id;

};
