#include "ControlCAN.h"

class pcan_device
{
public:
    explicit pcan_device(int id, int type);

    bool scan_device(void);
 
    bool open_device(void);

    bool init_device(void);

    bool set_filter(void);

    bool start_device(void);

    void reset_device(void);
 
    void close_device(void);

    int device_index(void);

private:
    int m_status;
    int m_id;
    int m_deviceIndex;
    int m_type;
};
