#ifndef webservicesubmit_H_
#define webservicesubmit_H_

#include "bms_values.h"
#include "settings.h"

class WebServiceSubmit {
  public:
    virtual void postData(eeprom_settings myConfig, cell_module (&cell_array)[20], int cell_array_max) = 0;        // Needs to be implemented by each subclass
};

class EmonCMS : public WebServiceSubmit {
  public:
    void postData(eeprom_settings myConfig, cell_module (&cell_array)[20], int cell_array_max);       // Needs to be implemented by each subclass
};


#endif

