#ifndef softap_H_
#define softap_H_


void setupAccessPoint(void);

void SetupManagementRedirect(void);

void HandleWifiClient(void);
uint16_t AboveAverageBalance();
void CancelAverageBalance();
bool SetVoltCalib(uint8_t module, float newValue);
bool SetTempCalib(uint8_t module, float newValue);


extern int cell_array_index;
extern int cell_array_max;
extern cell_module cell_array[20];

extern bool runProvisioning;


#endif

