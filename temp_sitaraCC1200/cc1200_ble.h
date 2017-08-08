#include <stdint.h>
#include <unistd.h>


/*void buttons_leds_init(bool * p_erase_bonds);
void ble_stack_init(void);
void gap_params_init(void);
extern void gatt_init(void);
void advertising_init(void);
void services_init(void);
void conn_params_init(void);
void advertising_start(bool erase_bonds);
void power_manage(void);*/
extern uint8_t rx_msg[];
extern int rx_fifo_bytes;
static uint8_t sample;