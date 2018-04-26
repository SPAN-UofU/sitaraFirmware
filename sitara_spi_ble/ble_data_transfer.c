#include "ble_data_transfer.h"
uint32_t total_mag[8] = {0};
uint8_t final_msg[20] = {};
int i = 0;
int average_mag;
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))
uint8_t int ble_data_compress(uint8_t spimsg[])
{
    total_mag[i] = ((spimsg[0]<<16)&0xFF0000) + ((spimsg[1]<<8)&0x00FF00) + (spimsg[2]&0x0000FF);
    final_msg[i+4] = spimsg[2];
    final_msg[i+12] = spimsg[4];
  	average_mag +=  total_mag[i];
    i++;
    if(i == 8)
    {
   	i = 0;
    average_mag = average_mag/ARR_SIZE(total_mag);
  	for(int k=0; k<8; k++)
    {
        if(CHECK_BIT(average_mag+(total_mag[k]&0x000000FF),17))
        {
            average_mag |= 1<<(k+18);
            //NRF_LOG_INFO("check\r\n");
        }                   
    }
    //NRF_LOG_INFO("avg %x\r\n",average_mag);
    //NRF_LOG_FLUSH();
   	final_msg[0] = (average_mag >> 24) & 0xff;  // overflow 7 bits(lower 7), reserved
 	final_msg[1] = (average_mag >> 16) & 0xff; // average magnitude 1 bit(bit 0), overflow 7 bits
    final_msg[2] = (average_mag >> 8) & 0xff; // average magnitude
    final_msg[3] = (average_mag) & 0xff; // average magnitude
    //NRF_LOG_HEXDUMP_INFO(final_msg,20);
    //NRF_LOG_FLUSH();
    average_mag = 0;
    memset(total_mag,0,sizeof(total_mag));
    return final_msg;
	}
	return 0;
}