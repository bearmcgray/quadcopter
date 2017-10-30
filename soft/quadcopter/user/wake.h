#ifndef __WAKE_H
#define __WAKE_H

#define FEND 0xc0
#define FESC 0xdb
#define TFEND 0xdc
#define TFESC 0xdd
#define CRCINIT 0xde

enum{wait_fend,wait_ac,wait_N,wait_data,wait_crc};

typedef struct{
	int pos;
	int esc;
	unsigned char crc;
} wake_data;

void WAKE_init(wake_data* w);
int WAKE_bytein(wake_data* w, unsigned char data, unsigned char* inpac);
int WAKE_pack(unsigned char* buf_out, unsigned char addr, unsigned char cmd, unsigned char N, void *data);
void Wake_step_crc(unsigned char b, unsigned char *crc );

#endif //__WAKE_H
