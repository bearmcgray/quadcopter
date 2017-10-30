#include "wake.h"

void WAKE_init(wake_data* w){
	w->pos = 0;
	w->esc = 0;
	w->crc = 0;
}

int WAKE_bytein(wake_data* w, unsigned char c, unsigned char* inpac) {
	if( c==FEND ){
		WAKE_init(w);
		w->crc = CRCINIT;
		inpac[w->pos++] = c;
		Wake_step_crc(c,&w->crc);
	}else{
		if( c==FESC ){
			w->esc = 1;
		}else{
			if( w->esc ){
				w->esc = 0;
				if( c==TFEND ){
					c = FEND;
				}else{
					if( c==TFESC ){
						c = FESC;
					}else{
						return 0;
					}
				}
			}
			if( w->pos == 1 ){
				if(c&0x80){
					inpac[w->pos++]=c&0x7f;//if high bit at 1 byte == 1 it is addr
					Wake_step_crc(c&0x7f,&w->crc);
				}else{
					inpac[w->pos++]=0;//addr = 0
					Wake_step_crc(0,&w->crc);
					inpac[w->pos++]=c&0x7f;//cmd = c
					Wake_step_crc(c&0x7f,&w->crc);
				}
			}else{
				if( w->pos == 2 ){
					inpac[w->pos++]=c&0x7f;//here if addr present
					Wake_step_crc(c&0x7f,&w->crc);
				}else{
					if(w->pos == 3){
						inpac[w->pos++]=c;
						Wake_step_crc(c,&w->crc);
					}else{
						if( (w->pos > 3)&&(w->pos< (inpac[3] + 4) ) ){
							inpac[w->pos++]=c;
							Wake_step_crc(c,&w->crc);
						}else{
							if( w->pos == (inpac[3] + 4) ){//crc
								inpac[w->pos++]=c;
								if( c == w->crc ){
									return inpac[3]+5;
								}else{
									WAKE_init(w);
								}
							}
						}
					}
				}
			}
		}
	}
	return 0;
}

int WAKE_pack(unsigned char* buf_out, unsigned char addr, unsigned char cmd,
		unsigned char N, void *data) {
	unsigned char temp = 0;
	unsigned char crc = CRCINIT;
	int pos_pac = 0;
	for (int i = 0; i < N + 5; ++i) {
		if ((i > 3) && (i < N + 4)) {
			temp = ((char*) data)[i - 4];
			Wake_step_crc(temp,&crc);
		} else {
			if (i == 0){
				temp = FEND;
				Wake_step_crc(temp,&crc);
			}
			if (i == 1) {
					temp = 0x80 | addr;
					//crc ^= 0x80;
				Wake_step_crc(temp&0x7f,&crc);
			}
			if (i == 2){
				temp = 0x7f & cmd;
				Wake_step_crc(temp&0x7f,&crc);
			}
			if (i == 3){
				temp = N;
				Wake_step_crc(temp,&crc);
			}
			if (i == N + 4)
				temp = crc;
		}

		//crc ^= temp;
		if ((temp == FEND) && (i)) {
			temp = FESC;
			buf_out[pos_pac++] = temp;/*UART_WritePoll(devid,&temp,1);*/
			temp = TFEND;
		}
		if (temp == FESC) {
			temp = FESC;
			buf_out[pos_pac++] = temp;/*UART_WritePoll(devid,&temp,1);*/
			temp = TFESC;
		}
		//send_byte(temp);
		//UART_WritePoll(devid,&temp,1);
		if( !((i==1)&&(addr==0)) )
			buf_out[pos_pac++] = temp;
	}
	return pos_pac;
}

void Wake_step_crc(unsigned char b, unsigned char *crc)
{
  for(unsigned int i = 0; i < 8; b = b >> 1, i++)
	if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
	 else *crc = (*crc >> 1) & ~0x80;
}
