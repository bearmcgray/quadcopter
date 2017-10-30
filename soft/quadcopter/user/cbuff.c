#include "cbuff.h"

int next(int p){
	int tmp = (p+1)%CBUFF_SIZE;
	return tmp;
}

void cbuff_init(cbuff_data *cb){
	cb->pin = 0;
	cb->pout = cb->pin;
}

int cbuff_put(cbuff_data *cb, char b){
	if( next(cb->pin)!=cb->pout ){
		cb->data[cb->pin] = b;
		cb->pin = next(cb->pin);
		return 1;
	}else{
		return 0;
	}
}

int cbuff_get(cbuff_data *cb, char *b){
	if( cb->pin!=cb->pout ){
		*b = cb->data[cb->pout];
		cb->pout = next(cb->pout);
		return 1;
	}else{
		return 0;
	}
}
