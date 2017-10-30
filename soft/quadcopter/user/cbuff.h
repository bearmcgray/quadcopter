#ifndef __CBUFF_H
#define __CBUFF_H

#define CBUFF_SIZE 255

typedef struct{
	int pin;
	int pout;
	char data[CBUFF_SIZE];
} cbuff_data;

void cbuff_init(cbuff_data *cb);
int cbuff_put(cbuff_data *cb, char  b);
int cbuff_get(cbuff_data *cb, char *b);

#endif //__CBUFF_H
