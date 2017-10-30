#ifndef __ANALOD_DRV_H
#define __ANALOD_DRV_H

#define MAX_STEPS 25

typedef struct{
	float ary[MAX_STEPS];
	int sptr;
	int steps;
	float midval;
} tmiddler;

void setzero(void* dat, int sz);

float middle(tmiddler *dat, float par);
void middle_init(tmiddler *dat, int p);

void Analog_Init();
int Analog_GetCurrent();


#endif //__ANALOD_DRV_H
