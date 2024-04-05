#include <stdio.h>

void p_controller(double *error, double *kp, double *output){
	*output = *kp * *error;
}

void adapter(double *kp){
	*kp += 0.1;
}

void main(void){
	double error = 1.0;
	double output;
	double kp = 0.5;
	adapter(&kp);
	p_controller(&error, &kp, &output); // in model of the kp, the type(double), its named as kp, and its initial value of 0.5
}



	
