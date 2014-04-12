#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "MotionSensor.h"

#define delay_ms(a) usleep(a*1000)

int main() {
	ms_open();
	do{
		ms_update();
		printf("yaw = %2.3f\tpitch = %2.3f\troll = %2.3f\ttemperature = %2.2f\tcompass = %u\n",
		 ypr[YAW], ypr[PITCH],
		 ypr[ROLL],temp,compass);
		delay_ms(5);
	}while(1);

	return 0;
}
