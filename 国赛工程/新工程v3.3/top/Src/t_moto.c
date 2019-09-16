#include "t_moto.h"
#include "m_imu.h"
#include "control.h"
#include "t_monitor.h"
#include "robodata.h"

int __sgn(float x){
	if(x > 1) return 1;
	else if (x < -1) return -1;
	else return 0;
}

/*-------------------------------Chassis--------------------------------------------*/

//X轴指向前;Y轴指向右

void RaisingSetSpeed(void){
	if(monitor_remote.status == monitor_regular){
		Raising_MOTO[0].set_speed = RoboData.raising_ctrl.raising_speed;
		Raising_MOTO[1].set_speed = RoboData.raising_ctrl.raising_speed;
	}
	else{
		int i = 0;
		for(i = 0; i < 2; i++){
			Raising_MOTO[i].set_speed = 0;
		}
	}
}
