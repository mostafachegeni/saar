#ifndef MPU_H
#define MPU_H

struct s_mympu {
	float ypr[3];
	float gyro[3];
};

extern struct s_mympu mympu;
extern short accel[3];

int mympu_open(unsigned int rate);
int mympu_update(void);

#endif

