#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>

typedef struct MPUReport {
	uint8_t		status;
	int16_t		accel_x;
	int16_t		accel_y;
	int16_t		accel_z;
	int16_t		temp;
	int16_t		gyro_x;
	int16_t		gyro_y;
	int16_t		gyro_z;
} MPUReport;

int main(int argc, char* argv[]) {

    int input_fd;
    MPUReport buffer;      /* Character buffer */

    /* Create input file descriptor */
    input_fd = open ("/dev/mpu_6000_device", O_RDONLY);

    if (input_fd == -1) {
            perror ("open");
            return 2;
    }

    /* Copy process */
    while(1) {
    	read (input_fd, &buffer, sizeof(MPUReport));

		printf("%d %d %d, %d, %d %d %d\n",
				buffer.accel_x,
				buffer.accel_y,
				buffer.accel_z,
				buffer.temp,
				buffer.gyro_x,
				buffer.gyro_y,
				buffer.gyro_z);
    }
}