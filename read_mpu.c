#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>
#include <time.h>

typedef struct MPUReport {
	struct timeval timestamp;
	unsigned long error_count;

	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;

	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;
	int16_t temperature_raw;
}MPUReport;

int main(int argc, char* argv[]) {

    int input_fd;
    MPUReport buffer;

    input_fd = open ("/dev/mpu6000", O_RDONLY);

    if (input_fd == -1) {
            perror ("open");
            return 2;
    }

    while(1) {
    	read (input_fd, &buffer, sizeof(MPUReport));

		printf("%10d %10d %10d | %10.2f | %10d %10d %10d | %ld.%ld\n",
				buffer.accel_x_raw,
				buffer.accel_y_raw,
				buffer.accel_z_raw,
				buffer.temperature_raw/340 + 36.53,
				buffer.gyro_x_raw,
				buffer.gyro_y_raw,
				buffer.gyro_z_raw,
				buffer.timestamp.tv_sec,buffer.timestamp.tv_usec);
    }
}
