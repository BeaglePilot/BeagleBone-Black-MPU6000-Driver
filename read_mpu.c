/****************************************************************************
 *
 *   Copyright (C) 2014 Beagle Pilot Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Beagle Pilot nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @author Jimmy Johnson <catch22@fastmail.net>
 * 
 * Certain parts ported from the PX4 MPU6000 code base
 * 
 * https://pixhawk.org
 * 
 */
 
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
