/**
 * Kondo B3M 0.1 Library
 *
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "kondo_driver/b3m.h"

/*-----------------------------------------------------------------------------
 * Convenience macros
 */
#define b3m_error(ki, err) { \
  snprintf(ki->error, 128, "ERROR: %s: %s\n", __func__, err); \
  return -1; }

/*-----------------------------------------------------------------------------
 * Iniitialize the B3M interface
 * Here we mainly setup the FTDI USB-to-serial communication
 * 115200 baud, 8 bits, even parity, 1 stop bit.
 * Returns: 0 if successful, < 0 otherwise
 */
int b3m_init(B3MData * r, const char* serial_port)
{
	printf("b3m_init\n");
	assert(r);
	r->debug = 1;

	struct termios tio;

	r->fd = open(serial_port, O_RDWR);
	if (ioctl(r->fd, TCGETS, &tio)){
		fprintf(stderr, "Error: Get serial port parameters\n");
	}
	tio.c_cflag &= ~CBAUD;
	tio.c_cflag |= B3M_BAUD;
	if (ioctl(r->fd, TCSETS, &tio)){
		fprintf(stderr, "Error: Set serial port parameters\n");
	}
	return 0;
}

/*-----------------------------------------------------------------------------
 * Close / Deinitialize the B3M Interface.
 * Mainly closes the USB device and deletes the allocated data.
 * Returns 0 if successful, < 0 otherwise
 */
int b3m_close(B3MData * r)
{
	printf("b3m_close\n");
	assert(r);

	close(r->fd);

	return 0;
}

/*-----------------------------------------------------------------------------
 * Write n bytes from the swap to the Kondo.
 * Returns >0 number of bytes written, < 0 if error
 */
int b3m_write(B3MData * r, int n)
{
	printf("b3m_write\n");
	assert(r);
	int i;
	if ((i = write(r->fd, r->swap, n)) < 0)
		fprintf(stderr, "Error: Send data\n");
	return i;
}

/*-----------------------------------------------------------------------------
 * Read n bytes from B3M. Reads immediately from the serial buffer.
 * See b3m_read_timeout for a version that blocks waiting for the data.
 * Returns < 0: error
 * Returns >= 0: number of bytes read
 */
int b3m_read(B3MData * r, int n)
{
	printf("b3m_read\n");
	assert(r);
	int i;
	if ((i = read(r->fd, r->swap, n)) < 0)
		fprintf(stderr, "Error: Read data\n");
	return i;
}

/*-----------------------------------------------------------------------------
 * Read n bytes from the B3M, waiting for at most usecs for the n bytes.
 * Performs this by continuously polling the serial buffer until either
 * all of the bytes are read or the timeout has been reached.
 * Returns < 0: error
 * Returns >= 0: number of bytes read
 */
int b3m_read_timeout(B3MData * r, int n, long usecs)
{
	printf("b3m_read_timeout\n");
	assert(r);
	static struct timeval tv, end;
	int i = 0, bytes_read = 0;
	gettimeofday(&tv, NULL);
	// determine end time
	end.tv_sec = tv.tv_sec + usecs / 1000000;
	end.tv_usec = tv.tv_usec + usecs % 1000000;
	if (end.tv_usec > 1000000) {
		end.tv_sec += 1;
		end.tv_usec -= 1000000;
	}
	// spam the read until data arrives
	do {
	    if ((i = read(r->fd, r->swap, n - bytes_read)) < 0) {
		fprintf(stderr, "Error: Read data\n");
	    }
	    bytes_read += i;
	    gettimeofday(&tv, NULL);
	} while (bytes_read < n && (tv.tv_sec <= end.tv_sec || tv.tv_usec < end.tv_usec));
	return bytes_read;
}

/*-----------------------------------------------------------------------------
 * Purge the serial buffers.
 * Returns 0 if successful, < 0 if error
 */
int b3m_purge(B3MData * r)
{
	printf("b3m_purge\n");
	assert(r);
	tcflush(r->fd, TCIOFLUSH);
	return 0;
}

/*-----------------------------------------------------------------------------
 * Transaction template: Purge, then send out_bytes, then receive in_bytes
 * Blocks for B3M_RX_TIMEOUT microseconds (default value)
 * Returns < 0: error
 * Returns >= 0: number of bytes read
 */
int b3m_trx(B3MData * r, UINT bytes_out, UINT bytes_in)
{
	return b3m_trx_timeout(r, bytes_out, bytes_in, B3M_RX_TIMEOUT);
}

/*-----------------------------------------------------------------------------
 * Transaction template: Purge, then send out_bytes, then receive in_bytes
 * On RX, blocks for at most timeout microseconds before giving up.
 * Returns < 0: error
 * Returns >= 0: number of bytes read
 */
int b3m_trx_timeout(B3MData * r, UINT bytes_out, UINT bytes_in, long timeout)
{
	assert(r);
	int i;
	int j;

	// debug printing
	if (r->debug) {
		printf("send %d bytes: ", bytes_out);
		for (j = 0; j < bytes_out; j++)
			printf("%x ", r->swap[j]);
		printf("\n");
	}

	if ((i = b3m_purge(r)) < 0)
		return i;
	if ((i = b3m_write(r, bytes_out)) < 0)
		return i;

	// clear swap
	for (i = 0; i < bytes_in; i++)
		r->swap[i] = 0;

	// read the return data
	i = b3m_read_timeout(r, bytes_in, timeout);

	// debug printing
	if (r->debug) {
		printf("recv %d bytes: ", i);
		for (j = 0; j < i; j++)
			printf("%x ", r->swap[j]);
		printf("\n");
	}

	return i;
}


/*!
 * @brief send command to servo motors
 *
 * @param[in] id servo id, 0-255 (255: broadcast)
 * @param[in] address servo address
 * @param[in] data servo data
 * @param[in] byte byte of data
 * @return error status
 */
int b3m_com_send(B3MData * r, UINT id, UINT address, UCHAR *data, int byte)
{
	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 7 + byte;				// length
	r->swap[n++] = B3M_CMD_WRITE;			// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	r->swap[n++] = id;						// id
	for(i = 0; i < byte; i ++){
		r->swap[n++] = data[i];
	}
	r->swap[n++] = address;
	r->swap[n++] = 0x01;					// number of ID
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 7 + byte, 5, B3M_POS_TIMEOUT)) < 0)
		return i;

	// return error status
	return r->swap[2];
}

/*!
 * @brief set servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] pos the position to set (angle * 100)
 * @return error status.
 */
int b3m_pos(B3MData * r, UINT id, UINT pos)
{
	printf("b3m_pos\n");
	assert(r);

	char data[2];
	data[0] = pos & 0xff;
	data[1] = pos >> 8;

	return b3m_com_send(r, id, B3M_SERVO_DESIRED_POSITION, data, 2);
}

/*!
 * @brief set servo mode
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] option B3M_OPTIONS_*
 * @return error status
 */
int b3m_servo_mode(B3MData * r, UINT id, UCHAR option)
{
	printf("b3m_servo_mode\n");
	assert(r);

	char data[2];
	data[0] = option;						// mode
	data[1] = 0x00;							// interpolation

	return b3m_com_send(r, id, B3M_SERVO_SERVO_MODE, data, 2);
}

/*-----------------------------------------------------------------------------
 * Get servo current
 * id: the servo id, 0-31
 * Returns: Value of current (>= 0) if successful, < 0 if error
 */
int b3m_get_current(B3MData * r, UINT id)
{
/*
	printf("b3m_get_current\n");
	assert(r);
	int i;

	// check valid id
	if (id > 31)
		b3m_error(r, "Invalid servo ID > 31.");

	// build command
	r->swap[0] = id | B3M_CMD_GET; // id and command
	r->swap[1] = B3M_SC_CURRENT; // subcommand

	// synchronize
	if ((i = b3m_trx_timeout(r, 2, 5, B3M_GET_TIMEOUT)) < 0)
		return i;

	// return stretch
	return r->swap[4];
*/
	return 0;
}

/*-----------------------------------------------------------------------------
 * Set stretch parameter of the servo
 * id: the id of the servo 0-31
 * stretch: the desired stretch 1(2)-127(254)
 * Returns: the stretch as reported by the servo (>= 0), or < 0 if error
 */
int b3m_set_stretch(B3MData * r, UINT id, UCHAR stretch)
{
/*
	printf("b3m_set_stretch\n");
	assert(r);
	int i;

	// check valid stuff
	if (id > 31)
		b3m_error(r, "Invalid servo ID > 31.");
	if (stretch < 1 || stretch > 127)
		b3m_error(r, "Invalid stretch, not [1-127]");

	// build command
	r->swap[0] = id | B3M_CMD_SET; // id and command
	r->swap[1] = B3M_SC_STRETCH; // subcommand
	r->swap[2] = stretch;

	// synchronize
	if ((i = b3m_trx_timeout(r, 3, 6, B3M_SET_TIMEOUT)) < 0)
		return i;

	return r->swap[5];
*/
	return 0;
}

/*-----------------------------------------------------------------------------
 * Set speed parameter of the servo
 * id: the id of the servo 0-31
 * stretch: the desired speed 1(2)-127(254)
 * Returns: the speed as reported by the servo (>= 0), or < 0 if error
 */
int b3m_set_speed(B3MData * r, UINT id, UCHAR speed)
{
/*
	printf("b3m_set_speed\n");
	assert(r);
	int i;

	// check valid stuff
	if (id > 31)
		b3m_error(r, "Invalid servo ID > 31.");
	if (speed < 1 || speed > 127)
		b3m_error(r, "Invalid speed, not [1-127]");

	// build command
	r->swap[0] = id | B3M_CMD_SET; // id and command
	r->swap[1] = B3M_SC_SPEED; // subcommand
	r->swap[2] = speed;

	// synchronize
	if ((i = b3m_trx_timeout(r, 3, 6, B3M_SET_TIMEOUT)) < 0)
		return i;

	return r->swap[5];
*/
	return 0;
}
/*-----------------------------------------------------------------------------
 * Set current limit parameter of the servo
 * id: the id of the servo 0-31
 * curlim: the desired current limit 1-63
 * Returns: the current limit as reported by the servo (>= 0), or < 0 if error
 */
int b3m_set_current_limit(B3MData * r, UINT id, UCHAR curlim)
{
	printf("b3m_set_current_limit\n");
	assert(r);
	int i;

	// check valid stuff
	if (id > 31)
		b3m_error(r, "Invalid servo ID > 31.");
	if (curlim < 1 || curlim > 63)
		b3m_error(r, "Invalid current, not [1-63]");

	// build command
	r->swap[0] = id | B3M_CMD_SET; // id and command
	r->swap[1] = B3M_SC_CURRENT; // subcommand
	r->swap[2] = curlim;

	// synchronize
	if ((i = b3m_trx_timeout(r, 3, 6, B3M_SET_TIMEOUT)) < 0)
		return i;

	return r->swap[5];
}

/*-----------------------------------------------------------------------------
 * Set temperature limit parameter of the servo
 * id: the id of the servo 0-31
 * templim: the desired current limit 1-127
 * Returns: the temperature limit as reported by the servo (>= 0), or < 0 if error
 */
int b3m_set_temperature_limit(B3MData * r, UINT id, UCHAR templim)
{
/*
	printf("b3m_set_temperature_limit\n");
	assert(r);
	int i;

	// check valid stuff
	if (id > 31)
		b3m_error(r, "Invalid servo ID > 31.");
	if (templim < 1 || templim > 127)
		b3m_error(r, "Invalid temperature, not [1-127]");

	// build command
	r->swap[0] = id | B3M_CMD_SET; // id and command
	r->swap[1] = B3M_SC_TEMPERATURE; // subcommand
	r->swap[2] = templim;

	// synchronize
	if ((i = b3m_trx_timeout(r, 3, 6, B3M_SET_TIMEOUT)) < 0)
		return i;

	return r->swap[5];
*/
	return 0;
}

/*-----------------------------------------------------------------------------
 * Get the ID of the connected servo
 * This command should be used with only one servo attached to the bus.
 * Returns: ID (0-31), or < 0 if error
 */
int b3m_get_id(B3MData * r)
{
	printf("b3m_get_id\n");
	assert(r);
	int i;

	// build command
	r->swap[0] = 0xFF; // command (0xFF for read)
	r->swap[1] = B3M_SC_READ; // subcommand (read)
	r->swap[2] = B3M_SC_READ; // subcommand (read)
	r->swap[3] = B3M_SC_READ; // subcommand (read)

	// synchronize
	if ((i = b3m_trx_timeout(r, 4, 5, B3M_ID_TIMEOUT)) < 0)
		return i;

	// return the ID
	return r->swap[4] & 0x1F;
}

/*-----------------------------------------------------------------------------
 * Set the ID of the connected servo
 * This command should be used with only one servo attached to the bus.
 * id: the desired ID of the servo 0-31
 * Returns: ID (0-31), or < 0 if error.
 */
int b3m_set_id(B3MData * r, UINT id)
{
	printf("b3m_set_id\n");
	assert(r);
	int i;

	// check id
	if (id > 31)
	    b3m_error(r, "Invalid servo ID, > 31.");

	// build command
	r->swap[0] = id | B3M_CMD_ID;
	r->swap[1] = B3M_SC_WRITE;
	r->swap[2] = B3M_SC_WRITE;
	r->swap[3] = B3M_SC_WRITE;
	if (r->debug) {
	    int i;
	    for (i=0; i<4; i++) {
		printf("%x ", r->swap[i]);
	    }
	    printf("\n");
	}
	// This seems bug. set_id must wait some time before read.
#if 0	
	// synchronize
	if ((i = b3m_trx_timeout(r, 4, 5, B3M_ID_TIMEOUT)) < 0)
		return i;
#endif
	if ((i=b3m_write (r, 4)) < 0) {
	    return -1;
	}
	sleep (3);
	if ((i=b3m_read(r, 5)) < 0) {
	    return -1;
	}	    

	// return the ID
	return r->swap[4] & 0x1F;
}
