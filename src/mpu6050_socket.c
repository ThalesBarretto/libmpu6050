#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <assert.h>
#include <sys/ioctl.h>
#include "mpu6050_core.h"

int mpu_socket_connect(int *sfd, char *host, char *port)
{
	struct addrinfo		hints;		/* search criteria		*/
	struct addrinfo		*result;	/* list of struct addrin	*/
	struct addrinfo		*rp;		/* list iterator		*/
	int 			s;		/* return val. from getaddrinfo	*/

        /* Prepare hints: datagram, any protocol, any address */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family		= AF_INET;	/* Allow IPv4 or IPv6		*/
	hints.ai_socktype	= SOCK_DGRAM;	/* For datagram socket		*/ 
	hints.ai_protocol	= 0;		/* For any protocol		*/
	hints.ai_flags		= 0;		/* no special flags		*/
	hints.ai_canonname	= NULL;		/* on hints must be NULL	*/
	hints.ai_addr		= NULL;		/* on hints must be NULL	*/
	hints.ai_next		= NULL;		/* on hints must be NULL	*/

	/* Build an addrinfo list matching the desired configuration */
        s = getaddrinfo(host, port, &hints, &result);
        if (s != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
		exit(EXIT_FAILURE);
        }

        /* Have list. Try socket(), connect(), proceed on success. */
	for (rp = result; rp != NULL; rp = rp->ai_next) {
		*sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (*sfd == -1)
			continue;	/* socket() failed, try next */

		if (connect(*sfd, rp->ai_addr, rp->ai_addrlen) != -1)
			break;		/* connect() success, proceed */

	       close(*sfd);		/* connect() failed, try next */
        }

	freeaddrinfo(result);	/* discard the list */

	if (rp == NULL) {	/* failed the whole list  */
		fprintf(stderr, "Could not connect\n");
		exit(EXIT_FAILURE);
	}

	/* success = open and connected socket */
	return 0;
}

int mpu_socket_sendmsg(int *sfd, char *msg)
{
	assert(NULL != sfd);
	assert(NULL != msg);

	/* Have channel. Send separate datagrams, read responses. */ 
	ssize_t len = strlen(msg) + 1;	/* mind the terminator	*/

	if (len > MPU_SOCK_BUFFSIZ) {		/* buffer overflow	*/
		fprintf(stderr, "Ignoring long message\n");
		return -1;
	}

	if (write(*sfd, msg, len) != len) {	/* write failed */
		fprintf(stderr,"partial/failed write\n");
		return -1;
	}
	//snprintf(msg, MPU_SOCK_BUFFSIZ, "\n");

	return 0;
}

int mpu_socket_makemsg(struct mpu_dev *dev, char *msg, char *buf)
{
	assert(NULL != msg); 
	assert(NULL != buf);
		
	snprintf(buf, MPU_SOCK_BUFFSIZ,"%12llu ", dev->samples);
	strcat(msg, buf);
	snprintf(buf, MPU_SOCK_BUFFSIZ,"%3.0lf Hz ", dev->sr);
	strcat(msg, buf);
	if(dev->cfg->temp_fifo_en) {
		snprintf(buf, MPU_SOCK_BUFFSIZ, "T:%+.1lf°C ",*(dev->t));
		strcat(msg, buf);
	}
	if(dev->cfg->accel_fifo_en) {
		snprintf(buf, MPU_SOCK_BUFFSIZ,"Ax:%+lf Ay:%+lf Az:%+lf ", *(dev->Ax), *(dev->Ay), *(dev->Az));
		strcat(msg, buf);
	}
	if(dev->cfg->xg_fifo_en) {
		snprintf(buf, MPU_SOCK_BUFFSIZ,"Gx:%+lf ", *(dev->Gx));
		strcat(msg, buf);
	}
	if(dev->cfg->yg_fifo_en) {
		snprintf(buf, MPU_SOCK_BUFFSIZ,"Gy:%+lf ", *(dev->Gy));
		strcat(msg, buf);
	}
	if(dev->cfg->zg_fifo_en) {
		snprintf(buf, MPU_SOCK_BUFFSIZ,"Gz:%+lf ", *(dev->Gz));
		strcat(msg, buf);
	}
	return 0;
}
