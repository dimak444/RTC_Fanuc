/* This program is client for data exchange over Socket Messaging 
 * for run-time robot controll.
 * Robot sends messages with max size = 126 byte
 *
 * Author: Makarov Dmitry, BMSTU, RK10 
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SERV_TCP_PORT 4000
#define SERV_HOST_ADDR "127.0.0.1"
#define MAXLINE 126

/* Codes of commands */
#define MOV "40 "
#define EOS "99 "

/* Uncomment for output debug info in stdout */
//#define DEBUG
#define DEBUG_OUT

/* Position in joint coordinates */
typedef struct joint_pos {
	float j[10]; /* joints from 1 to 9 index */
} jpos;

/* Position in XYZWPR coordinates */
typedef struct linear_pos {
	float x;
	float y;
	float z;
	float w;
	float p;
	float r;
} lpos;

/* Configuration of manipulator */
typedef struct robot_conf {
	unsigned short axis_cnt;
	jpos upper_lims;
	jpos lower_lims;
} robconf;

int sendall(int socket, char *buf, int buf_len);
int recvline(int socket, char *ptr, int maxlen);
int lpos_to_array(lpos l_coord, float* coords);

int start_server(robconf* r_cfg);
int stop_server(int sockfd);
int getRobotInfo(int sockfd, robconf* rcf);
int waitForRobotReady(int sockfd, unsigned int seconds);
int joint_move(int sockfd, robconf* rcf, jpos j_coord, unsigned int speed, unsigned int term);
int lin_move(int sockfd, lpos l_coord, unsigned int speed, unsigned int term);


int main(int argc, char *argv[]) {
	robconf r;
	int sockfd = start_server(&r);
	
	/* coords for debugging */
	jpos j_coord;
	unsigned int speed;
	/*
	srand(time(NULL));
	for (int i = 1; i <= r.axis_cnt; i++) {
		j_coord.j[i] = rand() % (int)(r.upper_lims.j[i]) + 1 -
						rand() % (int)(r.lower_lims.j[i]);
		j_coord.j[i] += 0.001 * (rand() % 1000);
	}*/
	for (int i = 1; i <= r.axis_cnt; i++) {
		j_coord.j[i] = 0;
	}
	j_coord.j[5] = -90;
	
	/* Perform run-time control */
	lpos l_coord;
	l_coord.x = 1010.0;
	l_coord.y = 1000.0;
	l_coord.z = 0.0;
	l_coord.w = 180.0;
	l_coord.p = 0.0;
	l_coord.r = 0.0;	
	unsigned int term = 50;
	
	speed = 1000;
	lin_move(sockfd, l_coord, speed, term);
	
	for (float i = 0.0; i <= 2000.0; i += 10.0) {
		l_coord.y = 1000.0 - i;
		l_coord.x = 1010.0 + sin(i / 100.0) * 200.0;
		lin_move(sockfd, l_coord, speed, term);
	}
	
	speed = 100;
	joint_move(sockfd, &r, j_coord, speed, term);
	/* Run-time control session end*/
	
	stop_server(sockfd);
	
	printf("Program finished without errors\n");
	exit(0);	
}


int start_server(robconf* r_cfg) {
	int sockfd;
	struct sockaddr_in serv_addr;
	
	/* Setting Robot-host address */
	memset((char*)&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family 		= AF_INET;
	serv_addr.sin_addr.s_addr 	= inet_addr(SERV_HOST_ADDR);
	serv_addr.sin_port 			= htons(SERV_TCP_PORT);
	
	/* Open client TCP socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("Can't Open Stream Socket");
        exit(-1);
	}
	
	/* Connect to the robot */
	printf("Connecting to " SERV_HOST_ADDR ":%d..\n", SERV_TCP_PORT);
	if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
		perror("Can't Connect to the server");
        exit(-2);
	}
	
	printf("Connected\n");
	getRobotInfo(sockfd, r_cfg);
	
	return sockfd;
}


int stop_server(int sockfd) {

	int n_bytes;
	char buf[MAXLINE];
	
	/* Send EOS comand */
	n_bytes = sprintf(buf, EOS);
	n_bytes = sendall(sockfd, buf, n_bytes);
	
	printf("Closing connection..\n");
	close(sockfd);
	return 0;
}


int getRobotInfo(int sockfd, robconf* rcf) {
	
	int i;
	int n_bytes;
	char buf[MAXLINE]; /* for received bytes */
	
	/* Recieving robot info string */
	n_bytes = recvline(sockfd, buf, MAXLINE);
	#ifdef DEBUG
		printf("Recieved %d bytes: %s", n_bytes, buf);
	#endif
	printf("Robot info:\n%s", buf);	

	/* Recieving robot axis count */
	n_bytes = recvline(sockfd, buf, MAXLINE);
	#ifdef DEBUG
		printf("Recieved %d bytes: %s", n_bytes, buf);
	#endif
	rcf->axis_cnt = atoi(buf);
	printf("Axis count: %d\n", rcf->axis_cnt);
	
	/* Recieving robot axis limits */
	for (i = 1; i <= rcf->axis_cnt; i++) {
		n_bytes = recvline(sockfd, buf, MAXLINE);
		#ifdef DEBUG
			printf("Recieved %d bytes: %s", n_bytes, buf);
		#endif
		rcf->lower_lims.j[i] = atof(buf);
		n_bytes = recvline(sockfd, buf, MAXLINE);
		#ifdef DEBUG
			printf("Recieved %d bytes: %s", n_bytes, buf);
		#endif
		rcf->upper_lims.j[i] = atof(buf);
		printf("J%d: %8.3f/%8.3f\n", i, rcf->lower_lims.j[i], rcf->upper_lims.j[i]);
	}
	
	/* Recieving menu */
	n_bytes = recvline(sockfd, buf, MAXLINE);
	unsigned short rows = atoi(buf);
	for (i = 0; i < rows; i++) {
		n_bytes = recvline(sockfd, buf, MAXLINE);
		printf("%s", buf);
	}
	
	return 0;
}


int waitForRobotReady(int sockfd, unsigned int seconds) {

	int n_bytes;
	char buf[MAXLINE];
		
	time_t start_time = time(NULL);
	printf("Waiting for ready state..\n");
	do {	
		n_bytes = recvline(sockfd, buf, MAXLINE);
		if (strstr(buf, "FALSE")) {
			fprintf(stderr, "Robot is not ready! \n");
			return -4;
		}
		if (time(NULL) - start_time > seconds) {
			fprintf(stderr, "Time exceed! Check robot state. \n");
			return -5;
		}
	} while (!strstr(buf, "TRUE"));
	printf("Robot ready.\n");
	
	return 0;
}


int joint_move(int sockfd, robconf* rcf, jpos j_coord, unsigned int speed, unsigned int term) {
	
	int n_bytes;
	char buf[MAXLINE];
	
	/* Send motion comand */
	n_bytes = sprintf(buf, MOV);
	n_bytes = sendall(sockfd, buf, n_bytes);
	
	printf("Start joint moving.. \n");	
	if (waitForRobotReady(sockfd, 60) != 0) {
		fprintf(stderr, "Robot doesnt ready!");
		return -5;
	}
	
	/* Send type of motion */
	n_bytes = sprintf(buf, "J");
	n_bytes = sendall(sockfd, buf, n_bytes);
	#ifdef DEBUG_OUT
		printf("Sended %d bytes: %s\n", n_bytes, buf);
	#endif
	
	/* Send coordinates */
	for (int i = 1; i <= rcf->axis_cnt; i++) {
		if (j_coord.j[i] >= rcf->lower_lims.j[i] &&
			j_coord.j[i] <= rcf->upper_lims.j[i]) {
				
			n_bytes = sprintf(buf, "%.3f ", j_coord.j[i]);
			n_bytes = sendall(sockfd, buf, n_bytes);
			#ifdef DEBUG_OUT
				printf("Sended %d bytes: %s\n", n_bytes, buf);
			#endif
		}
		else {
			fprintf(stderr, "Invalid coordinate: J%d = %8.3f \n", i, j_coord.j[i]);
			return -1;
		}
	}
	n_bytes = recvline(sockfd, buf, MAXLINE);
	printf("Position is reachable: %s", buf);
	if (!strstr(buf, "TRUE")) {
		fprintf(stderr, "Unreachable position! \n");
		return -2;
	}
	
	/* Send speed */
	if (speed > 100) {
		fprintf(stderr, "Invalid speed: %d \n", speed);
		return -3;
	}
	n_bytes = sprintf(buf, "%d ", speed);
	n_bytes = sendall(sockfd, buf, n_bytes);
	#ifdef DEBUG_OUT
		printf("Sended %d bytes: %s\n", n_bytes, buf);
	#endif
	
	/* Send termination */
	if (term > 100) {
		fprintf(stderr, "Invalid termination: %d \n", term);
		return -3;
	}
	n_bytes = sprintf(buf, "%d ", term);
	n_bytes = sendall(sockfd, buf, n_bytes);
	#ifdef DEBUG_OUT
		printf("Sended %d bytes: %s\n", n_bytes, buf);
	#endif
	
	/* Waiting for End Of Moving */
	time_t start_time = time(NULL);
	printf("Waiting for EOM..\n");
	do {	
		n_bytes = recvline(sockfd, buf, MAXLINE);
		if (time(NULL) - start_time > 300) {
			fprintf(stderr, "Time exceed! Check moving state. \n");
			return -5;
		}
	} while (!strstr(buf, "EOM"));	
	printf("Moving done.\n");
	
	/* Receieving current position */
	printf("Current position: \n");
	for (int i = 1; i <= rcf->axis_cnt; i++) {
		n_bytes = recvline(sockfd, buf, MAXLINE);
		j_coord.j[i] = atof(buf);
		printf("J%d: %8.3f\n", i, j_coord.j[i]);
	}
}


int lin_move(int sockfd, lpos l_coord, unsigned int speed, unsigned int term) {
	
	int n_bytes;
	char buf[MAXLINE];
	
	/* Send motion comand */
	n_bytes = sprintf(buf, MOV);
	n_bytes = sendall(sockfd, buf, n_bytes);
	
	printf("Start linear moving.. \n");	
	if (waitForRobotReady(sockfd, 60) != 0) {
		fprintf(stderr, "Robot doesnt ready!");
		return -5;
	}
	
	/* Send type of motion */
	n_bytes = sprintf(buf, "L");
	n_bytes = sendall(sockfd, buf, n_bytes);
	#ifdef DEBUG_OUT
		printf("Sended %d bytes: %s\n", n_bytes, buf);
	#endif
	
	/* Send coordinates */	
	float coords[6];
	lpos_to_array(l_coord, coords);
	for (int i = 0; i < 6; i++) {
		if (i > 3 && (coords[i] > 180.0 || coords[i] < -180.0)) {
			fprintf(stderr, "Invalid coordinate: L%d = %8.3f \n", i, coords[i]);
			return -1;
		}
		else {
			n_bytes = sprintf(buf, "%.3f ", coords[i]);
			n_bytes = sendall(sockfd, buf, n_bytes);
			#ifdef DEBUG_OUT
				printf("Sended %d bytes: %s\n", n_bytes, buf);
			#endif
		}
	}
	n_bytes = recvline(sockfd, buf, MAXLINE);
	printf("Position is reachable: %s", buf);
	if (!strstr(buf, "TRUE")) {
		fprintf(stderr, "Unreachable position! \n");
		return -2;
	}
	
	/* Send speed */
	if (speed > 2000) {
		fprintf(stderr, "Invalid speed: %d \n", speed);
		return -3;
	}
	n_bytes = sprintf(buf, "%d ", speed);
	n_bytes = sendall(sockfd, buf, n_bytes);
	#ifdef DEBUG_OUT
		printf("Sended %d bytes: %s\n", n_bytes, buf);
	#endif

	/* Send termination */
	if (term > 100) {
		fprintf(stderr, "Invalid termination: %d \n", term);
		return -3;
	}
	n_bytes = sprintf(buf, "%d ", term);
	n_bytes = sendall(sockfd, buf, n_bytes);
	#ifdef DEBUG_OUT
		printf("Sended %d bytes: %s\n", n_bytes, buf);
	#endif
	
	/* Waiting for End Of Moving */
	time_t start_time = time(NULL);
	printf("Waiting for EOM..\n");
	do {	
		n_bytes = recvline(sockfd, buf, MAXLINE);
		if (time(NULL) - start_time > 300) {
			fprintf(stderr, "Time exceed! Check moving state. \n");
			return -5;
		}
	} while (!strstr(buf, "EOM"));	
	printf("Moving done.\n");

}


int sendall(int socket, char *buf, int buf_len) {
    
	int total_sended = 0;
    int n_sended;
	
    while(total_sended < buf_len)
    {
        n_sended = send(socket, 
				buf + total_sended, 
				buf_len - total_sended,
				0);
        if(n_sended == -1) break;
        total_sended += n_sended;
    }
    return (n_sended == -1 ? -1 : total_sended);
}


int recvline(int socket, char *ptr, int maxlen) {
	
	int n;
	int received;
	char c;
	
	for (n = 0; n < maxlen; n++) {
		if ((received = recv(socket, &c, 1, 0)) == 1) {
			*ptr++ = c;
			
			if (c == '\n') break;
			else if (received == 0) {
				if (n == 0) return (n);
				else break;
			}
		}
		else return (-1);
	}
	
	*ptr = 0;
	return (n);
}

int lpos_to_array(lpos l_coord, float* coords) {
	coords[0] = l_coord.x;
	coords[1] = l_coord.y;
	coords[2] = l_coord.z;
	coords[3] = l_coord.w;
	coords[4] = l_coord.p;
	coords[5] = l_coord.r;	
	return 0;
}