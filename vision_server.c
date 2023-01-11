#include "vision.h"
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <sys/wait.h>         /*  for waitpid()             */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include "jpeglib.h"
#include <setjmp.h>


void error_report(int conn, char *code, char *title, char *msg)
{
	char buffer[TEXTLEN];
	sprintf(buffer, 
		"HTTP/1.0 %s %s\r\n" 
        "\r\n" 
        "<!DOCTYPE HTML PUBLIC \"-//IETF//DTD HTML 2.0//EN\">\r\n" 
        "<TITLE>%s %s</TITLE>\r\n" 
        "</HEAD><BODY>\r\n" 
        "<H1>%s</H1>\r\n%s<P>\r\n" 
        "<HR><ADDRESS>Truck web server\r\n" 
        "</BODY></HTML>\r\n",
		code,
		title,
		code,
		title,
		title,
		msg);
	int result = write(conn, buffer, strlen(buffer));
}

int send_response(int conn, 
	unsigned char *buffer, 
	int buffer_size, 
	char *content_type)
{
	char string[TEXTLEN];
	sprintf(string, "HTTP/1.0 200 OK\r\n" 
     	   "Content-Type: %s\r\n" 
     	   "Server: Truck web server\r\n\r\n",
		   content_type);
	int result = write(conn, string, strlen(string));
	result = write(conn, buffer, buffer_size);
}


int listener;
void* httpd_thread(void *ptr)
{
	int pid;
	while(1)
	{
		int conn = accept(listener, NULL, NULL);

		if ( (pid = fork()) == 0 ) 
		{
// child process
			close(listener);
			
			unsigned char buffer[TEXTLEN];
			int bytes_read = read(conn, buffer, TEXTLEN);
			buffer[bytes_read] = 0;
//			printf("init_httpd %d: '%s'\n", __LINE__, buffer);

// search for GET
			char *end = (char*)buffer + strlen((char*)buffer);
			char *ptr = strstr((char*)buffer, "GET");
			if(ptr)
			{
				ptr += 4;
// get path
				if(ptr < end)
				{
					char *ptr2 = strchr(ptr, ' ');
					if(ptr2 && ptr2 < end)
					{
						*ptr2 = 0;
//						printf("init_httpd %d: '%s'\n", __LINE__, ptr);

// strip arguments
						ptr2 = strchr(ptr, '?');
						if(ptr2)
						{
							*ptr2 = 0;
						}

// create an image
						if(!strcmp(ptr, "/latest.jpg"))
						{
// 							printf("init_httpd %d: latest_size=%d latest_image=%p\n", 
// 								__LINE__, 
// 								vision.latest_size,
// 								vision.latest_image);

							pthread_mutex_lock(&vision.latest_lock);
							unsigned char *buffer2 = (unsigned char*)malloc(vision.latest_size);
							int buffer2_size = vision.latest_size;
							memcpy(buffer2, vision.latest_image, vision.latest_size);
							pthread_mutex_unlock(&vision.latest_lock);
							
							send_response(conn, buffer2, buffer2_size, (char*)"image/jpeg");
							free(buffer2);
						}
						else
						if(!strcmp(ptr, "/total_frames.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.frames_written);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
						if(!strcmp(ptr, "/fps.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.fps);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
						if(!strcmp(ptr, "/path_x.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.bottom_x);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
						if(!strcmp(ptr, "/bottom_x.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.bottom_x);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
						if(!strcmp(ptr, "/top_x.txt"))
						{
							char string[TEXTLEN];
							sprintf(string, "%d", vision.vanish_x);
							send_response(conn, (unsigned char*)string, strlen(string) + 1, (char*)"text/html");
						}
						else
// file in the html directory
						if(ptr[0] = '/')
						{
							char string[TEXTLEN];
							if(!strcmp(ptr, "/"))
								sprintf(string, "html/index.html");
							else
								sprintf(string, "html%s", ptr);
							
//							printf("init_httpd %d: '%s'\n", __LINE__, string);
							FILE *fd = fopen(string, "r");
							if(fd)
							{
								fseek(fd, 0, SEEK_END);
								int size = ftell(fd);
								fseek(fd, 0, SEEK_SET);

// need to pad the buffer or fclose locks up
								unsigned char *buffer = (unsigned char*)malloc(size + 16);
								int result = fread(buffer, size, 1, fd);
								buffer[size + 1] = 0;
								fclose(fd);
								
								send_response(conn, buffer, size + 1, (char*)"text/html");
								free(buffer);
							}
							else
							{
								char string2[TEXTLEN];
								sprintf(string2, 
									"The requested file '%s' was not found on this server.",
									string);
								error_report(conn, 
									(char*)"404", 
									(char*)"Not Found",
			                    	string2);
							}
							
						}
						else
						{
							error_report(conn, 
								(char*)"404", 
								(char*)"Not Found",
			                    (char*)"The requested URL was not found on this server.");
						}
					}
				}
			}
			
			close(conn);
			exit(EXIT_SUCCESS);
			
		}
		else
		{
// parent process
			close(conn);
			waitpid(-1, NULL, WNOHANG);
		}
	}
}

// http://www.paulgriffiths.net/program/c/srcs/webservsrc.html
void init_httpd()
{
    listener = socket(AF_INET, SOCK_STREAM, 0);
	struct sockaddr_in servaddr;
	memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	
	int port = 8080;
	for(port = 8080; port < 8100; port++)
	{
    	servaddr.sin_port = htons(port);
    	if ( bind(listener, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 )
		{
			printf("init_httpd %d: couldn't bind to port %d.\n", __LINE__, port);
			if(port == 99) return;
		}
		else
		{
			printf("init_httpd %d: got port %d\n", __LINE__, port);
			break;
		}
	}
	
	listen(listener, TEXTLEN);
	
	pthread_t tid;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_create(&tid, &attr, httpd_thread, 0);
}

