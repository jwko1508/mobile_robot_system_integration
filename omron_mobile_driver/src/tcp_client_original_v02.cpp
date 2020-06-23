// status명령어를 사용함 그러나 이제 oneLineStatus를 사용할 생각임.


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <netdb.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>

#define MAX 4096
#define PORT 7171
#define SA struct sockaddr
#define TRUE 1
#define FALSE 0

unsigned char isFisrtFunc = FALSE;

void func(int sockfd)
{
    isFisrtFunc = TRUE;
    printf("first function!!\n");
    char buff[MAX];
    char read_buff[MAX];
    int n;
    for (;;) {
        bzero(buff, sizeof(buff));
        printf("Enter the string : ");
        n = 0;
        while ((buff[n++] = getchar()) != '\n');
        if ((strncmp(buff, "exit", 4)) == 0) {
            printf("inital func Exit...\n");
            break;
        }
        write(sockfd, buff, sizeof(buff));
        bzero(buff, sizeof(buff));
        read(sockfd, buff, sizeof(buff));
        printf("From Server : \n%s\n", buff);

    }
}

void statusCheck(int sockfd)
{
    isFisrtFunc = FALSE;
    printf("Second function!!\n");
    char buff[MAX];
    const char* buff_p;
    for (;;) {
        bzero(buff, sizeof(buff));
//        strcpy(buff, "status\r\n");
//        write(sockfd, buff, sizeof(buff));
//        bzero(buff, sizeof(buff));
        read(sockfd, buff, sizeof(buff));
        printf("result : %s\n", buff);
        if ((strncmp(buff, "exit", 4)) == 0) {
            printf("statusCheck Exit...\n");
            break;
        }
    }
}


  
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tcp_client_original_v02");
    int sockfd, connfd; 
    struct sockaddr_in servaddr, cli; 
  
    // socket create and varification 
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) { 
        printf("socket creation failed...\n"); 
        exit(0); 
    } 
    else
        printf("Socket successfully created..\n"); 
    bzero(&servaddr, sizeof(servaddr)); 
  
    // assign IP, PORT 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = inet_addr("192.168.0.104");
    servaddr.sin_port = htons(PORT); 
  
    // connect the client socket to server socket 
    if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) { 
        printf("connection with the server failed...\n"); 
        exit(0); 
    } 
    else
        printf("connected to the server..\n"); 
  
    // function for chat
    while(ros::ok())
    {
        if(!isFisrtFunc)
            func(sockfd);
        else{
            printf("statusCheck(sockfd) is started.\n");
            statusCheck(sockfd);
        }


    }



    // close the socket 
    close(sockfd); 
} 