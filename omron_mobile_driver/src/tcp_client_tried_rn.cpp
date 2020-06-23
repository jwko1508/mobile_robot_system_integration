#include "ros/ros.h"
#include "std_msgs/String.h"
#include <netdb.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>

#define TRUE 1
#define FALSE 0

#define MAX 4096
#define PORT 7171
#define SA struct sockaddr
char isLogin = FALSE;

void func(int sockfd)
{
    char buff[MAX];
    char buff_onelinestatus[MAX];
    char read_buff[MAX];
    int n;
    for (;;) {
        bzero(buff, sizeof(buff));
        printf("Enter the string : ");
        n = 0;
        if(!isLogin)
        {
            strcpy(buff, "admin\n");
            printf("admin buff : %s\n", buff);
            write(sockfd, buff, sizeof(buff));
            bzero(buff, sizeof(buff));
            isLogin = TRUE;
        }

        while ((buff[n++] = getchar()) != '\n')
        {
            printf("n: %d\n", n);
        }
        printf("final n: %d\n", n);

        /*
         * exit와 admin 확인
         */
        if ((strncmp(buff, "exit", 4)) == 0) {
            printf("inital func Exit...\n");
            break;
        }
//        else if((strncmp(buff, "admin", 5)) == 0)
//        {
//            write(sockfd, buff, sizeof(buff));
//            bzero(buff, sizeof(buff));
//            read(sockfd, buff, sizeof(buff));
//            printf("From Server : \n%s\n", buff);
//            continue;
//        }

        /*
         * \r\n을 만들기 위해 이렇게 작성함. while문을 빠져나온 n은 (문자열 길이 + 1)임.
            따라서 s 치고 엔터 눌렀으면 0: 's' 1: '\n' 2:(여기가 n이 가리키는 공간)
            후위 증감연산자라서 이런 결과가 나옴
         */
        buff[n-1] = ' ';
        buff[n] = '\r';
        buff[n+1] = '\n';
        printf("----------------1\n");
        printf("buff : %s\n", buff);
        printf("----------------2\n");

//        strcat(buff, "\r");
        write(sockfd, buff, sizeof(buff));
//        strcpy(buff_onelinestatus, "onelinestatus\r\n");
//        write(sockfd, buff_onelinestatus, sizeof(buff));
        bzero(buff, sizeof(buff));
        read(sockfd, buff, sizeof(buff));
        printf("From Server : \n%s\n", buff);

    }
}

void statusCheck(int sockfd)
{
    char buff[MAX];
    const char* buff_p;
    for (;;) {
        bzero(buff, sizeof(buff));
        strcpy(buff, "status\r\n");
        write(sockfd, buff, sizeof(buff));
        bzero(buff, sizeof(buff));
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
    ros::init(argc, argv, "tcp_client");
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
  
//     function for chat
    func(sockfd);

//    printf("statusCheck(sockfd) is started.\n");
//    statusCheck(sockfd);


    // close the socket 
    close(sockfd); 
} 