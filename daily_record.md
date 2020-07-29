# 2020-05-25 ~ 2020-05-29
## 학교 실습주간
### 오므론 모바일 로봇 데이터 수신
 <p align="center">
 그림 1
  <img width="550" src="https://user-images.githubusercontent.com/40682993/88031200-38a36300-cb77-11ea-9fba-b5f508a3c531.png"> 
 </p>
그림 1과 같이 네트워크를 구성함. Indy 7은 임시적으로 유선으로 연결하였지만 차후, 무선 네트워크로 변경할 예정

 <p align="center">
 그림 2
  <img width="350" src="https://user-images.githubusercontent.com/40682993/88031490-a3549e80-cb77-11ea-93f5-7324c9132dd9.png"> 
 </p>
그림 2의 네트워크 소켓을 통해 유선 연결 후, 무선인터넷 설정을 통해 공유기와 연결 가능함. 자세한 내용은 오므론 모바일 로봇 매뉴얼 참고 
[링크](https://www.ia.omron.co.kr/products/product_detail.asp?list_code2=031010&prodPk=2368)

오므론의 매뉴얼을 확인하면, (Advanced Robotics Command Language Reference Guide 의 141 페이지)
```
onelinestatus
```
명령어를 통해 다음의 정보를 얻을 수 있다.
```
Status: Arrived at <goal> BatteryVoltage: <volts_dc> Location: <X_mm> <Y_mm> <heading>
Temperature: <degrees>
```
즉, x,y,heading 값을 통해 현재 로봇의 위치를 알 수 있으며, tf 관계를 정해줄 수 있음. 

onelinestatus를 계속 입력해주어야 하므로 boost::thread를 이용해 병렬처리를 하여 계속 onelinestauts를 입력받을 수 있다. 그리고 통신은 tcp socket 통신을 이용함.

 <p align="center">
 그림 3
  <img width="550" src="https://user-images.githubusercontent.com/40682993/88032623-2de9cd80-cb79-11ea-9ecb-1b95b962b6c7.png"> 
 </p>

그림 3에서 Client는 개인 PC이고, Server는 omron mobile robot이다. 반복문을 통해 계속 onelinestuats를 요청하여 로봇으 현재 위치를 계속 수신할 수 있다.

### Indy7 (뉴로메카) 연결 방식

오므론과 동일하게 TCP 소켓 통신 방식이다. 다만, 명령을 위하여 뉴로메카에서 제공하는 IndyDCP 라이브러리를 이용해 명령을 STEP 컴퓨터에 전달한다.
[링크](http://docs.neuromeka.com/2.3.0/kr/C++/section1/)를 참고한다.

여기에서 C++ 를 사용하였는데, ROS에서 헤더파일을 추가하고자 한다면, 다음의 글[(링크)](https://roboticsbackend.com/ros-include-cpp-header-from-another-package/)을 참고해 헤더파일을 추가하면 된다. 

여기서 주의할 점은 위 링크에는 없는 내용이 있는데, 다음과 같이 CMakelist.txt파일에 
``` cpp
add_executable(indy_dcp_test src/indy_dcp_test.cpp)
target_link_libraries(indy_dcp_test ${catkin_LIBRARIES} ${PROJECT_NAME})
```
**${PROJECT_NAME})** 을 꼭 추가해주어야 빌드가 완성된다.


# 2020-07-02
## ipTime N3U doesn't work.
I had bought the ipTime N3U, but it doesn't work in ubuntu 14.04, so I tried to install the driver of N3U (Realtek rtl8192eu-linux-driver), it was impossible to install. Because linux kernel of STEP computer (embedded computer) is customed. so, I can't find this kernel. therefore I bought the new tp link wireless USB adapter. it was made before 2014. so I anticipate it will work properly in Step computer.

## iptime N3U가 작동이 안됨.
무선 인터넷이 필요해서 N3U를 구매했는데, step 컴퓨터에서 제대로 인식하지 않음. 그래서 드라이버를 설치하려 했으나 커스텀 kernel이라서 드라이버도 설치할 수 없었음. 그래서 새로운 어댑터를 구매하여 연결함.

# 2020-07-05
## The wireless Internet of the STEP is now working well.
After I use the tp-link wireless internet dongle, wireless internet worked very well. It's satisfying.

# 2020-07-16
## 다크넷 로스 디펜던시 설정
### how to set the build dependency for other package's message generation
[링크](https://stackoverflow.com/questions/51433055/how-to-setup-cmakelists-and-package-xml-for-generated-message-dependencies)에 들어가면 다른 패키지 빌드 디펜던시를 설정할 수 있다. 이런 방식으로 설정하면 됨.
add_dependencies(main_node darknet_ros_msgs_generate_messages_cpp)
이게 핵심이다.

# 2020-07-18
## 계획
오므론 모바일 로봇 Goal1 위치 이동 후 Goal2 위치 이동

# 2020-07-22
## 이슈
모바일 로봇은 성공적으로 움직이나 쓰레드가 과도하여 연산량을 100Hz로 제한할 예정

 <p align="center">
 그림 7-22-1
  <img width="550" src="https://user-images.githubusercontent.com/40682993/88146590-08bb9480-cc37-11ea-90d0-862b86b604a0.png"> 
 </p>
 그림  7-22-1과 같이 CPU와 과도할 때, 명령 전달 연산이 제대로 이뤄지지 않음. 즉, 쓰레드 연산이 우선순위에 밀린다. -> 반복문 연산을 100Hz로 제한하겠음.
 
 ---
 그건 아니었음, CPU 연산량을 11%에서 0%으로 줄였지만, 문제가 해결되지 않음.
 
 ``` cpp
   while(ros::ok())
  {
 //    if(recv(clientSocket, buffer, 1024, 0) < 0)
//    {
//      printf("Receive failed\n");
//    }
    char receiveStatus[MAX];
    strcpy(receiveStatus, buffer);
    char* receiveStatus_p = receiveStatus;
    ...
 ```
 
while문 초반에 **recv**를 해서 문제였음.  여기서 계속 수신값 받다가 delay가 생긴것.
그래도 100Hz로 바꾸면서 괜찮아짐.

## 해야할 일 
[ ] 백파일 녹화

<p align="center">
 그림 7-22-2
  <img width="550" src="https://user-images.githubusercontent.com/40682993/88177548-06bbfa80-cc64-11ea-946e-f08633e421bf.png"> 
 </p>
 
  그림 7-22-2와 같이 mobile_system_integration 에 command값을 공유하도록 (수정이 용이하도록) 헤더파일을 추가하였음.
  
  # 2020-07-27
  ## visual studio code 설치
  https://erdalpekel.de/?p=157 참고
  
  # 2020-07-29
  ## 윈도우에 있는 py 우분투에 옮길 때 이거 해줘야함.
  https://askubuntu.com/questions/896860/usr-bin-env-python3-r-no-such-file-or-directory
  
  # daily_pull_request test

https://github.com/jwko1508/test/pull/2 풀리퀘스트 하는 방법
  
  # 2020-07-30
  ## 작성자 : jwko1508
  1. pip install speech-recognition
  2. sudo apt install portaudio19-dev 
  3. pip install pyaudio
