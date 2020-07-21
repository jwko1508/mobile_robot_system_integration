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
그림 2의 네트워크 소켓을 통해 유선 연결 후, 무선인터넷 설정을 통해 공유기와 연결 가능함. 자세한 내용은 오므론 모바일 로봇 매뉴얼 참고 [링크](https://www.ia.omron.co.kr/products/product_detail.asp?list_code2=031010&prodPk=2368)

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
