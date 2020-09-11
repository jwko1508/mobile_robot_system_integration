#!/usr/bin/env python
#-*- coding: utf-8 -*-

# 출처: https://nowonbun.tistory.com/668 [명월 일지]

# 소켓을 사용하기 위해서는 socket을 import해야 한다.

import socket, threading

def binder(client_socket, addr):
  # 커넥션이 되면 접속 주소가 나온다.
  print('Connected by', addr);
  try:
    while True:
      # data = client_socket.recv(4);
      # length = int.from_bytes(data, "little");
      data = client_socket.recv(14);
      msg = data;
      print('Received from', addr, msg);
      # msg = "echo : " + msg;
      data = msg
      # length = len(data);
      # client_socket.sendall(length.to_bytes(4, byteorder="little"));
      client_socket.sendall(data);
  except Exception as e:
    print("except : " , addr);
    print("예외가 발생했습니다. ", e)
  finally:
    client_socket.close();

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1);
server_socket.bind(('', 9999));
server_socket.listen();

try:
  while True:
    client_socket, addr = server_socket.accept();
    th = threading.Thread(target=binder, args = (client_socket,addr));
    th.start();
except:
  print("server");
finally:
  server_socket.close();


# 출처: https://nowonbun.tistory.com/668 [명월 일지]
