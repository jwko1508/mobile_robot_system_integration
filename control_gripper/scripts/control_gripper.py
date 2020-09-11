#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""
author : JIWAN HAN
e-mail : hjw1399@gmail.com
"""

# 출처: https://nowonbun.tistory.com/668 [명월 일지]

import socket
# import rospy 
import sys
import time


class control_gripper():
  def __init__(self, HOST, PORT, Gripper_ID):
    """
    HOST = string, host ip
    PORT = int, host port
    """
    self.HOST = HOST
    self.PORT = PORT
    self.gripper_id = Gripper_ID
    self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.connect()

    self.frame_header = [0xFF,0xFE,0xFD,0xFC]
    self.frame_end = 0xfb
    self.data_length = 14
    self.command_data = bytearray(self.data_length)
    for i in range(0,4):
      self.command_data[i] = self.frame_header[i]
    self.command_data[4] = self.gripper_id
    self.command_data[13] = self.frame_end

    ### defined parameter
    self.WRITE = 0x01
    self.READ = 0x00
    self.INTERNAL_DIRECTION = 0x02
    self.EXTERNAL_DIRECTION = 0x03

    time.sleep(5)
    # print(self.command_data)
    # print(type(self.frame_end))
    # print(sys.getsizeof(self.command_data))



  def __def__(self):
    self.disconnect()

  def connect(self):
    print('connect the gripper')
    self.client_socket.connect((self.HOST, self.PORT))

  def disconnect(self):
    print('disconnect the gripper')
    self.client_socket.close()

  def gripper_initialize(self, sub_function = 0x02, read_or_write = 0x01):
    """
    sub_fuction : 0x01, 0x02
    0x01 : the function of this command is to read and write whether feedback after finish initialization.
    0x02 : the function of this command is to initialize gripper or read whether finish initialization.
            write mode : gripper initialization.
            read mode : read initialization
    ---
    read_or_write : 0x00, 0x01
    read : 0x00
    write : 0x01
    """
    print('initializing..')
    initialization_function_register = 0x08
    sub_function_register = sub_function

    if not (sub_function_register == 0x01 or sub_function_register == 0x02):
      print('Subfunction can be inputted only 0x01, 0x02. please check again.')
      return
      
    if not (read_or_write == 0x00 or read_or_write == 0x01):
      print('read_write can be inputted only 0x00, 0x01. please check again.')
      return

    self.command_data[5] = initialization_function_register
    self.command_data[6] = sub_function_register
    self.command_data[7] = read_or_write
    data = [0x00, 0x00, 0x00, 0x00]
    self.input_data_to_command_data(data)
    self.send_data()

    if sub_function_register == 0x01:
      print('read-write is initialized')
    elif sub_function_register == 0x02:
      print('gripper is initialized')
    else:
      print("[DEBUG] early-return failed. this branch cannot be reach here.")

    time.sleep(8)
    


  def set_force_of_gripper(self, force_value, direction):
    """
    force_value : int, unit : %, not N
    direction : 0x02 = internal direction
                0x03 = external direction
    """
    force_function_register = 0x05

    if not (20 <= force_value <= 100):
      print ("Internal forces should always be between 20%", "and 100%.")
      return
    if not (direction == self.INTERNAL_DIRECTION or direction == self.EXTERNAL_DIRECTION):
      print ("direction should always be 0x02 or 0x03, please check again.")
      return

    if direction == self.INTERNAL_DIRECTION:
      print('be setting the internal force : {} ...'. format(force_value))

    elif direction == self.EXTERNAL_DIRECTION:
      print('be setting the external force : {} ...'. format(force_value))

    sub_function_register = direction
    
    self.command_data[5] = force_function_register
    self.command_data[6] = sub_function_register
    self.command_data[7] = self.WRITE

    data = force_value.to_bytes(4, byteorder="little")
    self.input_data_to_command_data(data)
    self.send_data()
    
    if direction == self.INTERNAL_DIRECTION:
      print('has set the internal force : {} ...'. format(force_value))

    elif direction == self.EXTERNAL_DIRECTION:
      print('has set the external force : {} ...'. format(force_value))

  def set_position(self, position):
    """
    position : int, data range : 0-100%
    """
    print('be setting the position : {} ...'. format(position))
    position_function_register = 0x06

    if not (0 <= position <= 100):
      print ("position should always be between 0%", "and 100%.")
      return
    sub_function_register = 0x02 

    self.command_data[5] = position_function_register
    self.command_data[6] = sub_function_register
    self.command_data[7] = self.WRITE

    data = position.to_bytes(4, byteorder="little")
    self.input_data_to_command_data(data)
    self.send_data()
    print('has set the position : {} ...'. format(position))
    time.sleep(1)


  def get_position(self):
    """
    return : position : int
    """
    print("getting the gripper's position.")
    position_function_register = 0x06
    sub_function_register = 0x02 

    self.command_data[5] = position_function_register
    self.command_data[6] = sub_function_register
    self.command_data[7] = self.READ
    data = [0x00, 0x00, 0x00, 0x00]
    self.input_data_to_command_data(data)
    self.send_data()
    _, position = self.receive_data()
    return position

  def wait_for_moving(self):
    feedback_function_register = 0x0f
    sub_function_register = 0x01

    self.command_data[5] = feedback_function_register
    self.command_data[6] = sub_function_register
    self.command_data[7] = self.READ

    # data = [0x00, 0x00, 0x00, 0x00]
    # self.input_data_to_command_data(data)

    return_data = 0
    isArrived = False
    while not isArrived:
      data = [0x00, 0x00, 0x00, 0x00]
      self.input_data_to_command_data(data)
      self.send_data()
      _, return_data = self.receive_data()
      # print(return_command_data)
      print("return_data :", return_data)
      if return_data == 2:
        print("gripper position is arrived.")
        isArrived = True
      elif return_data == 3:
        print("be get object.")
        isArrived = True
      time.sleep(0.01)
      
    time.sleep(1)
    

  def input_data_to_command_data(self, data_):
    for i in range(4):
      self.command_data[9+i] = data_[i]

  def send_data(self):
    # self.client_socket.sendall(self.data_length.to_bytes(4, byteorder="little"));
    self.client_socket.sendall(self.command_data)

  def receive_data(self):
    """
    ---
    return value
    received_command_data : all received_command_data, 14bytes
    data : unsigned int, 4bytes
    """
    # data = self.client_socket.recv(4);
    # length = int.from_bytes(data, "little");
    received_command_data = self.client_socket.recv(self.data_length)
    # msg = data
    # print('Received from : ', msg);
    data_bytes = bytearray(4)
    for i in range(0,4):
      data_bytes[i] = received_command_data[9+i]
    data = int.from_bytes(data_bytes, "little", signed=False)
    
    return received_command_data, data

  def print_current_command_data(self):
    print(self.command_data)

  def doTest(self):
    for _ in range(0,5):
      msg = 'hello'
      data = msg.encode()
      length = len(data)
      self.client_socket.sendall(length.to_bytes(4, byteorder="little"))
      self.client_socket.sendall(data)
      data = self.client_socket.recv(4)
      length = int.from_bytes(data, "little")
      data = self.client_socket.recv(length)
      msg = data.decode()
      print('Received from : ', msg)

  def doTest_v02(self):
    self.send_data()
    self.receive_data()

# HOST = '127.0.0.1'
# PORT = 9999
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# client_socket.connect((HOST, PORT))



if __name__ == '__main__':
  Control_gripper = control_gripper('192.168.1.29', PORT=8888, Gripper_ID=1)
  # waiting_time = 4
  Control_gripper.gripper_initialize(sub_function=0x02)
  # time.sleep(8)

  Control_gripper.set_force_of_gripper(20, Control_gripper.EXTERNAL_DIRECTION)
  time.sleep(1)
  Control_gripper.set_force_of_gripper(20, Control_gripper.INTERNAL_DIRECTION)
  time.sleep(1)

  # while True:
  # Control_gripper.set_position(0)
  # time.sleep(2)
  # Control_gripper.wait_for_moving()

  # time.sleep(3)
  # Control_gripper.wait_for_moving()

  Control_gripper.set_position(100)
  time.sleep(2)
  # Control_gripper.set_position(30)
  # time.sleep(2)


  # Control_gripper.set_position(0)
  # time.sleep(2)

  while True:
    pass
  # Control_gripper.wait_for_moving()
  # print(1111)
  # Control_gripper.print_current_command_data()

  # Control_gripper.set_position(50)
  # Control_gripper.wait_for_moving()
  # # Control_gripper.wait_for_moving()

  # Control_gripper.set_position(10)
  # Control_gripper.wait_for_moving()

  # Control_gripper.set_position(90)
  # Control_gripper.wait_for_moving()

  # Control_gripper.set_position(10)
  # Control_gripper.wait_for_moving()

  # Control_gripper.wait_for_moving()
  # time.sleep(3)
  # position = Control_gripper.get_position()
  # print("position :", position)


  # time.sleep(waiting_time)
  # Control_gripper.set_position(0)
  # Control_gripper.wait_for_moving()
  # time.sleep(waiting_time)
  # Control_gripper.set_position(100)
  # Control_gripper.wait_for_moving()
  # time.sleep(waiting_time)

  # Control_gripper.set_position(100)
  # time.sleep(waiting_time)
  # Control_gripper.wait_for_moving()


  # Control_gripper.set_position(50)
  # time.sleep(waiting_time)
  # Control_gripper.wait_for_moving()



    # is_success_initialization = False
    # while not is_success_initialization:
    #   received_command_data, data = self.receive_data()

    #   if sub_function_register == 0x01:
    #     if data == 0xA5:
    #       print('read-write is initialized')
    #       is_success_initialization = True

    #   elif sub_function_register == 0x02:
    #     if received_command_data[7] == 0x00:
    #       print('gripper is initialized')
    #       is_success_initialization = True

    #   else:
    #     print("[DEBUG] early-return failed. this branch cannot be reach here.")
    #   print('hello?')
    #   print(data)
    #   print(received_command_data)
    #   time.sleep(0.01)
    # print()
