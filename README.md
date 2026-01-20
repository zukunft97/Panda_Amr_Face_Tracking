# 라이센스 관련

## Description
This project implements face tracking for a Panda robot mounted on an AMR using ROS Noetic, MoveIt, and YOLOv5.

## Features
- Panda arm motion control with MoveIt
- YOLOv5-based face detection
- ROS message-based integration

## Acknowledgements
- MoveIt: https://github.com/moveit/moveit
- MoveIt Tutorials: https://github.com/moveit/moveit_tutorials
- YOLOv5: https://github.com/ultralytics/yolov5

# 로봇 팔과 카메라를 통하여 얼굴 추적 하는 시스템 구현

## 1. moveit의 튜토리얼 코드를 받아서 시작

## 2. 로봇 팔의 제어 방법 2가지 테스트

  - 로봇 팔의 각 조인트 값을 통한 제어
    
  - x,y,z 좌표 값을 통한 로봇 팔 제어

  ### 2-1 각각의 조인트 값을 통한 제어 테스트 결과
  
    - 시작 위치의 joint 값과 로봇 팔이 움직일 공간을 파악하여 사각형 형태의 각 꼭지 점 위치의 joint값을 읽은 후 상하좌우 명령을 통해 로봇이 범위 내의 일정 위치 이동 테스트시 기대한 움직이이 이루어 지지 않음
    
    원인 파악 과정
    
    - 범위 내의 각 꼭지점 으로 이동 하는 로봇 팔의 자세의 joint 값들이 조금씩 변경 되는게 아닌 중간에 크게 변경 되는 부분으로 인해
    10x10의 각 포인트 joint 값을 계산한 방식이 a + (b-a)/n 식의 값으로 계산 하여 오차가 크게 발생 하는것으로 파악 됨
    
    - 따라서 a,b의 자세가 크게 변경 되지 않는것을 확인 하며 로봇 팔의 활동 범위 재설정 하였으나 해결 되지 않음 개발과정 작성 해도되나?
    
    - 로봇 팔이 주어진 joint 값으로 잘 이동 되는 지 확인 하기 위해 현재 자세에서 rostopic을 통해 읽어들인 joint 값으로 움직 이라는 명령을 주었을때 정지해 있는것을 예상 하고 명령을 주었으나 자세가 크게 변경됨
    
    - /move_group/fake_controller_joint_states 이라는 topic을 통하여 읽어 들인 joint 값이 가상의 값이라고 파악되어 joint값을 통한 제어가 어렵다고 판단
    
  ### 2-2 x,y,z 좌표 값을 통한 로봇 팔 제어 테스트 결과
  
    - 현재 위치에서 x 좌표로 +5 만큼 이동 명령
    
      x +4 , y + 1 으로 이동 확인
      
    - 로봇 팔이 정확한 자세 보단 얼굴을 따라서 비슷하게 움직이는 것을 구현 하는게 목표 이므로 이대로 진행
    
  ### 2-3 x,y,z 좌표 값을 통한 로봇 팔 제어 테스트2 결과
  
    - 현재 위치에서 x+3, y+3 만큼 이동 명령 일때 로봇 팔의 엔드 툴의 위치가 오른쪽으로 대략 4cm 이동 확인
    
    - 현재 위치에서 x-3, y+3 만큼 이동 명령 일때 로봇 팔의 엔드 툴의 위치가 아래쪽으로 대략 4cm 이동 확인
      
    - 로봇 팔이 움직일땐 위치 이동 명령을 무시하고 멈춰있을 때만 위치 최신화 실시
    
    - 시작 위치에서 상화좌우 최대 10번만 이동 하도록 제한

## 3. 얼굴 추적 시스템 구현

  - 카메라와 YoloV5를 이용하여 얼굴 감지 및 사람의 얼굴 위치 추정
  
  - 추정된 얼굴의 박스를 이용하여 토픽을 발행 하여 로봇 팔 제어 방향 지시
  
  - 감지된 얼굴이 2개 이상일 경우 박스가 가장 큰게 가까운 사람으로 판단하여 박스가 제일 큰 사람을 기준으로 방향 제어
  
## 4. 얼굴 추적 과 로봇 팔 연동

  - 2,3 을 통해서 카메라로 인식한 사람의 얼굴을 로봇 팔이 추적 하도록 설정
  
  - 제어 하기 위한 명령어
  	cd ~/ws 기준
	python3 src/yolov5_ros/src/face_detect.py
	roslaunch panda_moveit_config demo.launch
	roslaunch moveit_tutorials move_group_interface_tutorial.launch
	roslaunch panda_amr_tutorial eef_relative_move.launch

