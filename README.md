<div align="justify">
<h2>EV Autonomous System</h2>
  
**2022 국제대학생 창작자동차 경진대회 자율전기차 부문** 자율주행 시스템입니다.

자작차에 액추에이터를 설치해 플랫폼을 구동시키고, 3D LiDAR, GPS, 카메라를 사용해 미션을 수행했습니다. 
</div>

<p align="center">
  <br>
  <img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/5bf0c9d0-4ba8-4824-8d98-b836a714b646" width="400" height="400">  <img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/42fc286c-fb51-4d98-8771-7ffb3fbbafb5" width="400" height="400">
  <br>
</p>

## 목차
  - [개요](#개요) 
  - [구현 기능](#구현-기능)
  - [보완할 점](#보완할-점)

<br>

## 개요
- 프로젝트 지속기간: 2022.01-2022.09 (8개월)
- 개발 언어 및 기술: C++ & Python & ROS
- 팀 규모: 15인 1팀 *(개발 4인 1팀)*
- 센서 모델명
  - 3D LiDAR (OUSTER OS-1)
  - GPS (C099-f9p)
  - Camera (Logitech C270)

<br>

## 구현 기능

### Block Diagram

<img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/c9be8fd8-cb39-4c82-94fa-9574904eb983">
<br>

### Method
1. 3D LiDAR Object Detection: DBSCAN Clusteringimg
<img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/1cdac569-99b7-4ccc-b0b9-9013d2576361" width="200" height="300">

2. Camera Object Detection: Yolo Cone Detection
<img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/61875dd7-13c9-429c-b115-2479ef8b7085" width="400" height="200">

4. Local Planning

    3.1. Sensor Fusion: $Obj = {Obj_{LiDAR}+Obj_{camera}\over{2}}$

    3.2. Lane Keeping Algorithm

5. High-Level Control

    4.1. $Steering(\delta)=tan^{-1}(Path)$

    4.2. $Velocity(v)=f(v,\delta)$

6. Low-Level Control

    5.1. PID Controller for each state → $state=K_{P}e+K_{I}\int{e}dt+K_{D}\dot{e}$

    <img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/27d1bc34-106d-4c2e-8fe3-071f25a8486b" width="600" height="300">

<br>

## 보완할 점
- [ ] Planning 알고리즘을 더 정교하도록 수정하기 (현재: ***Pure*** Lane Keeping Algorithm)
- [ ] High-Level Control에 차량 모델을 적용하기 (현재: with ***No*** Vehicle Model)

</p>

<br>

## 라이센스

MIT &copy; [NoHack](mailto:lbjp114@gmail.com)
