# 아두이노 드론 프로젝트
## 개요
이 프로젝트는 송도고등학교 로봇동아리 (RnB)에서 진행하는 드론프로젝트의 소스코드를 공개하는 곳입니다. 라이센스는 MIT라이센스를 따릅니다.

## 진행현황
### 드론 내 소프트웨어(아두이노)

현재버전:Alpha v1.0.2

연혁

2016/05/24 Alpha v1.0 릴리즈

2016/05/29 Alpha v1.0.0.a 릴리즈(aimAngle, throttle에 대한 초기값 설정코드 추가)

2016/06/30 Alpha v1.0.2 릴리즈(Overflow문제 해결, 안정적인 동작을 위한 코드 추가)

### 조종 프로그램(프로세싱)(제작현황:계획중)

### 조종 프로그램(C#, Xamarin)

현재 버전:Alpha v.1.0

연혁

2016/05/29 Alpha v.1.0 릴리즈

링크:[https://github.com/Prokuma/ardino_drone_controller](https://github.com/Prokuma/ardino_drone_controller)

## 프로토콜 규격(Prokuma Drone Control Protocol v1)

명령자뒤에 값을 붙이는 형식으로 진행합니다. 현재 명령자의 종류는

T : 쓰로틀값 조정

R : Roll값 조정

P : Pitch값 조정

Y : Yaw값 조정

명령자는 char형태로, byte로 변환을 한 뒤에, byte로 변환한 조정값을 담아서 byte배열형태로 전송합니다. 예제처럼 써주시면 되겠습니다.

```C
{(byte)'T', (byte)255, (byte)'R', (byte)10, (byte)'P', (byte)5, (byte)'R',(byte)1}
```

### 프로토콜 업데이트 계획(Prokuma Drone Control Protocol v2)

구조 : 명령자 소명령자 값

명령자

C : Connection 관련 명령자
- s : 연결 개시 (값 X)
- e : 연결 종료 (값 X)
- k : 연결 유지 (값 X)

S : PID 상수 관련 명령자
- p : P제어 상수 설정
- i : I제어 상수 설정
- d : D제어 상수 설정

I : 정보 관련 명령자
- g : 관련 정보 얻어오기
- t : Throttle값 설정
- r : Roll값 설정
- p : Pitch값 설정
- y : Yaw값 설정

연결 유지는 5000ms동안에 관련 정보가 발견되지 않으면 연결이 끊긴것을 알고 자동적으로 종료.

## 아직 남아있는 개선사항
드론 내 소프트웨어

1.PID제어 -> 이중 PID제어

2.PID제어 상수 튜닝기능
