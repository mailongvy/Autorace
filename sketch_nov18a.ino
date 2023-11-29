#include <PID_v1.h>

// cảm biến siêu âm
const int trigPin = A0; // kết nối chân trig với chân 11 arduino
const int echoPin = A1; // kết nối chân echo với chân 12 arduino
long duration; //
int stop_distance = 12;// Khoảng cách phát hiện vật cản 
int distance;  // biến khoảng cách
int count = 1; // biến đếm các vật cản
// khi có 2 vật cản thì vật cản thứ 1 cho xe chạy vật cản thứ hai cho xe dừng


float Kp=19.34, Ki = 0, Kd=12;
int x = 95;
// P = error;
//   I = I + error;
//   D = error - previous_error;
//   PID_value = (Kp * P) + (Ki * I) + (Kd * D);
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
int first_speed = x;
int right_pid, left_pid;
float previous_error = 0;
int in1 = 2; //bánh trái
int in2 = 4; 
int in3 = 6; //bánh phải
int in4 = 7;
int ENA = 3; // bánh trái
int ENB = 5; // bánh phải

// int s1 = 3; // phải1  
// int s5 = 12; // phải2 
// int s2 = 4; // giữa
// int s3 = 5; // trái1
// int s4 = 13; // trái2
/// define sensor pinout
// #define line_1      13 // trái (hoặc ngược lại)
// #define line_2      5
// #define line_3      4 // giữa
// #define line_4      3 
// #define line_5      12 // phải

int sensor[5] = {0, 0, 0, 0, 0};



int Read_sensor();
void xuly();
void control();
void Dithangnhe();
void Diluinhe();
void Right();
void Left();

void setup() {
  pinMode(ENA, OUTPUT); // chân băm xung cho động cơ
  pinMode(ENB, OUTPUT); // chân băm xung cho động cơ
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
// sensor
  pinMode(8,INPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  Serial.begin(9600);

  // cảm biến siêu âm
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
}

void loop() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);

  xuly();

  if (distance <= stop_distance && count != 0) {
    Right();
    delay(300);
    Stop();
    delay(100);

    Dithangnhe();
    delay(300);
    Stop();
    delay(100);

    Left();
    delay(500);
    Stop();
    delay(100);

    Dithangnhe();
    while (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) {
      sensor[0] = digitalRead(8);
      sensor[1] = digitalRead(9);
      sensor[2] = digitalRead(10);
      sensor[3] = digitalRead(11);
      sensor[4] = digitalRead(12);
    }
  }

  else if (distance <= stop_distance && count == 0) {
    // khi biến đếm bằng 0 = gặp vật cản cuối cho xe dừng hẳn và cho dừng
    Stop();
    delay(100000);
  }
  
}
int Read_sensor() {
  /*
  1 la 0 nhan
  0 la nhan
  */
  int error = 0;
  sensor[0] = digitalRead(8);
  sensor[1] = digitalRead(9);
  sensor[2] = digitalRead(10);
  sensor[3] = digitalRead(11);
  sensor[4] = digitalRead(12);
  if((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) error = -4;
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) error = -3;
  else if((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) error = -2;
  else if((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) error = -1;
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) error = 0;
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) error = 1;
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1)) error = 2;
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) error = 3;
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) error = 4;
  //Trường hợp văng line, đi lùi
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) error = 100;
  //Chướng ngại vật đường vạch ngang, 5 line đều nhận, chạy thẳng nhẹ
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) error = 101;
  //TH nhiễu 3 line ở giữa nhận
  else if((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) error = 102;
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) error = 103;
  return error;
}



// Thêm biến để theo dõi trạng thái vạch line

// Thêm biến để theo dõi trạng thái vạch line và khoảng cách giữa các vạch line đứt
bool lineLost = false;
const int vachLineDutDistance = 10; // Khoảng cách giữa các vạch line đứt (đơn vị: cm)

bool Vatcan() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  if (distance <= stop_distance) {
    return true;
  }
  else {
    return false;
  }
}
void xuly() {
  error = Read_sensor();
  
  // TH nhiễu
  if (error == 102) {
    Dithangnhe();
    return;
  }
  else if (error == 101) {
    Dithangnhe();
    return;
  }
  else if (error != 100 && error != 101) {
    // Kiểm tra và cập nhật trạng thái vạch line
    if (lineLost) {
      // Nếu trạng thái trước đó là mất vạch line, điều chỉnh lại hướng lái và tăng tốc độ
      // theo thông tin về khoảng cách giữa các vạch line đứt
      int additionalSpeed = map(vachLineDutDistance, 0, 93, 0, 22); // Giả sử khoảng cách 0-100cm, tăng tốc độ từ 0-20
      int additionalSteering = map(vachLineDutDistance, 0, 93, 0, 35); // Giả sử khoảng cách 0-100cm, điều chỉnh lái từ 0-30
      analogWrite(ENA, x + additionalSpeed);
      analogWrite(ENB, x - additionalSpeed);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      // Điều chỉnh hướng lái
      int newSteering = x + additionalSteering;
      newSteering = constrain(newSteering, 50, 100); // Đảm bảo giữ giữa giá trị hướng lái trong khoảng 50-100
      analogWrite(ENA, newSteering);
      analogWrite(ENB, x - additionalSteering);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    } else {
      // Nếu vẫn ổn định, thực hiện điều khiển bình thường
      control();
    }
    lineLost = false;
  }
  else if (error == 100) {
    // Dithangnhe(); //
    if (error != 100 && error != 101) {
      // Gặp vạch line đứt, cập nhật trạng thái
      lineLost = true;
      control();
      delay(500);
      return;
    }
  }
  
  else {
    // Xử lý vạch line đứt
    if (error == 100) {
      // Thực hiện hành động khi gặp vạch line đứt
      // Ở đây, bạn có thể thực hiện các điều chỉnh cụ thể để giữ ổn định
      // Dưới đây là ví dụ tạm thời điều chỉnh hướng lái và tăng tốc độ
      int additionalSpeed = map(vachLineDutDistance, 0, 80, 0, 20); // Giả sử khoảng cách 0-100cm, tăng tốc độ từ 0-20
      int additionalSteering = map(vachLineDutDistance, 0, 80, 0,  50); // Giả sử khoảng cách 0-100cm, điều chỉnh lái từ 0-30
      analogWrite(ENA, x + additionalSpeed);
      analogWrite(ENB, x - additionalSpeed);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      // Điều chỉnh hướng lái
      int newSteering = x + additionalSteering;
      newSteering = constrain(newSteering, 50, 100); // Đảm bảo giữ giữa giá trị hướng lái trong khoảng 50-100
      analogWrite(ENA, newSteering);
      analogWrite(ENB, x - additionalSteering);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      delay(200);
      return;
    }
    
    control();
  }

  // trường hợp gặp vật cản
  if(Vatcan() == true && error == 103) // khi xe gặp vật cản và có nhận được line ngoài thì cho xe xử lí khi xe gặp line trái ngoài cùng(error = -4)
  { 
    error = -4;
    xuly();
    return;
  }
  else if(Vatcan == true && error != 103) {
    // khi cho xe bắt được vật cản cuối cùng mà ko nhận được line ngoài thì cho xe dừng và kết thúc
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }
  
}

/*them trương hợp khi gặp vật cản thì sẽ có 3 line nhận (1 1 1 0 0) thì cho xe chạy vs tốc độ giống như error = -4 
để cho xe rẽ sang trái để tiếp tục bắt line xử lí 
else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) error = -4
thêm dòng này vào phần đọc readsensor để bắt được vật cản
*/


/*trong trường hợp không có line (trường hợp bất đắc dĩ) thì điểu khiển thủ công:
- cho xe chạy lùi 
- cho xe rẽ trái đến khi cảm biến ko bắt được vật cản nữa
- cho xe rẽ lại hướng line cần tiếp tục
- cho xe bắt được line 
*/


void control() {
  error = Read_sensor();

  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  right_pid = first_speed - PID_value;
  left_pid = first_speed + PID_value;
  right_pid = constrain(first_speed - PID_value, 0, first_speed);
  left_pid = constrain(first_speed + PID_value, 0, first_speed);
  analogWrite(ENB, right_pid);
  analogWrite(ENA, left_pid);

  // test case
  Serial.print("error ");
  Serial.print(error);
  Serial.print("\t");
  // motorspeed
  Serial.print("\t");
  Serial.print("leftspeed ");
  Serial.print(left_pid);
  Serial.print("\t");
  Serial.print("rightspeedv");
  Serial.print(right_pid);
  Serial.println();
}

void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(in1, LOW); // bánh trái tiến
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); // bánh phải tiến
  digitalWrite(in4, LOW);
}

void Dithangnhe() {
  analogWrite(ENA, x - 33);
  analogWrite(ENB, x - 33);
  digitalWrite(in1, HIGH); // bánh trái tiến
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // bánh phải tiến
  digitalWrite(in4, LOW);
  delay(0);
} 

void Dilui() {
  analogWrite(ENA, x - 33);
  analogWrite(ENB, x - 33);
  digitalWrite(in1, LOW); // bánh trái lùi
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); // bánh phải lùi
  digitalWrite(in4, HIGH);
}

void Right() {
  analogWrite(ENA, x - 33);
  analogWrite(ENB, x - 33);
  digitalWrite(in1, HIGH); // bánh trái tiến
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); // bánh phải đứng yên
  digitalWrite(in4, LOW);
}

void Left() {
  analogWrite(ENA, x - 33);
  analogWrite(ENB, x - 33);
  digitalWrite(in1, LOW); // bánh trái đứng yên
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);// bánh phải tiến
  digitalWrite(in4, LOW);
}

