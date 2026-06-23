#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <time.h>


//======================================================
// WIFI
//======================================================

const char* ssid = "Q3 Pro";
const char* password = "12345678909";


//======================================================
// WEB SERVER
//======================================================

WebServer server(80);


//======================================================
// GPIO CONFIG
//======================================================

// IR SENSOR
#define IR1 4
#define IR2 14


// SERVO BARRIER
#define SERVO1_PIN 18
#define SERVO2_PIN 19


// LIMIT SWITCH
// LOW = ĐÃ NHẤN
// HIGH = CHƯA NHẤN

#define LIMIT1 33
#define LIMIT2 27


// ĐÈN ĐỎ BÊN TRÁI
#define RED1 25
#define RED2 26


// ĐÈN ĐỎ BÊN PHẢI
#define RED3 16
#define RED4 17


// ĐÈN VÀNG BÁO LỖI
#define YELLOW1 32
#define YELLOW2 13


// BUZZER
#define BUZZER 23


//======================================================
// SERVO
//======================================================

Servo barrier1;
Servo barrier2;


//======================================================
// SYSTEM MODE
//======================================================

enum SystemMode
{
  AUTO_MODE,
  MANUAL_MODE
};


SystemMode mode = AUTO_MODE;


//======================================================
// SYSTEM ERROR
//======================================================

bool systemError = false;

String errorMessage = "NONE";


//======================================================
// TRAIN STATE
//======================================================

enum TrainState
{
  NO_TRAIN,
  TRAIN_COMING,
  TRAIN_PASSING,
  TRAIN_LEAVING
};


TrainState trainState = NO_TRAIN;

const int HISTORY_MAX = 20;
String historyArrival[HISTORY_MAX];
String historyLeave[HISTORY_MAX];
int historyCount = 0;
time_t trainArrivalEpoch = 0;

//======================================================
// BARRIER STATE
//======================================================

enum BarrierState
{
  BARRIER_OPEN,
  BARRIER_CLOSING,
  BARRIER_CLOSED,
  BARRIER_OPENING,
  BARRIER_ERROR
};


BarrierState barrier1State = BARRIER_OPEN;
BarrierState barrier2State = BARRIER_OPEN;


//======================================================
// TIMER
//======================================================

// Đèn đỏ nhấp nháy
unsigned long blinkTimer = 0;
bool blinkState = false;


// Phát hiện tàu
unsigned long trainDetectTime = 0;


// Tàu rời khỏi IR2
unsigned long trainLeaveTime = 0;


// Kiểm tra thời gian đóng barrier
unsigned long barrierCloseTimer = 0;

// Limit switch debounce
const unsigned long LIMIT_DEBOUNCE_TIME = 200;
const unsigned long LIMIT_CLOSE_CONFIRM_TIMEOUT = 1000;
int limit1Raw = HIGH;
int limit2Raw = HIGH;
int limit1Stable = HIGH;
int limit2Stable = HIGH;
unsigned long limit1DebounceTime = 0;
unsigned long limit2DebounceTime = 0;

void updateLimitSwitches();

// Pending close (set when IR1 detects, close after delay)
bool pendingClose = false;
unsigned long pendingCloseStart = 0;

// IR1 must remain active for this long before we accept it as a true train detection.
const unsigned long IR1_DETECT_CONFIRM_MS = 100;

// If IR1 triggered but IR2 never sees the train, return to safe state after this timeout.
const unsigned long TRAIN_ABORT_TIMEOUT_MS = 20000;

unsigned long ir1DetectStart = 0;

//======================================================
// KIỂM TRA IR NHẤP NHÁY BẤT THƯỜNG
//======================================================

int lastIR1 = HIGH;
int lastIR2 = HIGH;


int ir1ToggleCount = 0;
int ir2ToggleCount = 0;


unsigned long irToggleTimer = 0;


//======================================================
// KIỂM TRA IR BỊ CHE QUÁ LÂU
//======================================================

unsigned long ir1ActiveTime = 0;
unsigned long ir2ActiveTime = 0;


//======================================================
// TRẠNG THÁI ĐIỀU KHIỂN TAY
//======================================================

bool manualBuzzer = false;

bool manualRedBlink = false;

void setFailSafeOutputs();

//======================================================
// HÀM BÁO LỖI
//======================================================

void setError(String message)
{
  systemError = true;

  mode = MANUAL_MODE;

  errorMessage = message;

  // cancel any pending close when error occurs
  pendingClose = false;
  pendingCloseStart = 0;

  // Fail-safe: treat error as if train is passing
  trainState = TRAIN_PASSING;

  setFailSafeOutputs();

  // Force barriers closed for fail-safe
  if(barrier1State != BARRIER_CLOSED || barrier2State != BARRIER_CLOSED)
  {
    for(int angle = 90; angle >= 0; angle--)
    {
      barrier1.write(angle);
      barrier2.write(angle);
      delay(15);
    }

    barrier1State = BARRIER_CLOSED;
    barrier2State = BARRIER_CLOSED;
    barrierCloseTimer = millis();

    updateLimitSwitches();
  }

  Serial.println("========================");
  Serial.println(" SYSTEM ERROR ");
  Serial.println(message);
  Serial.println(" SWITCH TO FAIL-SAFE MODE ");
  Serial.println("========================");
}
//======================================================
// ĐÓNG BARRIER
//======================================================

void closeBarrier()
{
  Serial.println("CLOSING BARRIER");

  barrier1State = BARRIER_CLOSING;
  barrier2State = BARRIER_CLOSING;

  for(int angle = 90; angle >= 0; angle--)
  {
    barrier1.write(angle);
    barrier2.write(angle);
    delay(15);
  }

  barrier1State = BARRIER_CLOSED;
  barrier2State = BARRIER_CLOSED;

  // clear pending close when barriers reach closed
  pendingClose = false;
  pendingCloseStart = 0;

  barrierCloseTimer = millis();

  // Wait for limit switches to settle, then use debounced stable values.
  unsigned long closeConfirmStart = millis();
  do
  {
    updateLimitSwitches();

    if(limit1Stable == LOW && limit2Stable == LOW)
    {
      break;
    }

    delay(10);
  }
  while(millis() - closeConfirmStart < LIMIT_CLOSE_CONFIRM_TIMEOUT);

  if(!(limit1Stable == LOW && limit2Stable == LOW))
  {
    setError("BARRIER NOT CLOSED");
  }
}


//======================================================
// MỞ BARRIER
//======================================================

void openBarrier()
{
  Serial.println("OPENING BARRIER");

  barrier1State = BARRIER_OPENING;
  barrier2State = BARRIER_OPENING;

  for(int angle = 0; angle <= 90; angle++)
  {
    barrier1.write(angle);
    barrier2.write(angle);
    delay(15);
  }

  barrier1State = BARRIER_OPEN;
  barrier2State = BARRIER_OPEN;
}


//======================================================
// TẮT TẤT CẢ ĐÈN
//======================================================

void allLedOff()
{
  digitalWrite(RED1, LOW);
  digitalWrite(RED2, LOW);

  digitalWrite(RED3, LOW);
  digitalWrite(RED4, LOW);

  digitalWrite(YELLOW1, LOW);
  digitalWrite(YELLOW2, LOW);
}


//======================================================
// NHÁY ĐÈN ĐỎ CẢNH BÁO
//======================================================

void redBlink()
{
  if(millis() - blinkTimer >= 300)
  {
    blinkTimer = millis();

    blinkState = !blinkState;

    digitalWrite(RED1, blinkState);
    digitalWrite(RED2, !blinkState);

    digitalWrite(RED3, blinkState);
    digitalWrite(RED4, !blinkState);
  }
}


//======================================================
// TẮT CẢNH BÁO
//======================================================

void warningOff()
{
  digitalWrite(BUZZER, LOW);

  digitalWrite(RED1, LOW);
  digitalWrite(RED2, LOW);

  digitalWrite(RED3, LOW);
  digitalWrite(RED4, LOW);
}


//======================================================
// RESET LỖI
//======================================================

void setFailSafeOutputs()
{
  digitalWrite(BUZZER, HIGH);

  digitalWrite(RED1, HIGH);
  digitalWrite(RED2, HIGH);
  digitalWrite(RED3, HIGH);
  digitalWrite(RED4, HIGH);

  digitalWrite(YELLOW1, HIGH);
  digitalWrite(YELLOW2, HIGH);
}


void resetError()
{
  Serial.println("RESET SYSTEM");

  systemError = false;
  mode = AUTO_MODE;

  errorMessage = "NONE";

  trainState = NO_TRAIN;


  manualBuzzer = false;
  manualRedBlink = false;


  ir1ToggleCount = 0;
  ir2ToggleCount = 0;

  ir1ActiveTime = 0;
  ir2ActiveTime = 0;

  // cancel pending close
  pendingClose = false;
  pendingCloseStart = 0;


  warningOff();

  allLedOff();

  openBarrier();

  Serial.println("SYSTEM BACK TO AUTO MODE");
}

//======================================================
// KIỂM TRA LIMIT BARRIER
//======================================================

void updateLimitSwitches()
{
  int raw1 = digitalRead(LIMIT1);
  int raw2 = digitalRead(LIMIT2);

  if(raw1 != limit1Raw)
  {
    limit1Raw = raw1;
    limit1DebounceTime = millis();
  }
  else if(raw1 != limit1Stable &&
          millis() - limit1DebounceTime >= LIMIT_DEBOUNCE_TIME)
  {
    limit1Stable = raw1;
  }

  if(raw2 != limit2Raw)
  {
    limit2Raw = raw2;
    limit2DebounceTime = millis();
  }
  else if(raw2 != limit2Stable &&
          millis() - limit2DebounceTime >= LIMIT_DEBOUNCE_TIME)
  {
    limit2Stable = raw2;
  }
}


void checkLimit()
{
  // Cả 2 barrier đã đóng
  if(limit1Stable == LOW && limit2Stable == LOW)
  {
    barrier1State = BARRIER_CLOSED;
    barrier2State = BARRIER_CLOSED;

    return;
  }


  // Quá 5 giây chưa đóng đủ
  if((millis() - barrierCloseTimer > 5000) &&
     (barrier1State == BARRIER_CLOSING ||
      barrier2State == BARRIER_CLOSING))
  {
    setError("BARRIER NOT CLOSED");
  }
}


//======================================================
// KIỂM TRA 2 IR CÙNG LÚC
//======================================================

void checkBothIR(int ir1, int ir2)
{
  if(ir1 == LOW && ir2 == LOW)
  {
    setError("BOTH IR DETECTED");
  }
}


//======================================================
// KIỂM TRA IR CHỚP TẮT BẤT THƯỜNG
//======================================================

void checkIRToggle(int ir1, int ir2)
{
  if(millis() - irToggleTimer >= 5000)
  {
    irToggleTimer = millis();

    ir1ToggleCount = 0;
    ir2ToggleCount = 0;
  }


  if(ir1 != lastIR1)
  {
    lastIR1 = ir1;
    ir1ToggleCount++;
  }


  if(ir2 != lastIR2)
  {
    lastIR2 = ir2;
    ir2ToggleCount++;
  }


  if(ir1ToggleCount >= 5)
  {
    setError("IR1 UNSTABLE");
  }


  if(ir2ToggleCount >= 5)
  {
    setError("IR2 UNSTABLE");
  }
}


//======================================================
// KIỂM TRA IR BỊ CHE QUÁ 3 PHÚT
//======================================================

void checkIRActive(int ir1, int ir2)
{
  // IR1
  if(ir1 == LOW)
  {
    if(ir1ActiveTime == 0)
    {
      ir1ActiveTime = millis();
    }

    if(millis() - ir1ActiveTime >= 180000)
    {
      setError("IR1 ACTIVE TOO LONG");
    }
  }
  else
  {
    ir1ActiveTime = 0;
  }


  // IR2
  if(ir2 == LOW)
  {
    if(ir2ActiveTime == 0)
    {
      ir2ActiveTime = millis();
    }

    if(millis() - ir2ActiveTime >= 180000)
    {
      setError("IR2 ACTIVE TOO LONG");
    }
  }
  else
  {
    ir2ActiveTime = 0;
  }
}


//======================================================
// KIỂM TRA TÀU TỚI IR2 KHI BARRIER CHƯA ĐÓNG
//======================================================

void checkTrainSafety(int ir2)
{
  if(ir2 == LOW)
  {
    if(barrier1State != BARRIER_CLOSED ||
       barrier2State != BARRIER_CLOSED)
    {
      setError("TRAIN PASSED BEFORE GATE CLOSED");
    }
  }
}
//======================================================
// GET TRAIN STATUS
//======================================================

String getTrainStatus()
{
  switch(trainState)
  {
    case NO_TRAIN:
      return "NO TRAIN";

    case TRAIN_COMING:
      return "TRAIN COMING";

    case TRAIN_PASSING:
      return "TRAIN PASSING";

    case TRAIN_LEAVING:
      return "TRAIN LEFT";
  }

  return "UNKNOWN";
}


//======================================================
// GET BARRIER STATUS
//======================================================

String getBarrierStatus(BarrierState state)
{
  switch(state)
  {
    case BARRIER_OPEN:
      return "OPEN";

    case BARRIER_CLOSING:
      return "CLOSING";

    case BARRIER_CLOSED:
      return "CLOSED";

    case BARRIER_OPENING:
      return "OPENING";

    case BARRIER_ERROR:
      return "ERROR";
  }

  return "UNKNOWN";
}


//======================================================
// GET SYSTEM MODE
//======================================================

String getMode()
{
  if(mode == AUTO_MODE)
    return "AUTO";

  return "MANUAL";
}

String getGateStatus()
{
  if(barrier1State == BARRIER_CLOSED && barrier2State == BARRIER_CLOSED)
    return "CLOSED";

  return "OPEN";
}

String formatTimeString(time_t t)
{
  if(t == 0)
    return String("-");

  struct tm timeinfo;
  localtime_r(&t, &timeinfo);

  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo.tm_year + 1900,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday,
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec);

  return String(buf);
}
//======================================================
// STATUS API
//======================================================

void handleStatus()
{
  updateLimitSwitches();

  String json = "{";

  json += "\"ir1\":\"";
  json += (digitalRead(IR1)==LOW) ?
          "DETECTED":"CLEAR";
  json += "\",";


  json += "\"ir2\":\"";
  json += (digitalRead(IR2)==LOW) ?
          "DETECTED":"CLEAR";
  json += "\",";


  json += "\"barrier1\":\"";
  json += getBarrierStatus(barrier1State);
  json += "\",";


  json += "\"barrier2\":\"";
  json += getBarrierStatus(barrier2State);
  json += "\",";

  json += "\"limit1\":\"";
  json += (limit1Stable==LOW) ? "CLOSED":"OPEN";
  json += "\",";

  json += "\"limit2\":\"";
  json += (limit2Stable==LOW) ? "CLOSED":"OPEN";
  json += "\",";


  json += "\"train\":\"";
  json += getTrainStatus();
  json += "\",";

  json += "\"gate\":\"";
  json += getGateStatus();
  json += "\",";

  json += "\"buzzer\":\"";
  json += digitalRead(BUZZER) ?
          "ON":"OFF";
  json += "\",";


  json += "\"system\":\"";
  json += getMode();
  json += "\",";


  json += "\"error\":\"";
  json += errorMessage;
  json += "\"";


  json += "}";


  server.send(200,
              "application/json",
              json);
}

//======================================================
// HISTORY API
//======================================================

void handleHistory()
{
  String json = "[";

  for(int i = 0; i < historyCount; ++i)
  {
    json += "{";
    json += "\"arrival\":\"" + historyArrival[i] + "\",";
    json += "\"leave\":\"" + historyLeave[i] + "\"";
    json += "}";

    if(i + 1 < historyCount) json += ",";
  }

  json += "]";
  server.send(200, "application/json", json);
}

//======================================================
// WEB PAGE
//======================================================

void handleRoot()
{

String html = R"rawliteral(

<!DOCTYPE html>
<html>

<head>

<meta name="viewport"
content="width=device-width, initial-scale=1">

<title>
HE THONG CANH BAO DUONG NGANG TU DONG
</title>

<style>

body{
background:#071738;
color:white;
font-family:Arial;
text-align:center;
}

h1{
color:#00c8ff;
margin-top:20px;
}


.container{

display:flex;
flex-wrap:wrap;
justify-content:center;

}


.card{

width:170px;
height:90px;

background:#152542;

margin:10px;

border-radius:12px;

box-shadow:0 0 10px black;

padding-top:15px;

}


.status{

font-size:22px;
margin-top:10px;

}


button{

width:180px;

height:40px;

margin:8px;

border:none;

border-radius:8px;

font-weight:bold;

cursor:pointer;

}


.open{

background:#00cc44;

}


.close{

background:#ff4444;
color:white;

}


.warn{

background:#ffcc00;

}


.reset{

background:#cc0000;

color:white;

}

.error{

color:#ff3333;
font-weight:bold;

}


</style>

</head>


<body>


<h1>
HE THONG CANH BAO DUONG NGANG TU DONG
</h1>


<div class="container">


<div class="card">

<h3>IR1</h3>

<div id="ir1"
class="status">

---</div>

</div>



<div class="card">

<h3>IR2</h3>

<div id="ir2"
class="status">

---</div>

</div>


<div class="card">

<h3>BARRIER 1</h3>

<div id="barrier1"
class="status">

---</div>

</div>


<div class="card">

<h3>BARRIER 2</h3>

<div id="barrier2"
class="status">

---</div>

</div>


<div class="card">

<h3>LIMIT 1</h3>

<div id="limit1"
class="status">

---</div>

</div>


<div class="card">

<h3>LIMIT 2</h3>

<div id="limit2"
class="status">

---</div>

</div>



<div class="card">

<h3>TRAIN STATUS</h3>

<div id="train"
class="status">

---</div>

</div>



<div class="card">

<h3>GATE</h3>

<div id="gate"
class="status">

---</div>

</div>



<div class="card">

<h3>BUZZER</h3>

<div id="buzzer"
class="status">

---</div>

</div>



<div class="card">

<h3>SYSTEM</h3>

<div id="system"
class="status error">

---</div>

</div>


<div class="card">

<h3>ERROR</h3>

<div id="error"
class="status error">

NONE</div>

</div>


</div>


<h2>
MANUAL CONTROL
</h2>
<a href="/open1">
<button class="open">
OPEN BARRIER 1
</button>
</a>


<a href="/close1">
<button class="close">
CLOSE BARRIER 1
</button>
</a>


<br>


<a href="/open2">
<button class="open">
OPEN BARRIER 2
</button>
</a>


<a href="/close2">
<button class="close">
CLOSE BARRIER 2
</button>
</a>


<br>


<a href="/buzzerOn">
<button class="warn">
BUZZER ON
</button>
</a>


<a href="/buzzerOff">
<button class="warn">
BUZZER OFF
</button>
</a>


<br>


<a href="/redOn">
<button class="close">
RED LIGHT ON
</button>
</a>


<a href="/redOff">
<button class="open">
RED LIGHT OFF
</button>
</a>


<br>


<a href="/resetError">
<button class="warn">
RESET ERROR
</button>
</a>


<br>


<a href="/resetESP">
<button class="reset">
RESET ESP32
</button>
</a>

<br>

<button class="warn" onclick="loadHistory()">
TIME
</button>

<div id="history" style="max-width:800px;margin:12px auto;color:#fff;text-align:left;">
<!-- history will appear here -->
</div>


<script>


function updateData()
{

fetch("/status")

.then(response => response.json())

.then(data =>
{


document.getElementById("ir1")
.innerHTML = data.ir1;


document.getElementById("ir2")
.innerHTML = data.ir2;


document.getElementById("barrier1")
.innerHTML = data.barrier1;


document.getElementById("barrier2")
.innerHTML = data.barrier2;


document.getElementById("train")
.innerHTML = data.train;

document.getElementById("limit1")
.innerHTML = data.limit1;

document.getElementById("limit2")
.innerHTML = data.limit2;


document.getElementById("gate")
.innerHTML = data.gate;


document.getElementById("buzzer")
.innerHTML = data.buzzer;


document.getElementById("system")
.innerHTML = data.system;


document.getElementById("error")
.innerHTML = data.error;


});

}


function loadHistory()
{
  fetch("/history")
  .then(response => response.json())
  .then(data =>
  {
    let out = "<h3 style='color:#00c8ff'>Train History</h3><ul>";

    data.forEach(item => {
      out += "<li>Train arrival:" + item.arrival + "; Train departure:" + item.leave + "</li>";
    });

    out += "</ul>";
    document.getElementById("history").innerHTML = out;
  });
}


setInterval(updateData,500);


updateData();


</script>


</body>

</html>


)rawliteral";


server.send(200,
            "text/html",
            html);

}
//======================================================
// MANUAL BARRIER 1
//======================================================

void handleOpen1()
{
  if(systemError)
  {
    server.sendHeader("Location","/");
    server.send(303);
    return;
  }

  for(int i = 0; i <= 90; i++)
  {
    barrier1.write(i);
    delay(15);
  }

  barrier1State = BARRIER_OPEN;

  server.sendHeader("Location","/");
  server.send(303);
}


void handleClose1()
{
  for(int i = 90; i >= 0; i--)
  {
    barrier1.write(i);
    delay(15);
  }

  barrier1State = BARRIER_CLOSED;

  // cancel pending close if user manually closed
  pendingClose = false;
  pendingCloseStart = 0;

  server.sendHeader("Location","/");
  server.send(303);
}


//======================================================
// MANUAL BARRIER 2
//======================================================

void handleOpen2()
{
  if(systemError)
  {
    server.sendHeader("Location","/");
    server.send(303);
    return;
  }

  for(int i = 0; i <= 90; i++)
  {
    barrier2.write(i);
    delay(15);
  }

  barrier2State = BARRIER_OPEN;

  server.sendHeader("Location","/");
  server.send(303);
}


void handleClose2()
{
  for(int i = 90; i >= 0; i--)
  {
    barrier2.write(i);
    delay(15);
  }

  barrier2State = BARRIER_CLOSED;

  // cancel pending close if user manually closed
  pendingClose = false;
  pendingCloseStart = 0;

  server.sendHeader("Location","/");
  server.send(303);
}


//======================================================
// BUZZER
//======================================================

void handleBuzzerOn()
{
  manualBuzzer = true;

  digitalWrite(BUZZER,HIGH);

  server.sendHeader("Location","/");
  server.send(303);
}


void handleBuzzerOff()
{
  if(systemError)
  {
    server.sendHeader("Location","/");
    server.send(303);
    return;
  }

  manualBuzzer = false;

  digitalWrite(BUZZER,LOW);

  server.sendHeader("Location","/");
  server.send(303);
}


//======================================================
// RED LIGHT
//======================================================

void handleRedOn()
{
  manualRedBlink = true;

  server.sendHeader("Location","/");
  server.send(303);
}


void handleRedOff()
{
  if(systemError)
  {
    server.sendHeader("Location","/");
    server.send(303);
    return;
  }

  manualRedBlink = false;

  digitalWrite(RED1,LOW);
  digitalWrite(RED2,LOW);
  digitalWrite(RED3,LOW);
  digitalWrite(RED4,LOW);

  server.sendHeader("Location","/");
  server.send(303);
}


//======================================================
// RESET ERROR
//======================================================

void handleResetError()
{
  resetError();

  server.sendHeader("Location","/");
  server.send(303);
}


//======================================================
// RESET ESP32
//======================================================

void handleResetESP()
{
  server.send(200,"text/plain",
              "ESP32 RESTART");

  delay(1000);

  ESP.restart();
}
//======================================================
// SETUP
//======================================================

void setup()
{
  Serial.begin(115200);

  // IR SENSOR
  pinMode(IR1, INPUT_PULLUP);
  pinMode(IR2, INPUT_PULLUP);


  // LIMIT SWITCH
  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);


  // LED
  pinMode(RED1, OUTPUT);
  pinMode(RED2, OUTPUT);
  pinMode(RED3, OUTPUT);
  pinMode(RED4, OUTPUT);

  pinMode(YELLOW1, OUTPUT);
  pinMode(YELLOW2, OUTPUT);


  // BUZZER
  pinMode(BUZZER, OUTPUT);


  allLedOff();
  digitalWrite(BUZZER, LOW);


  // SERVO
  barrier1.attach(SERVO1_PIN);
  barrier2.attach(SERVO2_PIN);


  // Mặc định mở barrier
  barrier1.write(90);
  barrier2.write(90);


  // WIFI
  WiFi.begin(ssid, password);

  Serial.print("CONNECTING WIFI");

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }


  Serial.println();
  Serial.println("WIFI CONNECTED");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Cấu hình đồng bộ thời gian từ Internet (Múi giờ Việt Nam: UTC+7)
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Da dong bo thoi gian thuc qua NTP");


  // WEB PAGE
  server.on("/", handleRoot);

  server.on("/status", handleStatus);
  server.on("/history", handleHistory);

  // MANUAL CONTROL
  server.on("/open1", handleOpen1);
  server.on("/close1", handleClose1);

  server.on("/open2", handleOpen2);
  server.on("/close2", handleClose2);


  server.on("/buzzerOn", handleBuzzerOn);
  server.on("/buzzerOff", handleBuzzerOff);


  server.on("/redOn", handleRedOn);
  server.on("/redOff", handleRedOff);


  server.on("/resetError", handleResetError);

  server.on("/resetESP", handleResetESP);


  server.begin();

  Serial.println("WEB SERVER START");
}
//======================================================
// LOOP
//======================================================

void loop()
{
  server.handleClient();


  int ir1 = digitalRead(IR1);
  int ir2 = digitalRead(IR2);

  updateLimitSwitches();

  if(ir1 == LOW)
  {
    if(ir1DetectStart == 0)
    {
      ir1DetectStart = millis();
    }
  }
  else
  {
    ir1DetectStart = 0;
  }

  bool ir1Detected = (ir1DetectStart != 0 &&
                      millis() - ir1DetectStart >= IR1_DETECT_CONFIRM_MS);


  //====================================================
  // MANUAL RED BLINK
  //====================================================

  if(systemError)
  {
    setFailSafeOutputs();
  }
  else if(manualRedBlink)
  {
    redBlink();
  }


  //====================================================
  // KHI LỖI -> CHỈ CHO PHÉP MANUAL
  //====================================================

  if(mode == MANUAL_MODE)
  {
    return;
  }


  //====================================================
  // KIỂM TRA CÁC LỖI
  //====================================================

  checkBothIR(ir1, ir2);

  checkIRToggle(ir1, ir2);

  checkIRActive(ir1, ir2);


  // Sau khi check lỗi có thể đã chuyển Manual
  if(mode == MANUAL_MODE)
  {
    return;
  }


  //====================================================
  // PHÁT HIỆN TÀU ĐẾN IR1
  //====================================================

  if(trainState == NO_TRAIN &&
     ir1Detected)
  {
    trainState = TRAIN_COMING;

    trainDetectTime = millis();
    trainArrivalEpoch = time(NULL);

    Serial.println("TRAIN DETECTED IR1");
    // start warning and close immediately
    digitalWrite(BUZZER, HIGH);
    redBlink();

    if(barrier1State != BARRIER_CLOSED || barrier2State != BARRIER_CLOSED)
    {
      closeBarrier();
    }

    // cancel any pending delayed close
    pendingClose = false;
    pendingCloseStart = 0;
  }


  //====================================================
  // TÀU ĐANG ĐẾN
  //====================================================

  if(trainState == TRAIN_COMING)
  {
    // Bật cảnh báo ngay lập tức
    digitalWrite(BUZZER, HIGH);

    redBlink();


    // Nếu barrier chưa đóng (vì lý do nào đó), cố đóng ngay
    if(barrier1State != BARRIER_CLOSED || barrier2State != BARRIER_CLOSED)
    {
      closeBarrier();
    }


    // Kiểm tra limit
    checkLimit();


    // Tàu tới IR2 khi chưa đóng xong
    checkTrainSafety(ir2);


    // Nếu không có IR2 trong thời gian chờ, hủy báo động và trả về trạng thái an toàn
    if(ir2 == HIGH && millis() - trainDetectTime >= TRAIN_ABORT_TIMEOUT_MS)
    {
      Serial.println("TRAIN DETECTION TIMEOUT, RETURN TO NO_TRAIN");
      warningOff();
      openBarrier();
      trainState = NO_TRAIN;
    }

    // Đã tới IR2
    if(ir2 == LOW &&
       barrier1State == BARRIER_CLOSED &&
       barrier2State == BARRIER_CLOSED)
    {
      trainState = TRAIN_PASSING;

      Serial.println("TRAIN PASSING");
    }
  }


  //====================================================
  // TÀU ĐANG QUA
  //====================================================

  if(trainState == TRAIN_PASSING)
  {
    digitalWrite(BUZZER, HIGH);

    redBlink();


    // Tàu rời IR2
    if(ir2 == HIGH)
    {
      trainState = TRAIN_LEAVING;

      trainLeaveTime = millis();
      time_t leaveEpoch = time(NULL);

      if(historyCount < HISTORY_MAX)
      {
        historyArrival[historyCount] = formatTimeString(trainArrivalEpoch);
        historyLeave[historyCount] = formatTimeString(leaveEpoch);
        historyCount++;
      }
      else
      {
        for(int k = 1; k < HISTORY_MAX; k++)
        {
          historyArrival[k - 1] = historyArrival[k];
          historyLeave[k - 1] = historyLeave[k];
        }
        historyArrival[HISTORY_MAX - 1] = formatTimeString(trainArrivalEpoch);
        historyLeave[HISTORY_MAX - 1] = formatTimeString(leaveEpoch);
      }

      Serial.println("TRAIN LEFT IR2");
    }
  }


  //====================================================
  // MỞ BARRIER KHI IR2 KHÔNG CÒN TÍN HIỆU
  //====================================================

  if(trainState == TRAIN_LEAVING)
  {
    digitalWrite(BUZZER, HIGH);

    redBlink();

    // Giữ trạng thái đóng cho tới khi IR2 hết tín hiệu (HIGH)
    if(ir2 == HIGH)
    {
      openBarrier();

      warningOff();

      trainState = NO_TRAIN;

      Serial.println("SYSTEM READY");
    }
  }

  delay(50);
}
