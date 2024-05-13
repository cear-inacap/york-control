/*  york_cfv2.ino
    Para el control del robot YORK con tarjeta de control CFV2.

    Autor: Claudio Morales Díaz @cmoralesd
    Código fuente e instrucciones de uso: http://github.com/cear-inacap/york-control 
    Versión: 1.0 - mayo de 2024

    La transacción de datos se realiza mediante protocolo Modbus-TCP sobre WiFi
    Requiere la librería modbus-esp8266 by Andre Sarmento Barbosa, versión 4.1.0
    https://github.com/emelianov/modbus-esp8266
    
*/

#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

/****** CONFIGURACION WIFI *********/
const char* ssid = "***wifi-ssid***";
const char* password = "***password***";

/**********************************/

// Bobinas (Coils)
#define   LED_COIL            10001
#define   BUZZER_COIL         10002

// Registos de propósito general (Holding Registers)
#define   VX_TARGET_VELOCITY    40001
#define   VY_TARGET_VELOCITY    40002
#define   OMEGA_R_TARGET_VELOCITY 40003
#define   M1_TARGET_VELOCITY    40101
#define   M2_TARGET_VELOCITY    40102
#define   M3_TARGET_VELOCITY    40103
#define   M4_TARGET_VELOCITY    40104

// Registros de entradas (input registers)


// Pines utilizados
#define   LED     2
#define   BUZZER  12
#define   ENABLE  23
#define   SHIFT   32
#define   LATCH   33
#define   DATA    25

// palabras clave
#define   PWM_RESOLUTION 100


// Crea un objeto ModbusIP
ModbusIP mb;

// Variables de sistema
hw_timer_t *pwm_timer = NULL;  // timer para generar PWM
int pwm_counter = 0;           // cuenta interna en el ciclo pwm
byte motors_out = B00000000;   // registro de activación de los motores
// valores pwm en rango 0-100, con signo para indicar dirección de movimiento.
int m1_pwm_value = 0;          
int m2_pwm_value = 0;
int m3_pwm_value = 0;
int m4_pwm_value = 0;
// para depuración
int init_time = 0;
int end_time = 0;

// Función callback para el timer_pwm.
// Genera pulsos PWM a los 4 motores y los envía mediante registro de desplazamiento.
void IRAM_ATTR send_pwm(){
  // prepara el registro de accionamiento de los motores
  motors_out = B00000000;
  if(m1_pwm_value > 0 & 100 - abs(m1_pwm_value) < pwm_counter) bitWrite(motors_out, 0, HIGH);
  if(m1_pwm_value < 0 & 100 - abs(m1_pwm_value) < pwm_counter) bitWrite(motors_out, 1, HIGH);
  if(m2_pwm_value > 0 & 100 - abs(m2_pwm_value) < pwm_counter) bitWrite(motors_out, 2, HIGH);
  if(m2_pwm_value < 0 & 100 - abs(m2_pwm_value) < pwm_counter) bitWrite(motors_out, 3, HIGH);
  if(m3_pwm_value > 0 & 100 - abs(m3_pwm_value) < pwm_counter) bitWrite(motors_out, 4, HIGH);
  if(m3_pwm_value < 0 & 100 - abs(m3_pwm_value) < pwm_counter) bitWrite(motors_out, 5, HIGH);
  if(m4_pwm_value > 0 & 100 - abs(m4_pwm_value) < pwm_counter) bitWrite(motors_out, 6, HIGH);
  if(m4_pwm_value < 0 & 100 - abs(m4_pwm_value) < pwm_counter) bitWrite(motors_out, 7, HIGH);
  
  // y envía los datos al registro de desplazamiento.
  shiftOut(DATA, SHIFT, MSBFIRST, motors_out);
  digitalWrite(LATCH, HIGH);
  digitalWrite(LATCH, LOW);

  // reinicia el ciclo
  if(pwm_counter < PWM_RESOLUTION) pwm_counter += 1; else pwm_counter = 0;
}

// activa el buzzer n veces
void beep(int n)
{
  for(int i = 0; i < n; i++)
  {
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
    delay(250);
  }
}

// funciones para gestionar la conexión WiFi
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Conexión exitosa al punto de acceso!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi conectada");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("");
  Serial.println("Desconectado del punto de acceso a WiFi");
  Serial.print("No se pudo conectar. Código de error: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("intentando conectar ...");
  WiFi.begin(ssid, password);
}

void setup() {
  Serial.begin(115200);   // Puerto serial se utiliza para desplegar mensajes de estado

  WiFi.disconnect(true);  // Borra la antigua configuración de conexión
  delay(500);             // espera a que reaccione el router

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
    
  Serial.println();
  Serial.println();
  Serial.println("Esperando conexión WiFi... ");

  // Inicializa el servidor modbus
  mb.server();
  mb.addCoil(LED_COIL);
  mb.addCoil(BUZZER_COIL);
  mb.addHreg(VX_TARGET_VELOCITY, 16384);
  mb.addHreg(VY_TARGET_VELOCITY, 16384);
  mb.addHreg(OMEGA_R_TARGET_VELOCITY, 16384);
  mb.addHreg(M1_TARGET_VELOCITY, 16384);
  mb.addHreg(M2_TARGET_VELOCITY, 16384);
  mb.addHreg(M3_TARGET_VELOCITY, 16384);
  mb.addHreg(M4_TARGET_VELOCITY, 16384);

  // configura pines GPIO
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(SHIFT, OUTPUT);
  pinMode(LATCH, OUTPUT);

  // inicia con motores deshabilitados
  digitalWrite(ENABLE, 1);

  // configura el timer para generar los pulsos pwm
  pwm_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pwm_timer, &send_pwm, true);
  timerAlarmWrite(pwm_timer, 50, true);
  timerAlarmEnable(pwm_timer);

}

void loop() {
  // verifica la conexión WiFi
  if (WiFi.status() == WL_CONNECTED) {
    // conectado! realiza las transacciones modbus
    mb.task();

    // procesa los datos
    digitalWrite(LED, mb.Coil(LED_COIL));
    //digitalWrite(BUZZER, mb.Coil(BUZZER_COIL));
    // actualiza el registro de velocidad de los motores
    m1_pwm_value = int((float(mb.Hreg(M1_TARGET_VELOCITY)) - 16384.0)/16384.0 * 100);
    m2_pwm_value = int((float(mb.Hreg(M2_TARGET_VELOCITY)) - 16384.0)/16384.0 * 100);
    m3_pwm_value = int((float(mb.Hreg(M3_TARGET_VELOCITY)) - 16384.0)/16384.0 * 100);
    m4_pwm_value = int((float(mb.Hreg(M4_TARGET_VELOCITY)) - 16384.0)/16384.0 * 100);
    digitalWrite(ENABLE, 1);    // habilita los motores


  }else{
    // desconectado! deshabilita los motores y espera por reconexión
    digitalWrite(ENABLE, 0);
    Serial.print('se perdió la conexión WiFi! intentando reconectar ...');
    delay(1000);
    beep(3);
  }

}