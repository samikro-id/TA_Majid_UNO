#include <ArduinoJson.h>        // ArduinoJson by Benoit Blanchon (5.13.5)

#include <Servo.h>
Servo servo;
#define SERVO_PIN       6
#define SERVO_OPEN      180
#define SERVO_CLOSE     0
bool servo_open         = false;

#define LIMIT_C_PIN     8
#define LIMIT_O_PIN     9

#define TRIG_PIN        3
#define ECHO_PIN        2

#define VOLTAGE_PIN     A0
#define CURRENT_PIN     A1

typedef enum RelayState {
    RELAY_OFF,
    RELAY_RIGHT,
    RELAY_LEFT
};
#define RELAY1_PIN      4
#define RELAY2_PIN      5
#define RELAY_DELAY     3000
RelayState relay_state  = RELAY_OFF;
uint32_t relay_time;

uint32_t led_time;
bool led_state          = false;

#define SENSOR_LOOP     200

#define VOLT_FULL       13.5    // V
#define VOLT_EMPTY      10.5    // V
float volt;

#define CURRENT_SENSITIVITY    0.185   // V tergantung sensor arus yang digunakan, ini 5A
#define CURRENT_OFFSET         2.5     // V
float current;

#define ENERGY_FULL     90
#define ENERGY_EMPTY    0
float energy = 0;

uint32_t sensor_time;

#define SERIAL_BAUD     9600
#define SERIAL_LEN      400
String serial_buff;
bool serial_complete    = false;

#define WATER_LIMIT     15      // cm
#define WATER_RELEASE   10      // cm
int distance;

void setup(){
    delay(100);

    Serial.begin(SERIAL_BAUD);
    ioInit();

    serial_buff.reserve(SERIAL_LEN);

    sensor_time = millis();
    led_time = millis();
}

void loop(){
    if(relay_state != RELAY_OFF){
        if(relay_state == RELAY_RIGHT && digitalRead(LIMIT_O_PIN) == LOW){
            relayState(RELAY_OFF);
        }

        if(relay_state == RELAY_LEFT && digitalRead(LIMIT_C_PIN) == LOW){
            relayState(RELAY_OFF);
        }
    }

    if((millis() - sensor_time) > 1000){
        bacaSensor();
        sensor_time = millis();
    }

    if((millis() - led_time) > 200){
        toggleLed();
        led_time - millis();
    }

    if(serial_complete){
        prosessData();

        serial_complete = false;
        serial_buff = "";
    }
}

void bacaSensor(){
    float pinvolt=0;
    float pinArus=0;

    for(uint8_t n=0; n<SENSOR_LOOP; n++){
        uint16_t voltRaw = analogRead(VOLTAGE_PIN);      
        pinvolt += (float) (5.0 * voltRaw) / 1024.0;  

        uint16_t arusRaw = analogRead(CURRENT_PIN);
        pinArus += (float) (5.0 * arusRaw) / 1024.0;

        delay(10);  
    }
    /* BACA TEGANGAN */   
    volt = (float) (pinvolt / SENSOR_LOOP) * 5;

    /* BACA arus */
    current = (float) ((pinArus / SENSOR_LOOP) - CURRENT_OFFSET) / CURRENT_SENSITIVITY;

    /* HITUNG ENERGY */
    energy += float((volt * current) / 3600);        // 3600 detik dalam 1 jam
    if(energy > ENERGY_FULL){    energy = ENERGY_FULL;  }
    if(energy <= ENERGY_EMPTY){  energy = ENERGY_EMPTY; }

    if(volt >= VOLT_FULL){    energy = ENERGY_FULL;   }
    if(volt <= VOLT_EMPTY){   energy = ENERGY_EMPTY;  }

    // Clears the TRIG_PIN condition
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH, 1000000);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

    if(distance >= WATER_LIMIT){
        servo.write(SERVO_OPEN);
        servo_open = true;
    }

    if(distance <= WATER_RELEASE){
        servo.write(SERVO_CLOSE);
        servo_open = false;
    }
}

void prosessData(){
    StaticJsonBuffer<SERIAL_LEN> JSONBuffer;

    JsonObject& root = JSONBuffer.parseObject(serial_buff);
    if(root.success()){
        const char * op = root["op"];
        
        if(strcmp(op, "data") == 0){
            const char * cmd = root["cmd"];

            if(strcmp(cmd, "get") == 0){
                serial_buff = "";
                serial_buff = "{\"op\":\"data\",\"tegangan\":" + String(volt, 1) +",\"arus\":"+ String(current, 3) + ",\"energy\":" + String(energy, 2) + ",\"distance\":"+ String(distance) + ",";
                
                switch(relay_state){
                    case RELAY_OFF      :   serial_buff += "\"relay\":\"OFF\"";
                        break;
                    case RELAY_RIGHT    :   serial_buff += "\"relay\":\"OPEN\"";
                        break;
                    case RELAY_LEFT     :   serial_buff += "\"relay\":\"CLOSE\"";
                        break;
                    default :   serial_buff += "\"relay\":\"OFF\"";
                        break;
                };

                switch(servo_open){
                    case true       :   serial_buff += ",\"servo\":\"OPEN\""; 
                        break;
                    case false      :   serial_buff += ",\"servo\":\"CLOSE\"";
                        break;
                    default         :   serial_buff += ",\"servo\":\"CLOSE\"";
                        break;
                }
                
                serial_buff += "}";

                Serial.print(serial_buff);
            }
        }
        else if(strcmp(op, "control") == 0){
            const char * cmd = root["cmd"];

            if(strcmp(cmd, "set") == 0){
                const char* relay = root["relay"];

                if(strcmp(relay, "OFF") == 0){          relayState(RELAY_OFF);  }
                else if(strcmp(relay, "OPEN") == 0){    relayState(RELAY_RIGHT);}
                else if(strcmp(relay, "CLOSE") == 0) {  relayState(RELAY_LEFT); }

                serial_buff = "";
                serial_buff = "{\"op\":\"data\",\"tegangan\":" + String(volt, 1) +",\"arus\":"+ String(current, 3) + ",\"energy\":" + String(energy, 2) + ",\"distance\":"+ String(distance) + ",";
                
                switch(relay_state){
                    case RELAY_OFF      :   serial_buff += "\"relay\":\"OFF\"";
                        break;
                    case RELAY_RIGHT    :   serial_buff += "\"relay\":\"OPEN\"";
                        break;
                    case RELAY_LEFT     :   serial_buff += "\"relay\":\"CLOSE\"";
                        break;
                    default :   serial_buff += "\"relay\":\"OFF\"";
                        break;
                }

                switch(servo_open){
                    case true       :   serial_buff += ",\"servo\":\"OPEN\""; 
                        break;
                    case false      :   serial_buff += ",\"servo\":\"CLOSE\"";
                        break;
                    default         :   serial_buff += ",\"servo\":\"CLOSE\"";
                        break;
                }

                serial_buff += "}";

                Serial.print(serial_buff);
            }
        }
    }
}

/******* Serial Interrupt Event Callback ********/
void serialEvent(){
    while(Serial.available()){
        char inChar = (char) Serial.read();
        if(inChar == '\n'){
            serial_complete = true;
        }else if(inChar == '\r'){
            // do nothing   
        }else{
            if(!serial_complete){
                serial_buff += inChar;
            }
        }
    }
}

/****** Relay function*******/
void relayState(RelayState state){
    if(state != relay_state){
        switch(state){
            case RELAY_OFF      :   digitalWrite(RELAY1_PIN, HIGH);
                                    digitalWrite(RELAY2_PIN, HIGH);
                break;
            case RELAY_RIGHT    :   if(digitalRead(LIMIT_O_PIN) == HIGH){
                                        digitalWrite(RELAY1_PIN, HIGH);
                                        digitalWrite(RELAY2_PIN, LOW);  
                                    }
                break;
            case RELAY_LEFT     :   if(digitalRead(LIMIT_C_PIN) == HIGH){
                                        digitalWrite(RELAY1_PIN, LOW);
                                        digitalWrite(RELAY2_PIN, HIGH);
                                    }
                break;
            default :
                break;
        }
    }
    
    relay_state = state;
}

/****** LED function ******/
void ledState(bool state){
  if(state){  digitalWrite(LED_BUILTIN, HIGH);  }
  else{       digitalWrite(LED_BUILTIN, LOW);   }
  led_state = state;
}

void toggleLed(){
  if(led_state){
    digitalWrite(LED_BUILTIN, LOW);
    led_state = false;
  }else{
    digitalWrite(LED_BUILTIN, HIGH);
    led_state = true;
  }
}

void ioInit(){
    pinMode(RELAY1_PIN, OUTPUT);
    digitalWrite(RELAY1_PIN, HIGH);
    pinMode(RELAY2_PIN, OUTPUT);
    digitalWrite(RELAY2_PIN, HIGH);
    relay_state = RELAY_OFF;

    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(LIMIT_C_PIN, INPUT_PULLUP);
    pinMode(LIMIT_O_PIN, INPUT_PULLUP);

    led_state = false;

    servo.attach(SERVO_PIN);
}

/*  command
 *  get data
 *  {"op":"data","cmd":"get"}
 * 
 *  control relay
 *  {"op":"control","cmd":"set","relay":"OFF"}
 *  {"op":"control","cmd":"set","relay":"OPEN"}
 *  {"op":"control","cmd":"set","relay":"CLOSE"}
 *  
 *  https://api.thingspeak.com/channels/1370408/charts/1?yaxismin=-2&yaxismax=2&days=1&height=0&width=0
 *  https://api.thingspeak.com/channels/1370408/charts/2?yaxismin=0&days=1&height=0&width=0
 *  https://api.thingspeak.com/channels/1370408/charts/3?yaxismin=0&days=1&height=0&width=0
 */