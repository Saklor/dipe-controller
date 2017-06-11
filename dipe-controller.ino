#include <UTFT.h>
#include <Wire.h>
#include <TaskScheduler.h>

#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define SSerialTxControl 2   //RS485 Direction control

// LEDS
#define LED_ARMADO      8
#define LED_PRESION     9
#define LED_ZONA        10
#define LED_BATERIA     11
#define LED_CA          12
#define LED_ACTIVADO    13

#define LED_AMOUNT      6

// PINES EXTRA
#define BUZZER              3
#define CA_220              4
#define RELE_BATTERY        4 // Aca se escribe HIGH para chequear el estado de bateria
#define RELE_TRIGGER        5 // Este es el disparador de los tanques
#define TRIGGER             6
// PINES IDENTIFICACION HARDWARE
#define PIN_IS_CENTRAL      7
#define PIN_ID_DIPE_H       8
#define PIN_ID_DIPE_L       9

// PANTALLA
#define SIPE_1_FILA     42
#define SIPE_2_FILA     90
#define SIPE_3_FILA     138
#define SIPE_4_FILA     186
#define LOCAL_FILA      234
#define ACT_REMOTA_FILA 282

// BATERIA
#define DELAY_TEST_BATERIA  7200000 // 2 horas

// BITS COMUNICACION
#define BIT_ID_H            7 // High bit del ID
#define BIT_ID_L            6 // Low bit del ID
#define BIT_MSG_CENTRAL     5 // Mensaje de central. 1 = Central, 0 = Dipe
#define BIT_COD_220         4 // Fallo de 220
#define BIT_COD_BATTERY     3 // Fallo de Bateria
#define BIT_COD_AREA        2 // Fallo de Area
#define BIT_COD_PRESSURE    1 // Fallo de Presion
#define BIT_COD_TRIGGERED   0 // Activado

// STATUS
#define STATUS_BATTERY      0
#define STATUS_CA           1
#define STATUS_PRESSURE_1   2
#define STATUS_AREA_1       3
#define STATUS_PRESSURE_2   4
#define STATUS_AREA_2       5
#define STATUS_PRESSURE_3   6
#define STATUS_AREA_3       7
#define STATUS_TRIGGERED    8

#define STATUS_AMOUNT       9

// SENSORES
#define SENSOR_GAS_1        A0
#define SENSOR_GAS_2        A1
#define SENSOR_GAS_3        A2
#define SENSOR_BATTERY      A11
#define SENSOR_CA           A3

#define SENSOR_GAS_1_ID     0
#define SENSOR_GAS_2_ID     1
#define SENSOR_GAS_3_ID     2
#define SENSOR_BATTERY_ID   3
#define SENSOR_CA_ID        4

// MINIMA VARIACION PARA ACTUALIZAR STATUS
#define MIN_VARIATION       5

#define SENSOR_AMOUNT       5

int led_array[LED_AMOUNT] = { LED_ARMADO, LED_PRESION, LED_ZONA, LED_BATERIA, LED_CA, LED_ACTIVADO };

bool se_disparo = false;

// Identificacion de Hardware
bool is_central = false;
uint8_t local_dipe_id = 0;

// Declare which fonts we will be using
extern uint8_t Grotesk16x32[];
extern uint8_t Grotesk24x48[];

// Este status es el que se manda via Serial1 a central
uint8_t status;

// Este status mantiene registro de la situacion.
// False es todo ok, True es hay error
bool local_status[STATUS_AMOUNT];
bool previous_local_status[STATUS_AMOUNT];
bool first_status_update = true;

// Valores de sensores y valores de sensores en previas lecturas.
const int sensor_array[SENSOR_AMOUNT] = { SENSOR_GAS_1, SENSOR_GAS_2, SENSOR_GAS_3, SENSOR_BATTERY, SENSOR_CA };
int sensor_value[SENSOR_AMOUNT];
int previous_sensor_value[SENSOR_AMOUNT];

// Scheduler
Scheduler runner;

// Callback a funciones de tasks
void testBatteryCallback();
void turnBuzzerOffCallback();

// Tasks
Task testBatteryTask(DELAY_TEST_BATERIA, TASK_FOREVER, &testBatteryCallback);
Task turnBuzzerOffTask(0, TASK_ONCE, &turnBuzzerOffCallback);


UTFT screen(ILI9481,38,39,40,41);

void setup() {
    // Aca comienza la inicializacion que se comparte entre DIPE y CENTRAL

    // Inicializo Seriales
    Serial.begin(9600);
    Serial1.begin(9600);

    // Inicializo TxControl
    pinMode(SSerialTxControl, OUTPUT);
    digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver

    // Inicializo pines de LED
    pinMode(LED_ARMADO,     OUTPUT);    // led Armado
    pinMode(LED_PRESION,    OUTPUT);    // led presión
    pinMode(LED_ZONA,       OUTPUT);    // led zona
    pinMode(LED_BATERIA,    OUTPUT);    // led bateria
    pinMode(LED_CA,         OUTPUT);    // led ca
    pinMode(LED_ACTIVADO,   OUTPUT);    // led activado

    // Inicializo pines extra
    pinMode(BUZZER,         OUTPUT);    // Buzzer
    pinMode(TRIGGER,        INPUT);     // Pulsador disparo
    
    pinMode(PIN_IS_CENTRAL, INPUT);
    pinMode(PIN_ID_DIPE_L,  INPUT);
    pinMode(PIN_ID_DIPE_H,  INPUT);

    // Inicializo pantalla
    screen.InitLCD();
    screen.setBackColor(4, 13, 33);
    screen.fillScr(4, 13, 33);
    screen.setColor(240, 225, 150);

    // Inicializo scheduler
    runner.init();

    screen.setFont(Grotesk16x32);
    screen.print("INICIALIZANDO CONTROLADOR...", CENTER, 10);
    delay(750);

    if (digitalRead(PIN_IS_CENTRAL) == HIGH) {
        is_central = true;
        screen.print("SELECIONADA CENTRAL", CENTER, 58);
    }
    else {
        is_central = false;
        screen.print("SELECIONADO SIPE", CENTER, 58);
        delay(750);
        local_dipe_id = 1;
        if (digitalRead(PIN_ID_DIPE_L) == HIGH) local_dipe_id += 1;
        if (digitalRead(PIN_ID_DIPE_H) == HIGH) local_dipe_id += 2;
        screen.print("SIPE ID:", CENTER, 106);
        screen.printNumI(local_dipe_id, CENTER, 154);
    }

    delay(5000);

    screen.fillScr(4, 13, 33);

    if (is_central)
        setupCentral();
    else
        setupDipe();
}


void setupCentral() {
    // Seteo pines extra especificos central
    pinMode(CA_220,         INPUT);     // Entrada CA 220

    // Relleno pantalla con info estatica.
    screen.setColor(118, 151, 226);
    screen.setFont(Grotesk16x32);
    screen.print("CONTROLE CENTRAL",CENTER, 10);
    screen.setColor(240, 225, 150);
    screen.setFont(Grotesk24x48);
    screen.print("SIPE 1", 10, SIPE_1_FILA);
    screen.print("SIPE 2", 10, SIPE_2_FILA);
    screen.print("SIPE 3", 10, SIPE_3_FILA);
    screen.print("SIPE 4", 10, SIPE_4_FILA);
    screen.print("LOCAL ", 10, LOCAL_FILA);
}


void setupDipe() {
    screen.setFont(Grotesk24x48);

    // Seteo pines extra especificos dipe
    pinMode(RELE_BATTERY,   OUTPUT);//rele medir bateria
    pinMode(RELE_TRIGGER,   OUTPUT);//rele salida EV

    // Inicializo status local en false (sin errores)
    for (int i = 0; i < STATUS_AMOUNT; i++) {
        local_status[i] = false;
        previous_local_status[i] = false;
    }

    // Inicializo valores de sensores y anteriores (en -1 asi puedo identificar que nunca los lei)
    for (int i = 0; i < SENSOR_AMOUNT; i++) {
        sensor_value[i] = 0;
        previous_sensor_value[i] = -1;
    }

    // Inicializo status
    status = 0;
    if (local_dipe_id > 2)
        bitSet(status, BIT_ID_H);
    if (local_dipe_id == 2 || local_dipe_id == 4)
        bitSet(status, BIT_ID_L);
    bitSet(status, BIT_MSG_CENTRAL);

    // Hago el enable de las tasks que tienen que empezar a ejecutarse
    runner.addTask(testBatteryTask);
    testBatteryTask.enable();
}


void loop() {
    // Ejecuto el scheduler para ver si hay alguna tarea pendiente
    runner.execute();


    Serial1.flush();

    if (is_central)
        loopCentral();
    else
        loopDipe();

    if (Serial1.available() > 0)
        handleByteReceived(Serial1.read());
}


void loopCentral() {
    int value;
    int byte_send;
    value = digitalRead(5);


    if (value == HIGH && !se_disparo) {
        byte_send = 88;
        screen.setColor(250, 35, 35);
        screen.setFont(Grotesk16x32);
        screen.print("ATIVACAO REMOTA SIPES ",CENTER, ACT_REMOTA_FILA);
        screen.setFont(Grotesk24x48);

        delay(500);
        Serial.write("["); Serial.write(byte_send); Serial.write("           se fue voló]");
        digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit

        Serial1.write(byte_send);                        // Send byte to Remote Arduino
        delay(10);
        Serial1.write(byte_send);                        // Send byte to Remote Arduino
        delay(10);

        digitalWrite(SSerialTxControl, RS485Receive);  // Disable RS485 Transmit
        digitalWrite(3,HIGH);
        
        se_disparo = true;
    }


    value = digitalRead(4);

    if ( value == LOW ) {
        screen.setColor(250, 35, 35);
        screen.print("FALHA CA 220V ",170, LOCAL_FILA);

        digitalWrite(3,HIGH);
        delay(500);
        digitalWrite(3,LOW);
        delay(500);
    } else {
        screen.setColor(152, 252, 106);
        screen.print("OK            ",170, LOCAL_FILA);
    }
}


void loopDipe() {
    int manual_trigger;

    manual_trigger = digitalRead(9);
    if (manual_trigger == HIGH) {
        // Disparo manual
        screen.clrScr();

        for (int i = 30; i >= 0; i--) {
            digitalWrite(BUZZER, HIGH);
            delay(300);
            digitalWrite(BUZZER, LOW);
            screen.printNumI(i, CENTER, 136);
            delay(700);
        }
        triggerDipe();
    }

    checkSensors();

    communicateStatus();
}


void handleByteReceived(int byte_received) {
    // Estructura de byte:
    // ID_H            7 // High bit del ID
    // ID_L            6 // Low bit del ID
    // RES             5 // Reservado por ahora
    // COD_220         4 // Fallo de 220
    // COD_BAT         3 // Fallo de Bateria
    // COD_ARE         2 // Fallo de Area
    // COD_PRE         1 // Fallo de Presion
    // COD_ACT         0 // Activado
    int sipe_id = ((bitRead(byte_received, BIT_ID_H) * 2) + bitRead(byte_received, BIT_ID_L));

    bool central_msg =      bitRead(byte_received, BIT_MSG_CENTRAL);

    bool error_220 =        bitRead(byte_received, BIT_COD_220);
    bool error_battery =    bitRead(byte_received, BIT_COD_BATTERY);
    bool error_area =       bitRead(byte_received, BIT_COD_AREA);
    bool error_pressure =   bitRead(byte_received, BIT_COD_PRESSURE);
    bool triggered =        bitRead(byte_received, BIT_COD_TRIGGERED);

    if (is_central) {
        int fila = 0;

        // Me fijo en que fila tengo que imprimir el error, de paso chequeo que haya venido bien el ID
        switch(sipe_id) {
            case 0:
                fila = SIPE_1_FILA;
                break;
            case 1:
                fila = SIPE_2_FILA;
                break;
            case 2:
                fila = SIPE_3_FILA;
                break;
            case 3:
                fila = SIPE_4_FILA;
                break;
            default:
                Serial.print("Error leyendo ID de SIPE. Se obtuvo el valor ");
                Serial.println(sipe_id);
                return;
        }

        if (!error_220 && !error_battery && !error_area && !error_pressure && !triggered) {
            screen.setColor(152, 252, 106);
            screen.print("OK            ", 170, fila);
            turnLEDOn(LED_ARMADO);
            digitalWrite(3,LOW);
        } else {
            // Hay al menos un error
            screen.setColor(250, 35, 35);
            if (error_220) {
                screen.print("FALHA CA 220V ",170, fila);
                turnLEDOn(LED_CA);
            }
            if (error_battery) {
                screen.print("FALHA BATERIA ",170, fila);
                turnLEDOn(LED_BATERIA);
            }
            if (error_area) {
                screen.print("FALHA AREA    ",170, fila);
                turnLEDOn(LED_ZONA);
            }
            if (error_pressure) {
                screen.print("FALHA PRESSAO ",170, fila);
                turnLEDOn(LED_PRESION);
            }
            if (triggered) {
                screen.print("ATIVADO       ",170, fila);
                turnLEDOn(LED_ACTIVADO);
            }
            buzzerAlarm(true);
        }
    } else if (!is_central && sipe_id == local_dipe_id) {
        if (triggered){
            screen.clrScr();
            screen.print("REMOTO", CENTER, 10);

            for (int i = 30; i >= 0; i--) {
                digitalWrite(BUZZER, HIGH);
                delay(300);
                digitalWrite(BUZZER, LOW);
                screen.printNumI(i, CENTER, 136);
                delay(700);
            }

            triggerDipe();
        }
    }
}

void buzzerAlarm(bool on) {
    if (on) {
        for (int i = 0; i < 8; ++i) {
            digitalWrite(3,HIGH);
            delay(500);
            digitalWrite(3,LOW);
            delay(500);
        }
    } else
        digitalWrite(3, LOW);
}


void turnLEDOn(int led_pin) {

    digitalWrite(led_pin, HIGH);

    // Apago el resto
    for (int i = 0; i < LED_AMOUNT; ++i) {
        if (led_array[i] != led_pin)
            digitalWrite(led_array[i], LOW);
    }
}


void testBatteryCallback()
{
    Serial.println("Realizando test de bateria.");
    screen.print("TESTE DE BATERIA", CENTER, 10);

    digitalWrite(RELE_BATTERY, HIGH);
    sensor_value[SENSOR_BATTERY_ID] = 100;

    delay(100); // TODO: Chequear si es necesario

    sensor_value[SENSOR_BATTERY_ID] = analogRead(SENSOR_BATTERY);
    digitalWrite(RELE_BATTERY,LOW);

    if (sensor_value[SENSOR_BATTERY_ID] <=450) {
        digitalWrite(BUZZER, HIGH);
        delay(500);
        digitalWrite(BUZZER, LOW);

        if (valueChanged(previous_sensor_value[SENSOR_BATTERY_ID], sensor_value[SENSOR_BATTERY_ID])) {
            screen.print("    FALHA BATERIA    ", CENTER, 10);
            previous_sensor_value[SENSOR_BATTERY_ID] = sensor_value[SENSOR_BATTERY_ID];

            updateStatus(STATUS_BATTERY, true);
        }
    } else {
        updateStatus(STATUS_BATTERY, false);
    }

    Serial.println("Test de bateria completado.");
}


void switchLED(int led_pin, bool on) {
    if (on)
        digitalWrite(led_pin, HIGH);
    else
        digitalWrite(led_pin, LOW);
}

void triggerDipe() {
    screen.clrScr();
    screen.print("ACTIVADO", CENTER, 136);

    updateStatus(STATUS_TRIGGERED, true);

    digitalWrite(BUZZER,        HIGH);
    digitalWrite(RELE_TRIGGER,  HIGH);

    delay(15000); // No tiene sentido dejarlo parado 15 segundos

    digitalWrite(BUZZER,        LOW);
    digitalWrite(RELE_TRIGGER,  LOW);
}


void checkSensors() {
    int id = 0;
    int id_pressure_status = 0;
    int id_area_status = 0;

    // Leo todos los sensores
    for (int i = 0; i < SENSOR_AMOUNT; ++i)
        sensor_value[i] = analogRead(sensor_array[i]);


    // Chequeo tanques
    for (int i = 0; i < 3; ++i) {
        if (i == 0) {
            id = SENSOR_GAS_1_ID;
            id_pressure_status = STATUS_PRESSURE_1;
            id_area_status = STATUS_PRESSURE_1;
        } else if (i == 1) {
            id = SENSOR_GAS_2_ID;
            id_pressure_status = STATUS_PRESSURE_2;
            id_area_status = STATUS_PRESSURE_2;
        } else if (i == 2) {
            id = SENSOR_GAS_3_ID;
            id_pressure_status = STATUS_PRESSURE_3;
            id_area_status = STATUS_PRESSURE_3;
        }

        if (sensor_value[id] >= 500 && sensor_value[id] <= 525) {
            // Si el valor esta entre 500 y 525 entonces no hay errores
            if (valueChanged(previous_sensor_value[id], sensor_value[id])) {
                previous_sensor_value[id] = sensor_value[id];

                updateStatus(id_pressure_status,   false);
                updateStatus(id_area_status,       false);
            }
        } else if (sensor_value[id] <= 25) {
            // Si el valor esta por debajo de 25 entonces falla presion
            buzzOnce(500);

            if (valueChanged(previous_sensor_value[id], sensor_value[id])) {
                screen.print("PRESSAO   FALHA", CENTER, 10);
                screen.printNumI(i, CENTER, 10);
                previous_sensor_value[id] = sensor_value[id];

                updateStatus(id_pressure_status, true);
            }
        } else if (sensor_value[id] >= 980) {
            // Si el valor esta por encima de 980 entonces falla area
            buzzOnce(500);

            if (valueChanged(previous_sensor_value[id], sensor_value[id])) {
                screen.print("AREA   FALHA", CENTER, 10);
                screen.printNumI(i, CENTER, 10);
                previous_sensor_value[id] = sensor_value[id];

                updateStatus(id_area_status, true);
            }
        }
    }

    // Chequeo CA
    if (sensor_value[SENSOR_CA_ID] <= 800) {
        // Si el valor del sensor CA es menor a 800 falla 220
        buzzOnce(500);

        if (valueChanged(previous_sensor_value[SENSOR_CA_ID], sensor_value[SENSOR_CA_ID])) {
            screen.print("FALHA AC 220V", CENTER, 136);
            previous_sensor_value[SENSOR_CA_ID] = sensor_value[SENSOR_CA_ID];

            updateStatus(STATUS_CA, true);
        }
    } else
        updateStatus(STATUS_CA, false);
}


bool valueChanged(int previous_value, int current_value) {
    return (previous_value < 0 || abs(current_value - previous_value) > MIN_VARIATION);
}

void updateStatus(int id, bool error) {
    // Me guardo en el local_status el estado de 'id'
    local_status[id] = error;

    // Prendo o apago dependiendo si hubo o no error el led correspondiente
    switch (id) {
        case STATUS_BATTERY:
            switchLED(LED_BATERIA, error);
            break;
        case STATUS_CA:
            switchLED(LED_CA, error);
            break;
        case STATUS_PRESSURE_1:
        case STATUS_PRESSURE_2:
        case STATUS_PRESSURE_3:
            switchLED(LED_PRESION, error);
            break;
        case STATUS_AREA_1:
        case STATUS_AREA_2:
        case STATUS_AREA_3:
            switchLED(LED_ZONA, error);
            break;
        case STATUS_TRIGGERED:
            switchLED(LED_ACTIVADO, error);
            break;
    }
}

bool statusChanged() {
    // Si cambio algun estado tengo que mandar comms
    for (int i = 0; i < STATUS_AMOUNT; ++i) {
        if (local_status[i] != previous_local_status[i]) return true;
    }
}

void checkErrorAndSetBit(bool condition, int bit_id) {
    if (condition)
        bitSet(status, bit_id);
    else
        bitClear(status, bit_id);
}

void transmitByte(uint8_t byte) {
    digitalWrite(SSerialTxControl, RS485Transmit);
    Serial1.write(byte);
    Serial1.flush();
    digitalWrite(SSerialTxControl, RS485Receive);
}

void communicateStatus() {
    // Updateo status bit a bit en base al status actual
    checkErrorAndSetBit(local_status[STATUS_BATTERY], BIT_COD_BATTERY);
    checkErrorAndSetBit(local_status[STATUS_CA], BIT_COD_220);
    checkErrorAndSetBit(local_status[STATUS_TRIGGERED], BIT_COD_TRIGGERED);
    checkErrorAndSetBit(
        (local_status[STATUS_PRESSURE_1] ||
        local_status[STATUS_PRESSURE_2] ||
        local_status[STATUS_PRESSURE_3]),
        BIT_COD_PRESSURE);
    checkErrorAndSetBit(
        (local_status[STATUS_AREA_1] ||
        local_status[STATUS_AREA_2] ||
        local_status[STATUS_AREA_3]),
        BIT_COD_AREA);

    // Updateo status anterior al que voy a mandar
    for (int i = 0; i < STATUS_AMOUNT; ++i) previous_local_status[i] = local_status[i];

    // Envio status
    transmitByte(status);
}

void buzzOnce(int ms) {
    digitalWrite(BUZZER, HIGH);
    turnBuzzerOffTask.setInterval(ms);
    turnBuzzerOffTask.enableDelayed();
}

// Esta funcion tiene el unico proposito de apagar el buzzer.
// La idea es que se llame desde una alarma para poder seguir mientras suena el buzzer.
void turnBuzzerOffCallback() {
    digitalWrite(BUZZER, LOW);
}
