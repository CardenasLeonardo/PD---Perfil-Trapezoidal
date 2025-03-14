#define ENCA 2 // Amarillo
#define ENCB 3 // Morado
#define PWM1 5
#define PWM2 6

int pos = 0; 
volatile int pos_act = 0; // Variable compartida con la interrupción

int ref = 2000;           // En pulsos
float ekT_ant = 0;        // Error anterior 

// *********** Parametros del PD *************
float Kp = 3.0;   // Reducir para que la posición no domine tanto
float Kd = 0.14;   // Agregar un poco de amortiguación
float Kv = 1.0;    // Aumentar para que la velocidad tenga más peso
const float Ts = 50;      // Periodo de muestreo (50 ms)
const float fc = 1000;    // Factor de conversion de ms a s
const float cv = fc/(4*Ts); // Factor de conversion para escalar vista de velocidad.

// *********** Parametros del filtro de la velocidad *************
#define WINDOW_SIZE 5  // Tamaño de la ventana para el filtro de media móvil
float vel_history[WINDOW_SIZE];  // Arreglo para almacenar las últimas velocidades
int vel_index = 0;               // Índice para acceder a las velocidades más recientes

// *********** Parámetros del perfil de velocidad trapezoidal *************
float vmax = 0.5;  // Velocidad máxima en pulsos/ms
float acc = 0.15;    // Aceleración en pulsos/ms^2
float t1, t2, t3;   // Tiempos de aceleración, velocidad constante y desaceleración
float pos_teorica = 0;
float vel_teorica = 0;
float tiempo_total = 0;
unsigned long t_inicio;

void calcularPerfilTrapezoidal() {
    t1 = vmax / acc;  // Tiempo para alcanzar vmax en ms
    t3 = t1;          // Tiempo de desaceleración simétrico en ms
    float d1 = 0.5 * acc * t1 * t1;  // Distancia recorrida en aceleración, medida en pulsos
    float d3 = d1;                   // Distancia en desaceleración
    float d2 = ref - (d1 + d3);       // Distancia en velocidad constante

    if (d2 < 0) {
        // Si no hay suficiente distancia para vmax, recalcular vmax
        vmax = sqrt(ref * acc);
        t1 = vmax / acc;
        t3 = t1;
        d1 = 0.5 * acc * t1 * t1;
        d3 = d1;
        d2 = 0;
    }

    t2 = d2 / vmax;  // Tiempo en velocidad constante
    tiempo_total = t1 + t2 + t3;
    t_inicio = millis();
}

void setup()
{
    Serial.begin(9600);
    
    // Configuración de pines de entrada (encoder)
    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);

    // Configuración de pines de salida (PWM)
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);

    // Se leerán los pulsos por medio de una interrupción
    attachInterrupt(digitalPinToInterrupt(ENCA), leePulsos, RISING);  
}

void loop()
{
    calcularPerfilTrapezoidal();  // Calcula los tiempos del perfil trapezoidal
  
    // Cálculo del error de posición
    int ekT = ref - pos;

    // Cálculo de la velocidad sin Filtrar
    float vel = pos_act - pos; // vel = pulsos/ms
    
    // Aplicar filtro de media móvil
    vel_history[vel_index] = vel;  // Guardamos la nueva velocidad
    vel_index = (vel_index + 1) % WINDOW_SIZE;  // Incrementamos el índice, asegurándonos de que no se salga del rango del arreglo

    // Calcular la media de los valores en el arreglo
    float vel_filtered = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        vel_filtered += vel_history[i];
    }
    vel_filtered /= WINDOW_SIZE;  // Promedio de las últimas velocidades

    // Generación del perfil trapezoidal
    float t_actual = (millis() - t_inicio);  // Tiempo en ms
    if (t_actual < t1) {
        vel_teorica = acc * t_actual;
    } else if (t_actual < (t1 + t2)) {
        vel_teorica = vmax;
    } else if (t_actual < tiempo_total) {
        vel_teorica = vmax - acc * (t_actual - (t1 + t2));
    } else {
        vel_teorica = 0;
    }

    // Integración de la velocidad teórica para obtener la posición teórica
    pos_teorica += vel_teorica * Ts;

    // Cálculo del error de velocidad
    float eV = vel_teorica - vel_filtered;

    // Derivada numérica (error por milisegundo)
    float dedt = (ekT - ekT_ant);  

    // Señal de control (PD)
    float mkT = Kp * ekT + Kd * dedt + Kv * eV;
    
    // Saturación del PWM
    float valpwm = fabs(mkT);
    if (valpwm > 255) valpwm = 255;

    // Aplicación al motor con cambio de giro
    (mkT > 0) ? sentidoAntiHor((byte) valpwm) : sentidoHor((byte) valpwm);

    // Envío de datos para depuración
    Serial.print("\t Pref:");
    Serial.print(ref);
    Serial.print("\t Pos:");
    Serial.print(pos);
    Serial.print("\t Pos Teorica:");
    Serial.print(pos_teorica);
    Serial.print("\t Vel p/s:");
    Serial.print(vel*cv); // Operación para mejorar la escala
    Serial.print("\t Vel filtrada p/s:");
    Serial.print(vel_filtered*cv); // Operación para mejorar la escala
    Serial.print("\t Vel teorica:");
    Serial.println(vel_teorica*cv);

    // Guardar valores previos
    pos = pos_act;
    ekT_ant = ekT;
    
    // Delay ajustado al periodo de muestreo
    delay(Ts);  // **Convertir Ts de segundos a milisegundos**
}

// Contador de pulsos del encoder
void leePulsos()
{
    int b = digitalRead(ENCB);
    if (b > 0) pos_act++;  // Incrementamos el contador de pulsos
    else pos_act--;        // Decrementamos el contador de pulsos
}

// El motor gira en sentido horario
void sentidoHor(byte vel)
{
    analogWrite(PWM1, 0);
    analogWrite(PWM2, vel);
}

// El motor gira en sentido antihorario
void sentidoAntiHor(byte vel)
{
    analogWrite(PWM1, vel);
    analogWrite(PWM2, 0);
}
