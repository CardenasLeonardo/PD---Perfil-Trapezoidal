/*
 * Observaciones:
 * Se muestra la grafica de la posicion en respuesta al perfil de velocidad trapezoidal como ejercicio de comparacion con respecto 
 *  al perfil de velocidad mostrado por el control.
 * En primera instancia se desea alcanzar un unico punto por lo que se coloca en el setup, para evolucionar con el igngreso de multiples puntos
 *  se debe tomar otra estrategia o modificar esta colocandola en el loop
 * 
 */
#define ENCA 2 // Amarillo
#define ENCB 3 // Morado
#define PWM1 5
#define PWM2 6

int pos = 0; 
volatile int pos_act = 0; // Variable compartida con la interrupción

int ref = 3000;           // En pulsos
float ekT_ant = 0;        // Error anterior 
float sum_ekT = 0;        // Suma del error 

// *********** Parametros del PD *************
float Kp = 1.0;           // Constante proporcional
float Kd = 0.5;  
const float Ts = 50;    // Periodo de muestreo (50 ms)
const float fc = 1000;   // Factor de conversion de ms a s
const float cv = fc/(2*Ts); // Factor de conversion para escalar vista de velocidad.


// *********** Parametros del filtro de la velocidad *************
#define WINDOW_SIZE 5  // Tamaño de la ventana para el filtro de media móvil
float vel_history[WINDOW_SIZE];  // Arreglo para almacenar las últimas velocidades
int vel_index = 0;               // Índice para acceder a las velocidades más recientes

// *********** Parametros del perfil trapezoidal *************
float vmax = 1000; // Velocidad máxima en pulsos/s
float acc = 500;   // Aceleración en pulsos/s^2
float t_acc, t_const, t_total;

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

    // Asegurar que las interrupciones estén activadas desde el inicio
    noInterrupts();  // Desactivar interrupciones al inicio si es necesario
    interrupts();    // Activar interrupciones después de la configuración

    //Calcular tiempos del perfil
    calcularPerfilTrapezoidal();
}

void loop()
{
    // Cálculo del error de posición
    int ekT = ref - pos;

    // Cálculo de la velocidad sin Filtrar
    float vel = pos_act - pos;            //vel = pulsos/ms
    //float vel = (pos_act - pos)*fc;       //vel = pulsos/s
    
    // Aplicar filtro de media móvil
    vel_history[vel_index] = vel;  // Guardamos la nueva velocidad
    vel_index = (vel_index + 1) % WINDOW_SIZE;  // Incrementamos el índice, asegurándonos de que no se salga del rango del arreglo

    // Calcular la media de los valores en el arreglo
    float vel_filtered = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        vel_filtered += vel_history[i];
    }
    vel_filtered /= WINDOW_SIZE;  // Promedio de las últimas velocidades

    // Derivada numérica (error por segundo)
    float dedt = (ekT - ekT_ant);  

    // Señal de control (PD)
    float mkT = Kp * ekT + Kd * dedt;
    
    // Saturación del PWM
    float valpwm = fabs(mkT);
    if (valpwm > 255) valpwm = 255;

    // Aplicación al motor con cambio de giro
    (mkT > 0) ? sentidoAntiHor((byte) valpwm) : sentidoHor((byte) valpwm);

    // Perfil para comparar
    float tiempo = millis() / 1000.0;
    float vel_trapezoidal = 0;
    if (tiempo < t_acc) {
        vel_trapezoidal = acc * tiempo;
    } else if (tiempo < t_acc + t_const) {
        vel_trapezoidal = vmax;
    } else if (tiempo < t_total) {
        vel_trapezoidal = vmax - acc * (tiempo - t_acc - t_const);
    }

    

    // Envío de datos para depuración
    Serial.print("\t Pref:");Serial.print(ref);
    Serial.print("\t Pos:");Serial.print(pos);
    Serial.print("\t Vel p/s:");Serial.print(vel*cv); // operacion para mejorar la escala
    Serial.print("\t Vel filtrada p/s:");Serial.print(vel_filtered*cv); // operacion para mejorar la escala
    Serial.print("\t Vel trapezoidal:"); Serial.println(vel_trapezoidal);
    
   
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

void calcularPerfilTrapezoidal() {
    t_acc = vmax / acc;
    float x_acc = 0.5 * acc * t_acc * t_acc;
    float x_remain = ref - 2 * x_acc;
    t_const = x_remain / vmax;
    t_total = 2 * t_acc + t_const;
}
