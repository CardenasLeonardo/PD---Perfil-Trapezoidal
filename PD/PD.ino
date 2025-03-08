/*
 *Tuneado con Kp=1 , Kd=0.5 , Ts =50
 *Observaciones: si se disminuye Ts, no alcanza ls referencia, si aumenta lo sobrepasa
 *Hipotesis: Se necesita demasiado error para mover el motor
 *  Un error pequeño se refleja en un pwm pequeño, pero no es suficiente para mover el motor
 *  Existe un rango de PWM donde el motor permanece inmovil, de tal forma que PWM < tolerancia = 0
 *  Incluir una correccion considerando un error incremental reduce el estado estacionario, se desea una respuesta  
 *    criticamente amortiguada, debe considerarse la parte I para resolverlo, esto tambien garantiza una solucion
 *    a errores donde se sobrepasa la referencia aunque generaria una respuesta subamortiguada (indeseada)
 *  Aumentar Kp vuelve mas agresivo el sistema pues un error pequeño provoca una correccion mayor y debe modificarse cada que se cambie el Ts, 
 *    haciendo esto no es necesario colocar la parte incremental del controlador.
 *  Incorporar la parte integral requeire incorporar una correcion al windup en cada cambio de signo del error
 */
#define ENCA 2 // Amarillo
#define ENCB 3 // Morado
#define PWM1 5
#define PWM2 6

int pos = 0; 
volatile int pos_act = 0; // Variable compartida con la interrupción

int ref = 800;           // En pulsos
float ekT_ant = 0;        // Error anterior 
float sum_ekT = 0;        // Suma del error 

// *********** Parametros del PD *************
float Kp = 1.0;           // Constante proporcional
float Kd = 0.5;  
/* Constante derivativa generalmente menor que 1 probar con la mitad, decima o quinta parte de Kp.
Incorporar filtro pasabajos, amortiguar el ruido antes de inyectarlo
promedio ponderado
Cosenos Senos?
medias deslizante
*/
const float Ts = 50;    // Periodo de muestreo (50 ms)

// *********** Parametros del Filtro de Media Movil *************
#define N 20              // Número de muestras para suavizar
float vel_buffer[N] = {0};  // Buffer circular de velocidades
int index = 0;            // Índice del buffer
float suma_vel = 0;       // Suma de las velocidades almacenadas

// Función para filtrar la velocidad en tiempo real
float filtrarVelocidad(float nueva_vel) {
    suma_vel -= vel_buffer[index];  // Restar la muestra más antigua
    vel_buffer[index] = nueva_vel;  // Almacenar la nueva velocidad
    suma_vel += nueva_vel;  // Sumar la nueva muestra

    index = (index + 1) % N;  //Avanzar el índice circular

    return suma_vel / N;  // Devolver la velocidad filtrada
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

    // Asegurar que las interrupciones estén activadas desde el inicio
    noInterrupts();  // Desactivar interrupciones al inicio si es necesario
    interrupts();    // Activar interrupciones después de la configuración
}

void loop()
{
    // Cálculo del error de posición
    int ekT = ref - pos;

    // Incremento de la posicion 
    //float delta_pos = pos_act - pos_ant;
    //float delta_t = t_act - t_ant;   
    /*
     * Entender velocidad como el incremento de la posicion con respecto al incremento del tiempo.
     * Incremento de la posicion = pos_act - pos_ant
     * Incremento del tiempo = Tiempo de ejecución de una iteración el ciclo loop conteniendo sus instrucciones
     * Incremento del tiempo = Diferencia del tiempo de ejecución (en constante aumento) entre lectura de posicion
     */
    
    // Aplicar el filtro de media móvil
    //float vel_act = filtrarVelocidad(vel_nueva);

    pos = pos_act;

    // Derivada numérica (error por segundo)
    float dedt = (ekT - ekT_ant);  

    // Señal de control (PD)
    float mkT = Kp * ekT + Kd * dedt;
    
    // Saturación del PWM
    float valpwm = fabs(mkT);
    if (valpwm > 255) valpwm = 255;

    // Aplicación al motor con cambio de giro
    (mkT > 0) ? sentidoAntiHor((byte) valpwm) : sentidoHor((byte) valpwm);


    // Guardar valores previos
    ekT_ant = ekT;

    // Envío de datos para depuración
    Serial.print("\t Pref:");
    Serial.print(ref);
    Serial.print("\t Pos:");
    Serial.println(pos);
    //Serial.print("\t Vel:");
    //Serial.println(vel_act); 

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
