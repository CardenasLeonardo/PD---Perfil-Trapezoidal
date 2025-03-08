/*
 * Observaciones:
 * La unidad de tiempo en los procesos del codigo son ms
 * La unidad de desplazamiento son pulsos
 * La unidad de velocidad son pulsos/Ts = pulsos/50 ms y puede convertirse a pulsos/ms al multiplicar la tasa resultate por Ts.
 *  Esto involucra varias consideraciones
 *  La diferencia entre la posicion actual con la anterior representa pulsos/ms, debe multiplicarse por 1000 para tener pulsos/s
 *  Esto en parte explicar porque el calculo de la parte derivativa del control funciona con una resta simplemente, resulta interesante
 *  considerar Ts y la conversion.
 * 
 *  Para lograr estose debe considerar el delay al final del void loop, encargado del retarde en cada iteracion, una consideracion es 
 *  implementar el un condicional con millis para lograr el mismo cometido sin las implicaciones de un delay.
 *  El delay usa ms de forma predeterminada, por conveniencia esto rige las unidades de tiempo.
 *  El Ts no esta presente en el calculo de la parte derivativa, y es absorbido por las constantes Kp y Kd.
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
const float fc = 1000;   // Factor de conversion de ms a s
const float cv = fc/(2*Ts);


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

    // Cálculo de la velocidad sin Filtrar
    float vel = pos_act - pos;            //vel = pulsos/ms
    //float vel = (pos_act - pos)*fc;       //vel = pulsos/s
    
    
    // Derivada numérica (error por segundo)
    float dedt = (ekT - ekT_ant);  

    // Señal de control (PD)
    float mkT = Kp * ekT + Kd * dedt;
    
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
    Serial.print("\t Vel p/s:");
    Serial.println(vel*cv); // operacion para mejorar la escala
   
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
