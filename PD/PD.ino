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
    Serial.println(pos);
   
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
