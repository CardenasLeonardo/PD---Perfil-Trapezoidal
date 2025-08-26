#define ENCA 2 // Amarillo
#define ENCB 3 // Morado
#define PWM1 5
#define PWM2 6
int pos = 0; 
volatile int pos_act = 0; // Variable compartida con la interrupción
// *********** Parametros del PD *************
float Kp = 2.8;           // Constante proporcional
float Kd = 0.314;  

float ekT_ant = 0;        // Error anterior 
float sum_ekT = 0;        // Suma del error 

// Parámetros de muestreo (en microsegundos)
const unsigned long TS_PROFILE_US = 100; // 0.1 ms
const unsigned long TS_CONTROL_US = 0.1;  // 0.10 ms
const unsigned long TS_POT_US = 10000;  // 1 ms

// Variables de trayectoria y potenciómetro
float theta_0 = 0;   // Posición inicial
float theta_f[20];   // Array de puntos finales (theta_f) (ahora dinámico)
int n_puntos = 0;    // Número de puntos finales (theta_f), comenzamos con 0

// Variables para el perfil trapezoidal
float u_max   = 1000.0;   // Vel max (pulsos/s)
float alpha   = 1000.0;    // Aceleración (pulsos/s^2)
float t_a, t_c, t_d, T; 
float ref       = 0;  // Posición de referencia
float vel_ref   = 0;  // Velocidad de referencia
float accel_ref = 0;  // Aceleración de referencia
int   phase     = 0;  // Fase del perfil (1,2,3,4)

// Variables para medir tiempo en microsegundos
unsigned long startTime    = 0; 
unsigned long lastProfileTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastPotTime = 0;
int current_target = 0;  // Índice del punto actual en theta_f

// Filtro de Media Móvil
const int N = 5; // Tamaño del filtro (número de muestras a promediar)
float position_history[N];  // Historial de las últimas N posiciones
int pos_index = 0;  // Índice para el historial circular

void setup() {
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
  pinMode(A0, INPUT);  // Pin analógico para el potenciómetro

  // Inicializar el historial de posiciones a cero
  for (int i = 0; i < N; i++) {
    position_history[i] = 0;
  }

  // Guardamos tiempo inicial en microsegundos
  startTime       = micros();
  lastProfileTime = startTime;
  lastControlTime = startTime;
}

void loop() {
  unsigned long now = micros();
  calcularTiempos();

  // 1) Actualizar theta_f cada TS_POT_US
  if (now - lastPotTime >= TS_POT_US) {
    lastPotTime = now;
    actualizarPuntosFinales();  // Leer el potenciómetro y actualizar el array
  }
  
  // 2) Actualizar trayectoria cada TS_PROFILE_US
  if (now - lastProfileTime >= TS_PROFILE_US) {
    lastProfileTime = now;
    generarTrayectoria();  // Generación de trayectoria
  }

  // 3) Actualizar PWM cada TS_CONTROL_US
  if (now - lastControlTime >= TS_CONTROL_US) {
    lastControlTime = now;
    control();
  }

  // Mostrar datos por el puerto serie
  Serial.print((micros() - startTime) / 1000000.0); // tiempo (s)
  Serial.print(" ");
  Serial.print(ref);         // referencia de posición
  Serial.print(" ");
  Serial.print(pos);         // posición
  Serial.print(" ");
  Serial.print(vel_ref);     // velocidad de referencia
  Serial.print(" ");
  Serial.print(accel_ref);   // aceleración de referencia
  Serial.println();
}


// Genera la trayectoria para alcanzar los puntos finales
void generarTrayectoria() {
  // t en segundos desde el inicio
  float t = (micros() - startTime) / 1000000.0;

  // Verificar si el siguiente theta_f es menor que el anterior
  bool invertir_senales = (theta_f[current_target] < theta_0);

  // Fase 1: aceleración
  if (t < t_a) {
    if (invertir_senales) {
      ref = theta_0 - 0.5 * alpha * t * t;
    } else {
      ref = theta_0 + 0.5 * alpha * t * t;
    }
    vel_ref = alpha * t;
    accel_ref = alpha;
  }
  // Fase 2: velocidad constante
  else if (t < (t_a + t_c)) {
    if (invertir_senales) {
      ref = theta_0 - (0.5 * u_max * t_a) - u_max * (t - t_a);
    } else {
      ref = theta_0 + (0.5 * u_max * t_a) + u_max * (t - t_a);
    }
    vel_ref = u_max;
    accel_ref = 0;
  }
  // Fase 3: desaceleración
  else if (t < (t_a + t_c + t_d)) {
    float t_d_phase = t - (t_a + t_c);
    if (invertir_senales) {
      ref = theta_f[current_target] + 0.5 * alpha * (t_d - t_d_phase) * (t_d - t_d_phase);
    } else {
      ref = theta_f[current_target] - 0.5 * alpha * (t_d - t_d_phase) * (t_d - t_d_phase);
    }
    vel_ref = alpha * (t_d - t_d_phase);
    accel_ref = -alpha;
  }
  // Fase 4: reposo final
  else {
    ref = theta_f[current_target];
    vel_ref = 0;
    accel_ref = 0;
    t = 0;

    // Solo cambiamos de objetivo cuando la trayectoria ha finalizado completamente
    if (current_target < n_puntos - 1) {
      current_target++;  // Avanzar al siguiente punto
      theta_0 = theta_f[current_target - 1];  // La nueva theta_0 es el anterior theta_f
      startTime = micros();  // Reiniciar el tiempo
      calcularTiempos();  // Recalcular tiempos con los nuevos valores
    }

    // Si hemos llegado al último punto, reiniciar
    if (current_target == n_puntos-1) {
      current_target = 0;   
    }
  }
}


// Calcula los tiempos para el perfil trapezoidal
void calcularTiempos() {
  // Calculamos el tiempo de aceleración, velocidad constante y desaceleración
  t_a = u_max / alpha;
  t_c = (theta_f[current_target] - theta_0 - (u_max * t_a)) / u_max;
  t_d = u_max / alpha;

  // Si t_c < 0 => el perfil es triangular
  if (t_c < 0) {
    t_a = sqrt(abs(theta_f[current_target] - theta_0) / alpha);
    t_d = t_a;
    t_c = 0;
  }
}


// Lee el valor del potenciómetro y actualiza el array de puntos finales
void actualizarPuntosFinales() {
  // Lee el valor del potenciómetro (de 0 a 1023)
  int potValue = analogRead(A0);

  // Mapea el valor del potenciómetro a un rango que tenga sentido (por ejemplo, de 0 a 2000)
  float mappedValue = map(potValue, 0, 1023, 0, 2000);

  // Evitar sobrescribir puntos sin necesidad
  if (n_puntos < 20) {
    // Si el valor mapeado no es igual al último valor agregado (evitar duplicados)
    if (n_puntos == 0 || mappedValue != theta_f[n_puntos - 1]) {
      theta_f[n_puntos] = mappedValue;
      n_puntos++;  // Incrementa el número de puntos
    }
  } else {
    // Si ya hay 20 puntos, se elimina el primero (desplaza los elementos del vector)
    for (int i = 0; i < 19; i++) {
      theta_f[i] = theta_f[i + 1];
    }
    theta_f[19] = mappedValue; // Añadimos el nuevo valor al final
  }

  // Si el número de puntos es 21, lo reiniciamos para empezar desde el principio
  if (n_puntos == 21) {
    n_puntos = 0;
  }

  // Aplicar el filtro de media móvil en la posición
  aplicarFiltroMediaMovil(mappedValue);
}

void aplicarFiltroMediaMovil(float nueva_posicion) {
  // Añadir el nuevo valor de la posición al historial
  position_history[pos_index] = nueva_posicion;

  // Mover el índice del historial de manera circular
  pos_index = (pos_index + 1) % N;

  // Calcular el promedio de las últimas N posiciones
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += position_history[i];
  }

  // El valor filtrado es el promedio
  ref = sum / N;
}

void control(){
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
  // Guardar valores previos
    pos = pos_act;
    ekT_ant = ekT;
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
