/*****************************************************/
/*                Variables globales                 */
/*****************************************************/
// Parámetros de muestreo (en microsegundos)
const unsigned long TS_PROFILE_US = 100; // 0.1 ms
const unsigned long TS_CONTROL_US = 100;  // 0.10 ms

// Variables para el perfil trapezoidal
float theta_0 = 0;   // Posición inicial
float theta_f = 0;   // Posición final (leída desde el pot)
float u_max   = 1000.0;   // Vel max (pulsos/s)
float alpha   = 350.0;    // Aceleración (pulsos/s^2)
float t_a, t_c, t_d, T; 
float ref       = 0;  // Posición de referencia
float vel_ref   = 0;  // Velocidad de referencia
float accel_ref = 0;  // Aceleración de referencia
int   phase     = 0;  // Fase del perfil (1,2,3,4)

// Variables para medir tiempo en microsegundos
unsigned long startTime    = 0; 
unsigned long lastProfileTime = 0;
unsigned long lastControlTime = 0;
int current_target = 0;  // Índice del punto actual en theta_f

/*****************************************************/
/*                     setup()                       */
/*****************************************************/
void setup() {
  Serial.begin(9600);

  // Configura el pin del potenciómetro
  pinMode(A0, INPUT); // Asumimos que el pot está conectado al pin A0

  // Guardamos tiempo inicial en microsegundos
  startTime       = micros();
  lastProfileTime = startTime;
  lastControlTime = startTime;
}

/*****************************************************/
/*                      loop()                       */
/*****************************************************/
void loop() {
  unsigned long now = micros();
  calcularTiempos();

  // 2) Actualizar trayectoria cada TS_PROFILE_US
  if (now - lastProfileTime >= TS_PROFILE_US) {
    lastProfileTime = now;
    generarTrayectoria();
  }

  Serial.print((micros() - startTime) / 1000000.0); // tiempo (s)
  Serial.print(" ");
  Serial.print(ref);         // referencia de posición
  Serial.print(" ");
  Serial.print(vel_ref);     // velocidad de referencia
  Serial.print(" ");
  Serial.print(accel_ref);   // aceleración de referencia
  Serial.println();
}

/*****************************************************/
/*    Generación de la trayectoria trapezoidal       */
/*****************************************************/
// Genera la trayectoria para alcanzar los puntos finales
void generarTrayectoria() {
  // t en segundos desde el inicio
  float t = (micros() - startTime) / 1000000.0;

  // Verificar si el siguiente theta_f es menor que el anterior
  bool invertir_senales = (theta_f < theta_0);
  
  // 1) Leer el valor del pot cada TS_PROFILE_US (solo si ha cambiado)
  int potValue = analogRead(A0); // Leer el valor del pot (0 a 1023)
  float theta_f_new = map(potValue, 0, 1023, 0, 1500); // Ajustar el rango según tu necesidad

  // Solo actualizar theta_f si el valor ha cambiado significativamente
  if (abs(theta_f_new - theta_f) > 5) {  // Umbral de cambio de 5 unidades (ajusta según sea necesario)
    theta_f = theta_f_new;
  }

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
      ref = theta_f + 0.5 * alpha * (t_d - t_d_phase) * (t_d - t_d_phase);
    } else {
      ref = theta_f - 0.5 * alpha * (t_d - t_d_phase) * (t_d - t_d_phase);
    }
    vel_ref = alpha * (t_d - t_d_phase);
    accel_ref = -alpha;
  }
  // Fase 4: reposo final
  else {
    ref = theta_f;
    vel_ref = 0;
    accel_ref = 0;
    t = 0;

    // Aquí verificamos si hemos alcanzado theta_f y cambiamos al siguiente
    // Si fuera necesario, puedes ajustar la lógica para cambiar entre varios puntos.
  }
}


/*****************************************************/
/*    Calcular los tiempos para la trayectoria       */
/*****************************************************/
void calcularTiempos() {
  // Calculamos el tiempo de aceleración, velocidad constante y desaceleración
  t_a = u_max / alpha;
  t_c = (theta_f - theta_0 - (u_max * t_a)) / u_max;
  t_d = u_max / alpha;

  // Si t_c < 0 => el perfil es triangular
  if (t_c < 0) {
    t_a = sqrt(abs(theta_f - theta_0) / alpha);
    t_d = t_a;
    t_c = 0;
  }
}
