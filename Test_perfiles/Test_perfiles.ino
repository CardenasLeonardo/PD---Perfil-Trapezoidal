/*****************************************************/
/*                Variables globales                 */
/*****************************************************/
// Parámetros de muestreo (en microsegundos)
const unsigned long TS_PROFILE_US = 1; // 0.1 ms
const unsigned long TS_CONTROL_US = 10;  // 0.10 ms
const unsigned long TS_POT_US = 10000;  // 1 ms

// Variables de trayectoria y potenciómetro
float theta_0 = 0;   // Posición inicial
float theta_f[20];   // Array de puntos finales (theta_f) (ahora dinámico)
int n_puntos = 0;    // Número de puntos finales (theta_f), comenzamos con 0

// Variables para el perfil trapezoidal
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
unsigned long lastPotTime = 0;
int current_target = 0;  // Índice del punto actual en theta_f

/*****************************************************/
/*                     setup()                       */
/*****************************************************/
void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);  // Pin analógico para el potenciómetro

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

  // Mostrar datos por el puerto serie
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

/*****************************************************/
/*    Calcular los tiempos para la trayectoria       */
/*****************************************************/
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

/*****************************************************/
/*    Actualizar los puntos finales con el potenciómetro       */
/*****************************************************/
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
}
