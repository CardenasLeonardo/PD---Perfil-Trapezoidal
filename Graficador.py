import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

ser = serial.Serial("COM3", 9600)  # Asegúrate de usar el puerto correcto

# Inicialización de listas para los datos a graficar
x_data, pos_data, vel_data = [], [], []

# Crear la figura y los subgráficos
fig, (ax1, ax2) = plt.subplots(2, 1)

# Función de actualización para la animación
def update(frame):
    # Leer la línea del puerto serial
    line = ser.readline().decode().strip()
    
    # Separar los valores de posición y velocidad
    values = line.split("\t")
    
    # Asegurarse de que haya dos valores (posición y velocidad)
    if len(values) == 2:
        try:
            # Añadir los datos a las listas
            x_data.append(frame)
            pos_data.append(int(values[0]))  # Convertir la posición a entero
            vel_data.append(float(values[1]))  # Convertir la velocidad a flotante
        except ValueError:
            return

        # Limitar la cantidad de datos para evitar problemas de memoria
        if len(x_data) > 100:  # Limita los datos a los últimos 100 puntos
            x_data.pop(0)
            pos_data.pop(0)
            vel_data.pop(0)

        # Limpiar los ejes y graficar los nuevos datos
        ax1.clear()
        ax2.clear()

        # Graficar los datos en los subgráficos
        ax1.plot(x_data, pos_data, 'r')
        ax1.set_title("Posición")

        ax2.plot(x_data, vel_data, 'b')
        ax2.set_title("Velocidad")

# Crear la animación con un número ilimitado de cuadros
ani = animation.FuncAnimation(fig, update, interval=100)

plt.show()
