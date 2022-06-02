<h1 align="center"; style="text-align:center;">Laboratorio 3: Cinemática Inversa - Robot Phantom X - ROS</h1>
<p align="center";style="font-size:50px; background-color:pink; color:red; text-align:center;line-height : 60px; margin : 0; padding : 0;">
Robótica</p1>
<p align="center";style="font-size:50px; text-align:center; line-height : 40px;  margin-top : 0; margin-bottom : 0; "> <br> Giovanni Andrés Páez Ujueta</p>
<p align="center";style="font-size:50px; text-align:center; line-height : 20px; margin-top : 0; "> email: gpaezu@unal.edu.co</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 40px;  margin-top : 0; margin-bottom : 0; "> <br> Daniel Esteban Bohórquez Cifuentes</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 20px; margin-top : 0; "> email: dbohorquezc@unal.edu.co</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 40px;  margin-top : 0; margin-bottom : 0; "> <br> Nicolas Pulido Gerena</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 20px; margin-top : 0; "> email: npulido@unal.edu.co</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 30px;  margin-top : 0; margin-bottom : 0; "> <br><br>INGENIERÍA MECATRÓNICA</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 30px; margin-top : 0; "> Facultad de Ingeniería</p>
<p align="center"; style="font-size:50px; text-align:center; line-height : 30px; margin-top : 0; "> Universidad Nacional de Colombia Sede Bogotá</p>
<br>
<p align="center">
  <img align="center"; width="100"  src="Fig/Escudo_UN.png">
</p>

<p align="center"; style="font-size:50px; text-align:center; line-height : 30px; margin-top : 0; "> <br>1ero de Junio de 2022</p>

## Metodología

### Cinemática Inversa

Se tiene el valor de $q_1$ como:

$$
\begin{gather*}
    q_1=\mathrm{atan2}(X_\mathrm{T},Y_\mathrm{T})
\end{gather*}
$$

donde los valores de $X_\mathrm{T}$ y $Y_\mathrm{T}$ corresponden a coordenadas del TCP.

Se realiza un desacople de la muñeca de la ultima articulación por lo que la posición de nuestro TCP se mueve la distancia correspondiente a $l_4$ en la dirección del vector "$a$"(approach en la nomenclatura NOA), por lo que la posición de la muñeca corresponde a:

$$
\begin{align*}
    W&=
    \begin{bmatrix}
        X_\mathrm{T}\\
        Y_\mathrm{T}\\
        Z_\mathrm{T}
    \end{bmatrix}
    -l_4
    \begin{bmatrix}
        a_X\\
        a_Y\\
        a_Z
    \end{bmatrix}
\end{align*}
$$

Donde los valores de $a_X$, $a_Y$ and $a_Z$ corresponden a los componentes cartesianos del vector "$a$". A partir de esto es posible determinar los componentes de la matriz de rotación de nuestro TCP. De esta forma es posible simplificar este problema de cinematica inversa al de un mecanismo 2R, donde se proponen dos soluciones: Codo arriba y Codo abajo:


$$
\begin{gather*}
    r = \sqrt{X_W^2+Y_W^2} \ \ \ \ \ \ h = z_w-l_1\\
    c = \sqrt{r^2+h^2}\\
    \\
    \beta = \mathrm{arctan2}(l_m,l_2) \ \ \ \ \ \ \psi = \frac{\pi}{2}-\beta\\
    l_r = \sqrt{l_m^2+l_2^2}\\
    \\
    \phi = \mathrm{arccos}(\frac{c^2-l_3^2-l_r^2}{-2l_rl_3}) \ \ \ \ \ \ \alpha =  \arccos{\frac{l_3^2-l_r^2-c^2}{-2l_rc}}
    \\
    \gamma = \arctan2{(h,r)}
\end{gather*}
$$

Se utilizan las siguientes ecuaciones para definir las soluciones de Codo arriba y Codo abajo:

#### CODO ARRIBA:

$$
\begin{gather*} 
  q_2 = \frac{\pi}{2}-\beta-\alpha-\gamma \\
  q_3 = \pi-\psi-\phi
\end{gather*}
$$

#### CODO ABAJO:

$$
\begin{gather*} 
  q_2 = \frac{\pi}{2}-(\gamma-\alpha+\beta)\\
  q_3 = -\pi+(\phi-\psi)
\end{gather*}
$$

Una vez definidos los ángulos se vuelve a acoplar la muñeca y se define su angulo como:

$$
\begin{gather*}
    \theta_a=\mathrm{\arctan2}\left(\sqrt{X_a^2+Y_a^2},Z_a\right)\\
    q_4=\theta_a-q_2-q_3-\frac{\pi}{2}
\end{gather*}
$$

Donde $\theta_a$ corresponde al angulo del vector "$a$" respecto al eje $Z_0$.

### ROS - Matlab: Aplicación de Pick and place
Para esta parte del laboratorio se hizo uso de la conexión de Matlab con ROS y su capacidad de permitir acceder y llamar los servicios que presente con respecto a una aplicación determinada por medio de código dentro del software. Es necesario mencionar que se tiene que relaizar un proceso previo el cual antecede al trabajo dentro Matlab, tal y como se menciono en los anteriores laboratorios se dan permisos de administrador al puerto donde se inserta la FTDI, se inicia el nodo maestro y se utiliza el launch que permite mover los motores. Se sigue el mismo proceso expresado en el Lab 1 donde se crea un nodo de MAtlab con el nodo maestro, y un cliente que permita ingresar a los servicios del Dynamixel Command por medio de un mensaje que indicará el movimiento de los motores a partir del siguiente código:

```Matlab
%%
clc
clear
rosshutdown
rosinit;
%%
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
motorCommandMsg= rosmessage(motorSvcClient);

```
Como primer análisis es necesario encontrar una forma en el que se realice un tipo de interpolación entre dos puntos que se conozca la rotación y traslación del efector final tal y como lo pide la cinemática inversa. Para esto se investigó una función del Toolbox de Peter Corke que permita realizar este proceso y tener una trayectoria más fluida y no solo dos puntos en el espacio, "ctraj" es la encargada de realizar este proceso, tiene como parámetros 

### ROS - Python: Aplicación de movimiento en el espacio de la tarea

Para el desarrollo del movimiento aplicado del manipulador en el espacio de la tarea se utilizó el entorno de Python, debido a la facilidad en la detección de las teclas "W", "A", "S" y "D". El script hecho es similar en algunas herramientas al desarrollado en la Practica 2, como lo es el uso de THERMIOS para la detección de teclas.

```Python
TERMIOS = termios
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c
```

Ahora bien, este script tiene en cuenta la función de cinematica inversa propuesta inicialmente en el repositorio del laboratorio, donde se entrega la función "qinv" obteniendo los valores de posición de las articulaciones.

```Python
def inv_kci(T):
    l = np.array([14.5, 10.7, 10.7, 9])
    Tw = T-(l[3]*T[0:4,2]).reshape(4,1)
    q1 = np.arctan2(Tw[1,3],Tw[0,3])
    # Solucion 2R
    h = Tw[2,3] - l[0]
    r = np.sqrt(Tw[0,3]**2 + Tw[1,3]**2)
    # Codo abajo
    the3 = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))
    the2 = np.arctan2(h,r) - np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2d = -(np.pi/2-the2)
    q3d = the3

    # Codo arriba
    the2 = np.arctan2(h,r) + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2u = -(np.pi/2-the2)
    q3u = -the3

    # Solucion q4
    Rp = (rotz(q1).T).dot(T[0:3,0:3])
    pitch = np.arctan2(Rp[2,0],Rp[0,0])
    q4d = pitch - q2d - q3d
    q4u = pitch - q2u - q3u
    if q4u > (7/6)*np.pi:
        q4u = q4u-2*np.pi
    qinv = np.empty((1,4))
    qinv[:] =np.NaN
    qinv[0,:] = np.array([q1*180/3.1416,q2u*180/3.1416,q3u*180/3.1416, q4u*180/3.1416])
    return qinv
```
Se propone una función que permita definir trayectoria.

```Python
def give_Traj(initia_pos, axe_movement, q1, MLD, MLA, n_points):
    print(initia_pos)
    initial_pos_matrix = SE3(initia_pos[0],initia_pos[1], initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4] ,unit='deg')
    if axe_movement == 1:
        future_pos = SE3(initia_pos[0]+ MLD,initia_pos[1], initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4], unit='deg')
        new_position = initia_pos
        new_position[0] = initia_pos[0]+ MLD
    elif axe_movement == 2:
        future_pos = SE3(initia_pos[0],initia_pos[1]+ MLD, initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4], unit='deg')
        new_position = initia_pos
        new_position[1] = initia_pos[1]+ MLD
    elif axe_movement == 3:
        future_pos = SE3(initia_pos[0],initia_pos[1], initia_pos[2]+MLD)*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4], unit='deg')
        new_position = initia_pos
        new_position[2] = initia_pos[2]+ MLD
    elif axe_movement == 4:
        future_pos = SE3(initia_pos[0],initia_pos[1], initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4]+MLA ,unit='deg')
        new_position = initia_pos
        new_position[4] = initia_pos[4]+ MLA
    print('Posicion Final')
    print(new_position)
    Ts = rtb.tools.trajectory.ctraj(initial_pos_matrix, future_pos, n_points)
    #print(initial_pos_matrix)
    #print(future_pos)
    Traj = np.zeros((n_points,4))
    for i in range(0,n_points):
        Traj[i,:] = inv_kci(Ts[i].A)
   
    return Traj, n_points, new_position
```


## Video en Youtube
[Robótica: Cinemática Inversa - Phantom X - ROS](https://youtu.be/5wIkKf9X7k8 "Robótica: Cinemática Inversa - Phantom X - ROS")

## Conclusiones 

* Como se muestra en el video la precisión del robot se puede ver afectada por la vida util de los componentes o por el mal uso como los golpes los cuales pueden afectar la integridad fisica del dispositivo llevando así a perturbaciones en la trayectado evidenciadas principalmente en las oscilaciones. 
