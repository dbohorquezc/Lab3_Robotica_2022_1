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

<p align="center">
  <img src="Fig/Codoarriba.jpeg"/>
</p>

$$
\begin{gather*} 
  q_2 = \frac{\pi}{2}-\beta-\alpha-\gamma \\
  q_3 = \pi-\psi-\phi
\end{gather*}
$$

#### CODO ABAJO:

<p align="center">
  <img src="Fig/Codoabajo.jpeg"/>
</p>

$$
\begin{gather*} 
  q_2 = \frac{\pi}{2}-(\gamma-\alpha+\beta)\\
  q_3 = -\pi+(\phi-\psi)
\end{gather*}
$$

Una vez definidos los ángulos se vuelve a acoplar la muñeca y se define su angulo como:

<p align="center">
  <img src="Fig/Inv.jpeg"/>
</p>

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

Como primer análisis es necesario encontrar una forma en el que se realice un tipo de interpolación entre dos puntos que se conozca la rotación y traslación del efector final tal y como lo pide la cinemática inversa. Para esto se investigó una función del Toolbox de Peter Corke que permita realizar este proceso y tener una trayectoria más fluida y no solo dos puntos en el espacio, "ctraj" es la encargada de realizar este proceso, tiene como parámetros el ingreso de una matriz de transformación homogenea inicial, una de objetivo y por ultimo el numero de puntos que se quieren en la trayectoria, entre mas se tengan se consume más tiempo de procesamiento. Ahora el siguiente problema es la obtención de dichas matrices, para esto se hizo uso del modelo utilizado en el Lab2 donde se puede ver de manera gráfica e interactiva la posicion dl robot y el marco de referencia del eslabón con respecto a la base.
<p align="center">
  <img align="center"; width="500"  src="Fig/ModeloTeach.png">
</p>
Para la obtención de la información del tipo de rotación  que presenta el marco de referencia de la herramienta con respecto a la base, se analiza la magnitud que debe tener dicha rotación de manera empírica y observación de los diferentes marcos, en algunos casos solo se realiza una modificación de la traslación, de esta manera se generan las suguientes matrices de transformacion homogenea.

```
%Matriz de Home
MTHinit=[1 0 0 0;0 1 0 0;0 0 1 44.9;0 0 0 1];
%Matriz intermedia entre el punto encima del poste y el Home
MTHinter1a=[0.7071 0 0.7071 14.483;0 1 0 0;-0.7071 0 0.7071 40.466;0 0 0 1];
%Matriz que representa la poscición a 10 cm encima del poste
MTHinter=[-1 0 0 15;0 1 0 0;0 0 -1 10;0 0 0 1];
%MTH de 8 cm encima del poste
MTHinter2=[-1 0 0 15;0 1 0 0;0 0 -1 8;0 0 0 1];
%MTH de la rotación a la izquierda del efector y posición de 10 cm encima del primer cilindro
MTHrotz1=trotz(pi/2)*MTHinter;MTHrotz1(1,4)=0;MTHrotz1(2,4)=15;
%MTH del acercamiento al primer cilindro
MTHFinal1=MTHrotz1;MTHFinal1(3,4)=4;
%MTH de la rotación a la derecha del efector y posición de 10 cm encima del segunda cilindro
MTHrotz2=trotz(-pi/2)*MTHinter;MTHrotz2(1,4)=0;MTHrotz2(2,4)=-15;
%MTH del acercamiento al segunda cilindro
MTHFinal2=MTHrotz2;MTHFinal2(3,4)=4;
```

Al tener estas matrices se puede hacer una rutina que permita ubicar los dos cilindros en el poste requerido, para evitar que se tenga una gran cantidad de lineas de codigo, se realizó una función que permitiera realizar la interpolación, la cinemática inversa y el llamado del servicio para enviar los ángulos requeridos en cada articulación:

```
function []= Move(MTH1,MTH2,n,motorSvcClient,motorCommandMsg)

TCP1=ctraj(MTH1,MTH2,n);

for i=1:length(TCP1)
    q_rad=invKin(TCP1(:,:,i));
    q_deg=q_rad(1,:)*(180/pi)
    for j=1:length(q_deg)
       
        motorCommandMsg.AddrName="Goal_Position";
        motorCommandMsg.Id=j;
        round(mapfun(q_deg(j),-150,150,0,1023))
        motorCommandMsg.Value=round(mapfun(q_deg(j),-150,150,0,1023));%bits
        call(motorSvcClient,motorCommandMsg);
        pause(0.1);
    end
end
end
```
Se evidencia que los parametros que ingresan con MTH de inicio y fin, el numeros de puntos, el cliente y el mensaje, como se menciono la interpolacion se ejecuta y se obtiene un arreglo en 3 dimendiones donde se guardan todas las MTH's de los puntos hallados permitiendo asi crear una iteracion que finalice cuando dichos puntos hallan sido evaluados con el fin de obtner los diferentes valores articulares y por último enviarlos  a cada uno de los motores. Como se hizo en lab 2 es necesario utilizar un mapeo para el envio de informacion a los motores, esto por medio de la función Mapfun la cual recibe los ángulo en radianes y otorga su valor respectivo en bits, con esto realizado lo que queda es realizar la rutina.
```
motorCommandMsg.AddrName="Goal_Position";
motorCommandMsg.Id=5;
motorCommandMsg.Value=round(mapfun(0,-150,150,0,1023));%bits
call(motorSvcClient,motorCommandMsg);


Move(MTHinit,MTHinter1a,10,motorSvcClient,motorCommandMsg);
Move(MTHinter1a,MTHinter,10,motorSvcClient,motorCommandMsg);
Move(MTHinter,MTHrotz1,10,motorSvcClient,motorCommandMsg);
Move(MTHrotz1,MTHFinal1,15,motorSvcClient,motorCommandMsg);
motorCommandMsg.AddrName="Goal_Position";
motorCommandMsg.Id=5;
motorCommandMsg.Value=round(mapfun(50,-150,150,0,1023));%bits
call(motorSvcClient,motorCommandMsg);
Move(MTHFinal1,MTHrotz1,10,motorSvcClient,motorCommandMsg);
Move(MTHrotz1,MTHinter,10,motorSvcClient,motorCommandMsg);
Move(MTHinter,MTHinter2,15,motorSvcClient,motorCommandMsg);
motorCommandMsg.AddrName="Goal_Position";
motorCommandMsg.Id=5;
motorCommandMsg.Value=round(mapfun(0,-150,150,0,1023));%bits
call(motorSvcClient,motorCommandMsg);
Move(MTHinter2,MTHinter,10,motorSvcClient,motorCommandMsg);
Move(MTHinter,MTHrotz2,10,motorSvcClient,motorCommandMsg);
Move(MTHrotz2,MTHFinal2,15,motorSvcClient,motorCommandMsg);
motorCommandMsg.AddrName="Goal_Position";
motorCommandMsg.Id=5;
motorCommandMsg.Value=round(mapfun(50,-150,150,0,1023));%bits
call(motorSvcClient,motorCommandMsg);
Move(MTHFinal2,MTHrotz2,10,motorSvcClient,motorCommandMsg);
Move(MTHrotz2,MTHinter,10,motorSvcClient,motorCommandMsg);
Move(MTHinter,MTHinter2,15,motorSvcClient,motorCommandMsg);
motorCommandMsg.AddrName="Goal_Position";
motorCommandMsg.Id=5;
motorCommandMsg.Value=round(mapfun(0,-150,150,0,1023));%bits
call(motorSvcClient,motorCommandMsg);
Move(MTHinter2,MTHinter,10,motorSvcClient,motorCommandMsg);
```
Cada uno de los llamados de la función Move indican un movimiento iniciando desde home, y se puede evidenciar que en los puntos donde se ubica el cilindro se agrega un llamado del servicio del motor de la pinza para ejecutar el agarre y como buen practica se agregan más puntos en el movimiento previo a esto para obtener una aproximación más precis


<p align="center">
  <img src="Fig/Robotpp.jpeg"/>
</p>

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
Se propone una función que permita definir trayectoria. Esta recibe como entradas el punto actual, y dependiendo del eje de movimiento y la tecla pulsada, va a agregar o disminuir la cantidad MLD en [cm] para los desplazamientos; y la cantidad MLA en grados, para la rotación en y respecto a q1. Además de esto, se agrega como ultimo parámetro la cantidad de interpolaciones que se quiere que realice la función Ctaj() del Toolbox de Peter Corke.

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
Posterior a esto solo es necesario agregar una cantidad positiva o negativa de MLD o MLA dependiendo si se pulsó la tecla ‘A’ o ‘D’,  y realizar el movimiento, para ello se creó la función move(), que recibe el número de iteraciones que se realizaron en la función del toolbox Ctraj() e itera por medio de un for para lograr el movimiento secuencial del robot.

## Video en Youtube

En el siguiente video se muestra de forma condensada lo realizado en la práctica. Primero se muestra el control por teclado y posteriormente la función de *Pick and place*.
 Cabe aclarar que el robot Phantom x número 2 dado en la práctica, poseía fallos en el mecanismo del *gripper*, ya que un eslabón estaba deformado generando fricción y movimientos no esperados; es por esto que al final de la prueba de  *Pick and place* no se puede insertar el aro en la base, ya que generó fricciones indeseadas que al abrir el *gripper* no permitieron volver a su estado abierto y soltar la pieza.


[Robótica: Cinemática Inversa - Phantom X - ROS](https://youtu.be/5wIkKf9X7k8 "Robótica: Cinemática Inversa - Phantom X - ROS")

## Conclusiones 

* Como se muestra en el video la precisión del robot se puede ver afectada por la vida util de los componentes o por el mal uso como los golpes los cuales pueden afectar la integridad fisica del dispositivo llevando así a perturbaciones en la trayectado evidenciadas principalmente en las oscilaciones. 
* El funcionamiento en conjunto de la función "ctraj" y la cinemática inversa en algunos casos presentaba soluciones que no era adecuadas o posibles para los motores, por ende fue necesario ingresar puntos intermedios manuales que indicaran un inicio de por donde se desea que siga la trayectoria, tal y como lo representa la MTH "MTHinter1a".
* Se pudo realizar satisfactoriamente la práctica, aprendiendo del proceso de interpolar rutas para generar trayectorias y hallar la cinemática inversa para cada punto, un futuro análisis para la mejorar el desempeño del robot consta en abrir un camino hacia el control de movimiento, y el análisis de fuerzas que intervienen en las articulaciones del robot para que realice la tarea efectivamente.
