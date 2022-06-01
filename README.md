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

<p align="center"; style="font-size:50px; text-align:center; line-height : 30px; margin-top : 0; "> <br>13 de mayo de 2022</p>

## Cinemática Inversa

Se tiene el valor de $q_1$ como:

$$
\begin{gather*}
    q_1=\mathrm{atan2}(X_\mathrm{T},Y_\mathrm{T})
\end{gather*}
$$

donde los valores de X_\mathrm{T} y $y_T$ corresponden a coordenadas del TCP.

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
    r = \sqrt{x_W^2+y_W^2}\\
    h = z_w-l_1\\
    \\
    c = \sqrt{r^2+h^2}\\
    \\
    \beta = \mathrm{arctan2}(l_m,l_2)\\
    \psi = \frac{\pi}{2}-\beta\\
    l_r = \sqrt{l_m^2+l_2^2}\\
    \\
    \phi = \mathrm{arccos}(\frac{c^2-l_3^2-l_r^2}{-2l_rl_3})\\
    \\
    \gamma = \arctan2{(h,r)}\\
    \alpha =  \arccos{\frac{l_3^2-l_r^2-c^2}{-2l_rc}}
\end{gather*}
$$

Se utilizan las siguientes ecuaciones para definir las soluciones de Codo arriba y Codo abajo:

CODO ARRIBA:
$$
\begin{gather*} 
q_2 = \frac{\pi}{2}-\beta-\alpha-\gamma \\
q_3 = \pi-\psi-\phi
\end{gather*}
$$

CODO ABAJO:

$$
\begin{gather*} 
q_2 = \frac{\pi}{2}-(\gamma-\alpha+\beta)\\
q_3 = -\pi+(\phi-\psi)
\end{gather*}
$$

## Metodología
