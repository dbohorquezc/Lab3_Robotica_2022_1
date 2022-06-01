function [qinv]=invKin(Tt)
l = [14.5, 10.7, 10.7, 9]; % Longitudes eslabones
    % Desacople
    T = Tt;
    Posw = T(1:3,4) - l(4)*T(1:3,3); % MTH Wrist
    
    % Solucion q1
    q1 = atan2(Posw(2),Posw(1));
    rad2deg(q1);
    
    % Solución 2R
    h = Posw(3) - l(1);
    r = sqrt(Posw(1)^2 + Posw(2)^2);
    
    % Codo abajo
    the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    the2 = atan2(h,r) - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
    
    q2d = -(pi/2-the2);
    q3d = the3;
    
    % Codo arriba
    % the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    the2 = atan2(h,r) + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
    q2u = -(pi/2-the2);
    q3u = -the3;
    
    
    
    % Solucion de q4
    Rp = (rotz(q1))'*T(1:3,1:3);
    pitch = atan2(Rp(3,1),Rp(1,1));
    
    q4d = pitch - q2d - q3d;
    q4u = pitch - q2u - q3u;
    %disp(rad2deg([q4d q4u]))
     if q4u>(7/6)*pi
         q4u=q4u-2*pi;
     end
    qinv(1,1:4) = [q1 q2u q3u q4u];
    qinv(2,1:4) = [q1 q2d q3d q4d];
    
end