function []= Move(MTH1,MTH2,n,motorSvcClient,motorCommandMsg)
TCP1=ctraj(MTH1,MTH2,n);

for i=1:length(TCP1)
    q_rad=invKin(TCP1(:,:,i));
    %q_deg=[0 0 0 0];
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