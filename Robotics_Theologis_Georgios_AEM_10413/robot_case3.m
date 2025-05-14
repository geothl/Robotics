clear;
Ts=0.01;
HA= human_arm();
robot=ur5robot();
pb=[0.5; -0.5; -0.5;];
gob=[1 0 0 0.5; 0 1 0 -0.5; 0 0 1 -0.5; 0 0 0 1];
[pw,Qw,pwdot,omegaw]=HA.get_arm_posture(0);
%Qw=[h e]; =[cos(th/2) ksin(th/2)] kai R=I3+2hS(e)+2*S(e)^2
q0=[-1.5692 -0.9318 1.211 -1.85 -1.5708 -0.522];
robot.fkine(q0);
% robot.plot(q0);
q=q0;
qdot=transpose(inv(robot.jacob0(q0))*[pwdot;omegaw]);
R=quat2rotm(transpose(Qw));
i=1;
Q_ar(:,1)=transpose(rotm2quat(robot.fkine(q0).R));
Qd_ar(:,1)=Qw;
K=3*eye(3);
ep=[0;0;0;];
ei=[0;0;0;];
pd=pw;
p_arr=pw;
pd_arr=pd;
pw_arr=pw;
pe_arr=pw-0.3*R*[1;0;0;];
for t=Ts:Ts:5
[pw,Qw,pwdot,omegaw]=HA.get_arm_posture(t);
R=quat2rotm(transpose(Qw));
pw_arr(:,i+1)=pw;
pe_arr(:,i+1)=pw-0.3*R*[1;0;0;];
% pd=pw-t*6*10^(-2)*R(:,1);
pd=pw-(3/25*0.3*t^2-2/(5*25)*0.3*t^3)*R(:,1);
pd_arr(:,i+1)=pd;
g=(robot.fkine(q(i,:)));
p=g.t;
p_arr(:,i+1)=p;
Rr=g.R;
Q=transpose(rotm2quat(Rr));
Qd=Qw;
Q_ar(:,i+1)=Q;
Qd_ar(:,i+1)=Qd;
Qd_r=[Qd(1);-Qd(2:4)];
QQd_r=[Q(1)*Qd_r(1)-transpose(Q(2:4))*Qd_r(2:4);  Q(1)*Qd_r(2:4)+Qd_r(1)*Q(2:4)+cross(Q(2:4),Qd_r(2:4))];
ep(:,i+1)=p-pd;
ei(:,i+1) = errorofquat(QQd_r);
omegaw_so3=[0 -omegaw(3) omegaw(2);omegaw(3) 0 -omegaw(1);-omegaw(2) omegaw(1) 0;];
% pddot=pwdot-t*6*10^(-2)*omegaw_so3*R*[1;0;0;]-6*10^(-2)*R(:,1);
pddot=pwdot-(3/25*0.3*t^2-2/(5*25)*0.3*t^3)*omegaw_so3*R*[1;0;0;]-(6/25*0.3*t-6/(5*25)*0.3*t^2)*R(:,1);

vh=pddot-K*ep(:,i+1);
omegad=omegaw;
omegah=omegad-K*ei(:,i+1);
Vh=[vh;omegah];
Vb=[transpose(R) zeros(3,3);zeros(3,3) transpose(R)]*Vh;
qdot=transpose(inv(robot.jacob0(q(i,:)))*Vh);
q(i+1,:)=q(i,:)+qdot*Ts;
i=i+1;
end
pe=pw-0.3*R*[1;0;0;];
theta=-acos(transpose(pe)*R(:,1)/norm(pe));
thstep=0;
thprevstep=0;
for t=5.01:Ts:10
[pw,Qw,pwdot,omegaw]=HA.get_arm_posture(t);
R=quat2rotm(transpose(Qw));
pe=pw-0.3*R*[1;0;0;];
pw_arr(:,i+1)=pw;
pe_arr(:,i+1)=pe;
thprev=theta;
theta=-acos(transpose(pe)*R(:,1)/norm(pe));
Ro_th=[cos(theta) 0 sin(theta);0 1 0; -sin(theta) 0 cos(theta)];
Re=R*Ro_th;
% pd=pe-(t-5)*6*10^(-2)*Re(:,1);
pd=pe-(3/25*0.3*(t-5)^2-2/(5*25)*0.3*(t-5)^3)*Re(:,1);
pd_arr(:,i+1)=pd;

omegaw_so3=[0 -omegaw(3) omegaw(2);omegaw(3) 0 -omegaw(1);-omegaw(2) omegaw(1) 0;];
pedot=pwdot-0.3*omegaw_so3*R*[1;0;0;];

dtheta_dt=(theta-thprev)/Ts;
omegae=omegaw+dtheta_dt*Re(:,2);
omegae_so3=skew(omegae);

g=(robot.fkine(q(i,:)));
p=g.t;
p_arr(:,i+1)=p;
Rr=g.R;
Q=transpose(rotm2quat(Rr));
Qd=transpose(rotm2quat(Re));
% pddot=pedot-(t-5)*6*10^(-2)*omegae_so3*Re*[1;0;0;]-6*10^(-2)*Re(:,1);
pddot=pedot-(3/25*0.3*(t-5)^2-2/(5*25)*0.3*(t-5)^3)*omegae_so3*Re*[1;0;0;]-(6/25*0.3*(t-5)-6/(5*25)*0.3*(t-5)^2)*Re(:,1);
omegad=omegae; 


if(t<8.01)
thstep=theta*(t-5)/3;
Rs_th=[cos(thstep) 0 sin(thstep);0 1 0; -sin(thstep) 0 cos(thstep)];
Qd=transpose(rotm2quat(R*Rs_th));
%is this modification needed--> for CLICK control to work omega_desired and 
%Q_desired must change in accordance with each other  
dtheta_dt=(thstep-thprevstep)/Ts;
thprevstep=thstep;
omegad=omegaw+dtheta_dt*Re(:,2);
end



Q_ar(:,i+1)=Q;
Qd_ar(:,i+1)=Qd;
Qd_r=[Qd(1);-Qd(2:4)];
QQd_r=[Q(1)*Qd_r(1)-transpose(Q(2:4))*Qd_r(2:4);  Q(1)*Qd_r(2:4)+Qd_r(1)*Q(2:4)+cross(Q(2:4),Qd_r(2:4))];
ep(:,i+1)=p-pd;
ei(:,i+1) = errorofquat(QQd_r);
vh=pddot-K*ep(:,i+1);
omegah=omegad-K*ei(:,i+1);

qdot=transpose(inv(robot.jacob0(q(i,:)))*[vh;omegah]);
q(i+1,:)=q(i,:)+qdot*Ts;
i=i+1;
end
n=i;

tf=10;


%%%%%%%%%%%%%%%%%% ANIMATION OF MOVEMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
robot.plot(q0);
c2=line([0;pe_arr(1,1);],[0;pe_arr(2,1);],[0;pe_arr(3,1);],'Color','black','LineStyle','--');
c3=line([pw_arr(1,1);pe_arr(1,1);],[pw_arr(2,1);pe_arr(2,1);],[pw_arr(3,1);pe_arr(3,1);],'Color','black','LineStyle','--');
title("Animation of Robotic arm and Human arm")
pause(0.01);


for i=1:25:n 
hold on;
delete(c2);
delete(c3);
c2=line([0;pe_arr(1,i);],[0;pe_arr(2,i);],[0;pe_arr(3,i);],'Color','black','LineStyle','--'); %bicep
c3=line([pw_arr(1,i);pe_arr(1,i);],[pw_arr(2,i);pe_arr(2,i);],[pw_arr(3,i);pe_arr(3,i);],'Color','black','LineStyle','--'); %forearm
robot.animate(q(i,:));
% pause(0.25);
pause(0.01); %if we want a smoother but faster animation
end


%%%%%%%%%%%%%%%%%% PLOTS  %%%%%%%%%%%%%%%%%%%%%%%%
px= p_arr(1,:);
py= p_arr(2,:);
pz= p_arr(3,:);
t=0:0.01:tf;

%%%% position
figure;
subplot(3,1,1)
plot(t,px,t,pd_arr(1,:),'--');
title('x άκρου βραχίωνα και x_d επιθυμητή του θέση')
xlabel("Time (seconds)")
ylabel("Position (m)");
legend("x","x_d");
grid on;
subplot(3,1,2)
plot(t,py,t,pd_arr(2,:),'--');
title('y άκρου βραχίωνα και y_d επιθυμητή του θέση')
xlabel("Time (seconds)")
ylabel("Position (m)");
legend("y","y_d");
grid on;
subplot(3,1,3)
plot(t,pz,t,pd_arr(3,:),'--');
title('z άκρου βραχίωνα και z_d επιθυμητή του θέση')
xlabel("Time (seconds)")
ylabel("Position (m)");
legend("z","z_d");
grid on;
sgt = sgtitle('Θέση άκρου Βραχίωνα και Επιθυμητή θέση αναφοράς','Color','black');
sgt.FontSize = 20;

ex=ep(1,:);
ey=ep(2,:);
ez=ep(3,:);
figure;
plot(t,ex,t,ey,t,ez);
xlabel("Time (seconds)")
ylabel("Position (m)");
title("Διαφορά θέσης άρκου p με επιθυμητή θέση p_d ep=p-p_d")
legend("e_x","e_y","e_z");
grid on;


%%%%  Orientation
figure;
subplot(2,2,1)
plot(t,Q_ar(1,:),t,Qd_ar(1,:),'--');
title('Q_1 άκρου βραχίωνα και Qd_1 επιθυμητή συνιστώσα κουατερνίου')
xlabel("Time (seconds)")
ylabel("1st Quaternion coordinate (η)");
legend("Q_1","Qd_1");
grid on;
subplot(2,2,2)
plot(t,Q_ar(2,:),t,Qd_ar(2,:),'--');
title('Q_2 άκρου βραχίωνα και Qd_2 επιθυμητή συνιστώσα κουατερνίου')
xlabel("Time (seconds)")
ylabel("2nd Quaternion coordinate (ε_1)");
legend("Q_2","Qd_2");
grid on;
subplot(2,2,3)
plot(t,Q_ar(3,:),t,Qd_ar(3,:),'--');
title('Q_3 άκρου βραχίωνα και Qd_3 επιθυμητή συνιστώσα κουατερνίου')
xlabel("Time (seconds)")
ylabel("3rd Quaternion coordinate (ε_2)");
legend("Q_3","Qd_3");
grid on;
subplot(2,2,4)
plot(t,Q_ar(4,:),t,Qd_ar(4,:),'--');
title('Q_4 άκρου βραχίωνα και Qd_4 επιθυμητή συνιστώσα κουατερνίου')
xlabel("Time (seconds)")
ylabel("4th Quaternion coordinate (ε_3)");
legend("Q_4","Qd_4");
grid on;
sgt = sgtitle('Θέση άκρου Βραχίωνα και Επιθυμητή θέση αναφοράς','Color','black');
sgt.FontSize = 20;



eheta=Q_ar(1,:)-Qd_ar(1,:);
ee1=Q_ar(2,:)-Qd_ar(2,:);
ee2=Q_ar(3,:)-Qd_ar(3,:);
ee3=Q_ar(4,:)-Qd_ar(4,:);
figure;
plot(t,eheta,t,ee1,t,ee2,t,ee3);
xlabel("Time (seconds)")
ylabel("Quaternion Units");
title("Διαφορά τιμών κουατερνίων Q-Q_d ")
legend("e_η (Q_1-Qd_1)","e_{ε_1} (Q_2-Qd_2)","e_{ε_2} (Q_3-Qd_3)","e_{ε_3} (Q_4-Qd_4)");
grid on;



figure;
subplot(2,3,1)
plot(t,q(:,1));
title('q_1')
xlabel("Time (seconds)")
ylabel("Joint angle (rad)");
grid on;


subplot(2,3,2)
plot(t,q(:,2));
title('q_2')
xlabel("Time (seconds)")
ylabel("Joint angle (rad)");
grid on;
subplot(2,3,3)
plot(t,q(:,3));
grid on;
title('q_3')
xlabel("Time (seconds)")
ylabel("Joint angle (rad)");
subplot(2,3,4)
plot(t,q(:,4));
grid on;
title('q_4')
xlabel("Time (seconds)")
ylabel("Joint angle (rad)");
subplot(2,3,5)
plot(t,q(:,5));
grid on;
title('q_5')
xlabel("Time (seconds)")
ylabel("Joint angle (rad)");
subplot(2,3,6)
plot(t,q(:,6));
grid on;
title('q_6')
xlabel("Time (seconds)")
ylabel("Joint angle (rad)");
grid on;
sgt = sgtitle('Γωνίες Αρθρώσεων','Color','black');
sgt.FontSize = 20;








