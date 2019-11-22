  
% /****************************************************************************
%  *   Copyright (c) 2019 Jesus Tordesillas Torres. All rights reserved.
%  *
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions
%  * are met:
%  *
%  * 1. Redistributions of source code must retain the above copyright
%  *    notice, this list of conditions and the following disclaimer.
%  * 2. Redistributions in binary form must reproduce the above copyright
%  *    notice, this list of conditions and the following disclaimer in
%  *    the documentation and/or other materials provided with the
%  *    distribution.
%  * 3. Neither the name of this repo nor the names of its contributors may be
%  *    used to endorse or promote products derived from this software
%  *    without specific prior written permission.
%  *
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
%  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
%  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  *
%  ****************************************************************************/

clear;close all ; clc;
global N_fases;
global n; %Matrix that contains the normal vectors to all the gates
global angle_cone %For the cone constraint, in degrees
n=[];
N_fases=7;
angle_cone=60; 
global Pxw0 Pxwf Pyw0 Pywf Pzw0 Pzwf yaw0 yawf pitch0 pitchf roll0 rollf dpx0 dpxf dpy0 dpyf dpz0 dpzf p0 pf q0 qf r0 rf T0 Tf tauy0 tauyf taup0 taupf taur0 taurf intguess;
global Pxw0_min Pxw0_max Pxw_min Pxw_max Pxwf_min Pxwf_max Pyw0_min Pyw0_max Pyw_min Pyw_max Pywf_min Pywf_max Pzw0_min Pzw0_max...
Pzw_min Pzw_max Pzwf_min Pzwf_max yaw0_min yaw0_max yaw_min yaw_max yawf_min yawf_max pitch0_min pitch0_max pitch_min pitch_max...
pitchf_min pitchf_max roll0_min roll0_max roll_min roll_max rollf_min rollf_max dpx0_min dpx0_max dpx_min dpx_max dpxf_min...
dpxf_max dpy0_min dpy0_max dpy_min dpy_max dpyf_min dpyf_max dpz0_min dpz0_max dpz_min dpz_max dpzf_min dpzf_max p0_min p0_max...
p_min p_max pf_min pf_max q0_min q0_max q_min q_max qf_min qf_max r0_min r0_max r_min r_max rf_min rf_max T0_min T0_max T_min...
T_max Tf_min Tf_max tauy0_min tauy0_max tauy_min tauy_max tauyf_min tauyf_max taup0_min taup0_max taup_min taup_max taupf_min...
taupf_max taur0_min taur0_max taur_min taur_max taurf_min taurf_max t0min t0max tfmin tfmax integral_min integral_max;
read_guesses();
read_constraints();

%% SETUP
[bounds, guess]=do_setup();

for i=1:(N_fases-1)
     bounds.eventgroup(i).lower = [zeros(1,12),0,0];
     bounds.eventgroup(i).upper = [zeros(1,12),0,5];
end

%% Execution

%Provide Mesh Refinement Method and Initial Mesh 
mesh.method = 'hp-PattersonRao';
mesh.tolerance = 1e-6;%Estaba en 1e-6, yo lo he reducido para debug
mesh.colpointsmin = 20;
mesh.colpointsmax = 40;

% Assemble Information into Problem Structure
setup.name = 'Project';
setup.scales.method = 'automatic-bounds';
setup.functions.continuous = @fContinuous ;
setup.functions.endpoint = @fEndpoint ;
setup.bounds = bounds ;
setup.guess = guess ;
setup.mesh = mesh ;
setup.nlp.solver = 'ipopt';
setup.nlp.ipoptoptions.tolerance=1e-7; %Default is 1e-7
setup.derivatives.supplier ='sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.method = 'RPM-Differentiation';

% Solve Problem Using GPOPS2
output = gpops2 ( setup );
solution = output.result.solution ;

%% PLOTS
close all
figure
hold on
for i=1:N_fases
    time=solution.phase(i).time;
    plot (time,solution.phase(i).state(:,1),'b--o')
    plot (time,solution.phase(i).state(:,2),'r--o')
    plot (time,solution.phase(i).state(:,3),'k--o')
    vline(time(end));
    legend({'X (m)','Y (m)','Z (m)'},'Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Position','Interpreter','latex')
end 

printeps(1,'position');


figure
hold on
for i=1:N_fases
    time=solution.phase(i).time;
    plot (time,(180/pi)*solution.phase(i).state(:,4),'b--o')
    plot (time,(180/pi)*solution.phase(i).state(:,5),'r--o')
    plot (time,(180/pi)*solution.phase(i).state(:,6),'k--o')
    vline(time(end));
    legend({'yaw (deg)','pitch (deg)','roll (deg)'},'Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Orientation','Interpreter','latex')
    
end 

printeps(2,'orientation');

figure
hold on
for i=1:N_fases
    time=solution.phase(i).time;
    plot (time,solution.phase(i).control(:,1),'b--o')
    plot (time,solution.phase(i).control(:,2),'r--o')
    plot (time,solution.phase(i).control(:,3),'k--o')
    plot (time,solution.phase(i).control(:,4),'g--o')
    vline(time(end));
    legend({'$T (N)$','$\tau_{yaw} (Nm)$','$\tau_{pitch} (Nm)$','$\tau_{roll} (Nm)$'},'Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Control','Interpreter','latex')
end 

printeps(3,'control');

figure
hold on
for i=1:N_fases
    time=solution.phase(i).time;
    plot (time,solution.phase(i).state(:,7),'b--o')
    plot (time,solution.phase(i).state(:,8),'r--o')
    plot (time,solution.phase(i).state(:,9),'k--o')
    vline(time(end));
    legend({'$\dot{x} (m/s)$','$\dot{y} (m/s)$','$\dot{z} (m/s)$'},'Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Velocities','Interpreter','latex')
    
end 
printeps(4,'velocities');

figure
hold on
for i=1:N_fases
    time=solution.phase(i).time;
    plot (time,solution.phase(i).state(:,10),'g--o')
    plot (time,solution.phase(i).state(:,11),'m--o')
    plot (time,solution.phase(i).state(:,12),'k--o')
    vline(time(end));
    legend({'$p (\frac{rd}{s})$','$q (\frac{rd}{s})$','$r (\frac{rd}{s})$'},'Interpreter','latex')
    xlabel('Time (s)','Interpreter','latex')
    title('Angular rates','Interpreter','latex')
    
end 
printeps(5,'angular_rates');


figure
hold on;
gates=[];
positions=[];
anglesypr=[];
for i=1:N_fases
positions=[positions [solution.phase(i).state(:,1),solution.phase(i).state(:,2),solution.phase(i).state(:,3)]'];
anglesypr=[anglesypr [solution.phase(i).state(:,4),solution.phase(i).state(:,5),solution.phase(i).state(:,6)]'];
end
s = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
last_cell="S19:"+s(18+N_fases)+"21";
gates = xlsread('data.xlsx',1, last_cell);


plot3 (positions(1,:),positions(2,:),positions(3,:))
scatter3(gates(1,:),gates(2,:),gates(3,:),100,'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
s = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
last_cell="S22:"+s(18+N_fases)+"22";
angles = xlsread('data.xlsx',1, last_cell);
last_cell="S24:"+s(18+N_fases)+"24";
rolls = xlsread('data.xlsx',1, last_cell);
% for i=1:length(gates)
for i=1:length(gates)
    translation=[gates(1,i), gates(2,i), gates(3,i)];
    plotgate(angles(i), rolls(i), translation);
end
xlabel('X (m)','Interpreter','latex');
ylabel('Y (m)','Interpreter','latex');
zlabel('Z (m)','Interpreter','latex');
title('3D trajectory','Interpreter','latex')


Utotal_vector=[];
U2=[];
S2=[];
time=[];
integ_U2=[];
integ_S2=[];
integ_total=[];
for i=1:N_fases
    U=solution.phase(i).control;  
    S=solution.phase(i).state;
    time=[time; solution.phase(i).time];
    U2=[U2; dot(U,U,2)];
    S2=[S2; dot(S,S,2)];
end

for k = 2:length(time)
    integ_U2=[integ_U2; trapz(time(1:k), U2(1:k))];
    integ_S2=[integ_S2; trapz(time(1:k), S2(1:k))];
    integ_total=[integ_total; integ_U2(end)+integ_S2(end)];
end 

    figure 
     h = animatedline('Color','g','LineWidth',4); 
     h2 = animatedline('Color','b','LineWidth',4);
    h3 = animatedline('Color','r','LineWidth',4);
    axis([0,time(end),min(min(positions)),max(max(positions))]) 
    xlabel('Time(s)')
    legend({'$X (m)$','$Y (m)$','$Z (m)$'},'Interpreter','latex')
    for k = 2:(length(time)-1) 
      addpoints(h,time(k),positions(1,k)) 
      addpoints(h2,time(k),positions(2,k)) 
      addpoints(h3,time(k),positions(3,k))
      drawnow update 
      pause(time(k)-time(k-1))
    end 
    
    anglesypr=anglesypr*180/pi;
      figure 
     h = animatedline('Color','g','LineWidth',4); 
     h2 = animatedline('Color','b','LineWidth',4);
    h3 = animatedline('Color','r','LineWidth',4);
    axis([0,time(end),min(min(anglesypr)),max(max(anglesypr))]) 
    xlabel('Time(s)')
    legend({'$yaw (deg)$','$pitch (deg)$','$roll (deg)$'},'Interpreter','latex')
    for k = 2:(length(time)-1) 
      addpoints(h,time(k),anglesypr(1,k)) 
      addpoints(h2,time(k),anglesypr(2,k)) 
      addpoints(h3,time(k),anglesypr(3,k))
      drawnow update 
      pause(time(k)-time(k-1))
    end 
    
disp('objective= ')
output.result.objective


%Commented out to not overwrite the existing one:
%writeSolution(solution);

function [bounds, guess]=do_setup()
    global N_fases;
    global Pxw0 Pxwf Pyw0 Pywf Pzw0 Pzwf yaw0 yawf pitch0 pitchf roll0 rollf dpx0 dpxf dpy0 dpyf dpz0 dpzf p0 pf q0 qf r0 rf T0 Tf tauy0 tauyf taup0 taupf taur0 taurf intguess;
    global Pxw0_min Pxw0_max Pxw_min Pxw_max Pxwf_min Pxwf_max Pyw0_min Pyw0_max Pyw_min Pyw_max Pywf_min Pywf_max Pzw0_min Pzw0_max...
    Pzw_min Pzw_max Pzwf_min Pzwf_max yaw0_min yaw0_max yaw_min yaw_max yawf_min yawf_max pitch0_min pitch0_max pitch_min pitch_max...
    pitchf_min pitchf_max roll0_min roll0_max roll_min roll_max rollf_min rollf_max dpx0_min dpx0_max dpx_min dpx_max dpxf_min...
    dpxf_max dpy0_min dpy0_max dpy_min dpy_max dpyf_min dpyf_max dpz0_min dpz0_max dpz_min dpz_max dpzf_min dpzf_max p0_min p0_max...
    p_min p_max pf_min pf_max q0_min q0_max q_min q_max qf_min qf_max r0_min r0_max r_min r_max rf_min rf_max T0_min T0_max T_min...
    T_max Tf_min Tf_max tauy0_min tauy0_max tauy_min tauy_max tauyf_min tauyf_max taup0_min taup0_max taup_min taup_max taupf_min...
    taupf_max taur0_min taur0_max taur_min taur_max taurf_min taurf_max t0min t0max tfmin tfmax integral_min integral_max;
    global n
    s = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
    last_cell="S22:"+s(18+N_fases)+"22";
    angles = xlsread('data.xlsx',1, last_cell);
    for i = 1:N_fases
        bounds.phase(i).initialtime.lower = t0min(i);
        bounds.phase(i).initialtime.upper = t0max(i);
        bounds.phase(i).finaltime.lower = tfmin(i);
        bounds.phase(i).finaltime.upper = tfmax(i);
        bounds.phase(i).initialstate.lower = [Pxw0_min(i), Pyw0_min(i), Pzw0_min(i), yaw0_min(i), pitch0_min(i), roll0_min(i), dpx0_min(i), dpy0_min(i), dpz0_min(i), p0_min(i), q0_min(i), r0_min(i)];
        bounds.phase(i).initialstate.upper = [Pxw0_max(i), Pyw0_max(i), Pzw0_max(i), yaw0_max(i), pitch0_max(i), roll0_max(i), dpx0_max(i), dpy0_max(i), dpz0_max(i), p0_max(i), q0_max(i), r0_max(i)];
        bounds.phase(i).state.lower = [Pxw_min(i), Pyw_min(i), Pzw_min(i), yaw_min(i), pitch_min(i), roll_min(i), dpx_min(i), dpy_min(i), dpz_min(i), p_min(i), q_min(i), r_min(i)];
        bounds.phase(i).state.upper = [Pxw_max(i), Pyw_max(i), Pzw_max(i), yaw_max(i), pitch_max(i), roll_max(i), dpx_max(i), dpy_max(i), dpz_max(i), p_max(i), q_max(i), r_max(i)];
        bounds.phase(i).finalstate.lower = [Pxwf_min(i), Pywf_min(i), Pzwf_min(i), yawf_min(i), pitchf_min(i), rollf_min(i), dpxf_min(i), dpyf_min(i), dpzf_min(i), pf_min(i), qf_min(i), rf_min(i)];
        bounds.phase(i).finalstate.upper = [Pxwf_max(i), Pywf_max(i), Pzwf_max(i), yawf_max(i), pitchf_max(i), rollf_max(i), dpxf_max(i), dpyf_max(i), dpzf_max(i), pf_max(i) ,qf_max(i), rf_max(i)];
        bounds.phase(i).control.lower = [T_min(i), tauy_min(i), taup_min(i), taur_min(i)];
        bounds.phase(i).control.upper = [T_max(i), tauy_max(i), taup_max(i), taur_max(i)];
        bounds.phase(i).integral.lower=integral_min(i);
        bounds.phase(i).integral.upper=integral_max(i);
        %Guesses for the solution
        guess.phase(i).time = [ t0min(i) ; tfmax(i) ];
        guess.phase(i).state = [[Pxw0(i) ; Pxwf(i)],[Pyw0(i); Pywf(i)],[Pzw0(i); Pzwf(i)],[yaw0(i); yawf(i)],[pitch0(i); pitchf(i)],[roll0(i); rollf(i)],[dpx0(i); dpxf(i)],[dpy0(i); dpyf(i)],...
            [dpz0(i); dpzf(i)],[p0(i); pf(i)],[q0(i); qf(i)],[r0(i); rf(i)]];
        guess.phase(i).control = [[T0(i); Tf(i)],[tauy0(i); tauyf(i)],[taup0(i); taupf(i)],[taur0(i); taurf(i)]];
        guess.phase(i).integral = intguess(i);
        yaw=angles(i);
        n=[n [cos(yaw), sin(yaw),0]'];
    end 

end

function read_guesses()
    global N_fases;
    global Pxw0 Pxwf Pyw0 Pywf Pzw0 Pzwf yaw0 yawf pitch0 pitchf roll0 rollf dpx0 dpxf dpy0 dpyf...
        dpz0 dpzf p0 pf q0 qf r0 rf T0 Tf tauy0 tauyf taup0 taupf taur0 taurf intguess;
    s = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
    last_cell="B2:"+s(1+N_fases)+"34";
    B = xlsread('data.xlsx',2,last_cell);
    B_cell= num2cell(B,2);
    [Pxw0 Pxwf Pyw0 Pywf Pzw0 Pzwf yaw0 yawf pitch0 pitchf roll0 rollf dpx0 dpxf dpy0 dpyf...
        dpz0 dpzf p0 pf q0 qf r0 rf T0 Tf tauy0 tauyf taup0 taupf taur0 taurf intguess]=deal(B_cell{:});
end 
function read_constraints()
    global N_fases;
    global Pxw0_min Pxw0_max Pxw_min Pxw_max Pxwf_min Pxwf_max Pyw0_min Pyw0_max Pyw_min Pyw_max Pywf_min Pywf_max Pzw0_min Pzw0_max...
    Pzw_min Pzw_max Pzwf_min Pzwf_max yaw0_min yaw0_max yaw_min yaw_max yawf_min yawf_max pitch0_min pitch0_max pitch_min pitch_max...
    pitchf_min pitchf_max roll0_min roll0_max roll_min roll_max rollf_min rollf_max dpx0_min dpx0_max dpx_min dpx_max dpxf_min...
    dpxf_max dpy0_min dpy0_max dpy_min dpy_max dpyf_min dpyf_max dpz0_min dpz0_max dpz_min dpz_max dpzf_min dpzf_max p0_min p0_max...
    p_min p_max pf_min pf_max q0_min q0_max q_min q_max qf_min qf_max r0_min r0_max r_min r_max rf_min rf_max T0_min T0_max T_min...
    T_max Tf_min Tf_max tauy0_min tauy0_max tauy_min tauy_max tauyf_min tauyf_max taup0_min taup0_max taup_min taup_max taupf_min...
    taupf_max taur0_min taur0_max taur_min taur_max taurf_min taurf_max t0min t0max tfmin tfmax integral_min integral_max;

    s = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
    last_cell="B2:"+s(2+N_fases)+"103";
    B = xlsread('data.xlsx',1, last_cell);
    B_cell= num2cell(B,2);
    [Pxw0_min Pxw0_max Pxw_min Pxw_max Pxwf_min Pxwf_max Pyw0_min Pyw0_max Pyw_min Pyw_max Pywf_min Pywf_max Pzw0_min Pzw0_max...
    Pzw_min Pzw_max Pzwf_min Pzwf_max yaw0_min yaw0_max yaw_min yaw_max yawf_min yawf_max pitch0_min pitch0_max pitch_min pitch_max...
    pitchf_min pitchf_max roll0_min roll0_max roll_min roll_max rollf_min rollf_max dpx0_min dpx0_max dpx_min dpx_max dpxf_min...
    dpxf_max dpy0_min dpy0_max dpy_min dpy_max dpyf_min dpyf_max dpz0_min dpz0_max dpz_min dpz_max dpzf_min dpzf_max p0_min p0_max...
    p_min p_max pf_min pf_max q0_min q0_max q_min q_max qf_min qf_max r0_min r0_max r_min r_max rf_min rf_max T0_min T0_max T_min...
    T_max Tf_min Tf_max tauy0_min tauy0_max tauy_min tauy_max tauyf_min tauyf_max taup0_min taup0_max taup_min taup_max taupf_min...
    taupf_max taur0_min taur0_max taur_min taur_max taurf_min taurf_max t0min t0max tfmin tfmax integral_min integral_max]=deal(B_cell{:});
end 

function writeSolution (solution)
    global N_fases
    time=[];
    yaw=[];
    pitch=[];
    roll=[];
    X=[];
    Y=[];
    Z=[];
    XQ=[];  YQ=[];  ZQ=[];  yawQ=[];  pitchQ=[];  rollQ=[]; TQ=[];
    for i=1:N_fases
        time=[time ; solution.phase(i).time];
        X=[X ; solution.phase(i).state(:,1)];
        Y=[Y ; solution.phase(i).state(:,2)];
        Z=[Z ; solution.phase(i).state(:,3)];
        yaw=[yaw ; solution.phase(i).state(:,4)];
        pitch=[pitch ; solution.phase(i).state(:,5)];
        roll=[roll ; solution.phase(i).state(:,6)];
        time_p=solution.phase(i).time;
        timeq = (time_p(1):0.005:time_p(end))';
        TQ=[TQ; timeq];
        XQ =[XQ; interp1(time_p,solution.phase(i).state(:,1),timeq,'spline')];
        YQ =[YQ; interp1(time_p,solution.phase(i).state(:,2),timeq,'spline')];
        ZQ =[ZQ; interp1(time_p,solution.phase(i).state(:,3),timeq,'spline')];
        yawQ =[yawQ; interp1(time_p,solution.phase(i).state(:,4),timeq,'spline')];
        pitchQ =[pitchQ; interp1(time_p,solution.phase(i).state(:,5),timeq,'spline')];
        rollQ =[rollQ; interp1(time_p,solution.phase(i).state(:,6),timeq,'spline')];
    end 

    delta=diff(TQ);
    delta=[delta;delta(end)];

    filename = 'solutions.xlsx';
    A = [12.7 5.02 -98 63.9 0 -.2 56];
    sheet = 1;
    xlRange = 'A2';
    xlswrite(filename,XQ,sheet,xlRange)
    xlRange = 'B2';
    xlswrite(filename,YQ,sheet,xlRange)
    xlRange = 'C2';
    xlswrite(filename,ZQ,sheet,xlRange)
    xlRange = 'D2';
    xlswrite(filename,yawQ,sheet,xlRange)
    xlRange = 'E2';
    xlswrite(filename,pitchQ,sheet,xlRange)
    xlRange = 'F2';
    xlswrite(filename,rollQ,sheet,xlRange)
    xlRange = 'G2';
    xlswrite(filename,delta,sheet,xlRange)
end 