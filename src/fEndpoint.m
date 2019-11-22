  
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


function output = fEndpoint (input)
global a n
a=input;
global angle_cone
global N_fases
for i=1:(N_fases-1)
    Si_f=input.phase(i).finalstate;
    Sip1_0=input.phase(i+1).initialstate;
    ti_f = input.phase(i).finaltime;
    tip1_0 = input.phase(i+1).initialtime;
    
    yaw=Si_f(:,4); pitch=Si_f(:,5); roll=Si_f(:,6);
    dpx=Si_f(:,7); dpy=Si_f(:,8); dpz=Si_f(:,9);
    
    %Velocity of the drone expressed in the world frame:
    Vx=dpz.*(sin(roll).*sin(yaw) + cos(roll).*cos(yaw).*sin(pitch)) - dpy.*(cos(roll).*sin(yaw) - cos(yaw).*sin(pitch).*sin(roll)) + dpx.*cos(pitch).*cos(yaw);
    Vy=dpy.*(cos(roll).*cos(yaw) + sin(pitch).*sin(roll).*sin(yaw)) - dpz.*(cos(yaw).*sin(roll) - cos(roll).*sin(pitch).*sin(yaw)) + dpx.*cos(pitch).*sin(yaw);
    Vz=dpz.*cos(pitch).*cos(roll) - dpx.*sin(pitch) + dpy.*cos(pitch).*sin(roll);
    vel=[Vx, Vy,Vz];

    proy_real=abs(dot(n(:,i)',vel)); 
    proy_min=abs(norm(vel)*cos(angle_cone*pi/180));
    difference=proy_real-proy_min; %diff should be >=0
    output.eventgroup(i).event = [Si_f-Sip1_0,ti_f-tip1_0, difference];
end

J_control=0;
for i=1:N_fases
    J_control = J_control +input.phase(i).integral;
end 

output.objective=25*input.phase(N_fases).finaltime + J_control;
