  
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

function output = fContinuous( input )
global N_fases

for i=1:N_fases
     U=input.phase(i).control;  
    S=input.phase(i).state;
    output(i).dynamics=return_dynamics(U,S);
    output(i).integrand=dot(U,U,2);
end

    
                             
function dynamics=return_dynamics(U,S)
g=-9.81;
Jx=2*4.8e-02;
Jy=2*4.8e-02;
Jz=2*8.8e-02;
m=0.5;
J = [Jx 0 0;0 Jy 0;0 0 Jz];
T=U(:,1); tauy=U(:,2); taup=U(:,3); taur=U(:,4);
Pxw=S(:,1); Pyw=S(:,2); Pzw=S(:,3); yaw=S(:,4); pitch=S(:,5); roll=S(:,6);
dpx=S(:,7); dpy=S(:,8); dpz=S(:,9); p=S(:,10); q=S(:,11); r=S(:,12);
dynamics=[...
dpz.*(sin(roll).*sin(yaw) + cos(roll).*cos(yaw).*sin(pitch)) - dpy.*(cos(roll).*sin(yaw) - cos(yaw).*sin(pitch).*sin(roll)) + dpx.*cos(pitch).*cos(yaw),...
dpy.*(cos(roll).*cos(yaw) + sin(pitch).*sin(roll).*sin(yaw)) - dpz.*(cos(yaw).*sin(roll) - cos(roll).*sin(pitch).*sin(yaw)) + dpx.*cos(pitch).*sin(yaw),...
dpz.*cos(pitch).*cos(roll) - dpx.*sin(pitch) + dpy.*cos(pitch).*sin(roll),...
(r.*cos(roll))./cos(pitch) + (q.*sin(roll))./cos(pitch),...
q.*cos(roll) - r.*sin(roll),...
p + (r.*cos(roll).*sin(pitch))./cos(pitch) + (q.*sin(pitch).*sin(roll))./cos(pitch),...
dpy.*r - dpz.*q - g.*sin(pitch),...
dpz.*p - dpx.*r + g.*cos(pitch).*sin(roll),...
dpx.*q - dpy.*p + T./m + g.*cos(pitch).*cos(roll),...
(taur + Jy.*q.*r - Jz.*q.*r)./Jx,...
(taup - Jx.*p.*r + Jz.*p.*r)./Jy,...
(tauy + Jx.*p.*q - Jy.*p.*q)./Jz];
                                                                                                               
                                                                                                               
                                 