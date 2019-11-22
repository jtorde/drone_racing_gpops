  
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
function plotgate(yaw,roll, trans)
%First I rotate the object, and then I translate it
yaw=yaw+pi/2;
fv=readSTL("window.stl");
offset = repmat([-0.12 0 -0.08],size(fv.vertices,1),1);
fv.vertices=fv.vertices+offset;


    T_trans=[[1 0 0 0];[0 1 0 0];[0 0 1 0];[trans(1) trans(2) trans(3) 1]];
    T_rot_yaw=[[cos(yaw) sin(yaw) 0 0];[-sin(yaw) cos(yaw) 0 0]; [0 0 1 0]; [0 0 0 1]];
    T_rot_roll=makehgtform('yrotate',roll); 
    T = repmat(trans,size(fv.vertices,1),1);
    gate=[];
    roll;
    for i=1:size(fv.vertices,1)
        point=([fv.vertices(i,:) 1]*T_rot_roll*T_rot_yaw*T_trans)';
        gate=[gate; point(1:3,1)'];
    end 
    fv.vertices=fv.vertices+T;
    fv.vertices=gate;
    hSurface=patch(fv,'FaceColor',      'red', ...
             'EdgeColor',       'red');

    % Add a camera light, and tone down the specular highlighting
    camlight('headlight');
    material('dull');

    % Fix the axes scaling, and set a nice view angle
    axis('image');
    view([-135 35]);
end 