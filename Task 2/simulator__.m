clc 
clear 
% constants 
t_final  = 15;
t_initial = 0;
h = 0.15;
t_vec = 0:0.15:15;
n = length(t_vec);
forces = [10;5;9];
Moments = [10; 20; 5];
Mass = 15;
I = [1 -2 -1;-2 5 -3;-1 -3 0.1];


%Initial Conditions
%[u; v; w; p; q; r; phi; theta; epsi; x; y; z]
IC_vec = [10; 2; 0; 2*pi/180; pi/180; 0; 20*pi/180; 15*pi/180; 30*pi/180; 2; 4; 7];


%Output_Matrix
Result = NaN(12,n);
K1vec = NaN(12,1);
K2vec = NaN(12,1);
K3vec = NaN(12,1);
K4vec = NaN(12,1);

Result(:,1) =IC_vec;

for i =1:n-1
    IC_vec_update = Result(:,i);
    states = f_x_y(IC_vec_update,forces,Moments,I,Mass);
    K1vec = K__1(states,h);
    states = f_x_y(IC_vec_update+0.5*K1vec,forces,Moments,I,Mass);
    K2vec = K__1(states,h);
    states = f_x_y(IC_vec_update+0.5*K2vec,forces,Moments,I,Mass);
    K3vec = K__1(states,h);
    states = f_x_y(IC_vec_update+K3vec,forces,Moments,I,Mass);
    K4vec = K__1(states,h);
    for ii=1:length(K1vec)
        Result(ii,i+1) = Result(ii,i)+1/6*(K1vec(ii)+2*K2vec(ii)+2*K3vec(ii)+K4vec(ii));
   
    end 
    
end 

function K1 =K__1(states,h)
    for i=1:length(states)
        K1(i,1) = h*states(i);
    end
  
end

