T0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 -1 -1 1];
T1 = [cos(30) 0 0 0; 0 sin(45) 0 0; 0 0 sin(25) 0; 10 5 2 1];
tc = ctraj(T0,T1,10);

x0 = [zeros(3,1); 1];

x = [];

x = [x,x0];

for i = 1 : 10
    M = tc(:,:,i)
    t = M*x0
    x = [x,t];
    x0 = t;
    pause
   
end