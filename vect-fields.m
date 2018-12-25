%MATH240_Project3

%potential function
syms x y t;
phixy = 2*x.^3-4*y.*x.^2+8*y.^2;
%F function
fx = diff(phixy,x);
fy = diff(phixy,y);
f = [fx,fy];
dumf = [fx,-fy];

%==========================================================================
%PART 1
%==========================================================================

%Part 1--------------------------------------------------------------------
%%%TWEEDLE DEE 
%  Here is a radial vector field F = r / r^3
%  Replace this with your non-radial conservative field
u = inline(fx,'x','y');
v = inline(fy,'x','y');
x = linspace(-10,10,11)  ;
y = linspace(-10,10,11)  ;
[X,Y] = meshgrid(x,y)  ;
U = u(X,Y)  ;
V = v(X,Y)  ;
quiver(X,Y,U,V,'color','red');
title('Tweedle Dee Vector Field (Red)')
xlabel('x');
ylabel('y');


%%%TWEEDLE DUM 
figure
%  Here is a circulation vector field F = r / r^3
%  Replace this with your non-circulation NONconservative field
u = inline(fx,'x','y');
v = inline(-fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
quiver(X,Y,U,V,'color','blue');
title('Tweedle Dum Vector Field (Blue)')
xlabel('x');
ylabel('y');

%Part 2--------------------------------------------------------------------
startpt = [-4,-2];
endpt = [2,4];
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TWEEDLE DEE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
disp('TWIDDLE DEE LINE INTEGRAL WORK:');
%straight path
syms x y t;
disp('Dee Straight Line Work:');
trange = [-4,2]; r = [t,t+2]; %t's range and r(t)
fsb=subs(f,[x,y],r); %parameterizing
dstr = dot(fsb,diff(r,t)); %dot product
z=matlabFunction(dstr); %creates function handle
straightWork=round(integral(z,trange(1),trange(2)));%finds the integral
disp(straightWork);

%GRAPH 1
figure
u = inline(fx,'x','y');
v = inline(fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = -4:0.1:2;
hold on;
fplot(r(1),r(2),'color','magenta');
quiver(X,Y,U,V,'color','red')
title('Tweedle Dee - Straight Line');
xlabel('x');
ylabel('y');
hold off

%parallel to the axis
syms x y t1 t2;
disp('Dee Parallel Axises Work:');
xrange = [-4,2]; %x range  
yrange = [-2,4]; %y range
r11 = [t1,4]; %r(right)
r12 = [-4,t2]; %r(up)
frightsub = subs(f,[x,y],r11); %parameterizing right 
fupsub = subs(f,[x,y],r12); %parameterizing up
dright = dot(frightsub,diff(r11,t1)); %dot product right
dup = dot(fupsub,diff(r12,t2)); %dot product up
rightFun = matlabFunction(dright); %right side function handle
upFun = matlabFunction(dup);%up side function handle
rightWork = integral(rightFun,xrange(1),xrange(2));%right side work
upWork = integral(upFun,yrange(1),yrange(2)); %up side work
parawork = round(rightWork+upWork);
disp(parawork);

%GRAPH 2
figure
u = inline(fx,'x','y');
v = inline(fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
hold on;
t1 = -4:1:2; % change in the x
fplot(r11(1),r11(2),'color','magenta'); %right line
t2 = -2:1:4; %changes in y
fplot(r12(1),r12(2),'color','magenta'); %up line
quiver(X,Y,U,V,'color','red')
title('Tweedle Dee - Parallel Axis Lines');
xlabel('x');
ylabel('y');
hold off

%parabolic
disp('Dee Parabolic Work:');
syms x y t;
rpara = [t,(1/6)*(t+4)^2-2];
bolicsub=subs(f,[x,y],rpara);
dbolic = dot(bolicsub,diff(rpara,t));
bolichandle=matlabFunction(dbolic);
bolicwork=round(integral(bolichandle,-4,2));
disp(bolicwork);

%GRAPH 3
figure
u = inline(fx,'x','y');
v = inline(fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = -4:1:2;
hold on;
fplot(rpara(1),rpara(2),'color','magenta');
quiver(X,Y,U,V,'color','red');
title('Tweedle Dee - Parabolic Line');
xlabel('x');
ylabel('y');
hold off

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TWEEDLE DUM~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
disp('TWIDDLE DUM LINE INTEGRAL WORK:');
%straight path
syms x y t;
disp('Dum Straight Line Work:');
trange = [-4,2]; r = [t,t+2]; %t's range and r(t)
fsb=subs(dumf,[x,y],r); %parameterizing
dstr = dot(fsb,diff(r,t)); %dot product
z=matlabFunction(dstr); %creates function handle
straightWork=round(integral(z,trange(1),trange(2)));%finds the integral
disp(straightWork);

%GRAPH 1
figure
u = inline(fx,'x','y');
v = inline(-fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = -4:0.1:2;
hold on;
fplot(r(1),r(2),'color','magenta');
quiver(X,Y,U,V,'color','blue');
title('Tweedle Dum - Straight Line')
xlabel('x');
ylabel('y');
hold off

%parallel to the axis
syms x y t1 t2;
disp('Dum Parallel Axises Work:');
xrange = [-4,2]; %x range  
yrange = [-2,4]; %y range
r11 = [t1,4]; %r(right)
r12 = [-4,t2]; %r(up)
frightsub = subs(dumf,[x,y],r11); %parameterizing right 
fupsub = subs(dumf,[x,y],r12); %parameterizing up
dright = dot(frightsub,diff(r11,t1)); %dot product right
dup = dot(fupsub,diff(r12,t2)); %dot product up
rightFun = matlabFunction(dright); %right side function handle
upFun = matlabFunction(dup);%up side function handle
rightWork = integral(rightFun,xrange(1),xrange(2));%right side work
upWork = integral(upFun,yrange(1),yrange(2)); %up side work
parawork = round(rightWork+upWork);
disp(parawork);

%GRAPH 2
figure
u = inline(fx,'x','y');
v = inline(-fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
hold on;
t1 = -4:1:2; % change in the x
fplot(r11(1),r11(2),'color','magenta'); %right line
t2 = -2:1:4; %changes in y
fplot(r12(1),r12(2),'color','magenta'); %up line
quiver(X,Y,U,V,'color','blue');
title('Tweedle Dum - Parallel Axis Lines');
xlabel('x');
ylabel('y');
hold off

%parabolic
disp('Dum Parabolic Work:');
syms x y t;
tse = [-4,2]; 
rpara = [t,(1/6)*(t+4)^2-2];
bolicsub=subs(dumf,[x,y],rpara);
dbolic = dot(bolicsub,diff(rpara,t));
bolichandle=matlabFunction(dbolic);
bolicwork=round(integral(bolichandle,-4,2));
disp(bolicwork);

%GRAPH 3
figure
u = inline(fx,'x','y');
v = inline(-fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = -4:0.1:2;
hold on;
fplot(rpara(1),rpara(2),'color','magenta');
quiver(X,Y,U,V,'color','blue');
title('Tweedle Dum - Parabolic Line');
xlabel('x');
ylabel('y');
hold off

%==========================================================================
%PART 2
%==========================================================================
%circle
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TWEEDLE DEE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
disp('CIRCLE PATH INTEGRAL WORK:');
syms x y t;
%Vector field: f
%Parametrization of the Curve:
r = [7*cos(t),7*sin(t)]; 
%Parametrization of F:
%Dot product F*dr:
pf = subs(f,[x,y],r); 
rprime = [diff(r(1),t),diff(r(2),t)];
D = dot(pf,rprime);
handleD = matlabFunction(D);
%Integral of line respect of t (dt):
circleWorkDee = round(integral(handleD,0,2*pi));
disp('Dee Circle R=7 Work:')
disp(circleWorkDee);

%Graph Circle 'ArrayValued',true
figure
u = inline(fx,'x','y');
v = inline(fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = 0:0.1:2*pi;
hold on;
fplot(r(1),r(2),'color','magenta');
quiver(X,Y,U,V,'color','red');
title('Tweedle Dee - Circle');
xlabel('x');
ylabel('y');
hold off

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TWEEDLE DUM~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
syms x y t;
%Vector field: dumf
%Parametrization of the Curve:
r = [7*cos(t),7*sin(t)]; 
%Parametrization of F:
%Dot product F*dr:
pfdum = subs(dumf,[x,y],r); 
rprime = [diff(r(1),t),diff(r(2),t)];
D = dot(pf,rprime);
handleD = matlabFunction(D);
%Integral of line respect of t (dt):
circleWorkDum = round(integral(handleD,0,2*pi));
disp('Dum Circle R=7 Work:')
disp(circleWorkDum);

%Graph Circle
figure
u = inline(fx,'x','y');
v = inline(-fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = 0:0.1:2*pi;
hold on;
fplot(r(1),r(2),'color','magenta');
quiver(X,Y,U,V,'color','blue');
title('Tweedle Dum - Circle');
xlabel('x');
ylabel('y');
hold off

%rectangle
%(t,0)(6,0)()()
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TWEEDLE DEE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%Start at (8,-2) and perform the same integration 
%over a rectangular path (up 6, left 8, down 6, right 8).
disp('RECTANGLE PATH INTEGRAL WORK:');
syms x y t;

%Green's Theorum
fy = diff(f(1),y);
gx = diff(f(2),x);
handleGreen = matlabFunction(gx-fy);
partx = int(handleGreen,x,0,8);
partall = int(partx,y,-2,4);
disp('Dee Rectangle(8w,6h) Work:');
disp(partall);

%Rectangle Graph
figure
u = inline(fx,'x','y');
v = inline(fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
hold on;
%plotted rectangle 
rectangle('Position',[0 -2 8 6],'EdgeColor', 'magenta');
%vector field
quiver(X,Y,U,V,'color','red');
title('Tweedle Dee - Rectangle');
xlabel('x');
ylabel('y');
hold off

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TWEEDLE DUM~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
syms x y t;

%Green's Theorum
fy = diff(dumf(1),y);
gx = diff(dumf(2),x);
handleGreen = matlabFunction(gx-fy);
partx = int(handleGreen,x,0,8);
partall = int(partx,y,-2,4);
disp('Dum Rectangle(8w,6h) Work:');
disp(partall);
%Rectangle Circle
figure
u = inline(fx,'x','y');
v = inline(-fy,'x','y');
x = linspace(-10,10,11);
y = linspace(-10,10,11);
[X,Y] = meshgrid(x,y);
U = u(X,Y);
V = v(X,Y);
t = 0:0.1:2*pi;
hold on;
%plotted rectangle 
rectangle('Position',[0 -2 8 6],'EdgeColor', 'magenta');
%vector field
quiver(X,Y,U,V,'color','blue');
title('Tweedle Dum - Rectangle');
xlabel('x');
ylabel('y');
hold off

