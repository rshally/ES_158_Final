close all

finalIndex = 100;
deltaT = 0.05;
Q = eye(3);
F = 1000*eye(3);
R = eye(2);
%what you measure is just x and y
C = [1 0 0; 0 1 0; 0 0 0];

%standard deviation for the two covariance matrices
sigmaV = .01;
sigmaW = .1;

%Initial x and xhat
x = cell(finalIndex,1);
%The values are the difference between x and the reference (i.e. [0 0 0]
%would start on the reference path
x{1} = [1 0 pi/4]';
xhat = x;

%so covariance matrices are sigma^2*identity matrix (3x3)
noiseV = cell(finalIndex,1);
for i = 1:finalIndex
    noiseV{i} = [normrnd(0,sigmaV); normrnd(0,sigmaV); normrnd(0,sigmaV)];
end

noiseVcovar = sigmaV^2*eye(3);
noiseWcovar = sigmaW^2*eye(3);

noiseW = cell(finalIndex,1);
for i = 1:finalIndex
    noiseW{i} = [normrnd(0,sigmaW); normrnd(0,sigmaW); normrnd(0,sigmaW)];
end

A = cell(finalIndex,1);
B = A;
for i = 1:finalIndex
    A{i} = eye(3)+deltaT*[0 0 -cos((i-1)*deltaT); 0 0 -sin((i-1)*deltaT); 0 0 0];
    B{i} = deltaT*[-sin((i-1)*deltaT) 0; cos((i-1)*deltaT) 0; 0 1];
end

P = cell(finalIndex,1);
P{1} = zeros(3);
for i = 1:finalIndex
    P{i+1} = A{i}*(P{i}-P{i}*transpose(C)*inv(C*P{i}*transpose(C)+noiseWcovar)...
        *C*P{i})*transpose(A{i})+noiseVcovar;
end


S = cell(finalIndex,1);
S{finalIndex} = F;
for i = finalIndex-1:-1:1
S{i} = transpose(A{i})*(S{i+1}-S{i+1}*B{i}*...
    inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1})*A{i}+Q;
end

K = cell(finalIndex,1);
for i = 1:finalIndex
K{i} = P{i}*transpose(C)*inv(C*P{i}*transpose(C)+noiseWcovar);
end

L = cell(finalIndex-1,1);
u = L;
y = L;
for i = 1:finalIndex-1
L{i} = inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1}*A{i};
u{i} = -L{i}*x{i};
y{i} = C*x{i}+noiseW{i};
x{i+1} = A{i}*x{i}+B{i}*u{i}+noiseV{i};
end

for i = 1:finalIndex-2
xhat{i+1} = A{i}*xhat{i}+B{i}*u{i}+K{i+1}*(y{i+1}-C*(A{i}*xhat{i}+B{i}*u{i}));
end

xTilda = 1:finalIndex-1;
for i = 1:finalIndex-1
    xTilda(i) = xhat{i}(1);
end

yTilda = 1:finalIndex-1;
for i = 1:finalIndex-1
    yTilda(i) = xhat{i}(2);
end

xRef = 1:finalIndex-1;
for i = 1:finalIndex-1
    xRef(i) = cos(i*deltaT);
end

yRef = 1:finalIndex-1;
for i = 1:finalIndex-1
    yRef(i) = sin(i*deltaT);
end

%Because, e.g. x tilda = xhat-xRef
xValues = xTilda+xRef;
yValues = yTilda+yRef;

plot(xRef,yRef)
hold on
plot(xValues,yValues)
hold off