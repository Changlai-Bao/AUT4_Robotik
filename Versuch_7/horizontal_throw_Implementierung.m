close;
clear;
clc;

data = importdata('horizontal_throw.txt');
t = data(:,1);
x_meas = data(:,2);
y_meas = data(:,3);

% Initialisierungen
dT = 1.0;

% Zustandsübergangsmatrix
A = [1, 0, dT, 0; 
     0, 1, 0, dT; 
     0, 0, 1, 0; 
     0, 0, 0, 1];

B = eye(4);

u = [1e-5; 1e-5; 0; dT*-9.81e-4];

% Zustand: [x; y; vx; vy]
x = [x_meas(1); y_meas(1); 0; 0];

% Beobachtungsmatrix (H)
H = [1 0 0 0;
     0 1 0 0];

% Systemrauschen (Q)
ax = 1e-5;
ay = 1e-5;
Q = [0.25*dT^2*ax,    0,         0.5*dT*ax,    0;
     0,               0.25*dT^2*ay, 0,        0.5*dT*ay;
     0.5*dT*ax,       0,         ax,           0;
     0,               0.5*dT*ay,     0,        ay];

% Messrauschen (R)
r = 1;
R = [r 0; 0 r];

% Anfangskovarianz
P = eye(4);

% Speicher für Ergebnisse
x_filtered = zeros(length(t), 4);
x_filtered(1, :) = x';

% Kalman-Filter-Schleife
for k = 2:length(t)
% Prediction
    x = A * x + B*u;
    P = A * P * A' + Q;
% Messung
    z = [x_meas(k); y_meas(k)];
% Kalman Gain
    K = P * H' / (H * P * H' + R);
% Update
    x = x + K * (z - H * x);
    P = (eye(4) - K * H) * P;
    x_filtered(k, :) = x';
end

% Plot
figure;
plot(x_meas, y_meas, 'rx', 'DisplayName', 'Messdaten');
hold on;
plot(x_filtered(:,1), x_filtered(:,2), 'b-', 'LineWidth', 1, 'DisplayName', 'Kalman Filter');
title('Kalman-Filter auf Messdaten des freien Falls');
legend show;
grid on;