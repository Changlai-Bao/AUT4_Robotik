close;
clear;
clc;

% Daten laden
poses = load('poses.txt', '-ascii');
samples = size(poses, 1) - 1;

% Datensätze extrahieren
odom = poses(2:end, 1:2);
gps = poses(2:end, 3:4);
ground_truth = poses(2:end, 5:6);

% Kalman-Filter Parameter
dT = 0.999;

% Zustandsübergangsmatrix (Konstante Geschwindigkeit)
A = [1, 0, dT, 0;
     0, 1, 0,  dT;
     0, 0, 1,  0;
     0, 0, 0,  1];

% Steuermatrix (Positionskorrektur durch Odometrie)
B = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0];

% Beobachtungsmatrix (nur Position messbar)
H = [1, 0, 0, 0;
     0, 1, 0, 0];

% Anfangszustand [x, y, vx, vy]
x = [poses(1, 1:2)'; 0; 0];

% Anfangskovarianz
P = diag([1, 1, 0, 0]);

% Prozessrauschkovarianz
ax = 1e-5;
ay = 1e-5;
Q = [0.25*dT^2*ax,    0,         0.5*dT*ax,    0;
     0,               0.25*dT^2*ay, 0,        0.5*dT*ay;
     0.5*dT*ax,       0,         ax,           0;
     0,               0.5*dT*ay,     0,        ay];

% Messrauschkovarianz
R = eye(2);

% Speicher für Ergebnisse vorallokieren
x_history = zeros(4, samples + 1);
x_history(:, 1) = x;

% Odometrie-Tracking
odom_prev = poses(1, 1:2);

% Hauptschleife des Kalman-Filters
for k = 1:samples
    % GPS-Messung
    z = gps(k, :)';
    
    % Steuervektor aus Odometrie berechnen
    delta_odom = odom(k, :) - odom_prev;
    u = [delta_odom'; 0; 0];
    odom_prev = odom(k, :);
    
    % Prädiktionsschritt
    x_pred = A * x + B * u;
    P_pred = A * P * A' + Q;
    
    % Korrekturschritt
    y_innovation = z - H * x_pred;
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;                  
    
    % Zustandsupdate
    x = x_pred + K * y_innovation;
    P = (eye(4) - K * H) * P_pred;
    
    % Ergebnis speichern
    x_history(:, k + 1) = x;
end

% Visualisierung
figure;
plot(x_history(1, :), x_history(2, :), 'r-', 'LineWidth', 2, 'DisplayName', 'Kalman-Filter');
hold on;
plot(ground_truth(:, 1), ground_truth(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
plot(odom(:, 1), odom(:, 2), 'b--', 'LineWidth', 1, 'DisplayName', 'Odometrie');
plot(gps(:, 1), gps(:, 2), 'kx', 'MarkerSize', 3, 'DisplayName', 'GPS');
xlabel('X-Position [m]');
ylabel('Y-Position [m]');
title('Sensor-Fusion: GPS + Odometrie');
legend('show');
grid on;
hold off;