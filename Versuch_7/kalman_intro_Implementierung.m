close;
clear;
clc;

% Parameter definieren
vals = 100;                   % Anzahl der Messwerte
sigma = 5.0;                  % Standardabweichung des Messrauschens
dT = 1;                     % Zeitschritt

% Zustandsübergangsmatrix (Konstante Geschwindigkeit Modell)
A = [1, 0, dT, 0;
     0, 1, 0,  dT;
     0, 0, 1,  0;
     0, 0, 0,  1];

% Wahrer Anfangszustand [x, y, vx, vy]
x_wahr = [0; 0; 1; 1];

% Messmatrix (nur Position messbar)
H = [1, 0, 0, 0;
     0, 1, 0, 0];

% Messrauschkovarianz
R = [sigma 0; 0 sigma];

% Prozessrauschkovarianz
Q = [0 0 0 0; 0 0 0 0; 0 0 1e-4 0; 0 0 0 1e-4]; 

% Anfangszustand und Kovarianz für Filter
x_schaetzung = [0; 0; 1; 1];        % Konservative Anfangsschätzung
P = [0 0 0 0; 0 0 0 0; 0 0 0.1 0; 0 0 0 0.1];              % Große Anfangsunsicherheit

% Messdaten generieren
messdaten = generate_noisy_data(A, x_wahr, vals, sigma);

% Speicher für Filterergebnisse vorallokieren
x_gefiltert = zeros(vals, 2);

% Kalman-Filter Hauptschleife
for i = 1:vals
    % Prädiktionsschritt
    x_praediktion = A * x_schaetzung;
    P_praediktion = A * P * A' + Q;
    
    % Korrekturschritt
    y_innovation = messdaten(i, :)' - H * x_praediktion;
    S = H * P_praediktion * H' + R;
    K = P_praediktion * H' / S;              % Effizienter als inv()
    
    % Zustandsupdate
    x_schaetzung = x_praediktion + K * y_innovation;
    P = (eye(4) - K * H) * P_praediktion;
    
    % Ergebnis speichern (nur Position)
    x_gefiltert(i, :) = x_schaetzung(1:2)';
end

% Visualisierung
figure;
plot(messdaten(:,1), messdaten(:,2), 'bx', 'MarkerSize', 6, 'DisplayName', 'Verrauschte Messungen');
hold on;
plot(x_gefiltert(:,1), x_gefiltert(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'Kalman-Filter');
xlabel('X-Position');
ylabel('Y-Position');
title('Kalman-Filter Tracking');
legend('show');
grid on;
hold off;