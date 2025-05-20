% RANSAC Algorithmus für Kreismodell
% Dieser Algorithmus versucht, einen Kreis in einem Datensatz mit Ausreißern zu finden
%
% Autor: Changlai Bao
% Datum: 20.05.2025

close;
clear;
clc;

% Modellparameter (unbekannt -> zu suchen)
x0 = 3;
y0 = 4;
r = 2;

% Anzahl Messwerte
vals = 200;

% Generiere Messwerte
data = generate_data(x0, y0, r, vals);

best_match = 0;
consensus_max = 0;
best_x0 = 0;
best_y0 = 0;
best_r = 0;
best_inliers = [];

% Anzahl an Stichprobenüberprüfungen
trials = 200;

for i = 1:trials
    n = 3;
    % 3 Zufallszahlen im gültigen Wertebereich suchen
	idx1 = round(rand * (vals-1)+0.5);
	idx2 = round(rand * (vals-1)+0.5);
    idx3 = round(rand * (vals-1)+0.5);

	% Sicherstellen, dass nicht gleiche Indizes gewählt wurden
    if idx1 == idx2 || idx1 == idx3 || idx2 == idx3
      continue
    end

    % Sample set
	x = [data(idx1, 1), data(idx2, 1), data(idx3, 1)];
	y = [data(idx1, 2), data(idx2, 2), data(idx3, 1)];

    % Modellparameter bestimmen: (x-x0)²+(y-y0)²=r²
    % Berechne die Parameter a1, a2, b1, b2, c1, c2

    % Berechne die Summen für alle Datenpunkte
    sum_x = sum(x);
    sum_y = sum(y);
    sum_x2 = sum(x.^2);
    sum_y2 = sum(y.^2);
    sum_xy = sum(x.*y);
    sum_x3 = sum(x.^3);
    sum_y3 = sum(y.^3);
    sum_x2y = sum(x.^2.*y);
    sum_xy2 = sum(x.*y.^2);

    % Berechne Parameter
    a1 = 2 * ((sum_x)^2 - n * sum_x2);
    a2 = 2 * (sum_x * sum_y - n * sum_xy);
    b1 = a2;
    b2 = 2 * ((sum_y)^2 - n * sum_y2);

    c1 = sum_x2 * sum_x - n * sum_x3 + sum_x * sum_y2 - n * sum_xy2;
    c2 = sum_x2 * sum_y - n * sum_y3 + sum_y * sum_y2 - n * sum_x2y;

    % Löse das lineare Gleichungssystem für x0 und y0
    det = a1 * b2 - a2 * b1;

    x0_Lsg = (c1 * b2 - c2 * b1) / det;
    y0_Lsg = (a1 * c2 - a2 * c1) / det;

    % Berechne Radius r
    r_Lsg = sqrt(1/n * (sum_x2 - 2*x0_Lsg*sum_x + n*x0_Lsg^2 + sum_y2 - 2*y0_Lsg*sum_y + n*y0_Lsg^2));

    % Consensus set finden und bewerten
    dist = abs(sqrt((data(:,1) - x0_Lsg).^2 + (data(:,2) - y0_Lsg).^2) - r_Lsg);
    inliers = find(dist < 0.1);
    consensus_size = length(inliers);

    % Beste Übereinstimmung merken
    if consensus_size > consensus_max
        consensus_max = consensus_size;
        best_x0 = x0_Lsg;
        best_y0 = y0_Lsg;
        best_r = r_Lsg;
        best_inliers = inliers;
    end
end

% Beste Übereinstimmung ausgeben
theta = linspace(0, 2*pi, 100);
x_Kries = best_x0 + best_r * cos(theta);
y_Kreis = best_y0 + best_r * sin(theta);
fprintf('Bestes Kreismodell: (x-%f)²+(y-%f)²=%f² mit %d Inliern (%.1f%%)\n', best_x0, best_y0, best_r, consensus_max, 100*consensus_max/vals);

% Plotte Messwerte
clf;
plot(data(:,1), data(:,2), 'x');
hold on;
plot(x_Kries, y_Kreis, 'r-', 'LineWidth', 2);
plot(best_x0, best_y0, 'r+', 'MarkerSize', 10, 'LineWidth', 2);
title('RANSAC Kreisschätzung');
xlabel('x');
ylabel('y');
legend('Datenpunkte', 'Geschätzte Kreis');
grid on;
axis equal;

