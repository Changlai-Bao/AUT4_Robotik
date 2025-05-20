% RANSAC Algorithmus für Linienmodell
% Dieser Algorithmus versucht, eine Linie in einem Datensatz mit Ausreißern zu finden
%
% Autor: Changlia Bao
% Datum: 20.05.2025

close;
clear;
clc;

% Modellparameter (unbekannt -> zu suchen)
x0 = 3;
y0 = 4;
r = 2;

% Anzahl Messwerte
vals = 100;

% Generiere Messwerte
data=generate_data(x0, y0, r, vals);

best_match = 0;
consensus_max = 0;

% Anzahl an Stichprobenüberprüfungen
trials = 20;

for i=1:trials
	n = 2;
	% 2 Zufallszahlen im gültigen Wertebereich suchen
	idx1 = round(rand * (vals-1)+0.5);
	idx2 = round(rand * (vals-1)+0.5);

	% Sicherstellen, dass nicht gleiche Indizes gewählt wurden
	if idx1 == idx2
		continue
	end

	% Sample set
	x = [data(idx1, 1), data(idx2, 1)];
	y = [data(idx1, 2), data(idx2, 2)];

	% Modellparameter bestimmen: y = m*x + t
    m = (y(2) - y(1)) / (x(2) - x(1));
    t = y(1) - m * x(1);

	% Consensus set finden und bewerten
    inliers = 0;
    for j = 1:vals
        % Berechne quadratischen Abstand des Punktes zur Gerade
        dist = (data(j, 2) - (m * data(j, 1) + t))^2;
        if dist < 0.01
            inliers = inliers + 1;
        end
    end

	% Beste Übereinstimmung merken
    if inliers > consensus_max
        best_match = [m; t];
        consensus_max = inliers;
    end
end

% Beste Übereinstimmung ausgeben
x_range = linspace(min(data(:,1))-1, max(data(:,1))+1, 100);
y_line = best_match(1) * x_range + best_match(2);
fprintf('Bestes Linienmodell: y = %.4f*x + %.4f mit %d Inliern (%.1f%%)\n', best_match(1), best_match(2), consensus_max, 100*consensus_max/vals)

% Plotte Messwerte
clf;
plot(data(:,1),data(:,2), 'x');
hold on;
plot(x_range, y_line, 'r-', 'LineWidth', 2);
title('RANSAC Linieschätzung');
xlabel('x');
ylabel('y');
legend('Datenpunkte', 'Geschätzte Linie');
grid on;
axis equal;

