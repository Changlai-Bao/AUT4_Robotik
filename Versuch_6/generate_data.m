% Funktion zur Erzeugung pseudozufälliger Daten auf einer Kreisbahn/Geraden
%
% Parameter:
% x0: x-Koordinate des Datenmittelpunkts
% y0: y-Koordinate des Datenmittelpunkts
% vals: Anzahl an Messwerten
% r: Kreisradius
% data: nx2-Matrix mit Kreisbahn-/Geradenkoordinaten
%
% Autor: Changlai Bao
% Datum: 20.05.2025

function data=generate_data(x0, y0, r, vals)
  s = 1;
  noise = rand*2 - 4;
  r_with_noise = r + noise;

    for i=1:vals/2
		rnd = (rand - 0.5)*2;
		s = -s;
		x =  rnd*r + x0;
		y = s*sqrt(r^2 - (x - x0)^2) + y0;

    % Optional: Add noise to the radius for variability
		% x =  rnd*r_with_noise + x0;
		% y = s*sqrt(r_with_noise^2 - (x - x0)^2) + y0;

		data(i,1) = x;
		data(i,2) = y;
    end

	for i=1:vals/2
		rnd = (rand - 0.5)*2;
		x =  rnd*r + x0;
		y = x + y0 - x0;

    % Optional: Add noise to the radius for variability
		% x =  rnd*r_with_noise + x0;
		% y = x + y0 - x0;

		data(vals/2+i,1) = x;
		data(vals/2+i,2) = y;
    end

end

