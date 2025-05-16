% Funktion zur Erzeugung pseudozufälliger Daten auf einer Kreisbahn/Geraden
%
% Parameter:
% x0: x-Koordinate des Datenmittelpunkts
% y0: y-Koordinate des Datenmittelpunkts
% vals: Anzahl an Messwerten
% r: Kreisradius
% data: nx2-Matrix mit Kreisbahn-/Geradenkoordinaten
%
% Autor: Stefan May
% Datum: 02.05.2013

function data=generate_data(x0, y0, r, vals)
	s = 1;
	for i=1:vals/2
		rnd = (rand - 0.5)*2;
		s = -s;
		x =  rnd*r + x0;
		y = s*sqrt(r^2 - (x - x0)^2) + y0;
		data(i,1) = x;
		data(i,2) = y;
	end;
	
	for i=1:vals/2 
		rnd = (rand - 0.5)*2;
		x =  rnd*r + x0;
		y = x + y0 - x0;
		data(vals/2+i,1) = x;
		data(vals/2+i,2) = y;
	end;
	
end;
