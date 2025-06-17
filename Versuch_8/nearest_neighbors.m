function pairs = nearest_neighbors(model, scene, iter)
   pairs = [];
   zaehler = 0;
   
   % Dynamischer Distanzfilter - Schwellwert verringert sich mit jeder Iteration
   anfangsdistanz = 5;  % Startdistanz
   max_distanz = anfangsdistanz / (2*(iter-1));
   
   for i = 1:size(scene,1)
      dist_min = 1e20;
      j_min = 0;
      
      % Suche nach nächstem Nachbarn
      for j = 1:size(model,1)
         dist = norm(scene(i,:) - model(j,:));
         if(dist < dist_min)
            dist_min = dist;
            j_min = j;
         end
      end
      
      % Distanzfilter: Nur Punktpairs mit geringer Distanz verwenden
      if (dist_min < max_distanz && j_min > 0)
        zaehler = zaehler + 1;
        pairs(zaehler,1) = model(j_min,1);
        pairs(zaehler,2) = model(j_min,2);
        pairs(zaehler,3) = scene(i,1);
        pairs(zaehler,4) = scene(i,2);
      end
   end
end