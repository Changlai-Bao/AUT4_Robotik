function T = icp(model, scene, iter)

  scene_temp = scene;
  T = eye(3);

  % Sicherstellen, dass nur die angegebene Anzahl von Iterationen durchgeführt wird
  for i = 1:iter

    % Punktpaarsuche mit dynamischem Distanzfilter
    pairs = nearest_neighbors(model, scene_temp, i);
    
    % Prüfung ob Punktpairs gefunden wurden
    if isempty(pairs)
        fprintf('Keine gültigen Punktpairs in Iteration %d gefunden\n', i);
        break;
    end

    % Berechnung der Centroiden
    c = mean(pairs);
    cm = [c(1,1); c(1,2)];
    cd = [c(1,3); c(1,4)];

    % Bestimmung der Transformation
    n = size(pairs,1);
    M_komma = [(pairs(:,1) - cm(1,1)*ones(n,1)) (pairs(:,2) - cm(2,1)*ones(n,1))];
    D_komma = [(pairs(:,3) - cd(1,1)*ones(n,1)) (pairs(:,4) - cd(2,1)*ones(n,1))];

    zaehler = 0;
    nenner = 0;
    for j = 1:n
      xm = M_komma(j,1);
      ym = M_komma(j,2);
      xd = D_komma(j,1);
      yd = D_komma(j,2);
      zaehler = zaehler + ym*xd - xm*yd;
      nenner = nenner + xm*xd + ym*yd;
    end
    phi = atan2(zaehler, nenner);
    R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
    t = cm - R*cd;

    % Transformation
    Rt = [R t; 0 0 1];
    P = [(scene_temp(:,1))'; (scene_temp(:,2))'; ones(1,size(scene_temp,1))];
    T_Matrix = Rt * P;

    % Aufmultiplizieren der iterativen Transformationen
    scene_temp = [(T_Matrix(1,:))' (T_Matrix(2,:))'];
    T = T * Rt;

    figure(1);
    set(gcf, 'Name', 'Verlauf');
    hold on;
    plot(model(:,1), model(:,2), 'xg', 'MarkerSize', 3);
    plot(scene(:,1), scene(:,2), '*r', 'MarkerSize', 3);
    plot(scene_temp(:,1), scene_temp(:,2), 'xb', 'MarkerSize', 3);
    legend('model', 'Original scene', 'Transformierte scene');
    title(['Iteration ' num2str(i) ' von ' num2str(iter)]);
    hold off;
    drawnow;

  end

  fprintf('ICP-Algorithmus abgeschlossen nach %d Iterationen\n', iter);

end