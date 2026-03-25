function plotHeatMaps(heatMap_KF, heatMap_EKF, heatMap_UKF, xlim, ylim, params)
% Plots 3 heat maps side by side
% Shows "certainty of having been here" for each filter
%
% INPUTS:
%   heatMap_KF/EKF/UKF - Heat maps (visit counts)
%   xlim, ylim         - Grid bounds
%   params             - Parameters (for colors in titles)

%% Normalize heat maps (0 to 1)
max_KF = max(heatMap_KF(:));
max_EKF = max(heatMap_EKF(:));
max_UKF = max(heatMap_UKF(:));

% Avoid division by zero
if max_KF > 0
    heatMap_KF_norm = heatMap_KF / max_KF;
else
    heatMap_KF_norm = heatMap_KF;
end

if max_EKF > 0
    heatMap_EKF_norm = heatMap_EKF / max_EKF;
else
    heatMap_EKF_norm = heatMap_EKF;
end

if max_UKF > 0
    heatMap_UKF_norm = heatMap_UKF / max_UKF;
else
    heatMap_UKF_norm = heatMap_UKF;
end

%% KF Heat Map
subplot(2, 3, 4);
imagesc([xlim(1), xlim(2)], [ylim(1), ylim(2)], heatMap_KF_norm);
set(gca, 'YDir', 'normal');
colormap(gca, hot);
caxis([0, 1]);
axis equal tight;
colorbar;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
title('KF: Certainty of Path', 'Interpreter', 'latex', 'FontSize', 11, ...
      'FontWeight', 'bold', 'Color', params.colorKF);
grid on;

%% EKF Heat Map
subplot(2, 3, 5);
imagesc([xlim(1), xlim(2)], [ylim(1), ylim(2)], heatMap_EKF_norm);
set(gca, 'YDir', 'normal');
colormap(gca, hot);
caxis([0, 1]);
axis equal tight;
colorbar;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
title('EKF: Certainty of Path', 'Interpreter', 'latex', 'FontSize', 11, ...
      'FontWeight', 'bold', 'Color', params.colorEKF);
grid on;

%% UKF Heat Map
subplot(2, 3, 6);
imagesc([xlim(1), xlim(2)], [ylim(1), ylim(2)], heatMap_UKF_norm);
set(gca, 'YDir', 'normal');
colormap(gca, hot);
caxis([0, 1]);
axis equal tight;
colorbar;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
title('UKF: Certainty of Path', 'Interpreter', 'latex', 'FontSize', 11, ...
      'FontWeight', 'bold', 'Color', params.colorUKF);
grid on;

end
