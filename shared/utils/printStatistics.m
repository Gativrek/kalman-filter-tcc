function printStatistics(handles)
    rmse_KF  = sqrt(mean(handles.errorX_KF.^2  + handles.errorY_KF.^2));
    rmse_EKF = sqrt(mean(handles.errorX_EKF.^2 + handles.errorY_EKF.^2));
    rmse_UKF = sqrt(mean(handles.errorX_UKF.^2 + handles.errorY_UKF.^2));
    heading_KF  = sqrt(mean(handles.errorTheta_KF.^2));
    heading_EKF = sqrt(mean(handles.errorTheta_EKF.^2));
    heading_UKF = sqrt(mean(handles.errorTheta_UKF.^2));
    fprintf('KF  - RMSE: %.3f m, Heading: %.2f deg\n', rmse_KF,  heading_KF);
    fprintf('EKF - RMSE: %.3f m, Heading: %.2f deg\n', rmse_EKF, heading_EKF);
    fprintf('UKF - RMSE: %.3f m, Heading: %.2f deg\n', rmse_UKF, heading_UKF);
end