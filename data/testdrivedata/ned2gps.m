function dy = ned2gps(dx, y)
    
    lat = deg2rad(y(1));
    long = deg2rad(y(2));
    alt = deg2rad(y(3));

    R_e = 6378.137;     % earth radius @ equator
    R_p = 6356.752;     % earth radius @ pole
    
    R = sqrt(...
        (((R_e^2)*cos(lat))^2 + ((R_p^2)*sin(lat))^2) /...
        (((R_e)*cos(lat))^2 + ((R_p)*sin(lat))^2)...
        ) * 1000 + alt;
         % earth radius @ lat [m]
    
    dlat = rad2deg(dx(1) / R);
    dlong = rad2deg(dx(2) / R);
    dalt = -dx(3);
    dy = [dlat; dlong; dalt];
end