mass = 15102.8; % kg
Ixx = 135582;   % kg.m^2
Iyy = 149140.0; % kg.m^2
Izz = 135582;   % kg.m^2

%% Lunar Fixed-Frame (ME) Initial Conditions
% Circular parking orbit (~107.7 km altitude)
Xff0 = [-1.1121, -1.3527, -0.5813]*1.0e+06; % m (Mean Earth/Polar Axis frame)
r0 = norm(Xff0);
mu_moon = 4.9048695e12; % m^3/s^2
v_orbit = sqrt(mu_moon / r0); % ~1630.43 m/s

% Tangent velocity vector in Fixed Frame (perpendicular to position vector r0)
v_dir = [1.2383, -1.0300, 0.0264];
v_hat = v_dir / norm(v_dir);
r_hat = Xff0 / r0;
t_hat = v_hat - dot(v_hat, r_hat)*r_hat;
t_hat = t_hat / norm(t_hat);
Vff0 = v_orbit * t_hat; % m/s

% Legacy variables for backward compatibility
Xicrf0 = Xff0;
Vicrf0 = Vff0;

% Attitude aligned with orbit velocity tangent in NED frame
lat0 = asind(Xff0(3)/r0);
lon0 = atan2d(Xff0(2), Xff0(1));
slat = sind(lat0); clat = cosd(lat0);
slon = sind(lon0); clon = cosd(lon0);
C_ME2NED = [-slat*clon, -slat*slon,  clat;
            -slon,       clon,       0;
            -clat*clon, -clat*slon, -slat];
V_ned = C_ME2NED * Vff0';
heading0 = atan2d(V_ned(2), V_ned(1)); % ~89.01 deg (due East)
pitch0   = atan2d(-V_ned(3), norm(V_ned(1:2))); % 0.00 deg (horizontal tangent)

Att0 = [0, pitch0*0, heading0*0]; % [Roll, Pitch, Yaw] in deg
Omg0 = [0, 0, 0];             % body frame rates (deg/s)

%% Moon Elevation DEM (LOLA 16 px/deg) to Simulink Lookup Table

% 1. Define dimensions (180 deg * 16 px/deg = 2880 rows, 360 deg * 16 px/deg = 5760 cols)
num_lat = 2880;
num_lon = 5760;

% 2. Read the binary elevation dataset
fid = fopen('ldem_16_float.img', 'r', 'ieee-le');
if fid == -1
    error('File not found. Make sure ldem_16_float.img is in your current directory.');
end

% Data is stored row-major, transpose to match MATLAB column-major ordering
raw_data = fread(fid, [num_lon, num_lat], 'single')';
fclose(fid);

% 3. Create strictly monotonic coordinate vectors for Simulink
% Note: Simulink requires breakpoint vectors to be strictly monotonically INCREASING.
% The raw file is ordered from +90 (North) to -90 (South), so we flip it.
lat_grid = linspace(-90, 90, num_lat);     % Breakpoint 1 (Row): -90 to +90 deg
lon_grid = linspace(-180, 180, num_lon);   % Breakpoint 2 (Col): -180 to +180 deg

% Flip rows to align with increasing latitude (-90 to +90)
moon_dem_table = flipud(raw_data);

%% Simulink 2-D Lookup Table Block Configuration:
% - Table data:           moon_dem_table
% - Breakpoints 1 (Row):   lat_grid
% - Breakpoints 2 (Col):   lon_grid
% - Interpolation:        Linear
% - Extrapolation:        Clip (or Linear)