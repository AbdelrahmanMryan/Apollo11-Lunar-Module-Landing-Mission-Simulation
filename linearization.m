initialization
%% Specify the model name
model = 'LMOL';

%% Create the operating point specification object.
opspec = operspec(model);

%% Set the constraints on the states in the model.
% - The defaults for all states are Known = false, SteadyState = true,
%   Min = -Inf, Max = Inf, dxMin = -Inf, and dxMax = Inf.

% State (1) - LMOL/Lunar Dynamics & Environment/Spacecraft Dynamics
% - Default model initial conditions are used to initialize optimization.
opspec.States(1).SteadyState = [false;false;false;false;false;false;true;true;true;true;true;true;false];

%% Set the constraints on the inputs in the model.
% - The defaults for all inputs are Known = false, Min = -Inf, and
% Max = Inf.

% Input (1) - LMOL/Fx
% - Default model initial conditions are used to initialize optimization.

% Input (2) - LMOL/Fy
% - Default model initial conditions are used to initialize optimization.

% Input (3) - LMOL/Fz
% - Default model initial conditions are used to initialize optimization.

% Input (4) - LMOL/Mx
% - Default model initial conditions are used to initialize optimization.

% Input (5) - LMOL/My
% - Default model initial conditions are used to initialize optimization.

% Input (6) - LMOL/Mz
% - Default model initial conditions are used to initialize optimization.

%% Set the constraints on the outputs in the model.
% - The defaults for all outputs are Known = false, Min = -Inf, and
% Max = Inf.

% Output (1) - LMOL/Alt
% - Default model initial conditions are used to initialize optimization.

% Output (2) - LMOL/Attitude
% - Default model initial conditions are used to initialize optimization.

% Output (3) - LMOL/Rate
% - Default model initial conditions are used to initialize optimization.

% Output (4) - LMOL/Velocity
% - Default model initial conditions are used to initialize optimization.


%% Create the options
opt = findopOptions('DisplayReport','off');

%% Perform the operating point search.
[op,opreport] = findop(model,opspec,opt);
M = linearize(model,op);
M.OutputName = {'Alt','phi','theta','psi','p','q','r',...
                'Vx','Vy','Vz'};
M.InputName = {'Fx','Fy','Fz','Mx','My','Mz'};
M.StateName = {'Xff','Yff','Zff','Vxff','Vyff','Vzff',...
               'phi','theta','psi','p','q','r','t'};

%% SISO Extract
G = tf(M);
%Angles
G_Mx2phi = G(2,4);
G_My2tht = G(3,5);
G_Mz2psi = G(4,6);
%Rates
G_Mx2p = G(5,4);
G_My2q = G(6,5);
G_Mz2r = G(7,6);
%Speeds
G_Fx2Vx = G(8,1);
G_Fy2Vy = G(9,2);
G_Fz2Vz = G(10,3);
% Altitude
G_Fz2Alt = G(1,3);