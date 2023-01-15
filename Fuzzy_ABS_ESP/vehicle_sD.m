%% ******************************************************************
%% S-function describing the behaviour of the full vehicle model
%% [Based on the MathWorks S-function template]
%% ******************************************************************
function vehicle_sD(block)
%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 10;
block.NumOutputPorts = 19;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
for i = 1:block.NumInputPorts
    block.InputPort(i).Dimensions        = 1;
    block.InputPort(i).DatatypeID  = 0;  % double
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).SamplingMode = 'Sample';
    block.InputPort(i).DirectFeedthrough = true;
end

% Override output port properties
for j = 1:block.NumOutputPorts
    block.OutputPort(j).Dimensions  = 1;
    block.OutputPort(j).DatatypeID  = 0; % double
    block.OutputPort(j).Complexity  = 'Real';
    block.OutputPort(j).SamplingMode = 'Sample';
end

% Register parameters
block.NumDialogPrms     = 0;

block.NumContStates = 9;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

r = 0.3;        % wheel radius [m]
v_0 = 27.78;    % initial vehicle velocity [m/s]
w_0 = v_0/r;    % initial wheel angular velocity [rad/s]
v_y0 = 0;       % intial vehicle velocity along axis y [m/s]
psi_0 = 0;      % initial yaw rate [rad]
beta = 0;       % intial side slip angle [rad]

block.ContStates.Data = [w_0; w_0; w_0; ...
     w_0; v_0; v_y0; psi_0; beta; v_0];
    
%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
function Start(block)

block.Dwork(1).Data = 0;

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

%% Default parameters

m = 1280;           % total mass of the vehicle [kg]
g = 9.81;           % gravitational acceleration [m/s^2]
r = 0.3;            % wheel's radius [m]
a = 1.203;          % distance of the front axle from the COG [m]
b = 1.217;          % distance of the rear axle from the COG [m]
c_alpha = 50000;    % wheel's cornering stiffness [N/rad]
c_sig = 200000;     % wheel's longitudinal tire stiffness [N/s.u.]
L = a + b;          % wheelbase [m]
w = 1.9;            % vehicle's width [m]

%% Caluclating the weight at the wheels

F_wF = 0.5*(m*b*g)/L;   % front wheels
F_wR = 0.5*(m*a*g)/L;   % rear wheels

%% Inputs

% Braking torques
mu_FL = block.InputPort(5).Data;    % front left
mu_FR = block.InputPort(6).Data;    % front right
mu_RL = block.InputPort(7).Data;    % rear left
mu_RR = block.InputPort(8).Data;    % rear right

% Corrective yaw moment
M_z = block.InputPort(9).Data;

% Steering angle
delta = block.InputPort(10).Data;

%% Continous states

% Wheel angular velocities
w_FL = block.ContStates.Data(1);
w_FR = block.ContStates.Data(2);
w_RL = block.ContStates.Data(3);
w_RR = block.ContStates.Data(4);

% Vehicle body velocities
v_vx = block.ContStates.Data(5);
v_vy = block.ContStates.Data(6);
% Vehicle body orientation
psi = block.ContStates.Data(7);

% Sideslip angle
beta = block.ContStates.Data(8);
% Vehicle COG velocity
v_g = block.ContStates.Data(9);

%% Wheel slip angles

alpha_FL = delta - atan((v_vy + a*psi)/(v_vx));
alpha_FR = delta - atan((v_vy + a*psi)/(v_vx));
alpha_RL = -atan((v_vy - b*psi)/(v_vx));
alpha_RR = -atan((v_vy - b*psi)/(v_vx));

%% Wheel velocities

v_wxFL = (sqrt((v_vx)^2 + (v_vy + a*psi)^2))*cos(alpha_FL);
v_wxFR = (sqrt((v_vx)^2 + (v_vy + a*psi)^2))*cos(alpha_FR);
v_wxRL = v_vx;
v_wxRR = v_vx;

%% Longitudinal slip calculation

if v_wxFL <= 0
    sig_FL = 0;
else
    sig_FL = (r*w_FL - v_wxFL)/max(v_wxFL, r*w_FL);
end
if v_wxFR <= 0
    sig_FR = 0;
else
    sig_FR = (r*w_FR - v_wxFR)/max(v_wxFR, r*w_FR);
end
if v_wxRL <= 0
    sig_RL = 0;
else
    sig_RL = (r*w_RL - v_wxRL)/max(v_wxRL, r*w_RL);
end
if v_wxRR <= 0
    sig_RR = 0;
else
    sig_RR = (r*w_RR - v_wxRR)/max(v_wxRR, r*w_RR);
end

%% 'Lambda' function

lam_FL = (mu_FL*F_wF*(1 + abs(sig_FL)))/(2*sqrt((c_sig*sig_FL)^2 + (c_alpha*tan(alpha_FL))^2));
lam_FR = (mu_FR*F_wF*(1 + abs(sig_FR)))/(2*sqrt((c_sig*sig_FR)^2 + (c_alpha*tan(alpha_FR))^2));
lam_RL = (mu_RL*F_wR*(1 + abs(sig_RL)))/(2*sqrt((c_sig*sig_RL)^2 + (c_alpha*tan(alpha_RL))^2));
lam_RR = (mu_RR*F_wR*(1 + abs(sig_RR)))/(2*sqrt((c_sig*sig_RR)^2 + (c_alpha*tan(alpha_RR))^2));

if lam_FL < 1
    f_FL = (2 - lam_FL)*lam_FL;
elseif lam_FL >= 1 || isnan(lam_FL)
    f_FL = 1;
end

if lam_FR < 1
    f_FR = (2 - lam_FR)*lam_FR;
elseif lam_FR >= 1 || isnan(lam_FR)
    f_FR = 1;
end

if lam_RL < 1
    f_RL = (2 - lam_RL)*lam_RL;
elseif lam_RL >= 1 || isnan(lam_RL)
    f_RL = 1;
end

if lam_RR < 1
    f_RR = (2 - lam_RR)*lam_RR;
elseif lam_RR >= 1 || isnan(lam_RL)
    f_RR = 1;
end

%% Longitudinal force calculation

G_sFL = (1.15 - 0.75*mu_FL)*sig_FL^2 - (1.63 - 0.75*mu_FL)*abs(sig_FL) + 1.27;
G_sFR = (1.15 - 0.75*mu_FR)*sig_FR^2 - (1.63 - 0.75*mu_FR)*abs(sig_FR) + 1.27;
G_sRL = (1.15 - 0.75*mu_RL)*sig_RL^2 - (1.63 - 0.75*mu_RL)*abs(sig_RL) + 1.27;
G_sRR = (1.15 - 0.75*mu_RR)*sig_RR^2 - (1.63 - 0.75*mu_RR)*abs(sig_RR) + 1.27;

F_xFL = c_sig*(sig_FL/(1 + abs(sig_FL)))*f_FL*G_sFL;
F_xFR = c_sig*(sig_FR/(1 + abs(sig_FR)))*f_FR*G_sFR;
F_xRL = c_sig*(sig_RL/(1 + abs(sig_RL)))*f_RL*G_sRL;
F_xRR = c_sig*(sig_RR/(1 + abs(sig_RR)))*f_RR*G_sRR;

%% Lateral force calculation

G_aFL = (mu_FL - 1.6)*tan(alpha_FL) + 1.155;
G_aFR = (mu_FR - 1.6)*tan(alpha_FR) + 1.155;
G_aRL = (mu_RL - 1.6)*tan(alpha_RL) + 1.155;
G_aRR = (mu_RR - 1.6)*tan(alpha_RR) + 1.155;

F_yFL = c_alpha*(tan(alpha_FL)/(1 + abs(sig_FL)))*f_FL*G_aFL;
F_yFR = c_alpha*(tan(alpha_FR)/(1 + abs(sig_FR)))*f_FR*G_aFR;
F_yRL = c_alpha*(tan(alpha_RL)/(1 + abs(sig_RL)))*f_RL*G_aRL;
F_yRR = c_alpha*(tan(alpha_RR)/(1 + abs(sig_RR)))*f_RR*G_aRR;

%% Additional braking forces

if M_z > 0
    F_xFLc = (F_wF/(F_wF + F_wR))*((4*abs(M_z))/(2*w));
    F_xRLc = (F_wR/(F_wR + F_wR))*((4*abs(M_z))/(2*w));
    F_xFRc = 0;
    F_xRRc = 0;
elseif M_z < 0
    F_xFRc = (F_wF/(F_wF + F_wR))*((4*abs(M_z))/(2*w));
    F_xRRc = (F_wR/(F_wR + F_wR))*((4*abs(M_z))/(2*w));
    F_xFLc = 0;
    F_xRLc = 0;
else
    F_xFLc = 0;
    F_xRLc = 0;
    F_xFRc = 0;
    F_xRRc = 0; 
end

F_xFL = F_xFL + F_xFLc;
F_xFR = F_xFR + F_xFRc;
F_xRL = F_xRL + F_xRLc;
F_xRR = F_xRR + F_xRRc;

%% Outputs

% Vehicle velocities
if v_vx <= 0
    block.OutputPort(1).Data = 0;
else
    block.OutputPort(1).Data = v_vx;
end

block.OutputPort(2).Data = v_vy;

% Yaw rate and accelerations
block.OutputPort(3).Data = psi;

if v_vx <= 0
    block.OutputPort(4).Data = 0;
else
    block.OutputPort(4).Data = v_vy*psi + (1/m)*((F_xFL + F_xFR)*cos(delta) - (F_yFL + F_yFR)*sin(delta) + (F_xRL + F_xRR));
end

block.OutputPort(5).Data = -v_vx*psi + (1/m)*((F_xFL + F_xFR)*sin(delta) + (F_yFL + F_yFR)*cos(delta) + (F_yRL + F_yRR));
    
% Wheel velocities
if v_wxFL <= 0
    block.OutputPort(6).Data = 0;
else
    block.OutputPort(6).Data = v_wxFL;
end
if v_wxFR <= 0
    block.OutputPort(7).Data = 0;
else
    block.OutputPort(7).Data = v_wxFR;
end
if v_wxRL <= 0
    block.OutputPort(8).Data = 0;
else
    block.OutputPort(8).Data = v_wxRL;
end
if v_wxRR <= 0
    block.OutputPort(9).Data = 0;
else
    block.OutputPort(9).Data = v_wxRR;
end

% Wheel angular velocities
if w_FL <= 0
    block.OutputPort(10).Data = 0;
else
    block.OutputPort(10).Data = w_FL;
end
if w_FR <= 0
    block.OutputPort(11).Data = 0;
else
    block.OutputPort(11).Data = w_FR;
end
if w_RL <= 0
    block.OutputPort(12).Data = 0;
else
    block.OutputPort(12).Data = w_RL;
end
if w_RR <= 0
    block.OutputPort(13).Data = 0;
else
    block.OutputPort(13).Data = w_RR;
end

% Slips
if abs(sig_FL) <= 0
    block.OutputPort(14).Data = 0;
elseif abs(sig_FL) >= 1
    block.OutputPort(14).Data = 1;
else
    block.OutputPort(14).Data = abs(sig_FL);
end
if abs(sig_FR) <= 0
    block.OutputPort(15).Data = 0;
elseif abs(sig_FR) >= 1
    block.OutputPort(15).Data = 1;
else
    block.OutputPort(15).Data = abs(sig_FR);
end
if abs(sig_RL) <= 0
    block.OutputPort(16).Data = 0;
elseif abs(sig_RL) >= 1
    block.OutputPort(16).Data = 1;
else
    block.OutputPort(16).Data = abs(sig_RL);
end
if abs(sig_RR) <= 0
    block.OutputPort(17).Data = 0;
elseif abs(sig_RR) >= 1
    block.OutputPort(17).Data = 1;
else
    block.OutputPort(17).Data = abs(sig_RR);
end

% Velocitiy of the COG and side slip angle
block.OutputPort(18).Data = v_g;
block.OutputPort(19).Data = beta;


%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Update(block)

block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

%% Default parameters

m = 1280;           % total mass of the vehicle [kg]
I_w = 2.1;          % moment of inertia of the wheel [kgm^2]
I_z = 2500;         % moment of inertia of the chassis [kgm^2]
g = 9.81;           % gravitational acceleration [m/s^2]
r = 0.3;            % wheel's radius [m]
a = 1.203;          % distance of the front axle from the COG [m]
b = 1.217;          % distance of the rear axle from the COG [m]
w = 1.9;            % vehicle's width [m]
c_alpha = 50000;    % wheel's cornering stiffness [N/rad]
c_sig = 200000;     % wheel's longitudinal tire stiffness [N/s.u.]
L = a + b;          % wheelbase [m]

%% Calculating the weight of the wheels

F_wF = 0.5*(m*b*g)/L;
F_wR = 0.5*(m*a*g)/L;

%% Inputs

% Braking torques
Tb_FL = block.InputPort(1).Data;
Tb_FR = block.InputPort(2).Data;
Tb_RL = block.InputPort(3).Data;
Tb_RR = block.InputPort(4).Data;

% Max. friction coefficient
mu_FL = block.InputPort(5).Data;
mu_FR = block.InputPort(6).Data;
mu_RL = block.InputPort(7).Data;
mu_RR = block.InputPort(8).Data;

% Corrective yaw moment
M_z = block.InputPort(9).Data;

% Steering angle
delta = block.InputPort(10).Data;

%% Continous states

w_FL = block.ContStates.Data(1);
w_FR = block.ContStates.Data(2);
w_RL = block.ContStates.Data(3);
w_RR = block.ContStates.Data(4);

v_vx = block.ContStates.Data(5);
v_vy = block.ContStates.Data(6);
psi = block.ContStates.Data(7);

beta = block.ContStates.Data(8);
v_g = block.ContStates.Data(9);

%% Wheel slip angles

alpha_FL = delta - atan((v_vy + a*psi)/(v_vx));
alpha_FR = delta - atan((v_vy + a*psi)/(v_vx));
alpha_RL = -atan((v_vy - b*psi)/(v_vx));
alpha_RR = -atan((v_vy - b*psi)/(v_vx));

%% Wheel velocities

v_wxFL = (sqrt((v_vx)^2 + (v_vy + a*psi)^2))*cos(alpha_FL);
v_wxFR = (sqrt((v_vx)^2 + (v_vy + a*psi)^2))*cos(alpha_FR);
v_wxRL = v_vx;
v_wxRR = v_vx;

%% Longitudinal slip calculation

if v_wxFL <= 0
    sig_FL = 0;
else
    sig_FL = (r*w_FL - v_wxFL)/max(v_wxFL, r*w_FL);
end
if v_wxFR <= 0
    sig_FR = 0;
else
    sig_FR = (r*w_FR - v_wxFR)/max(v_wxFR, r*w_FR);
end
if v_wxRL <= 0
    sig_RL = 0;
else
    sig_RL = (r*w_RL - v_wxRL)/max(v_wxRL, r*w_RL);
end
if v_wxRR <= 0
    sig_RR = 0;
else
    sig_RR = (r*w_RR - v_wxRR)/max(v_wxRR, r*w_RR);
end

%% Lambda function

lam_FL = (mu_FL*F_wF*(1 + abs(sig_FL)))/(2*sqrt((c_sig*sig_FL)^2 + (c_alpha*tan(alpha_FL))^2));
lam_FR = (mu_FR*F_wF*(1 + abs(sig_FR)))/(2*sqrt((c_sig*sig_FR)^2 + (c_alpha*tan(alpha_FR))^2));
lam_RL = (mu_RL*F_wR*(1 + abs(sig_RL)))/(2*sqrt((c_sig*sig_RL)^2 + (c_alpha*tan(alpha_RL))^2));
lam_RR = (mu_RR*F_wR*(1 + abs(sig_RR)))/(2*sqrt((c_sig*sig_RR)^2 + (c_alpha*tan(alpha_RR))^2));

if lam_FL < 1
    f_FL = (2 - lam_FL)*lam_FL;
elseif lam_FL >= 1 || isnan(lam_FL)
    f_FL = 1;
end

if lam_FR < 1
    f_FR = (2 - lam_FR)*lam_FR;
elseif lam_FR >= 1 || isnan(lam_FR)
    f_FR = 1;
end

if lam_RL < 1
    f_RL = (2 - lam_RL)*lam_RL;
elseif lam_RL >= 1 || isnan(lam_RL)
    f_RL = 1;
end

if lam_RR < 1
    f_RR = (2 - lam_RR)*lam_RR;
elseif lam_RR >= 1 || isnan(lam_RR)
    f_RR = 1;
end
%% Longitudinal force calculation

G_sFL = (1.15 - 0.75*mu_FL)*sig_FL^2 - (1.63 - 0.75*mu_FL)*abs(sig_FL) + 1.27;
G_sFR = (1.15 - 0.75*mu_FR)*sig_FR^2 - (1.63 - 0.75*mu_FR)*abs(sig_FR) + 1.27;
G_sRL = (1.15 - 0.75*mu_RL)*sig_RL^2 - (1.63 - 0.75*mu_RL)*abs(sig_RL) + 1.27;
G_sRR = (1.15 - 0.75*mu_RR)*sig_RR^2 - (1.63 - 0.75*mu_RR)*abs(sig_RR) + 1.27;

F_xFL = c_sig*(sig_FL/(1 + abs(sig_FL)))*f_FL*G_sFL;
F_xFR = c_sig*(sig_FR/(1 + abs(sig_FR)))*f_FR*G_sFR;
F_xRL = c_sig*(sig_RL/(1 + abs(sig_RL)))*f_RL*G_sRL;
F_xRR = c_sig*(sig_RR/(1 + abs(sig_RR)))*f_RR*G_sRR;

%% Lateral force calculation

G_aFL = (mu_FL - 1.6)*tan(alpha_FL) + 1.155;
G_aFR = (mu_FR - 1.6)*tan(alpha_FR) + 1.155;
G_aRL = (mu_RL - 1.6)*tan(alpha_RL) + 1.155;
G_aRR = (mu_RR - 1.6)*tan(alpha_RR) + 1.155;

F_yFL = c_alpha*(tan(alpha_FL)/(1 + abs(sig_FL)))*f_FL*G_aFL;
F_yFR = c_alpha*(tan(alpha_FR)/(1 + abs(sig_FR)))*f_FR*G_aFR;
F_yRL = c_alpha*(tan(alpha_RL)/(1 + abs(sig_RL)))*f_RL*G_aRL;
F_yRR = c_alpha*(tan(alpha_RR)/(1 + abs(sig_RR)))*f_RR*G_aRR;

%% Additional braking forces

if M_z > 0
    F_xFLc = (F_wF/(F_wF + F_wR))*((4*abs(M_z))/(2*w));
    F_xRLc = (F_wR/(F_wR + F_wR))*((4*abs(M_z))/(2*w));
    F_xFRc = 0;
    F_xRRc = 0;
elseif M_z < 0
    F_xFRc = (F_wF/(F_wF + F_wR))*((4*abs(M_z))/(2*w));
    F_xRRc = (F_wR/(F_wR + F_wR))*((4*abs(M_z))/(2*w));
    F_xFLc = 0;
    F_xRLc = 0;
else
    F_xFLc = 0;
    F_xRLc = 0;
    F_xFRc = 0;
    F_xRRc = 0; 
end

F_xFL = F_xFL + F_xFLc;
F_xFR = F_xFR + F_xFRc;
F_xRL = F_xRL + F_xRLc;
F_xRR = F_xRR + F_xRRc;

%% FrontLeft wheel dynamics

if w_FL <= 0
    block.Derivatives.Data(1) = 0;
else
    block.Derivatives.Data(1) = (1/I_w)*(-F_xFL*r - Tb_FL);
end

%% FrontRight wheel dynamics

if w_FR <= 0
    block.Derivatives.Data(2) = 0;
else
    block.Derivatives.Data(2) = (1/I_w)*(-F_xFR*r - Tb_FR);
end

%% RearLeft wheel dynamics

if w_RL <= 0
    block.Derivatives.Data(3) = 0;
else
    block.Derivatives.Data(3) = (1/I_w)*(-F_xRL*r - Tb_RL);
end

%% RearRight wheel dynamics

if w_RR <= 0
    block.Derivatives.Data(4) = 0;
else
    block.Derivatives.Data(4) = (1/I_w)*(-F_xRR*r - Tb_RR);
end

%% Vehicle dynamics

% Longitudinal acceleration
if v_vx <= 0
    block.Derivatives.Data(5) = 0;
else
    block.Derivatives.Data(5) = v_vy*psi + (1/m)*((F_xFL + F_xFR)*cos(delta) - (F_yFL + F_yFR)*sin(delta) + (F_xRL + F_xRR));
end

% Lateral acceleration
block.Derivatives.Data(6) = -v_vx*psi + (1/m)*((F_xFL + F_xFR)*sin(delta) + (F_yFL + F_yFR)*cos(delta) + (F_yRL + F_yRR));


% Derivative of the yaw rate
block.Derivatives.Data(7) = (1/I_z)*(a*((F_xFL + F_xFR)*sin(delta) + (F_yFL + F_yFR)*cos(delta)) - ...
        b*(F_yRL + F_yRR) + w/2*((F_xFR - F_xFL)*cos(delta) + (F_xRR - F_xRL) + (F_yFL - F_yFR)*sin(delta)) + M_z);


% Derivative of side slip angle
block.Derivatives.Data(8) = -psi + (1/(m*v_g))*(-(F_xFL + F_xFR)*sin(beta - delta) + ...
    (F_yFL + F_yFR)*cos(beta - delta) + (F_yRL + F_yRR)*cos(beta) - (F_xRL + F_xRR)*sin(beta));

% Derivative of the velocity of the COG
block.Derivatives.Data(9) = (1/m)*((F_xFL + F_xFR)*cos(delta - beta) - (F_yFL + F_yFR)*sin(delta - beta) + ...
    (F_xRL + F_xRR)*cos(beta) + (F_yRL + F_yRR)*sin(beta));

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate