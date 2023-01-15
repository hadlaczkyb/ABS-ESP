% PSO          (Particle Swarm Optimization)
%
% This script implements the Particle Swarm Opitimization Algorithm.

clear Kpid;
clc;
rng default;

%% Simulation parameters

% Vehicle parameters
T_B = 0.1; J = 1.13; g = 9.81;

% Road parameters from the Burckhardt model
road_coeff = [
                1.2801 23.990 0.5200
                0.8570 33.822 0.3470
                1.1973 25.168 0.5373
                0.4004 33.708 0.1204
                0.1946 94.129 0.0646
                0.0500 306.39 0.0010
              ];
lam_opt = [
           0.17
           0.13
           0.16
           0.14
           0.06
           0.03
          ];
if ((1 <= sel) && (sel <= 6))
    ce = road_coeff(sel,:);
    lam_d = lam_opt(sel);
end
if(sel == 7)
    lam_d = 0.2;
end

%% Details of the optimization problem

fcn = @ObjFun;         % cost function
nVar = 3;              % number of optimization variables
lb = [0 0 0];          % lower bound for variables
ub = [1000 1000 1000]; % upper bound for variables

%% Define the PSO's parameters 

%NoP = 30;               % number of particles
%maxIter = 20;           % max iteration number
wMax = 0.9;              % weight max
wMin = 0.2;              % weight min
c1 = 2;                  % individual weight
c2 = 2;                  % global weight
vMax = (ub - lb) .* 0.2; % velocity max
vMin  = -vMax;           % velocity min

%% The PSO algorithm 
%% Initialize the particles

for k = 1 : NoP
    Swarm.Particles(k).X = (ub-lb) .* rand(1,nVar) + lb; % particle position
    Swarm.Particles(k).V = zeros(1, nVar);               % particle velocity
    Swarm.Particles(k).PBEST.X = zeros(1,nVar);          % individual best's position
    Swarm.Particles(k).PBEST.O = inf;                    % individual best's cost
    
    Swarm.GBEST.X = zeros(1,nVar); % global best's position
    Swarm.GBEST.O = inf;           % global best's cost
end


%% Main loop
for t = 1 : maxIter
    
    % Calcualte the objective value
    for k = 1 : NoP
        Kpid = Swarm.Particles(k).X;
        Swarm.Particles(k).O = fcn(Kpid);
        
        % Update the PBEST
        if Swarm.Particles(k).O < Swarm.Particles(k).PBEST.O 
            Swarm.Particles(k).PBEST.X = Kpid;
            Swarm.Particles(k).PBEST.O = Swarm.Particles(k).O;
        end
        
        % Update the GBEST
        if Swarm.Particles(k).O < Swarm.GBEST.O
            Swarm.GBEST.X = Kpid;
            Swarm.GBEST.O = Swarm.Particles(k).O;
        end
    end
    
    % Update the X and V vectors 
    w = wMax - t .* ((wMax - wMin) / maxIter);
    
    for k = 1 : NoP
        Swarm.Particles(k).V = w .* Swarm.Particles(k).V + c1 .* rand(1,nVar) .* (Swarm.Particles(k).PBEST.X - Swarm.Particles(k).X) ...
                                                                                     + c2 .* rand(1,nVar) .* (Swarm.GBEST.X - Swarm.Particles(k).X);
                                                                                 
        
        % Check velocities 
        index1 = find(Swarm.Particles(k).V > vMax);
        index2 = find(Swarm.Particles(k).V < vMin);
        
        Swarm.Particles(k).V(index1) = vMax(index1);
        Swarm.Particles(k).V(index2) = vMin(index2);
        
        Swarm.Particles(k).X = Swarm.Particles(k).X + Swarm.Particles(k).V;
        
        % Check positions 
        index1 = find(Swarm.Particles(k).X > ub);
        index2 = find(Swarm.Particles(k).X < lb);
        
        Swarm.Particles(k).X(index1) = ub(index1);
        Swarm.Particles(k).X(index2) = lb(index2);
        
    end
    
    outmsg = ['Iteration# ', num2str(t) , ' Swarm.GBEST.O = ' , num2str(Swarm.GBEST.O)];
    disp(outmsg);
    
    cgCurve(t) = Swarm.GBEST.O;
end

%% Display result

semilogy(cgCurve);
xlabel('Iteration#')
ylabel('Weight')
