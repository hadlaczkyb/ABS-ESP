% SGA          (Simple Genetic Algorithm) 
% 
% This script implements the Simple Genetic Algorithm. 
% Binary representation for the individuals is used. 

clear Kpid;
clc;
 
%% Algorithm parameters

%NIND = 30;           % number of individuals per subpopulations 
%MAXGEN = 80;         % maximal number of generations 
GGAP = 0.9;          % generation gap, how many new individuals are created 
NVAR = 3;            % number of optimization variables
PRECI = 10;          % precision of binary representation

% Based on the Chipperfield, Fonseca & Pohlheim GA Toolbox
SEL_F = 'sus';       % name of selection function 
XOV_F = 'xovsp';     % name of recombination function for individuals 
MUT_F = 'mut';       % name of mutation function for individuals 
OBJ_F = 'ObjFun';    % name of function for objective values 

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


%% Build field description matrix 
   FieldD = [rep([PRECI],[1, NVAR]);... 
              rep([0; 1000], [1, NVAR]);...
              rep([1; 0; 1; 1], [1, NVAR])]; 
 
%% Create population 
   Chrom = crtbp(NIND, NVAR*PRECI); 
 
%% Reset count variables 
   gen = 0; 
   Best = NaN*ones(MAXGEN,1);
 
%% Evaluate initial population
   generation = bs2rv(Chrom,FieldD);
   
   %generation(1,:) = BestKpid;
 
   tryKpid = rep([0 0 0],[MAXGEN*NIND,1]);
   tryKpid(1:NIND,:) = generation;
   tryObjV = rep(0, [MAXGEN*NIND,1]);
   
   ObjV = rep(0,[NIND, 1]);
   for i=1:NIND
       Kpid = generation(i,:);
       ObjV(i) = ObjFun();
   end
   tryObjV(1:NIND) = ObjV;
   
%% Track best individual and display convergence
   Best(gen+1) = min(ObjV);
   figure(1);
   plot(log10(Best), 'ro');
   xlabel('generation', 'FontSize', 18);
   ylabel('log(ITAE)', 'FontSize', 18);
   text(0.5, 0.95, ['Best = ', num2str(Best(gen+1))], 'Units', 'normalized', 'FontSize', 18);
   drawnow;
   
% Iterate population 
   while gen < MAXGEN-1 
 
   % Fitness assignement to whole population 
      FitnV = ranking(ObjV); 
             
   % Select individuals from population 
      SelCh = select(SEL_F, Chrom, FitnV, GGAP); 
      
   % Recombine selected individuals (crossover) 
      SelCh = recombin(XOV_F, SelCh, 0.7); 
 
   % Mutate offspring 
      SelCh = mut(SelCh);
     
   % Evaluate offspring, call objective function
      actSelGen = bs2rv(SelCh, FieldD);
      ObjVSel = rep(0, [length(actSelGen), 1]);
      for i=1:length(actSelGen)
        Kpid = actSelGen(i,:);
        ObjVSel(i) = ObjFun();
      end
 
   % Insert offspring in population replacing parents 
      [Chrom ObjV] = reins(Chrom, SelCh, 1, 1, ObjV, ObjVSel); 
      
   % Saving Kpid values
      actgen = bs2rv(Chrom, FieldD);
      tryKpid((1+gen)*NIND+1: (2+gen)*NIND, :) = actgen;
      tryObjV((1+gen)*NIND+1: (2+gen)*NIND) = ObjV;
 
   % Increment generational counter
      gen = gen+1; 
       
   % Update display and record current best individual
      Best(gen+1) = min(ObjV);
      plot(log10(Best), 'ro');
      xlabel('generation', 'FontSize', 18);
      ylabel('log(ITAE)', 'FontSize', 18);
      text(0.5, 0.95, ['Best = ', num2str(Best(gen+1))], 'Units', 'normalized', 'FontSize', 18);
      drawnow;
      
   end 
   
      [minobjval,ind] = min(tryObjV);
      BestKpid = tryKpid(ind,:);
      figure(2);
      plot3(tryKpid(:,1), tryKpid(:,2), tryKpid(:,3), 'rx');
      grid on;
      xlabel('Kp', 'FontSize', 18);
      ylabel('Ki', 'FontSize', 18);
      zlabel('Kd', 'FontSize', 18);
 