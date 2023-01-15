% Objective Function for GA and PSO
%
% Criterion: integral of time-multiplied absolute value of error (ITAE)

function ObjVal = ObjFun()

    simOut = sim('ABS_opt', 'SaveOutput', 'on');
    t = simOut.get('tout');                              % time data
    results = simOut.get('yout');                        % slip data
    e = results{2}.Values.Data - results{1}.Values.Data; % error
    
ObjVal = (sum(abs(e).*t))*t(length(t));                  % ITAE

end