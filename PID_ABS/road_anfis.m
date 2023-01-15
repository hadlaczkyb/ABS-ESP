clc;

%% Set road coefficients and optimal slip values according to the Burckhardt model

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
      
lam_1 = 0.1:0.001:1;
len = length(lam_1);
mu = rep(0,[6*len,1]);
slip = rep(0.2,[6*len,1]);

for i = 0:5
    road_coeff_i = road_coeff(i+1,:);
    mu_i = road_coeff_i(1)*(1-exp(-road_coeff_i(2)*lam_1))-road_coeff_i(3)*lam_1;
    mu(i*len+1 : (i+1)*len) = mu_i;
    slip(i*len+1 : (i+1)*len) = rep(lam_opt(i+1),[len,1]);
end

lam = rep(lam_1',[6,1]);

%% Set parameters for generating ANFIS

td_vec = [lam mu slip];
option = genfisOptions('GridPartition');
%option = genfisOptions('SubtractiveClustering','ClusterInfluenceRange',0.2);
option.NumMembershipFunctions = [6 6];
init_fis = genfis([td_vec(:,1) td_vec(:,2)], td_vec(:,3), option);
train_fis = anfis(td_vec, init_fis, 100);

%% Generate ANFIS

coeff_sim = road_coeff(1,:);
simOut = sim('anfis_test','SaveOutput','on');
r = simOut.get('tout');
res = simOut.get('yout');
l = length(res);
all = rep(0,[1,6]);

for i = 0:5
    coeff_sim = road_coeff(i+1,:);
    simOut = sim('anfis_test','SaveOutput','on');
    r = simOut.get('tout');
    res = simOut.get('yout');
    all(1:l,i+1) = res(:,1);
end

%% Plot results

figure(1);
plot(0.1*r,all(:,1),'k'); 
hold on;
plot(0.1*r,all(:,2),'b');
plot(0.1*r,all(:,3),'g');
plot(0.1*r,all(:,4),'r');
plot(0.1*r,all(:,5),'c');
plot(0.1*r,all(:,6),'m');
hold off;
grid minor;
title('ANFIS results');
xlabel('\lambda');
ylabel('\lambda_d');


PID = [
    2.638939022838257e+02 0 50.597476085793020
    4.165192145690851e+02 1000 1.127282083249942e+02
    93.352418685335120 0 51.977497754382250
    3.885045690906933e+02 1000 97.882933276027880
    3.612335217514744e+02 6.594113059585302e+02 1.764244611373335e+02
    4.923934389147504e+02 1000 1.746267870650558e+02
    ];
con_fis = genfis(lam_opt, PID, option);

