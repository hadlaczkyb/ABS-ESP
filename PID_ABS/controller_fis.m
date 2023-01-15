lam_opt = [
           0.17
           0.13
           0.16
           0.14
           0.06
           0.03
          ];
         
option = genfisOptions('SubtractiveClustering','ClusterInfluenceRange',0.2);
con_fis = genfis(lam_opt, PID, option);