close all
clear
clc

load('T2_polynomial_aerodatabase.mat')
% set(0,'DefaultLineLineWidth',1);

%% Body axes
figure
for ii = 1:6
    subplot(2,3,ii)
    plot(dC6_ail.alpha,squeeze(dC6_ail.data(:,14,7,ii)).'+(-1)^(ii-1)*squeeze(dC6_ail.data(:,14,1,ii)).','--')
    hold on
    xlim([-5 40])
    switch ii
        case {1,3,5}
            plot(dC3_ele.alpha,squeeze(dC3_ele.data(:,14,3,2,(ii+1)/2)).','--')
        case {2,4,6}
            plot(dC3_ele.alpha,dC3_ele.alpha*0,'--')
    end
    plot(dC6_rud.alpha,squeeze(dC6_rud.data(:,14,1,ii)).','--')
    plot(dC6_damage.bas.alpha,squeeze(dC6_damage.bas.data(:,7,4,ii)).','LineWidth',2)
    xlabel('Angle of attack, deg')
    grid on
    grid minor
    switch ii
        case 1
            ylabel('\DeltaC_X (\beta=0)')
            ylim([-1 1]*0.04)
        case 2
            ylabel('\DeltaC_Y (\beta=0)')
            ylim([-1 1]*0.02)
        case 3
            ylabel('\DeltaC_Z (\beta=0)')
            ylim([-1 1]*0.16)
        case 4
            ylabel('\DeltaC_l (\beta=0)')
            ylim([-1 1]*0.04)
        case 5
            ylabel('\DeltaC_m (\beta=0)')
            ylim([-1 1]*0.8)
        case 6
            ylabel('\DeltaC_n (\beta=0)')
            ylim([-1 1]*0.08)
    end
    legend({'Aileron','Elevator','Rudder','Morphing'},'Location','Best')
end

%% Wind Axes
% figure
% for ii = 1:6
%     for jj = 1:numel(dC6_damage.bas.alpha)
%         XYZ = squeeze(dC6_damage.bas.data(:,7,4,1:3)).';
%         alpha = dC6_damage.bas.alpha(jj);
%         body2wind = [cosd(alpha) 0 sind(alpha);
%             -cosd(alpha) 0 -sind(alpha);
%             -sind(alpha) 0 cosd(alpha)];
%         DCL(:,jj) = -body2wind*XYZ(:,jj);
%     end
%     subplot(2,3,ii)
%     switch ii
%         case {1,2,3}
%             plot(dC6_damage.bas.alpha,DCL(ii,:).')
%         case {4,5,6}
%             plot(dC6_damage.bas.alpha,squeeze(dC6_damage.bas.data(:,7,4,ii)).')
%     end
% end
% 
% for ii = 1:6
%     for jj = 1:numel(dC6_ail.alpha)
%         XYZ = squeeze(dC6_ail.data(:,14,7,1:3)).'+(-1)^(ii-1)*squeeze(dC6_ail.data(:,14,1,1:3)).';
%         alpha = dC6_ail.alpha(jj);
%         body2wind = [cosd(alpha) 0 sind(alpha);
%             -cosd(alpha) 0 -sind(alpha);
%             -sind(alpha) 0 cosd(alpha)];
%         DCL(:,jj) = -body2wind*XYZ(:,jj);
%     end
%     subplot(2,3,ii)
%     hold on
%     switch ii
%         case {1,2,3}
%             plot(dC6_ail.alpha,DCL(ii,:).')
%         case {4,5,6}
%             plot(dC6_ail.alpha,squeeze(dC6_ail.data(:,14,7,ii)).'+(-1)^(ii-1)*squeeze(dC6_ail.data(:,14,1,ii)).')
%     end
%     xlabel('Angle of attack, deg')
%     grid on
%     switch ii
%         case 1
%             ylabel('\DeltaC_D (\beta=0)')
%         case 2
%             ylabel('\DeltaC_C (\beta=0)')
%         case 3
%             ylabel('\DeltaC_L (\beta=0)')
%         case 4
%             ylabel('\DeltaC_l (\beta=0)')
%         case 5
%             ylabel('\DeltaC_m (\beta=0)')
%         case 6
%             ylabel('\DeltaC_n (\beta=0)')
%     end
%     legend('Morphing','Aileron')
% end
