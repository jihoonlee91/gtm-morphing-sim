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
            ylim([-1 1]*0.15)
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

%%
figure
for ii = 1:3
    subplot(1,3,ii)
    plot(C6_bas.alpha,C6_bas.data(:,14,ii*2-1),'-o')
    xlim([-5 40])
    xlabel('Angle of attack, deg')
    grid on
    grid minor
    switch ii
        case 1
            ylabel('\DeltaC_X (\beta=0)')
%             ylim([-1 1]*0.04)
        case 2
            ylabel('\DeltaC_Z (\beta=0)')
%             ylim([-1 1]*0.16)
        case 3
            ylabel('\DeltaC_m (\beta=0)')
%             ylim([-1 1]*0.8)
    end
end

%%
CX = C6_bas.data(1:23,14,1);
CZ = C6_bas.data(1:23,14,3);
CD = -CX.*cosd(C6_bas.alpha(1:23))-CZ.*sind(C6_bas.alpha(1:23));
CL = CX.*sind(C6_bas.alpha(1:23))-CZ.*cosd(C6_bas.alpha(1:23));
dCX = 2*dC6_damage.bas.data(:,7,4,1);
dCZ = 2*dC6_damage.bas.data(:,7,4,3);
dCD = -dCX.*cosd(dC6_damage.bas.alpha)-dCZ.*sind(dC6_damage.bas.alpha);
dCL = dCX.*sind(dC6_damage.bas.alpha)-dCZ.*cosd(dC6_damage.bas.alpha);
dCD_int = interp1(dC6_damage.bas.alpha,dCD,C6_bas.alpha(1:23));
dCL_int = interp1(dC6_damage.bas.alpha,dCL,C6_bas.alpha(1:23));

figure
plot(C6_bas.alpha(1:23),(CL+2*dCL_int)./(CD+2*dCD_int),'-o')
hold on
plot(C6_bas.alpha(1:23),(CL+1*dCL_int)./(CD+1*dCD_int),'-o')
plot(C6_bas.alpha(1:23),(CL+0*dCL_int)./(CD+0*dCD_int),'-o')
plot(C6_bas.alpha(1:23),(CL-1*dCL_int)./(CD-1*dCD_int),'-o')
plot(C6_bas.alpha(1:23),(CL-2*dCL_int)./(CD-2*dCD_int),'-o')
xlabel('Angle of attack, deg')
ylabel('C_L/C_D')
legend('50% Span','75% Span','100% Span','125% Span','150% Span')
grid on
grid minor

%%
figure
plot((CD+2*dCD_int),(CL+2*dCL_int),'-o')
hold on
plot((CD+1*dCD_int),(CL+1*dCL_int),'-o')
plot((CD+0*dCD_int),(CL+0*dCL_int),'-o')
plot((CD-1*dCD_int),(CL-1*dCL_int),'-o')
plot((CD-2*dCD_int),(CL-2*dCL_int),'-o')
xlim([0 1.2])
ylim([-0.6 1.6])
xlabel('C_D')
ylabel('C_L')
legend('50% Span','75% Span','100% Span','125% Span','150% Span','location','best')
grid on
grid minor

%%
figure
plot(C6_bas.alpha(1:23),(CD+2*dCD_int),'-o')
hold on
plot(C6_bas.alpha(1:23),(CD+1*dCD_int),'-o')
plot(C6_bas.alpha(1:23),(CD+0*dCD_int),'-o')
plot(C6_bas.alpha(1:23),(CD-1*dCD_int),'-o')
plot(C6_bas.alpha(1:23),(CD-2*dCD_int),'-o')
% xlim([0 2])
% ylim([0 2])
xlabel('Angle of attack, deg')
ylabel('C_D')
legend('50% Span','75% Span','100% Span','125% Span','150% Span','location','best')
grid on
grid minor

%%
figure
plot(C6_bas.alpha(1:23),(CL+2*dCL_int),'-o')
hold on
plot(C6_bas.alpha(1:23),(CL+1*dCL_int),'-o')
plot(C6_bas.alpha(1:23),(CL+0*dCL_int),'-o')
plot(C6_bas.alpha(1:23),(CL-1*dCL_int),'-o')
plot(C6_bas.alpha(1:23),(CL-2*dCL_int),'-o')
% xlim([0 2])
% ylim([0 2])
xlabel('Angle of attack, deg')
ylabel('C_L')
legend('50% Span','75% Span','100% Span','125% Span','150% Span','location','best')
grid on
grid minor

%%
alpha_index = 10;
CX = C6_bas.data(alpha_index,6:22,1).';
CZ = C6_bas.data(alpha_index,6:22,3).';
CY = C6_bas.data(alpha_index,6:22,2).';
CC = CX.*cosd(alpha_index).*sind(C6_bas.beta(6:22))-CY.*sind(C6_bas.beta(6:22))+CZ.*sind(alpha_index).*sind(C6_bas.beta(6:22));
dCX = dC6_damage.bas.data(alpha_index,:,4,1).';
dCZ = dC6_damage.bas.data(alpha_index,:,4,3).';
dCYL = dC6_damage.bas.data(alpha_index,:,4,2).';
dCYR = flip(dCYL);
dCCL = dCX.*cosd(alpha_index).*sind(dC6_damage.bas.beta)-dCYL.*sind(dC6_damage.bas.beta)+dCZ.*sind(alpha_index).*sind(dC6_damage.bas.beta);
dCCR = dCX.*cosd(alpha_index).*sind(dC6_damage.bas.beta)-dCYR.*sind(dC6_damage.bas.beta)+dCZ.*sind(alpha_index).*sind(dC6_damage.bas.beta);
dCC = dCCL-dCCR;
dCC_int = interp1(dC6_damage.bas.beta,dCC,C6_bas.beta(6:22));

figure
plot(C6_bas.beta(6:22),CC+0*dCC_int,'-o')
hold on
plot(C6_bas.beta(6:22),CC+1*dCC_int,'-o')
plot(C6_bas.beta(6:22),CC+2*dCC_int,'-o')
xlabel('Angle of sideslip, deg')
ylabel('C_C')
legend('100%-100% Span','75%-125% Span','50%-150% Span')
grid on
grid minor
