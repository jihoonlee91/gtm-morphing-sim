close all
clear
clc

load('T2_polynomial_aerodatabase.mat')
[Alpha,Beta] = meshgrid(dC6_damage.bas.alpha,dC6_damage.bas.beta);

figure
for ii = 1:6
    subplot(2,3,ii)
    contourf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,ii)).')
    xlabel('Angle of attack, deg')
    ylabel('Angle of sideslip, deg')
    grid on
    colorbar
    switch ii
        case 1
            title('\DeltaC_X')
        case 2
            title('\DeltaC_Y')
        case 3
            title('\DeltaC_Z')
        case 4
            title('\DeltaC_l')
        case 5
            title('\DeltaC_m')
        case 6
            title('\DeltaC_n')
    end
end

figure
for ii = 1:6
    subplot(2,3,ii)
    s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,ii)).','FaceAlpha',0.8);
    s.EdgeColor = 'none';
    xlabel('Angle of attack, deg')
    ylabel('Angle of sideslip, deg')
    grid on
    switch ii
        case 1
            title('\DeltaC_X')
        case 2
            title('\DeltaC_Y')
        case 3
            title('\DeltaC_Z')
        case 4
            title('\DeltaC_l')
        case 5
            title('\DeltaC_m')
        case 6
            title('\DeltaC_n')
    end
end

figure
for ii = 1:6
    subplot(2,3,ii)
    plot(dC6_damage.bas.alpha,squeeze(dC6_damage.bas.data(:,7,4,ii)).')
    xlabel('Angle of attack, deg')
    grid on
    switch ii
        case 1
            ylabel('\DeltaC_X (\beta=0)')
        case 2
            ylabel('\DeltaC_Y (\beta=0)')
        case 3
            ylabel('\DeltaC_Z (\beta=0)')
        case 4
            ylabel('\DeltaC_l (\beta=0)')
        case 5
            ylabel('\DeltaC_m (\beta=0)')
        case 6
            ylabel('\DeltaC_n (\beta=0)')
    end
end
