clear;
clear
close all;
clc;
nann=1;

[label,t,Stillness,GyroXYZ,AcceleroXYZ,MagnetoXYZ,falpha, qGMVD, qKalman] = readRecordingFileKalmanFilter('rec010GMV1.txt');

GYRO_THRSHLDx = 0.0271;
GYRO_THRSHLDy = 0.0232;
GYRO_THRSHLDz = 0.0305;
buffSize = 3;
B_SCALING = 1.00;
TM=0.2;
TA=0.9;

N = length(t);
SR = N / t(end);

gyroBuff = zeros(buffSize,3);
gyroMaxAbsAvg = zeros(N,3);
acceleroBuff = zeros(buffSize,3);
acceleroAvg = zeros(N,3);
magnetoBuff = zeros(buffSize,3);
magnetoAvg = zeros(N,3);

Bias = zeros(N,3);
BiasBuff = zeros(N,3);

UnbiasedXYZ = zeros(N,3);

StageM = zeros(size(Stillness));
dt = 1/SR;

accelInert = zeros(N,3);
magnetInert = zeros(N,3);
minertmag = ones(N, 1);

angchg = zeros(N, 1);
NMagnitudeMagInert = zeros(N, 1);
Magpenalty = zeros(N, 1);
kmmag1 = zeros(N, 1);
kmmag = zeros(N, 1);
kmmerge = zeros(N, 1);
mufake = zeros(N, 1);
KM = zeros(N, 1);
alphamin = zeros(N, 1);
alphamin2 = zeros(N, 1);

qG = zeros(N,4);
qG_pl = zeros(N,4);
qG(1,:) = [0 0 0 1];

dqG = zeros(N,4);

qGA = zeros(N,4);
qGA_pl = zeros(N,4);
qGA(1,:) = [0 0 0 1];
dqGA = zeros(N,4);

qGM = zeros(N,4);
qG_M = zeros(N,4);
qGM_pl = zeros(N,4);
qGM(1,:) = [0 0 0 1];
qG_M(1,:) = [0 0 0 1];
dqGM = zeros(N,4);

qOUT0 = zeros(N,4);
qSA = zeros(N,4);
qSM = zeros(N,4);
qOUT1 = zeros(N,4);

qNorm = zeros(N);

SlerpM = zeros(N,4);
SlerpA = zeros(N,4);

kmuang= zeros(N,1);

A_int = [AcceleroXYZ(1,:) 0];
a4 = zeros(N,4);
a3 = zeros(N,3);
M_int = [MagnetoXYZ(1,:) 0];
m4 = zeros(N,4);
m3 = zeros(N,3);

MINTM = repmat(MagnetoXYZ(1,:),N,1);

e = zeros(N,3);
e_magnitude = zeros(N,1);
trigcountX=0;
trigcountY=0;
trigcountZ=0;

alpha = getalphafromaccel(AcceleroXYZ);

for i=1:1:N-buffSize

    gyroBuff(i:i+buffSize-1,:) = GyroXYZ(i:i+buffSize-1,:);
    gyroMaxAbsAvg(i,1) = max(abs(mean(gyroBuff(i:i+buffSize-1,1)))); % High Pass Filter
    gyroMaxAbsAvg(i,2) = max(abs(mean(gyroBuff(i:i+buffSize-1,2))));
    gyroMaxAbsAvg(i,3) = max(abs(mean(gyroBuff(i:i+buffSize-1,3))));
    acceleroBuff = AcceleroXYZ(i:i+buffSize-1,:); % Low Pass Filter
    acceleroAvg(i,:) = mean(acceleroBuff);
    magnetoBuff = MagnetoXYZ(i:i+buffSize-1,:); % Low Pass Filter
    magnetoAvg(i,:) = mean(magnetoBuff);

    % if(i~=1)
    %     Bias(i,:) = Bias(i-1,:);
    % end
    % 
    % if(gyroMaxAbsAvg(i,1) < GYRO_THRSHLDx)
    %     trigcountX = trigcountX+1;
    %     BiasBuff(1,1) = BiasBuff(1,1)+GyroXYZ(i,1);
    % 
    % else
    %     trigcountX = 0;
    %     BiasBuff(1,1) = 0;
    % end
    % 
    % if(trigcountX == 5)
    % 
    %     Bias(i,1) =   BiasBuff(1,1)/5.0;
    %     trigcountX = 0;
    %     BiasBuff(1,1) = 0;
    % end
    % if(gyroMaxAbsAvg(i,2) < GYRO_THRSHLDy)
    %     trigcountY = trigcountY+1;
    %     BiasBuff(1,2) = BiasBuff(1,2)+GyroXYZ(i,2);
    % 
    % else
    %     trigcountY = 0;
    %     BiasBuff(1,2) = 0;
    % end
    % 
    % if(trigcountY == 5)
    % 
    %     Bias(i,2) =   BiasBuff(1,2)/5.0;
    %     trigcountY = 0;
    %     BiasBuff(1,2) = 0;
    % end
    % 
    % if(gyroMaxAbsAvg(i,3) < GYRO_THRSHLDz)
    %     trigcountZ = trigcountZ+1;
    %     BiasBuff(1,3) = BiasBuff(1,3)+GyroXYZ(i,3);
    % 
    % else
    % 
    %     trigcountZ = 0;
    %     BiasBuff(1,3) = 0;
    % end
    % 
    % if(trigcountZ == 5)
    % 
    %     Bias(i,3) =   BiasBuff(1,3)/5.0;
    %     trigcountZ = 0;
    %     BiasBuff(1,3) = 0;
    % end
    UnbiasedXYZ(i,:) = GyroXYZ(i,:); % - (B_SCALING * Bias(i,:));

    if(i > 1)
        mufake(i) = KM(i-1);
        w = [UnbiasedXYZ(i,1),UnbiasedXYZ(i,2),UnbiasedXYZ(i,3),0];

        dqG(i,:) = 0.5 * myQuatProd(qG(i-1,:),w);
        qG(i,:) = myQuatIntegrate(dqG(i,:),qG(i-1,:),(t(i)-t(i-1)));
        qG(i,:) = myQuatNormalize(qG(i,:));
        
        qGA(i, :) = qG(i, :);
        qGM(i, :) = qG(i, :);
    end

    a4(i,:) = myQuatProd(myQuatConj(qGA(i,:)),myQuatProd(A_int,qGA(i,:)));
    a3(i,:) = a4(i,1:3);

    m4(i,:) = myQuatProd(myQuatConj(qGM(i,:)),myQuatProd(M_int,qGM(i,:)));
    m3(i,:) = m4(i,1:3);

    v2 = a3(i,:);
    v1 = acceleroAvg(i,:);
    qv = cross(v1,v2);
    qw = sqrt((v1(1)^2 + v1(2)^2 + v1(3)^2)*(v2(1)^2 + v2(2)^2 + v2(3)^2)) + dot(v1,v2);
    deltaQa = myQuatNormalize([qv,qw]);

    vm2 = m3(i,:);
    vm1 = magnetoAvg(i,:);
    qmv = cross(vm1,vm2);
    qmw = sqrt((vm1(1)^2 + vm1(2)^2 + vm1(3)^2)*(vm2(1)^2 + vm2(2)^2 + vm2(3)^2)) + dot(vm1,vm2);
    deltaQm = myQuatNormalize([qmv,qmw]);

    qGM(i,:) = myQuatNormalize(myQuatProd(qGM(i,:),deltaQm));
    qGA(i,:) = myQuatNormalize(myQuatProd(qGA(i,:),deltaQa));

    qG_pl(i,:)=qG(i,:);
    qGA_pl(i,:)=qGA(i,:);
    qGM_pl(i,:)=qGM(i,:);

    qOUT0(i,:) = QSLERP(qGM(i,:),qGA(i,:),alpha(i));

    qSM(i,:) = QSLERP(qG(i,:),qGM(i,:),mufake(i));
    qSA(i,:) = QSLERP(qG(i,:),qGA(i,:),alpha(i));
    qOUT1(i,:) = QSLERP(qSM(i,:),qSA(i,:),alpha(i));
    qNorm(i, :) = norm(qOUT1(i,:));

    if (nann==1)
        qG(i,:) = qOUT1(i,:);
        qGA(i,:) = qOUT1(i,:);
        qGM(i,:) = qOUT1(i,:);
    elseif (nann==0)
        qG(i,:) = qOUT0(i,:);
        qGA(i,:) = qOUT0(i,:);
        qGM(i,:) = qOUT0(i,:);
    end

    accelInert(i,:) = qrotbak(qOUT1(i,:), acceleroAvg(i,:));
    magnetInert(i,:) = qrotbak(qOUT1(i,:), magnetoAvg(i,:));

    magn30 = mean(magnetInert(1:30,:));
    magnMag30 = sqrt(magn30 * (magn30'));

    % Prof's version
    % coskmuang = (dot(magnetInert(i,:),M_int(1:3))) / ((norm(magnetInert(i,:))) * (norm(M_int(1:3))) );
    % gammaKmuang = acos(coskmuang);
    
    gammaKmuang = anginertchg(magnetInert(i,:), M_int(1:3));
    slopekmua = 3;
    km1 = 1 + (-1 * slopekmua) * gammaKmuang;
    kmuang(i) = (1 + km1 + abs(1 + km1)) / 4;
    gammas(i) = gammaKmuang;

    minertmag(i) = my3dvnorm(magnetInert(i,:))';
    NMagnitudeMagInert(i) = minertmag(i) / magnMag30;
    magnitudes(i) = NMagnitudeMagInert(i);
    angchg(i) = anginertchg(magnetInert(i,:), magn30);

    Magpenalty(i) = NMagnitudeMagInert(i) .* angchg(i);

    kmmag1(i) =  1 - Magpenalty(i);
    kmmag(i) = (kmmag1(i) + abs(kmmag1(i)) )/2;

    kmmerge(i) = mean([kmuang(i) kmmag(i)]);

    win_s = 5;
    if i > win_s
        tempkm = min(kmmerge(i-win_s:i));
    else
        tempkm = kmmerge(i);
    end
    if i > 8
        alphamin(i) = min(alpha(i-8:i));
    else
        alphamin(i) = alphamin(i);
    end

    aminslope = 4;
    alphamin1 = (alphamin(i) * aminslope) - aminslope + 1;
    alphamin2(i) = (alphamin1 + abs(alphamin1)) / 2;

    KM(i) = tempkm .* alphamin2(i);
end

if (nann==1)
    fprintQ(qG,qOUT1);
elseif (nann==0)
    fprintQ(qG,qOUT0);
end

tiledlayout(5, 1)

ax0 = nexttile;
plot(ax0, falpha, 'r');
hold on;
plot(ax0, alpha, 'b');
legend('alpha file', 'alpha calc')
title('Alpha comparison');


ax1 = nexttile;
plot(ax1, kmuang, '.');
hold on
plot(ax1, kmmag);
legend('\mu_{KA}', '\mu_{KM}')
title('\mu_{KA} and \mu_{KM}');

ax2 = nexttile;
plot(ax2, KM);
title('\mu_K'); grid on;

ax3 = nexttile;
plot(ax3, qOUT1); grid on;
title('Components of q_{out} from \mu_K and \alpha_{accel}')

ax4 = nexttile;
plot(ax4, qOUT0); grid on;
title('Components of q_{out} from GMVD')