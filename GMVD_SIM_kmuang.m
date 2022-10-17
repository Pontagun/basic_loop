%% Demonstrate Real-Time Behavior
%  kMUang ver 2 applying lin eq to accelerate kmuang decrease
% ### The first output sample is determind after knowing first fifty samples of the sensors' data ###
%  % 2021-12-24 - This new ver. MAPS accels BACK to INERT Frame
% and uses ADAPTIVE HPF there,to isolate LIN ACCEL(opposite of alpha)
% 307: With a lot of Magnetic Interference (Only)
% 308: With Magnetic Interference (Only)
% 309: With Linear acceleration added (Only)
clear
close all;
clc;
nann=1; % CONTROLS VERS: nann= 0 GMVS; nann= 1 GMVD

%% Read recording data from file
% [label,t,Stillness,GyroXYZ,AcceleroXYZ,MagnetoXYZ,alpha, mu] = readRecordingFile('rec133GMV1.txt');
[label,t,Stillness,GyroXYZ,AcceleroXYZ,MagnetoXYZ,alpha, qGMVD] = readRecordingFileMuRM('rec133GMV1.txt');
% mu4plot = mu;  % Because "mu" is changed later in the program

% if exist('kmuang', 'var') && exist('kmmag', 'var')
%     mufake = circshift(kmuang_avg',[0,-1])';
% else
%     mufake = mu;
%     mufake(1408:3099, :) = 0;
%     disp('mufake is mu from reading, execute program again to remove 0');
% end
% 
% figure; plot( ((mu4plot * 6)-3),'m');


% FROM AB 2021 08 15- to study dT variance
% figure; %subplot(2,1,1);plot(t);grid;subplot(2,1,2);
% ddT=diff(t);plot(ddT);grid
% ddTMEAN = mean(ddT(10:length(ddT)))
% ddTVAR = var(ddT(10:length(ddT)))
%
% cutlead = 0;
% if cutlead ~= 0
% AcceleroXYZ = AcceleroXYZ(cutlead:end,:);
% GyroXYZ = GyroXYZ(cutlead:end,:);
% MagnetoXYZ = MagnetoXYZ(cutlead:end,:);
% Stillness = Stillness(cutlead:end,:);
% t = t(cutlead:end,:);
% IMUquat = IMUquat(cutlead:end,:);
% end

%% Settings of Parameters
GYRO_THRSHLDx = 0.0271;
GYRO_THRSHLDy = 0.0232;
GYRO_THRSHLDz = 0.0305;
buffSize = 3;
B_SCALING = 1.00;
TM=0.2;
TA=0.9;

%% Initilization of Arrays
N = length(t); % Number of Samples
SR = N / t(end); % SR = Sampling Rate

gyroBuff = zeros(buffSize,3);
gyroMaxAbsAvg = zeros(N,3);
acceleroBuff = zeros(buffSize,3);
acceleroAvg = zeros(N,3);
magnetoBuff = zeros(buffSize,3);
magnetoAvg = zeros(N,3);
Bias = zeros(N,3);
BiasBuff = zeros(N,3);
UnbiasedXYZ = zeros(N,3);
%alpha = ones(size(Stillness));
%AlpW = zeros(size(Stillness));
StageM = zeros(size(Stillness));
dt = 1/SR;

accelInert = zeros(N,3);  % Will store accel mapped BACK to inert frm
magnetInert = zeros(N,3);  % Will store magnt mapped BACK to inert frm
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

kmuang= zeros(N,1);    % Array to hold all the values of kmuang to be calculated

A_int = [AcceleroXYZ(1,:) 0]; % First measured gravity vector sample
a4 = zeros(N,4); % Computed Gravity Vector (Pure Quaternion)
a3 = zeros(N,3); % Computed Gravity Vector (3D-vector)
M_int = [MagnetoXYZ(1,:) 0];
m4 = zeros(N,4);
m3 = zeros(N,3);

MINTM = repmat(MagnetoXYZ(1,:),N,1);  % results in N-by-3 mat

e = zeros(N,3); % Error between computed and measured gravity vector
e_magnitude = zeros(N,1);
trigcountX=0;
trigcountY=0;
trigcountZ=0;

%% Clean up spikes in data Gyroscope data
% GyroXYZ(:,1)= hampel(GyroXYZ(:,1),10,0.01);
% GyroXYZ(:,2)= hampel(GyroXYZ(:,2),10,0.01);
% GyroXYZ(:,3)= hampel(GyroXYZ(:,3),10,0.01);

%% For every single sample into the buffer

for i=1:1:N-buffSize

    gyroBuff(i:i+buffSize-1,:) = GyroXYZ(i:i+buffSize-1,:);
    %     %Ong's- gyroMaxAbsAvg(i) = max(abs(mean(gyroBuff)));
    %     gyroMaxAbsAvg(i,1) = max(abs(mean(gyroBuff(i,1))));
    %     gyroMaxAbsAvg(i,2) = max(abs(mean(gyroBuff(i,2))));
    %     gyroMaxAbsAvg(i,3) = max(abs(mean(gyroBuff(i,3))));
    gyroMaxAbsAvg(i,1) = max(abs(mean(gyroBuff(i:i+buffSize-1,1))));
    gyroMaxAbsAvg(i,2) = max(abs(mean(gyroBuff(i:i+buffSize-1,2))));
    gyroMaxAbsAvg(i,3) = max(abs(mean(gyroBuff(i:i+buffSize-1,3))));
    acceleroBuff = AcceleroXYZ(i:i+buffSize-1,:);
    acceleroAvg(i,:) = mean(acceleroBuff);
    magnetoBuff = MagnetoXYZ(i:i+buffSize-1,:);
    magnetoAvg(i,:) = mean(magnetoBuff);

    if(i~=1)
        Bias(i,:) = Bias(i-1,:);
    end

    if(gyroMaxAbsAvg(i,1) < GYRO_THRSHLDx)   %Check static
        trigcountX = trigcountX+1;
        %BiasBuff(i,:) = BiasBuff(i,:)+GyroXYZ(i,:);
        BiasBuff(1,1) = BiasBuff(1,1)+GyroXYZ(i,1);

    else
        trigcountX = 0;
        %Bias(i,:) = mean(Bias(1:i-1,:));
        BiasBuff(1,1) = 0;
    end

    if(trigcountX == 5) % Recalculate Bias if gyroMaxAbsAvg(i) < GYRO_THRSHLD for 25 samples

        Bias(i,1) =   BiasBuff(1,1)/5.0;
        trigcountX = 0;
        BiasBuff(1,1) = 0;
    end
    if(gyroMaxAbsAvg(i,2) < GYRO_THRSHLDy)   %Check static
        trigcountY = trigcountY+1;
        %BiasBuff(i,:) = BiasBuff(i,:)+GyroXYZ(i,:);
        BiasBuff(1,2) = BiasBuff(1,2)+GyroXYZ(i,2);

    else
        trigcountY = 0;
        %Bias(i,:) = mean(Bias(1:i-1,:));
        BiasBuff(1,2) = 0;
    end

    if(trigcountY == 5) % Recalculate Bias if gyroMaxAbsAvg(i) < GYRO_THRSHLD for 25 samples

        Bias(i,2) =   BiasBuff(1,2)/5.0;
        trigcountY = 0;
        BiasBuff(1,2) = 0;
    end

    if(gyroMaxAbsAvg(i,3) < GYRO_THRSHLDz)   %Check static
        trigcountZ = trigcountZ+1;
        %BiasBuff(i,:) = BiasBuff(i,:)+GyroXYZ(i,:);
        BiasBuff(1,3) = BiasBuff(1,3)+GyroXYZ(i,3);

    else

        trigcountZ = 0;
        %Bias(i,:) = mean(Bias(1:i-1,:));
        BiasBuff(1,3) = 0;
    end

    if(trigcountZ == 5) % Recalculate Bias if gyroMaxAbsAvg(i) < GYRO_THRSHLD for 25 samples

        Bias(i,3) =   BiasBuff(1,3)/5.0;
        trigcountZ = 0;
        BiasBuff(1,3) = 0;
    end
    %% Removing Gyroscope Bias
    UnbiasedXYZ(i,:) = GyroXYZ(i,:) - (B_SCALING * Bias(i,:));
    %UnbiasedXYZ(i,:) = GyroXYZ(i,:) - (2 * Bias(i,:));

    %% Compute Quaternion
    if(i > 1)
        mufake(i) = KM(i-1);
%         mufake(i) = mu(i-1);
        w = [UnbiasedXYZ(i,1),UnbiasedXYZ(i,2),UnbiasedXYZ(i,3),0]; % Augment 0 to Angular velocity

        dqG(i,:) = 0.5 * myQuatProd(qG(i-1,:),w);
        % qG(i,:) = myQuatIntegrate(dqG(i,:),qG(i-1,:),dt);
        qG(i,:) = myQuatIntegrate(dqG(i,:),qG(i-1,:),(t(i)-t(i-1)));
        qG(i,:) = myQuatNormalize(qG(i,:));
        %qG_pl(i,:)=qG(i,:);

        dqGA(i,:) = 0.5 * myQuatProd(qGA(i-1,:),w);
        % qGA(i,:) = myQuatIntegrate(dqGA(i,:),qGA(i-1,:),dt);
        qGA(i,:) = myQuatIntegrate(dqGA(i,:),qGA(i-1,:),(t(i)-t(i-1)));
        qGA(i,:) = myQuatNormalize(qGA(i,:));

        dqGM(i,:) = 0.5 * myQuatProd(qGM(i-1,:),w);
        % qGM(i,:) = myQuatIntegrate(dqGM(i,:),qGM(i-1,:),dt);
        qGM(i,:) = myQuatIntegrate(dqGM(i,:),qGM(i-1,:),(t(i)-t(i-1)));
        qGM(i,:) = myQuatNormalize(qGM(i,:));


    end

    %% 2021-12-24_AB  Computer 3 accelerations @ Inert Frm, from qGA
    % the results in accelInert will be used ONLY FOR MONITORING
    %accelInert(i,:) = qrotbak(qGA(i,:) , acceleroAvg(i,:));
    %magnetInert(i,:) = qrotbak(qGA(i,:) , magnetoAvg(i,:));


    %% Compute Gravity Vector (y = q' * m * q)
    a4(i,:) = myQuatProd(myQuatConj(qGA(i,:)),myQuatProd(A_int,qGA(i,:)));
    a3(i,:) = a4(i,1:3); % Convert pure quaternion to 3D Vector

    %% Compute Magnetic North Vector (y= q' * m * q)
    m4(i,:) = myQuatProd(myQuatConj(qGM(i,:)),myQuatProd(M_int,qGM(i,:)));
    m3(i,:) = m4(i,1:3); % Convert pure quaternion to 3D Vector

    %% Compute error between computed and measured gravity vector
    e(i,:) = acceleroAvg(i,:)-a3(i,:);
    e_magnitude(i) = sqrt(e(i,1).^2 + e(i,2).^2 + e(i,3).^2);

    %% Compute Difference in Quaternion

    %if(gyroMaxAbsAvg(i) < GYRO_THRSHLD)
    v2 = a3(i,:); % Computed Gravity Vector
    v1 = acceleroAvg(i,:); % Measured Gravity Vector from Accelerometer
    qv = cross(v1,v2);
    qw = sqrt((v1(1)^2 + v1(2)^2 + v1(3)^2)*(v2(1)^2 + v2(2)^2 + v2(3)^2)) + dot(v1,v2);
    deltaQa = myQuatNormalize([qv,qw]);

    vm2 = m3(i,:);
    vm1 = magnetoAvg(i,:);
    qmv = cross(vm1,vm2);
    qmw = sqrt((vm1(1)^2 + vm1(2)^2 + vm1(3)^2)*(vm2(1)^2 + vm2(2)^2 + vm2(3)^2)) + dot(vm1,vm2);
    deltaQm = myQuatNormalize([qmv,qmw]);

    % Quaternion Interpolation
    %alpha(i) = mean(Stillness(i:i+buffSize-1))^2;
    %alpha(i)= 1;
    %get alpha directly from C# Nann edit 8/11/21
    %COMMENT for AB : below is commented out to use the alpha from the file
    %          if (i~=1)
    %              AlpW=0.25;
    %              alpha(i) = AlpW*(Stillness(i-1)^2)+(1-AlpW)*alpha(i-1); %Gamma Memory Filter
    %              alpha(i) = alpha(i)*1;%Put in by nann 11/5/2020
    %          end
    %

    qGM(i,:) = myQuatNormalize(myQuatProd(qGM(i,:),deltaQm));
    qGA(i,:) = myQuatNormalize(myQuatProd(qGA(i,:),deltaQa)); %Put in by nann 11/5/2020
    %qGA(i,:) = QSLERP(qG(i,:),temp_qGA,alpha(i)); %Ong's

    qG_pl(i,:)=qG(i,:);
    qGA_pl(i,:)=qGA(i,:);
    qGM_pl(i,:)=qGM(i,:);

    qOUT0(i,:) = QSLERP(qGM(i,:),qGA(i,:),alpha(i)); %Original Ong's
    
%     j4db
%     if (i >= 1420 && i <= 1450)
%         mufake(i) = 0;
%     end

    %edit by Nann 8/11/2021
    qSM(i,:) = QSLERP(qG(i,:),qGM(i,:),mufake(i));
    qSA(i,:) = QSLERP(qG(i,:),qGA(i,:),alpha(i));
    qOUT1(i,:) = QSLERP(qSM(i,:),qSA(i,:),alpha(i));
    qNorm(i, :) = norm(qOUT1(i,:));
    %edit by nann 11/4/2020


%     j4db
%     if (i == 1500)
%         qOUT1(i,:) = qOUT1Baseline(i,:);
%     end
    %***************COMMENT FOR AB
    %The version that is effectively used is control by the variable NANN
    %  nann=1;  % Variable nann is now set at the BEGINNING
    % with nann=1 --> implement GMV-D   for GMV-S change to nann=0
    if (nann==1)
        qG(i,:) = qOUT1(i,:);
        qGA(i,:) = qOUT1(i,:);
        qGM(i,:) = qOUT1(i,:);
    elseif (nann==0)
        qG(i,:) = qOUT0(i,:);
        qGA(i,:) = qOUT0(i,:);
        qGM(i,:) = qOUT0(i,:);
    end

    %% 2022-01-26_AB/PS  Computer 3 accelerations @ Inert Frm, from qOUT
    % the results in accelInert will be used ONLY FOR MONITORING
    accelInert(i,:) = qrotbak(qOUT1(i,:), acceleroAvg(i,:));
    magnetInert(i,:) = qrotbak(qOUT1(i,:), magnetoAvg(i,:));
    
    magn30 = mean(magnetInert(1:30,:));
    magnMag30 = sqrt(magn30 * (magn30'));

    %% mapBtoI functioanlly equals to qrotbak

    %% Calculating kmuang
    coskmuang = (dot(magnetInert(i,:),M_int(1:3)))/((norm(magnetInert(i,:))) * (norm(M_int(1:3))) );
%     cosk2 = (coskmuang + 1)/2
    gammaKmuang = acos(coskmuang);
    slopekmua = 3;
    km1 = 1 + (-1 * slopekmua) * gammaKmuang; % ------------------------------(14)
    kmuang(i) = (1 + km1 + abs(1 + km1)) / 4;
    %% With slopekmua = 3, the  above is the same as kmuang(i) = 1 - (1.5*gammaKmuang), forcing non-negative
    %% This is to understand equation 14 in the 4 pages Sensors2022 conference paper.
%     km2 = (1 + km1)/2;
%     kmuang(i) =  ( km2 + (abs(km2)))/2;
    %% Calculating kmmag
    minertmag(i) = my3dvnorm(magnetInert(i,:))';
    NMagnitudeMagInert(i) = minertmag(i) / magnMag30;
    angchg(i) = anginertchg(magnetInert(i,:), magn30);

    Magpenalty(i) = NMagnitudeMagInert(i) .* angchg(i);

    kmmag1(i) =  1 - Magpenalty(i);
    kmmag(i) = (kmmag1(i) + abs(kmmag1(i)) )/2;
    
%     kmmerge(i) = (kmuang(i) .* kmmag(i));
    kmmerge(i) = mean([kmuang(i) kmmag(i)]);
    
    % Find the minimum value in WINDOW_SIZE included itself and backward to drop the
    % calculated kmu faster.
    win_s = 5;
    if i > win_s 
        tempkm = min(kmmerge(i-win_s:i));
    else 
        tempkm = kmmerge(i);
    end
%     if i > 1408 && i < 1441
%         KM(i) = 0;
%     else
%     KM(i) = tempkm; 
% end
    if i > 8
        alphamin(i) = min(alpha(i-8:i));
    else
        alphamin(i) = alphamin(i);
    end

    % Linear function to accelerate decay 
    aminslope = 4;
    alphamin1 = (alphamin(i) * aminslope) - aminslope + 1;
    alphamin2(i) = (alphamin1 + abs(alphamin1)) / 2; 

    KM(i) = tempkm .* alphamin2(i); 
end

% 2021-12-24 AB, Fill last vals in accelInert  with same last value
for nbx = (N-buffSize+1):N
    accelInert(nbx,:) = accelInert(N-buffSize,:);
end


%% Print New Quaternion to a file

% fprintQ(qG,qOUT);  % THIS IS ALSO CONTROLLED by var  nann
if (nann==1)
    fprintQ(qG,qOUT1);
elseif (nann==0)
    fprintQ(qG,qOUT0);
end

% Plot the accelerations mapped back to Inert Frame {AB Jan 2022}
% figure; plot(accelInert);
% figure; plot(magnetInert);

% minertmag = my3dvnorm(magnetInert); % 1        3099
% angchg = anginertchg(magnetInert, magn100); % 3099           1
% 
% NMagnitudeMagInert = minertmag / magnMag100; % 1        3099
% Magpenalty = NMagnitudeMagInert' .* angchg;
% 
% 
% kmmag1 =  1 - Magpenalty;
% kmmag = (kmmag1 + abs(kmmag1) )/2;



tiledlayout(4, 1)

ax1 = nexttile;
plot(ax1, kmuang, '.');
hold on
plot(ax1, kmmag);
legend('\mu_{KA}', '\mu_{KM}')
title('\mu_{KA} and \mu_{KM}');
% title('');

ax2 = nexttile;
plot(ax2, KM);
title('\mu_K'); grid on;
% title(' ');

ax3 = nexttile;
plot(ax3, qOUT1); grid on;
title('Components of q_{out} from \mu_K')
% title(' ');

ax4 = nexttile;
plot(ax4, qGMVD); grid on;
title('Components of q_{out} from GMVD')

% ax4 = nexttile;
% plot(ax4, mu); grid on;
% % title('\mu (requires postion information)')
% title(' ');
% 
% % To get values, muanully run muq = qOUT1; with mu in file, after that run
% % with MUk later
% ax5 = nexttile;
% plot(ax5, mu); grid on;
% % title('Components of q_{out} from \mu')
% title(' ');

% 
% ax6 = nexttile;
% plot(ax6, muq); grid on;
% title('Components of q_{out} from Kalman filter')


% figure; plot(hpfax); hold on; plot(hpfay, 'r'); plot(hpfaz,'g')

%  FOR MAGNETOMETER mapped back to inertial frame:
% figure; plot(magnetInert);grid on; hold on;
% plot( ((mu4plot * 6)-3),'m');
% title('Magnetometer readings MAPPED BACK TO INERTIAL FRAME')
% MGNTDIF = magnetInert - MagnetoXYZ(1,:) ;
% maginfrmmg = sqrt(dot( MGNTDIF, MGNTDIF, 2));
% figure; plot(maginfrmmg); hold on;plot(mu4plot * 4.5,'m');grid
% title('MU * 4.5 and mag of magnet diff at inert frm')

%% Display Outputs

% figure;clf;
%
% % 1st Column Plot:
%
%     sp(1)=subplot(4,3,1);
%     plot(t(1:end-buffSize),GyroXYZ(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -3 3]);grid on;title('Raw Gyroscopre Data');
% %     legend('x','y','z','Location','BestOutside');
%     ylabel('Angular velocity (rad/s)');
%
%     sp(2)=subplot(4,3,4);plot(t(1:end-buffSize),Bias(1:end-buffSize,:),'Linewidth',1);
%     xlim([0 t(end)]);grid on;title('Predicted Biases');
% %     legend('x','y','z','Location','BestOutside');
%
%     sp(3)=subplot(4,3,7);plot(t(1:end-buffSize),UnbiasedXYZ(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -3 3]);grid on;title('Unbiased Gyroscope Data');
% %     legend('x','y','z','Location','BestOutside');
%     ylabel('Angular velocity (rad/s)');
%
%
%     sp(4)=subplot(4,3,10);plot(t(1:end-buffSize),qG_pl(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -1.1 1.1]);grid on;title('Quaternion (without Gravity-Vector Correction): q_G');
% %     legend('x','y','z','w','Location','BestOutside');
%     xlabel('Time (seconds)');
%
% % 2nd Column Plot:
%
%     sp(5)=subplot(4,3,2);plot(t(1:end-buffSize),acceleroAvg(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -1.1 1.1]);grid on;
% %     legend('x','y','z','Location','BestOutside');
%     title('Measured Gravity Vector Data from Accelerometer');
%     ylabel('Gravity (g)');
%
%     sp(6)=subplot(4,3,5);plot(t(1:end-buffSize),qGA_pl(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -1.1 1.1]);grid on;title('q_G_A');
% %     legend('x','y','z','w','Location','BestOutside');
%
%     sp(7)=subplot(4,3,8);plot(t(1:end-buffSize),magnetoAvg(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -3 3]);grid on;title('Measured Magnetic North from Magnetometer');
% %     legend('x','y','z','Location','BestOutside');
%
%     sp(8)=subplot(4,3,11);plot(t(1:end-buffSize),qGM_pl(1:end-buffSize,:),'Linewidth',1);
%     axis([0 t(end) -1.1 1.1]);grid on;title('q_G_M');xlabel('Time (seconds)');
% %     legend('X','Y','Z','Location','BestOutside');
%
% % 3rd Column Plot:
%
%     subplot(4,3,3);
%     text(0.1,0.5,['Recording ID: ',label,'BuffSize',string(buffSize),'GyroTh',string(GYRO_THRSHLDx),string(GYRO_THRSHLDy),string(GYRO_THRSHLDz)],'FontSize',10);
%     set(gca,'Visible','off');
%
%     sp(9)=subplot(4,3,6);
%     plot(t(1:end-buffSize),Stillness(1:end-buffSize));
%     axis([0 t(end) -0.1 1.1]);grid on;title('Stillness');
%
%     sp(10)=subplot(4,3,9);
%     plot(t(1:end-buffSize),alpha(1:end-buffSize),'r','LineWidth',1.0);
%     axis([0 t(end) -0.1 1.1]);grid on;title('Alpha');
%
%     sp(11)=subplot(4,3,12);plot(t(1:end-buffSize),qOUT1(1:end-buffSize,:),'LineWidth',1.0);
%     axis([0 t(end) -1.1 1.1]);grid on;title('q_O_U_T');
%     xlabel('Time (seconds)');
%
%     linkaxes(sp,'x');
%
%


