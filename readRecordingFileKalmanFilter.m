function [label,t,Stillness,GyroXYZ,AcceleroXYZ,MagnetoXYZ,alpha, qGMVD, qKalman] = readRecordingFileKalmanFilter(FILENAME)

label=FILENAME;
fileID = fopen(FILENAME);
readCell=textscan(fileID,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','delimiter',',');
fclose(fileID);

t = readCell{1};
Stillness = readCell{2};
GyroXYZ = [readCell{3},readCell{4},readCell{5}];
AcceleroXYZ = [readCell{6},readCell{7},readCell{8}];
%IMUquat = [readCell{9},readCell{10},readCell{11},readCell{12}];
MagnetoXYZ = [readCell{9},readCell{10},readCell{11}];
alpha = [readCell{12}];
% mu = [readCell{13}];
qGMVD = [readCell{13}, readCell{14}, readCell{15}, readCell{16}];
qKalman = [readCell{17}, readCell{18}, readCell{19}, readCell{20}];
end