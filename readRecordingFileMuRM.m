function [label,t,Stillness,GyroXYZ,AcceleroXYZ,MagnetoXYZ,alpha] = readRecordingFileMuRM(FILENAME)

label=FILENAME;
fileID = fopen(FILENAME);
readCell=textscan(fileID,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','delimiter',',');
fclose(fileID);

t = readCell{1};
Stillness = readCell{2};
GyroXYZ = [readCell{3},readCell{4},readCell{5}];
AcceleroXYZ = [readCell{6},readCell{7},readCell{8}];
%IMUquat = [readCell{9},readCell{10},readCell{11},readCell{12}];
MagnetoXYZ = [readCell{9},readCell{10},readCell{11}];
alpha = [readCell{12}];
% mu = [readCell{13}];

end

