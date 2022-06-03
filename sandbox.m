[label,t,Stillness,GyroXYZ,AcceleroXYZ,MagnetoXYZ,alpha,mu] = readRecordingFile('Data001.txt');
t_diff = diff(t);
t_diff_avg = mean(t_diff);

t_diff_max = max(t_diff);



figure
plot(t_diff(3:end));
title('Average sampling interval is ', (t_diff_avg));

