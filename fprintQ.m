function fprintQ(q_old,q_new)

fp_old = fopen('Results/qBefore.txt','w');
fp = fopen('Results/qAfter.txt','w');

for i=1:1:length(q_old)-50
    fprintf(fp_old,'%f,%f,%f,%f\n',q_old(i,1),q_old(i,2),q_old(i,3),q_old(i,4));
    fprintf(fp,'%f,%f,%f,%f\n',q_new(i,1),q_new(i,2),q_new(i,3),q_new(i,4));
end

fclose(fp_old);
fclose(fp);
end

