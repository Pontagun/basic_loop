% varrowtrc  draws a 3d graph where the arrowhead 
% of a vector that keeps changing direction
% creates a trace
% AssumptionMTXvect is numvects by 3
%
% SYNTAX:  numvects = varrowtrc( MTXvect);
%
function numvects = varrowtrc(MTXvect);

[numvects,nc] = size(MTXvect);
if(nc~= 3)
    display('num of columns in mtx is not 3');
end

ihat = [1, 0, 0];
orig3 = [0, 0, 0];
% Qnorms = sqrt( dot(Q,Q,2)); % Result should be a long vector of ~1s

figure;
%axis([-1, 1 , -1 , 1, -1, 1]); 
%view(45, 30);grid
axis([-2, 2 , -2 , 2, -2, 2]); 
view(90,30);grid

halfvects = 1431;
for r=1:halfvects   % 1st segment (blue), up tyo halfvects
    thisv = MTXvect(r,:);
    % draw 3-d line from 0,0,0 tp vout
    pairs = [ orig3 ; thisv ];
    line(pairs(:,1), pairs(:,2), pairs(:,3)  );
    pause(.005)
end
for r=(halfvects + 1):numvects   % second segment (red)
    thisv = MTXvect(r,:);
    % draw 3-d line from 0,0,0 tp vout
    pairs = [ orig3 ; thisv ];
    line(pairs(:,1), pairs(:,2), pairs(:,3),'Color','red');
    pause(.005)
end
xlabel('X'); ylabel('Y'); zlabel('Z');


end
