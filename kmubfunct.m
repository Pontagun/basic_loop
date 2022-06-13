% kmubfunct - function to draw the log function
% described for OPTION [B] to create kmuB
% in the word document
% This can be used to visualize the effects of changing
% the 2 adjustable parameters, r and p
%
% SYNTAX: kmub = kmubfunct(r,p);
%
function kmub = kmubfunct(r,p);

% heightscale = 4.605 * r ;
% create values of cosine from 0 to, every 0.01
cosk2 = linspace(0,100,101) /100;
% NOW RECOVERING THE ANGLE" (all 101 values)
kang = acos(cosk2);
kmub = zeros(size(cosk2));
% apply the function
for n=1:101
    kmub(n) = (-r) * log(p * kang(n)); 
    
    % Scale it
    % kmua(n) = kmua(n) / heightscale;
    
end
 % open a new figure
 figure;
 % Plot the trace
 plot(kang,kmub); grid on;
 xlabel('angle in radians'); ylabel('resulting kMUang B')
 % axis([0,1,0,inf]);
    
end
