% kmuafunct - function to draw the log function
% described for OPTION [A] to create kmuA
% in the word document
% This can be used to visualize the effects of changing
% the 2 adjustable parameters, r and p
%
% SYNTAX: kmua = kmuafunct(r,p);
%
function kmua = kmuafunct(r,p);

heightscale = 4.605 * r ;
% create values of cosine fomr 0 to, every 0.01
cosk2 = linspace(0,100,101) /100;
kmua = zeros(size(cosk2));
% apply the function
for n=1:101
    kmua(n) = (-r) * log( p *(1 - cosk2(n) ) ) ; 
    % Scale it
    kmua(n) = kmua(n) / heightscale;
end
 % open a new figure
 figure;
 % Plot the trace
 plot(cosk2,kmua); grid on;
  xlabel('cosine of angle'); ylabel('resulting kMUang A')
 % axis([0,1,0,inf]);
    
end
