function [angchg] = anginertchg(MTXvect)
%ANGINERTCHG Summary of this function goes here
%   Detailed explanation goes here
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;
% SYNTAX:  [angchg] = anginertchg(MTXvect);

[numvects,nc] = size(MTXvect);

angchg = zeros(numvects,1);

% getting avg of mag vector from initial 30 samples
Minit = mean(MTXvect(1:30,:));
MagMinit = sqrt(Minit * (Minit'));

% at each sampling instant, calculte angle between MTX and Minit

for n = 1:numvects
    MTXnow = MTXvect(n,:);
    MagMTXnow = sqrt(MTXnow * (MTXnow'));
    % calculate cos of eta , hich is the angle beteen MTXnow and Minit
    coseta = (MTXnow * (Minit')) / (MagMinit * MagMTXnow);
    % the take inverse cosine to obtain anle eta which is returned  angchg
    angchg(n) = acos(coseta);
end


end
