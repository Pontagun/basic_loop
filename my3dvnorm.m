% my3dvnorm computes norms of multiple 3D vectors as rows in a matric
%
% syntax norms = my3dvnorm(m)
% 
function norms = my3dvnorm(m)
  norms = sqrt(sum((m.*m)'));

end