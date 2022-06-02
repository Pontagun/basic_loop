% syntax: r = myQuatProd(w,q)
% where r, q and w are 4-element row vectors
% representing quaternion with the scalar component at last

function r = myQuatProd(q,w)

r = [ w(4), w(3),-w(2),w(1);
      -w(3), w(4), w(1),w(2);
       w(2),-w(1), w(4),w(3);
      -w(1),-w(2),-w(3),w(4)] * transpose(q);
  
r = transpose(r);

end