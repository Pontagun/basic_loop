% qROT - Function ROTATE a 3D vector v1 WITHIN A SINGLE REFERENCE FRAME
% according to a UNIT quaternion q
% from an intial direction v1 to a ROTATED direction v2
% It implements vout = q * vi * q'
% q - is passed as a 1 x 4 row vector, scalar last
% v1 is passed as a 3-element vector (v2 returned same format)
%
% SYNTAX:  v2 = qROT(q , v1);

function v2 = qROT(q , v1);
v14 = [vb,0];  % Appends 0 in last place to make v1 "pure quat"
   v24 = myQuatProd(myQuatConj(q),myQuatProd(v14,q));
   v2 = v24(1:3); % Convert pure quaternion result to 3D Vector
end