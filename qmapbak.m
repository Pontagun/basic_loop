% qmapbak-Fnct. to rotate a 3d vector vin according to invere of quat q
% It implements vout = q * vin * q'
% q - is passed as a 1 x 4 row vector, scalar last
% vin is passed as a 3-element vector (vout returned same format)
% ( Paired with qrotfwd to perform the "forward- rotation")
% SYNTAX:  vout = qmapbak(q , vin);

function vout = qmapbak(q , vin);
vin4 = [vin,0];  % Appends 0 in last place to make vin "pure quat"
   vout4 = myQuatProd(q,myQuatProd(vin4,myQuatConj(q)));
   vout = vout4(1:3); % Convert pure quaternion result to 3D Vector
end