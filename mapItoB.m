% mapItoB - Function to rotate a 3D vector vi according to UNIT quat q
% from the INERTIAL frame to the BODY frame
% It implements vout = q' * vi * q
% q - is passed as a 1 x 4 row vector, scalar last
% vi is passed as a 3-element vector (vb returned same format)
% ( Paired with mapBtoI to perform the "backwards - opposite - mapping")
% SYNTAX:  vb = mapItoB(q , vi);

function vb = mapItoB(q , vi);
vi4 = [vi,0];  % Appends 0 in last place to make vi "pure quat"
   vb4 = myQuatProd(myQuatConj(q),myQuatProd(vi4,q));
   vb = vb4(1:3); % Convert pure quaternion result to 3D Vector
end