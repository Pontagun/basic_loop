% mapBtoI - Function to map a 3D vector vb according to UNIT quat q
% from the BODY frame to the INERTIAL frame
% It implements vout = q * vi * q'
% q - is passed as a 1 x 4 row vector, scalar last
% vb is passed as a 3-element vector (vi returned same format)
% ( Paired with mapItoB to perform the "backwards - opposite - mapping")
% SYNTAX:  vi = mapItoB(q , vb);

function vi = mapBtoI(q , vb);
vb4 = [vb,0];  % Appends 0 in last place to make vb "pure quat"
   vi4 = myQuatProd(q,myQuatProd(vb4,myQuatConj(q)));
   vi = vi4(1:3); % Convert pure quaternion result to 3D Vector
end