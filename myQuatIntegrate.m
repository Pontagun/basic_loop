function q1 = myQuatIntegrate(dq,q0,dt)

w = 2 * myQuatProd(dq,myQuatConj(q0));
exp = myQuatExponential((w * dt)/2);
q1 = myQuatProd(exp,q0);

end