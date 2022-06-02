function exp_q = myQuatExponential(q)

    qvnorm2 = sqrt(q(1)^2 + q(2)^2 + q(3)^2);

    if(qvnorm2 ~= 0)
        exp_q = exp(q(4)) * [(sin(qvnorm2)/qvnorm2) * q(1:3), cos(qvnorm2)];
    else
        exp_q = exp(q(4)) * [q(1:3), cos(qvnorm2)];
    end
    
end