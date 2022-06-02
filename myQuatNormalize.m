function normalized_q = myQuatNormalize(q)

normalized_q = q/(sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2));

end