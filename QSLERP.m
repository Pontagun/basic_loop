function Q_INTPL = QSLERP( Q_START, Q_END, ALPHA )

    OMEGA = acos( dotQuat(Q_START,Q_END) );
    
    Q_INTPL = ((Q_START*sin((1-ALPHA)*OMEGA)+Q_END*sin((ALPHA)*OMEGA))/sin(OMEGA));

end

