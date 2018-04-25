function stepCheck(r, S, stLim, rtLim, osLim, errTol, inputs)
%checkResponse Prints response info.
%Determines if control effort and settling meets criteria.
    
%Tests
err = r - S.SettlingMax;
stPass = S.SettlingTime < stLim;
rtPass = S.RiseTime < rtLim;
osPass = rad2deg(S.Peak) < osLim;
errPass = abs(err) < errTol;

%Printed Results
if (inputs(1))
fprintf("Error of %f [m] (%.1f%%)\n", ...
        err, err/r*100);
end

resp = ["BAD","OK"];
if (inputs(2))
fprintf("SettlingTime %s. value = %.2f / %d [s]\n", ...
        resp(stPass+1), S.SettlingTime, stLim);
end
if (inputs(3))
fprintf("RiseTime %s. value = %.2f / %d [s]\n", ...
        resp(rtPass+1), S.RiseTime, rtLim);
end
if (inputs(4))
fprintf("Overshoot %s. value = %.2f / %d [deg]\n", ...
        resp(osPass+1), rad2deg(S.Peak), osLim);
end 
if (inputs(5))
fprintf("Settling %s. value = %.2f / %d [s]\n", ...
        resp(errPass+1), S.SettlingMax, r);
end

end