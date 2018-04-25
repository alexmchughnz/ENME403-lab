function checkResponse(V, dV, y, r, setTime, VLim, dVLim, setTol)
%checkResponse Prints response info.
%Determines if control effort and settling meets criteria.
    
%Tests
err = r - y(end);
VPass = isempty(find(abs(V) > VLim, 1));
dVPass = isempty(find(abs(dV) > dVLim, 1));
setPass = abs(y(end) - r) < setTol;

%Printed Results
fprintf("Error of %f [m] (%.1f%%)\n", ...
        err, err/r*100);
fprintf("Settling time of %f [s]\n", ...
        setTime);

resp = ["BAD","OK"];
fprintf("Voltage %s. max = %.2f / %d [V]\n", ...
        resp(VPass+1), max(abs(V)), VLim);
fprintf("Slew Rate %s. max = %.2f / %d [V/s]\n", ...
        resp(dVPass+1), max(abs(dV)), dVLim);
fprintf("Settling %s. final = %.5f / %.3f [m]\n", ...
        resp(setPass+1), y(end), r);

end