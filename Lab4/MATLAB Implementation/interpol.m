function a = interpol(startP,endP,startV,endV,startA,endA,tend,deg)
%interpol returns the 'deg' degree's polynomials parameters based on starting and ending points/velocities.
%   interpol(startP,endP,startV,endV,tend,deg) returns the a parameters of
%   a polynomial based on starting and ending values, velocities
%   accelerations.
%
%   Examples:
%       For the 3rd degree polynomial:
%           a = interpol(startP,endP,startV,endV,startA,endA,tend,deg);
%           P=a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3;
%
%       For the 4th degree polynomial:
%           a = interpol(startP,endP,startV,endV,startA,endA,tend,deg);
%           P=a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4;
%
%       For the 5th degree polynomial:
%           a = interpol(startP,endP,startV,endV,startA,endA,tend,deg);
%           P=a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4+a(:,6)*(t-t0).^5;
%
%----------------------------PROJECT VERSION-------------------------------
    if (deg==1)
        
        a0=startP;
        a1=(endP-startP)/tend;
        a=[a0,a1];
        
    elseif (deg==3)
        
        a0=startP;
        a1=startV;
        a2=(3/tend^2)*(endP-startP)-(2/tend)*startV-endV/tend;
        a3=-(2/tend^3)*(endP-startP)+(startV+endV)/tend^2;
        a=[a0, a1, a2, a3];
        
    elseif (deg==4)
        %To be used only in case of:
        % startA=endA=0 & (endP-startP)/tend=(startV+endV)/2
        
        statement=(startA~=0)|(endA~=0)|(((endP-startP)/tend)~=((startV+endV)/2));
        if statement
            warning('Interpolation for degree 4 is not right')
        end
        
        a0=startP;
        a1=startV;
        a2=startA/2;
        a3=(10/tend^3)*(endP-startP)-(4*endV+6*startV)/tend^2;
        a4=-(15/tend^4)*(endP-startP)+(7*endV+8*startV)/tend^3;
        a=[a0, a1, a2, a3, a4];
        
    elseif (deg==5)
        a0=startP;
        a1=startV;
        a2=startA/2;
        a3=(10/tend^3)*(endP-startP)-(4*endV+6*startV)/tend^2-(3*startA-endA)/(2*tend);
        a4=-(15/tend^4)*(endP-startP)+(7*endV+8*startV)/tend^3+(3*startA-2*endA)/(2*tend^2);
        a5=(6/tend^5)*(endP-startP)-(3*endV+startV)/tend^4-(startA-endA)/(2*tend^3);
        a=[a0, a1, a2, a3, a4, a5];
        
    else
        error('Not Supported Degree')
    end
end

