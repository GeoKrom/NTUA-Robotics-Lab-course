function [Pd,Vd,Ad] = trajGeneration(Parameters,tspan,space,type)
%trajGeneration returns the trajectory points needed for a motion
%   trajGeneration(Parameters,tspan,space,type) generates the result of 
%   the interpolation of a motion based on the type of motion and
%   using the given time vector. Rarameters is a struct containing the
%   start trajectory point, end trajectory point and the extra parameters
%   used. trajGeneration supports the 'Joint' and 'Cartesian' space options.
%
%   trajGeneration supports the 'parabMix', 'Standard'(3rd degree
%   polynomial), '5thDegree' (5th Degree polynomial), '3Phase' and 'Spline'
%   types.
%
%   trajGeneration supports the 'Line', 'Circle', 'Eclipse' and 'Spline'
%   types. The interpolation of the Parameter used from the curve
%   parameterization can be obtained from a 3rd degree polynomial, a 5th
%   degree polynomial and a 3 phase trajectory methodology.
%
%   Parameters is a struct containing (when the parameter is needed):
%   1)StartPosition and EndPosition
%   2)StartVelocity and EndVelocity
%   3)StartAcceleration and EndAcceleration
%   4)PhaseTime
%   5)Parameterization ('Standard','5thDegree','ParabMix','3Phase')
%   
%   Example 1:
%     Parameters.StartPosition = [10,30,10];
%     Parameters.StartVelocity = [0,0,0];
%     Parameters.EndPosition = [15,20,10];
%     Parameters.EndVelocity = [0,0,0];
%     pd = trajGeneration(Parameters,0:step:T,'Joint','Standard')
%
%   Example 2:
%     Parameters.StartPosition = [10,30,10];
%     Parameters.StartVelocity = [0,0,0];
%     Parameters.StartAcceleration = [0,0,0];
%     Parameters.EndPosition = [15,20,10];
%     Parameters.EndVelocity = [0,0,0];
%     Parameters.EndAcceleration = [0,0,0];
%     pd = trajGeneration(Parameters,0:step:T,'Joint','5thDegree')
%
%   Example 3:
%     Parameters.StartPosition = [10,30,10];
%     Parameters.EndPosition = [15,20,10];
%     Parameters.PhaseTime = 2;
%     pd = trajGeneration(Parameters,0:step:T,'Joint','3Phase')
%
%   Example 4:
%     Parameters.StartPosition = [10,30,10];
%     Parameters.StartVelocity = [0,0,0];
%     Parameters.StartAcceleration = [0,0,0];
%     Parameters.EndPosition = [15,20,10];
%     Parameters.EndVelocity = [0,0,0];
%     Parameters.EndAcceleration = [0,0,0];
%     Parameters.Parameterization = 'Standard'; %Uses the 3rd degree pol.
%     pd = trajGeneration(Parameters,0:step:T,'Cartesian','Line')
%
%   Example 5:
%     Parameters.StartPosition = [10,30,10];
%     Parameters.StartVelocity = [0,0,0];
%     Parameters.StartAcceleration = [0,0,0];
%     Parameters.EndPosition = [15,20,10];
%     Parameters.EndVelocity = [0,0,0];
%     Parameters.EndAcceleration = [0,0,0];
%     pd = trajGeneration(Parameters,0:step:T,'Cartesian','Line')
%
%----------------------------PROJECT VERSION-------------------------------

if strcmp(space,'Joint')
    %Creating a trajectory on the Joint space
    if strcmp(type,'Standard')
        %Creating a 3rd degree interpolation on Joint space
        
        %Parameter Init
        q0=Parameters.StartPosition;
        q0_dot=Parameters.StartVelocity;
        q0_ddot=[];
        qf=Parameters.EndPosition;
        qf_dot=Parameters.EndVelocity;
        qf_ddot=[];
        t0=tspan(1); %StartingTime need to be scalar
        t=tspan;
        
        %For each Joint creating the interpolation using 3rd degree
        a=interpol(q0,qf,q0_dot,qf_dot,q0_ddot,qf_ddot,tspan(end),3);
        Pd = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3;
        
    elseif strcmp(type,'5thDegree')
        %Creating a 5th degree interpolation on Joint space
        
        %Parameter Init
        q0=Parameters.StartPosition;
        q0_dot=Parameters.StartVelocity;
        q0_ddot=Parameters.StartAcceleration;
        qf=Parameters.EndPosition;
        qf_dot=Parameters.EndVelocity;
        qf_ddot=Parameters.EndAcceleration;
        t0=tspan(1);
        t=tspan;
        
        %For each Joint creating the interpolation using 3rd degree
        a=interpol(q0,qf,q0_dot,qf_dot,q0_ddot,qf_ddot,tspan(end),5);
        Pd = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4+a(:,6)*(t-t0).^5;
        
    elseif strcmp(type,'parabMix')
        %Creating a parabolic mix interpolation on Joint space (Trapezodial Velocity)
        
        %Parameter Init
        q0=Parameters.StartPosition;
        qf=Parameters.EndPosition;
        V=Parameters.Velocity;
        tb=Parameters.PhaseTime;
        t0=tspan(1); %StartingTime needs to be scalar
        a=V/tb;
        
        %Parabolic Blend - Trapezodial Velocity
        %Phase 1
        t=tspan(tspan<=tb);
        Pd_1 = q0+a*(t-t0).^2;
        %Phase 2
        t=tspan((tspan>=(tb)) & (tspan<=(tspan(end)-tb)));
        Pd_2 = (q0+qf-V*tspan(end))/2+V*(t-t0); 
        %Phase 3
        t=tspan(tspan>=(tspan(end)-tb));
        Pd_3 = qf-a*tspan(end)^2/2+a*tspan(end)*(t-t0)-(a/2)*(t-t0).^2; 
        %Total Trajectory
        Pd=[Pd_1(:,1:(end-1)),Pd_2,Pd_3(:,2:end)];

    elseif strcmp(type,'3Phase')
        %Creating a 3 phase interpolation on Joint space
        
        %Parameter Init
        q0=Parameters.StartPosition;
        qf=Parameters.EndPosition; 
        t0=tspan(1); %Starting time needs to be scalar
        Delta=Parameters.PhaseTime/2; %Half of the Acceleration and Decceleration time
        
        %Intermediate Positions and Velocities
        q02_dot=(qf-q0)/(tspan(end)-2*Delta);
        q02=q0+Delta*q02_dot;
        qf2=q0+(tspan(end)-3*Delta)*q02_dot;
        
        %Phase 1 of the Trajectory - Acceleration
        t=tspan(tspan<=2*Delta);
        %t0=tspan(1);
        a=interpol(q0,q02,... %Position
                zeros(size(q0)),q02_dot,... %Velocity
                zeros(size(q0)),zeros(size(q0)),... %Acceleration
                2*Delta,4); %Finish time and Degree of polynomial
        Pd_1 = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4;
        
        %Phase 2 of the Trajectory - Acceleration
        t=tspan((tspan>=(2*Delta)) & (tspan<=(tspan(end)-2*Delta)));
        t0=2*Delta;
        a=interpol(q02,qf2,... %Position
                q02_dot,q02_dot,... %Velocity (Not used for Degree==1)
                zeros(size(q0)),zeros(size(q0)),... %Acceleration (Not used for Degree==1)
                tspan(end)-4*Delta,1); %Finish time and Degree of polynomial
        Pd_2 = a(:,1)+a(:,2)*(t-t0);
        
        %Phase 3 of the Trajectory - Acceleration
        t=tspan(tspan>=(tspan(end)-2*Delta));
        t0=tspan(end)-2*Delta;
        a=interpol(qf2,qf,... %Position
                q02_dot,zeros(size(qf)),... %Velocity
                zeros(size(qf)),zeros(size(qf)),... %Acceleration
                2*Delta,4); %Finish time and Degree of polynomial
        Pd_3 = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4;
        
        %Total Trajectory Points
        Pd=[Pd_1(:,1:(end-1)),Pd_2,Pd_3(:,2:end)];
        
    elseif strcmp(type,'Spline')
        %Creating a multi point 3rd degree interpolation on Joint space
        %Computed Velocity Approach
        
        %Joint points used
        q = Parameters.Points;
        Npoints=size(q,2);
        Tsteps=Parameters.TimeSteps; %size of this is 1xNpoints
        
        %Cubic interpolation for constant velocity
        Pd = [];
        q_dot=zeros(size(q));
        v=zeros(size(q));
        
        %For each Point
        for i=1:(Npoints-1)
            
            if (i==1)
                q_dot(i)=zeros(size(q,1),1);
            else
                v(:,i)=(q(:,i)-q(:,i-1))/(Tsteps(i)-Tstep(i-1));
                v(:,i+1)=(q(:,i+1)-q(:,i))/(Tsteps(i+1)-Tstep(i));
                %For Each joint calculate the velocity
                for j=1:size(q,1)
                    if sgn(v(j,i))~=sgn(v(j,i+1))
                        q_dot(j,i)=(v(j,i)+v(j,i+1))/2;
                    else
                        q_dot(j,i)=0;
                    end
                end
            end
            
            %Interpolation using the 3rd Degree Polynomial
            a=interpol(q(:,i),q(:,i+1),... %Position
                    q_dot(:,i),q_dot(:,i+1),... %Velocity
                    [],[],Tsteps(i+1),3);
             t=tspan((tspan>=Tsteps(i)) & (tspan<=Tsteps(i+1)));
             t0=Tsteps(i);
             P=a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3;
             Pd=[Pd,P]; %#ok<AGROW>
        end
        
    else
        %Not supported Type
        error('Wrong Type Argument')
    end
    
elseif strcmp(space,'Cartesian')
    %Creating a trajectory on the Cartesian/Work space
    if strcmp(type,'Line')
        %Creating a line on Cartesian space
        
        %Parameter Init
        P0=Parameters.StartPosition;
        Pf=Parameters.EndPosition;
        s0=0;
        sf=norm(Pf-P0);
        s0_dot=0;
        sf_dot=0;
        s0_ddot=0;
        sf_ddot=0;
        t0=tspan(1); %StartingTime needs to be scalar
        t=tspan;
        Parameterization=Parameters.Parameterization;
        
        %Interpolation
        if strcmp(Parameterization,'5thDegree')        
            %Interpolation using 5th degree
        
            a=interpol(s0,sf,s0_dot,sf_dot,s0_ddot,sf_ddot,tspan(end),5);
            s = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4+a(:,6)*(t-t0).^5;
        
        elseif strcmp(Parameterization,'parabMix')
            Delta=Parameters.PhaseTime;
            V = (sf -s0)/(tspan(end)-t0 - Delta);
            t0 = tspan(1); %StartingTime needs to be scalar
            a=V/Delta;
            %Parabolic Blend - Trapezodial Velocity
            %Phase 1
            t = tspan(tspan<=(t0+Delta));
            s_1 = s0+a/2*(t-t0).^2;
            %Phase 2
            t=tspan((tspan>=(t0+Delta)) & (tspan<=(tspan(end)-Delta)));
            s_2 = (s0+sf-V*(tspan(end)-t0))/2+V*(t-t0); 
            %Phase 3
            t=tspan(tspan>=(tspan(end)-Delta));
            s_3 = sf-a*(tspan(end)-t0)^2/2+a*(tspan(end)-t0)*(t-t0)-(a/2)*(t-t0).^2; 
            %Total Trajectory
             s = [s_1(:,1:(end-1)),s_2,s_3(:,2:end)];
        elseif strcmp(Parameterization,'3Phase')
            %Creating a 3 phase interpolation on a Line in the 
            %Cartesian space.For 2 points with starting and finishing 
            %velocity/acceleration equal to zero.
            Delta=Parameters.PhaseTime/2; %Half of the Acceleration and Decceleration time
        
            %Intermediate Positions and Velocities
            s02_dot=(sf-s0)/(tspan(end)-2*Delta-t0);
            s02=s0+Delta*s02_dot;
            sf2=s0+(tspan(end)-3*Delta-t0)*s02_dot;

            %Phase 1 of the Trajectory - Acceleration
            t=tspan(tspan<=(t0+2*Delta));
            %t0=tspan(1);
            a=interpol(s0,s02,... %Position
                    0,s02_dot,... %Velocity
                    0,0,... %Acceleration
                    2*Delta,4); %Finish time and Degree of polynomial
            s_1 = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4;

            %Phase 2 of the Trajectory - Acceleration
            t=tspan((tspan>=(t0+2*Delta)) & (tspan<=(tspan(end)-2*Delta)));
            t0=t0+2*Delta;
            a=interpol(s02,sf2,... %Position
                    s02_dot,s02_dot,... %Velocity (Not used for Degree==1)
                    0,0,... %Acceleration (Not used for Degree==1)
                    tspan(end)-2*Delta-t0,1); %Finish time and Degree of polynomial
            s_2 = a(:,1)+a(:,2)*(t-t0);

            %Phase 3 of the Trajectory - Acceleration
            t=tspan(tspan>=(tspan(end)-2*Delta));
            t0=tspan(end)-2*Delta;
            a=interpol(sf2,sf,... %Position
                    s02_dot,0,... %Velocity
                    0,0,... %Acceleration
                    2*Delta,4); %Finish time and Degree of polynomial
            s_3 = a(:,1)+a(:,2)*(t-t0)+a(:,3)*(t-t0).^2+a(:,4)*(t-t0).^3+a(:,5)*(t-t0).^4;

            %Total Trajectory Points
            s = [s_1(:,1:(end-1)),s_2,s_3(:,2:end)];

        elseif strcmp(Parameterization,'Standard') 
            %3rd degree polynomial is used
            a=interpol(s0,sf,s0_dot,sf_dot,tspan(end),3);
            s = a(1)+a(2)*(t-t0)+a(3)*(t-t0).^2+a(4)*(t-t0).^3;
        else
            error('Wrong Parameterization for the s parameter')
        end

        %Points using the s Parameter
        Pd = P0+(Pf-P0)*s/norm(Pf-P0);
        
    elseif strcmp(type,'Circle')
        %Creating a circle on Cartesian space
        error('Not Implemented')
    elseif strcmp(type,'Spline')
        %Creating a spline trajectory on Cartesian space
        error('Not Implemented')
    else
        %Not supported Type
        error('Wrong Type Argument')
    end
else
    error('Wrong Space Argument')
end

%Velocity and Acceleration Calculation
n=size(Parameters.StartPosition,1);
Vd=zeros(n,length(tspan)); %Position Velocity for each time step
Ad=zeros(n,length(tspan)); %Position Acceleration for each time step
Vd(:,1) = Parameters.StartVelocity;
Ad(:,1) = Parameters.StartAcceleration;
for t=2:length(tspan)
    %Velocity and Acceleration Calculation from the trajectory created
    %using the differentiation (Euler Method) on each time step
    Vd(:,t) = (Pd(:,t) - Pd(:,t-1))/(tspan(:,t)-tspan(:,t-1));
    Ad(:,t) = (Vd(:,t) - Vd(:,t-1))/(tspan(:,t)-tspan(:,t-1));
end

end