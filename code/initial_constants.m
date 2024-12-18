function constants=initial_constants()
    
    % Constants 
    Ix = Ix; %kg*m^2
    Iy = Iy; %kg*m^2
    Iz  = Iz; %kg*m^2
    m  = m; %kg
    g  = 9.81; %m/s^2
    Jtp= Jtp; %N*m*s^2=kg*m^2
    Ts=0.1; %s
    
    % Matrix weights for the cost function (They must be diagonal)
    Q=[10 0 0;0 10 0;0 0 10]; % weights for outputs (output x output)
    S=[10 0 0;0 10 0;0 0 10]; % weights for the final horizon outputs (output x output)
    R=[10 0 0;0 10 0;0 0 10]; % weights for inputs (input x input)
    
    ct = 7.6184*10^(-8)*(60/(2*pi))^2; %N*s^2
    cq = 2.6839*10^(-9)*(60/(2*pi))^2; %N*m^2
    l = 0.171; %m;
    
    controlled_states=3;
    hz = 4; % horizon period
    
    innerDyn_length=4; % Number of inner control loop iterations
    
    px=[-1 -2];
    py=[-1 -2];
    pz=[-1 -2];
    
    % Choose your trajectory (1,2,3,4,5)
    trajectory=1;
    
    constants={Ix Iy Iz m g Jtp Ts Q S R ct cq l controlled_states hz innerDyn_length px py pz trajectory};

end
