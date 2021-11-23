
function MEC529_FINAL_PROJECT_111270310
%%MEC 529 MOTION PLANNING ALGORITHM
%%BY ALEXANDER RUFRANO
%%111270310
hold on
%%JOINT ANGLE LIMITS
S0 = [0.89,-2.461];
S1 = [1.047,-2.147];
E0 = [3.028,-3.028];
E1 = [2.618,-0.052];
W0 = [3.059,-3.059];
W1 = [2.094,-1.571];
W2 = [3.059,-3.059];
Limits = [S0;S1;E0;E1;W0;W1;W2];

%%Link Lengths %%IN MM
l0 = 270.35;%%IN MM
l1 = 69;%%IN MM
l2 = 364.35;%%IN MM
l23 = 69;%%IN MM
l3 = 370.82;%%IN MM
l4 = 374.29;%%IN MM
l45 = 10;%%IN MM
l5 = 374.42;%%IN MM
l6 = 229.525;%%IN MM

%%%Joint Axis Direction
w1r = [0;0;1]; %%S0
w2r = [1;0;0]; %%S1
w3r = [0;1;0]; %%EO
w4r = w2r; %%E1
w5r = w3r; %%WO
w6r = w2r; %%W1
w7r = w3r; %%W2

%%POINTS ON AXES
q1 = [0;0;l0]/1000; %%IN METERS SO
q2 = [0;l1;l0]/1000;%%IN METERS S1
q3 = [0;l1+l2;l0]/1000;%%IN METERS EO
q4 = [0;l1+l2;l0-l23]/1000;%%IN METERS E1
q5 = [0;l1+l2+l4;l0-l23]/1000;%%IN METERS W0
q6 = [0;l1+l2+l4;l0-l23-l45]/1000;%%IN METERS W1
q7 = [0;l1+l2+l4+l6;l0-l23-l45]/1000;%%IN METERS W2
q = [q1,q2,q3,q4,q5,q6,q7];%%IN METERS
i=1;

ERROR = 100; %%START AT 100%
jh = 1; %%USED TO
RPY_angles=[0;0;-45]; %%ANGLES END EFFECTOR
beta = 0.1;%%ANGLE INCREMENT TERM
tau = 0.1; %%EXPONENT VALUE
limitcount = 1;%%FOR CHECKING HOW MANY TIMES IT WENT NEAR JOINT LIMITS AND WAS CORRECTED
while ERROR > 0.3 %% 0.1% ERROR
    
    if i == 1
        %%STEP ZERO
        %%OBTAIN ALL KNOWN VALUES AND EVALUATE THEM
        
        %%INITIAL POSITION
        gst_step(1:4,1:4,i) = [eye(3),q7;zeros(1,3),1]; %%INITAL POSITION OF END EFFECTOR
        Rinitial = gst_step(1:3,1:3,i); %%ROTATIONAL MATRIX FROM GST0
        pinital = gst_step(1:3,4,i);    %%POSITION TO END EFF FROM GST0
        p_step(1:3,i)=pinital;          %%INITAL POSITION FROM GST0
        RPY_angles=deg2rad(RPY_angles); %%ANGLES TO ROTATE END EFFECTOR FRAME
        R_test=RPY_to_Rot(RPY_angles);  %%ROTATIONAL MATRIX BASED ON THE ABOVE ANGLES
        g_final = [R_test,[0.5;0.5;0.5];zeros(1,3),1]; %%FINAL CONFIG GST(FINAL)
        %g_final = [[1,0,0;0,0,1;0,1,0],[0.4;0.5;0.1];zeros(1,3),1];
        Rf = g_final(1:3,1:3); %%FINAL ROTATIONAL MATRIX
        pf = g_final(1:3,4);   %%FINAL END EFFECTOR POSITION
        Qf = Rot_to_Quat(Rf);  %%FINAL QUARTERION VALUES
        Arf = Qf;              %%FINAL Ar
        Adf = Adi(pf,Arf);     %%FINAL Ad
        Af = [Arf,Adf];        %%Afinal
    end
    %%GO BACK TO INITIAL CONFIGUATION ONCE SATISFIED WITH FINAL POSITION
    %%UNHIGHLIGHT THIS IF YOU WANT THE ROBOT TO GO BACK INTO ITS INITAL CONFIGURATION
    %         if ERROR < .5 %%THIS IS TO BRING IT BACK TO ITS INITAL CONFIGURATION
    %
    %             if jh == 1
    %                 itemp = i; %%RECORD THE i VALUE WHEN IT IS GOING BACK TO INITAL CONFIG
    %             end
    %             g_final = [eye(3),q7;zeros(1,3),1]; %%NEW FINAL CONFIGURATION
    %             Rf = g_final(1:3,1:3); %%new FINAL ROTATIONAL MATRIX
    %             pf = g_final(1:3,4);   %%new FINAL END EFFECTOR POSITION
    %             Qf = Rot_to_Quat(Rf);  %%new QUARTERION
    %             Arf = Qf;              %%new Ar Final
    %             Adf = Adi(pf,Arf);     %%new Ad Final
    %             Af = [Arf,Adf];        %%new A Final
    %             jh=2;                  %%STRICTLY FOR PLOTTING PURPOSES
    %         end
    
    %%STEP1
    Rref(1:3,1:3,i) = gst_step(1:3,1:3,i)'*Rf; %%REFERENCE ROTATIONAL MATRIX
    p = gst_step(1:3,4,i);                     %%REFERENCE POSITION VECTOR
    Q = Rot_to_Quat(gst_step(1:3,1:3,i));      %%REFERENCE QUARTERION
    pref = pf - p;                             %%FIND P on the REF FRAME
    gamma_t_plus_one(1:7,i) = [p;Q];           %%GAMMA(t)
    Ar = Q;                                    %%Ar BASED ON REF QUART
    Ad = Adi(p,Ar);                            %%Ad BASED ON REF p AND QUART
    A(1:4,1:2,i) = [Ar,Ad]; %%A(t),i           %%A Reference
    
    %%STEP2
    At_star = Astar(A(1:4,1:2,i)); %%A(t)*
    At_star_Af = DQuartMulti(At_star,Af);%%A*(t) x Af
    %%pref is only used when there is no rotational motion and only
    %%translational in the function below to find the unit vector, u
    [theta_At_star_Af(i),u_At_star_Af(1,i),u_At_star_Af(2,i),u_At_star_Af(3,i)] = Q_2_THETA_U(At_star_Af(1:end,1),pref);%%INPUT OF Ar from A*(t) x Af
    p_Astar_Af(1:4,i) = 2*QuartProduct(At_star_Af(1:4,2),[At_star_Af(1,1);-1*At_star_Af(2:end,1)]); %%P of A*(t)xAf
    At_star_Af_tau = DualQuartExpTau(theta_At_star_Af(i), tau, u_At_star_Af(1:3,i),p_Astar_Af(2:4,i)); %%(A*(t) x Af)^tau
    At_plus_one =   DQuartMulti(A(1:4,1:2,i),At_star_Af_tau); %%A(t+1)
    
    %%STEP 3
    P_At_plus_one = 2*QuartProduct(At_plus_one(1:4,2),[At_plus_one(1,1);-1*At_plus_one(2:end,1)]);%%P(t+1)
    gamma_t_plus_one(1:7,i+1) = [P_At_plus_one(2:end);At_plus_one(1:end,1)]; %%Gamma(t+1)
    
    %%STEP4
    if i == 1 %%WHEN i IS ONE, ESSENTIALLY IT IS ZERO BECAUSE I TREAT IT AS INITAL CONDITION FROM STEP 0
        q = [q1,q2,q3,q4,q5,q6,q7]; %%POINTS ON AXIS'S
        w = [w1r,w2r,w3r,w4r,w5r,w6r,w7r]; %%UNIT AXIS OF ROTATION
        type = [1;1;1;1;1;1;1]; %%ALL REVOLUTE JOINTS
        typedirect = ['R','R','R','R','R','R','R'];%%ALL REVOLUTE JOINTS
        thetaplus1(1:7,1,i) = [0;0;0;0;0;0;0]; %%JOINT SPACE theta(t), t=0
        Jsi(1:6,1:7,i) = SpatialmanipJac(w,q,type,thetaplus1(1:7,1,i));%%JACOBIAN(t=0)
    else
        Jsi(1:6,1:7,i) = SpatialmanipJac(w,q,type,thetaplus1(1:7,1,i));%%JACOBIAN(t>0)
    end
    
    J1 = J_1(Ar); %%J1 MATRIX
    J2 = J_2(p,J1); %%J2 MATRIX
    B = transpose(Jsi(1:6,1:7,i))*(Jsi(1:6,1:7,i)*Jsi(1:6,1:7,i)')^(-1)*J2;%%B MATRIX
    
    %%Theta(t+1)
    thetaplus1(1:7,1,i+1) = thetaplus1(1:7,1,i) + beta*B*(gamma_t_plus_one(1:7,i+1)-gamma_t_plus_one(1:7,i));
    
    %     %%CHECK IF JOINT ANGLES WITHIN LIMITS
    HL = Limits(1:7,1)-thetaplus1(1:7,1,i+1);%%CHECK IF NEGATIVE
    LL = Limits(1:7,2)-thetaplus1(1:7,1,i+1); %%CHECK IF POSITIVE
    
    %%CHECK IF ANY JOINTS ARE BEYOND THE REQUIRED LIMITS OF THE MACHINE
    if (any(thetaplus1(1:7,1,i+1) > Limits(1:7,1)) || any(thetaplus1(1:7,1,i+1)< Limits(1:7,2))) == 1
        %%IF LOWER LIMIT IS REACHED, CORRECT AND ADD 10% CUSHION
        if any(thetaplus1(1:7,1,i+1) < Limits(1:7,2)) == 1
            LL = LL*1.1;%%ADD 10% CUSHION
            LL(LL<0)=0;%%ELIMITATE ALL VALUES OTHER THAN THE LIMIT RESTRICTING ONE
            thetaplus1(1:7,1,i+1) = thetaplus1(1:7,1,i+1) + LL;%%CORRECT THETA(t+1)
            disp("LOW LIMIT REACHED");%%DISPLAY FOR INFO PURPOSES
            limitlog(limitcount) = sprintf("LOW %1d",i);%%SAVE LOG OF EVENT
            limitcount = limitcount + 1;%%ADD NEW TO COUNT
        end
        %%IF HIGHER LIMIT IS REACHED, CORRECT AND ADD 10% CUSHION
        if any(thetaplus1(1:7,1,i+1) > Limits(1:7,1)) == 1
            HL = HL*1.1; %%ADD 10% CUSHION
            HL(HL>0) = 0; %%ELIMITATE ALL VALUES OTHER THAN THE LIMIT RESTRICTING ONE
            thetaplus1(1:7,1,i+1) = thetaplus1(1:7,1,i+1) - HL; %%CORRECT THETA(t+1)
            disp("HIGH LIMIT REACHED"); %%DISPLAY FOR INFO PURPOSES
            limitlog(limitcount) = sprintf("HIGH %1d",i); %%SAVE LOG OF EVENT
            limitcount = limitcount + 1; %%ADD NEW TO COUNT
        end
        
    end
    
    %%STEP 5
    %%DETERMINE NEW CONFIGURATION BASED ON THETA(t+1)
    [gst, transform_upto_joint] = direct_kin(gst_step(1:4,1:4,1), typedirect, w, q, thetaplus1(1:7,1,i+1));
    gst_step(1:4,1:4,i+1) = gst; %%NEW CONFIGURATION
    p_step(1:3,i+1) = gst_step(1:3,4,i+1); %%NEW POSITION VECTOR
    %%CHECK ERROR
    GSTERROR = (g_final-gst_step(1:4,1:4,i+1))*100; %%BY HOW MUCH IS IT FROM GOAL IN %
    ERROR = max(max(abs(GSTERROR))); %%FIND THE HIGHEST VALUE OF ERROR
    ERROR  %%DISPLAY ERROR FOR TESTING PURPOSES
    
    
    i=i+1; %%GO TO NEXT ITERATION
    
end
% axis([-2 2 -2 2 -2 2])
i
%%%
% % if RPY_angles(2) == 1 %%Eliminates round-off error from calculations.
% %
% % else
% gst_step(1,4,1:i) = gst_step(1,4,10);
% gst_step(2,4,1:i) = gst_step(2,4,10);
% gst_step(3,4,1:i) = gst_step(3,4,10);
% % end

text(p_step(1,1),p_step(2,1),p_step(3,1),"inital position");
text(p_step(1,i),p_step(2,i),p_step(3,i),"    final position");
hold on
xlabel("x-axis");
ylabel("y-axis");
zlabel("z-axis");
plot3(p_step(1,1:i),p_step(2,1:i),p_step(3,1:i),'-o');%%PLOT ONCE ALL VALUES ARE OBTAINED
%quiver3(gst_step(1,4,1:i),gst_step(2,4,1:i),gst_step(3,4,1:i),gst_step(1,1,1:i),gst_step(2,1,1:i),gst_step(3,1,1:i));
quiver3(gst_step(1,4,1:i),gst_step(2,4,1:i),gst_step(3,4,1:i),gst_step(1,3,1:i),gst_step(2,3,1:i),gst_step(3,3,1:i));
quiver3(gst_step(1,4,1:i),gst_step(2,4,1:i),gst_step(3,4,1:i),gst_step(1,2,1:i),gst_step(2,2,1:i),gst_step(3,2,1:i));


%%STRICTLY FOR MOTION VIDEO PURPOSES
figure
hold on
xlabel("x-axis");
ylabel("y-axis");
zlabel("z-axis");
robot = loadrobot('rethinkBaxter','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');

axis([-1 1 -1 1 -0.1 1.5]);
% timeStep = 0.2; % seconds
% toolSpeed = 0.1; % m/s
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
trajTimes = 1:1:i;
timeInterval = [trajTimes(1); trajTimes(end)];
endEffector = "left_gripper";
for i=1:length(trajTimes)
    configNow = [0,thetaplus1(1:7,1,i)',zeros(1,7)]; %%USE MY VALUES OF THETA
    poseNow = getTransform(robot,configNow,endEffector); %%FUNCTION FROM ROBOT TOOLBOX
    show(robot,configNow,'PreservePlot',false,'Frames','off');%%PLOTS BAXTER MOVEMENT
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20) %%PLOTS POINT
    drawnow;
end
end