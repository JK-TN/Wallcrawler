close all
clear all
clc

theta = [0;0;0];
a = [0.0254;0.05398;0.073025];
d = [0;0;0];
alpha = [pi/2;0;0];
gain = 20;
goal = [0.1;0.039;0.044];
pose = [0;0;0];
ts = 0.001;
Str = [];
t = [];
diff = [];
it = 1000;

for i = 1:it

    H1_0 = [[cos(theta(1)),-sin(theta(1))*cos(alpha(1)),sin(theta(1))*sin(alpha(1)),a(1)*cos(theta(1))];
        [sin(theta(1)),cos(theta(1))*cos(alpha(1)),-cos(theta(1))*sin(alpha(1)),a(1)*sin(theta(1))];
        [0,sin(alpha(1)),cos(alpha(1)),d(1)];
        [0,0,0,1]];

    H2_1 = [[cos(theta(2)),-sin(theta(2))*cos(alpha(2)),sin(theta(2))*sin(alpha(2)),a(2)*cos(theta(2))];
        [sin(theta(2)),cos(theta(2))*cos(alpha(2)),-cos(theta(2))*sin(alpha(2)),a(2)*sin(theta(2))];
        [0,sin(alpha(2)),cos(alpha(2)),d(2)];
        [0,0,0,1]];

    H3_2 = [[cos(theta(3)),-sin(theta(3))*cos(alpha(3)),sin(theta(3))*sin(alpha(3)),a(3)*cos(theta(3))];
        [sin(theta(3)),cos(theta(3))*cos(alpha(3)),-cos(theta(3))*sin(alpha(3)),a(3)*sin(theta(3))];
        [0,sin(alpha(3)),cos(alpha(3)),d(3)];
        [0,0,0,1]];

    T2_0 = H1_0*H2_1;
    T3_0 = T2_0*H3_2;
    pose = T3_0(1:3,4);
    Str = [Str,T3_0(1:3,4)];

    R1_0 = H1_0(1:3,1:3);
    R2_0 = T2_0(1:3,1:3);

    d3_0 = [T3_0(1,4);T3_0(2,4);T3_0(3,4)];
    d2_0 = [T2_0(1,4);T2_0(2,4);T2_0(3,4)];
    d1_0 = [H1_0(1,4);H1_0(2,4);H1_0(3,4)];

    d3_1 = d3_0-d1_0;
    d3_2 = d3_0-d2_0;

    rot1 = [0;0;1];
    rot2 = R1_0*rot1;
    rot3 = R2_0*rot1;

    tran1 = transpose(cross(transpose(rot1),transpose(d3_0)));
    tran2 = transpose(cross(transpose(rot2),transpose(d3_1)));
    tran3 = transpose(cross(transpose(rot3),transpose(d3_2)));
    J = [tran1,tran2,tran3];

    deltaE = gain*(goal-pose);
    diff = [diff, deltaE];
    deltaQ = pinv(J)*deltaE;
    Q = deltaQ*ts;
    theta = theta+Q;
    t = [t,theta];

end

figure(1)
plot3(goal(1),goal(2),goal(3),'o')
hold on
plot3(Str(1,1),Str(2,1),Str(3,1),'x')
hold on
plot3(Str(1,2:it),Str(2,2:it),Str(3,2:it),'.')
legend("goal","start")
xlabel("x")
ylabel("y")
zlabel("z")
title("End-Effector Pose vs Goal Pose")

figure(2)
plot(diff(1,1:it))
hold on
plot(diff(2,1:it))
hold on
plot(diff(3,1:it))
legend('x','y','z')
title("Error vs Time")
xlabel("time (ms)")
ylabel("Error")

figure(3)
plot(t(1,1:it))
hold on
plot(t(2,1:it))
hold on
plot(t(3,1:it))
legend("motor1","motor2","motor3")
xlabel("time")
ylabel("angle")
title("Motor angles vs time")