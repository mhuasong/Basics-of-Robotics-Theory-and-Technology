robot.NB = 6;
robot.parent = [0 1 2 3 4 5];
robot.jtype = { 'Rz', 'Rz', 'Rz', 'Rz', 'Rz', 'Rz'};

%空间向量表示的惯性矩阵
Ilink1 = mcI(1.576,[0, 0.0053, -0.0088],[40.640,0,0;0,29.286,0;0,0,30.869]*10^-4);
Ilink2 = mcI(4.041,[0.2039, 0, 0.0127],[96.539,0,0;0,1449.938,0;0,0,1426.071]*10^-4);
Ilink3 = mcI(2.271,[0.1889, 0.0, 0.0981],[21.432,0,0;0,443.926,0;0,0,441.273]*10^-4);
Ilink4 = mcI(0.500,[0, 0.0062, -0.0039],[7.119,0,0;0,4.058,0;0,0,6.855]*10^-4);
Ilink5 = mcI(0.500,[0, -0.0062, -0.0039],[7.119,0,0;0,4.058,0;0,0,6.855]*10^-4);
Ilink6 = mcI(0.158,[0, 0.00017,-0.0213],[0.7313,0,0;0,0.7195,0;0,0,1.0877]*10^-4);

%MDH参数
alpha = [0,pi/2,pi,pi,pi/2,pi/2];
a = [0,0,0.408,0.376,0,0];
d = [0.122,0.1405,0,-0.019,0.1025,0.094];
offset = [0,pi/2,0,pi/2,pi,0];


theta = [0 pi/2 0 pi/2 pi 0];

T = zeros(robot.NB,4,4);

for i = 1:robot.NB
T(i,:,:) = [cos(theta(i)),-sin(theta(i)),0,a(i);
    sin(theta(i))*cos(alpha(i)),cos(theta(i))*cos(alpha(i)),-sin(alpha(i)),-sin(alpha(i))*d(i);
    sin(theta(i))*sin(alpha(i)),cos(theta(i))*sin(alpha(i)),cos(alpha(i)),cos(alpha(i))*d(i);
    0,0,0,1];
end
Xt = Ad(robot.NB,T);
robot.Xtree = {model.smds.Xtree{1},model.smds.Xtree{2},model.smds.Xtree{3},...
    model.smds.Xtree{4},model.smds.Xtree{5},model.smds.Xtree{6}};
robot.I = {Ilink1, Ilink2, Ilink3, Ilink4, Ilink5, Ilink6};

torque = ID(robot,[0,pi/2,0,pi/2,0,0], [0,0,0,0,0,0],[0,2,0,0,0,0])
%6维空间向量动力学输出扭矩验证
qdd = FDab(robot, [0,pi/2,0,pi/2,0,0], [0,0,0,0,0,0], torque)

%3维向量动力学输出扭矩验证
% qdd = FDab(robot, [0,pi/2,0,pi/2,0,0], [0,0,0,0,0,0], [0.0181626704240002 -26.5205316694276 7.18116660804760 0.0154915367323999...
%  -0.00296997844399997 0.000223055432400000])

torque = ID(robot,[0,-pi/2,0,-pi/2,0,0], [0,0,0,0,0,0],[0,2,0,0,0,0])
%6维空间向量动力学输出扭矩验证
qdd = FDab(robot, [0,-pi/2,0,-pi/2,0,0], [0,0,0,0,0,0], torque)

%3维向量动力学输出扭矩验证
% qdd = FDab(robot, [0,-pi/2,0,-pi/2,0,0], [0,0,0,0,0,0], [0.0181626704240002 34.0540954465724 -9.77835942995240 0.0154915367324000...
%  -0.00296997844400000 0.000223055432400000])

%输入轨迹计算正逆动力学，其中ID为6维空间向量逆动力学算法，FDab为featherstone的ABA算法
Tau = zeros(6,63);
Acc = zeros(6,63);
for i = 1:63
    Tau(:,i) = ID(robot,[0,q(i),0,0,0,0], [0,dq(i),0,0,0,0],[0,ddq(i),0,0,0,0]);
    Acc(:,i) = FDab(robot, [0,q(i),0,0,0,0], [0,dq(i),0,0,0,0], Tau(:,i));
end

figure;
plot(Tau(2,:));
xlabel("采样点");
ylabel("关节2扭矩(N.m)");

figure;
plot(Acc(2,:),'-o');
hold on;
plot(ddq);
legend('show','正动力学计算的加速度','给定期望加速度');
xlabel("采样点");
ylabel("加速度(rad/s2)");

%齐次变换矩阵转换为空间向量表示的坐标变换矩阵
function Xt = Ad(NB,T)
    Xt = zeros(NB,6,6);
    for i = 1:NB
        R = T(i,1:3, 1:3);
        P = T(i,1:3, 4);
        R = squeeze(R);
        Xt(i,:,:) = [R,zeros(3,3);
        skew(P)*R,R];
    end
end