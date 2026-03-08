T = 0.00005;%采样时间
%%
%涓嬪垪鍙傛暟娉ㄩ噴%锛熺殑閮芥槸闇?瑕佺‘璁ょ殑
%鍗曚綅鍧囦负鏍囧噯鍥介檯鍗曚綅 濡? m銆乲g銆乻

r=10;%骞叉壈鍊肩殑澶у皬锛屼负璐熻浇骞叉壈锛圢锛?

%%
%AUM4-S4鐢垫満鍙傛暟
fai=1;%唯f涓哄姩瀛愮閾惧湪瀹氬瓙涓婄殑鑰﹀悎   %锛?
d=60;%涓篜MLSM 鐨勬瀬璺?,mm
%kf=(3*fai*pi)/(2*d);%鏄浆鐭╁父閲?  96.0N/Arms
kf=96.0;
%ke=fai*pi/d;%鏄弽鐢靛姩鍔垮父鏁?   78.4m/s
ke=78.4;

L=15.33;%鐢垫劅  惟
R=18.62;%鐢甸樆  R

%%
% 榫欓棬骞冲彴妯″瀷

%鍩虹鍙傛暟
m1=18.526;       %妯璐ㄩ噺kg
m2=3.923;      %璐熻浇璐ㄩ噺kg
J=(m1+m2)/2;

l=0.97;    %0.97m
y_move=0;   %璐熻浇绉诲姩浣嶇Щ锛岃嚜宸辨ā鎷熻瀹?   鐩存帴璁惧畾锛岄粯璁よ礋杞藉湪妯涓偣锛屽悜鍙充负姝ｏ紝y鏈夋璐?
y=abs(y_move);
%y=0.2;

I1=m1*l^2/12;   %杞姩鎯噺
gama=0.003;     %鏄矘婊炴懇鎿︾郴鏁? %锛熺矘婊炴懇鎿︾郴鏁?0.003
u=0.005;    %搴撲鸡鎽╂摝绯绘暟u=0.005
g=9.8;


%鎽╂摝鍔涙ā鍨?
f1=u*(m1*g/2+(1/2-y/l)*m2*g);   %X1鎽╂鍔?
f2=u*(m1*g/2+(1/2+y/l)*m2*g);   %X2鎽╂鍔?

%鍔ㄥ姏瀛︽柟绋?
U= m2^2*y^2+2*m1*m2*y^2+I1*(m1+m2);
Q11=(2*m2*y^2+m2*l*y+I1+l^2*(m1+m2)/4)/U;       
Q12=(2*m2*y^2++I1-l^2*(m1+m2)/4)/U;
Q21=(2*m2*y^2++I1-l^2*(m1+m2)/4)/U;
Q22=(2*m2*y^2-m2*l*y+I1+l^2*(m1+m2)/4)/U;

%%
% 涓夌幆鎺у埗鍣?

%pid鍙傛暟

%浣嶇疆鐜?

kp=150;%  ok

%閫熷害鐜?
kpv=100*6*J/(kf);  %ok  100涓轰覆鑱攑id鐨刱i鍊?
kiv=kpv*100; % 璋冩暣 ok

%鐢垫祦鐜?
kpi=1061*L; %椋炰功   ok
kii=1061*R; %椋炰功   ok
%kii=0;

y_safe = min(max(y, 0.05), l-0.05);

I1=m1*l^2/12;
meq1 = (m1+m2)/2+2*(I1+m2*y_safe^2)/l^2;
meq2 = (m1+m2)/2+2*(I1+m2*y_safe^2)/(l-y_safe)^2;
b_nominal=2/(meq1+meq2);
b = max(b_nominal, 0.01);

tao = 0.003;

%改
%kii = 0;

%% Datafile

% Simscape(TM) Multibody(TM) version: 23.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(14).translation = [0.0 0.0 0.0];
smiData.RigidTransform(14).angle = 0.0;
smiData.RigidTransform(14).axis = [0.0 0.0 0.0];
smiData.RigidTransform(14).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 -100 0];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[platform-1:-:]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [0 0 0];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(2).ID = "F[platform-1:-:]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [597.34389296834797 262.50000000000205 -800];  % mm
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = "B[platform-1:-:Y1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [597.34389296834536 262.50000000000239 -743.9499999999997];  % mm
smiData.RigidTransform(4).angle = 0;  % rad
smiData.RigidTransform(4).axis = [0 0 0];
smiData.RigidTransform(4).ID = "F[platform-1:-:Y1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-470.00000001269302 428.50000000000091 -736.34389298103906];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(5).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(5).ID = "B[Y1-1:-:Z1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-515.51865060097612 428.5000000000008 -736.34389298103906];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(6).ID = "F[Y1-1:-:Z1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [597.34389296834797 262.50000000000205 -800];  % mm
smiData.RigidTransform(7).angle = 0;  % rad
smiData.RigidTransform(7).axis = [0 0 0];
smiData.RigidTransform(7).ID = "B[platform-1:-:Y2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [597.3438929683474 262.50000000000142 -856.05000000000075];  % mm
smiData.RigidTransform(8).angle = 0;  % rad
smiData.RigidTransform(8).axis = [0 0 0];
smiData.RigidTransform(8).ID = "F[platform-1:-:Y2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [469.99999998730095 428.50000000000097 736.34389298104304];  % mm
smiData.RigidTransform(9).angle = 2.0943951023931988;  % rad
smiData.RigidTransform(9).axis = [-0.57735026918962573 -0.57735026918962684 0.57735026918962462];
smiData.RigidTransform(9).ID = "B[Y2-1:-:Z2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [505.54999998730892 428.5000000000008 736.34389298104327];  % mm
smiData.RigidTransform(10).angle = 2.0943951023931988;  % rad
smiData.RigidTransform(10).axis = [-0.57735026918962573 -0.57735026918962684 0.57735026918962462];
smiData.RigidTransform(10).ID = "F[Y2-1:-:Z2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [409.8752423800629 471.95000000000107 -577];  % mm
smiData.RigidTransform(11).angle = 2.0943951023931944;  % rad
smiData.RigidTransform(11).axis = [-0.5773502691896254 -0.57735026918962651 -0.5773502691896254];
smiData.RigidTransform(11).ID = "B[Z1-1:-:actuator1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [409.8752423800621 526.50000000000216 -577.00000000000307];  % mm
smiData.RigidTransform(12).angle = 2.0943951023931944;  % rad
smiData.RigidTransform(12).axis = [-0.5773502691896254 -0.57735026918962651 -0.5773502691896254];
smiData.RigidTransform(12).ID = "F[Z1-1:-:actuator1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [-419.84389299373919 471.94999999999806 577];  % mm
smiData.RigidTransform(13).angle = 2.0943951023931988;  % rad
smiData.RigidTransform(13).axis = [0.57735026918962695 -0.57735026918962351 0.57735026918962684];
smiData.RigidTransform(13).ID = "B[Z2-1:-:actuator2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [-419.84389299372918 526.50000000001762 576.99999999999818];  % mm
smiData.RigidTransform(14).angle = 2.0943951023931988;  % rad
smiData.RigidTransform(14).axis = [0.57735026918962695 -0.57735026918962351 0.57735026918962684];
smiData.RigidTransform(14).ID = "F[Z2-1:-:actuator2-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(7).mass = 0.0;
smiData.Solid(7).CoM = [0.0 0.0 0.0];
smiData.Solid(7).MoI = [0.0 0.0 0.0];
smiData.Solid(7).PoI = [0.0 0.0 0.0];
smiData.Solid(7).color = [0.0 0.0 0.0];
smiData.Solid(7).opacity = 0.0;
smiData.Solid(7).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 1.7729725010170971;  % kg
smiData.Solid(1).CoM = [331.12130998947737 413.22762873334017 -637.74479311260063];  % mm
smiData.Solid(1).MoI = [10390.81931262237 11451.539837504497 11840.694402386565];  % kg*mm^2
smiData.Solid(1).PoI = [2706.894765622601 -5.4115450637870959 -0.32959854240882236];  % kg*mm^2
smiData.Solid(1).color = [1 1 1];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Z1*:*Ä¬ÈÏ";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 12.789817477120538;  % kg
smiData.Solid(2).CoM = [-6.4599912645737714 352.12478804929481 682.67121618020985];  % mm
smiData.Solid(2).MoI = [76514.773450817171 1949784.2785941928 1960075.1270618383];  % kg*mm^2
smiData.Solid(2).PoI = [-996.54962402849492 8666.724939204596 2027.415962017682];  % kg*mm^2
smiData.Solid(2).color = [1 1 1];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Y2*:*Ä¬ÈÏ";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 2.1287403257069766;  % kg
smiData.Solid(3).CoM = [-337.6486712945956 372.80371733589772 541.9896887031839];  % mm
smiData.Solid(3).MoI = [19829.092005946055 9107.2099410224 25319.249538732864];  % kg*mm^2
smiData.Solid(3).PoI = [-2921.7974725069789 349.19731941189582 644.36107019049462];  % kg*mm^2
smiData.Solid(3).color = [1 1 1];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "actuator2*:*Ä¬ÈÏ";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 12.798869976065784;  % kg
smiData.Solid(4).CoM = [6.0524544313182984 352.08369872505028 -682.57465013154751];  % mm
smiData.Solid(4).MoI = [76852.409769676844 1953195.4806511919 1963213.7982109089];  % kg*mm^2
smiData.Solid(4).PoI = [1083.1418697025003 9714.2993794917784 -2397.0805139335703];  % kg*mm^2
smiData.Solid(4).color = [1 1 1];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Y1*:*Ä¬ÈÏ";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 483.19326490008126;  % kg
smiData.Solid(5).CoM = [-0.024473936784169075 36.711613274736727 0.64920642253170702];  % mm
smiData.Solid(5).MoI = [105191842.01424366 171191284.21135354 74023802.858655378];  % kg*mm^2
smiData.Solid(5).PoI = [-60455.275277256012 7065.6506539571856 1591.9750770724636];  % kg*mm^2
smiData.Solid(5).color = [1 1 1];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "platform*:*Ä¬ÈÏ";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 2.1291325102187906;  % kg
smiData.Solid(6).CoM = [327.68785479094259 372.78699136512927 -541.99488680735851];  % mm
smiData.Solid(6).MoI = [19832.656647999858 9108.2482001690332 25323.204116651556];  % kg*mm^2
smiData.Solid(6).PoI = [2920.7923932809608 349.66301478569386 -642.84625680845011];  % kg*mm^2
smiData.Solid(6).color = [1 1 1];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "actuator1*:*Ä¬ÈÏ";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 1.7727061174895422;  % kg
smiData.Solid(7).CoM = [-341.09012439090276 413.24524446160882 637.74498075591259];  % mm
smiData.Solid(7).MoI = [10387.153305841097 11451.489772350411 11836.987112762172];  % kg*mm^2
smiData.Solid(7).PoI = [-2706.8557655809659 -5.4119076788741811 0.29555667336708885];  % kg*mm^2
smiData.Solid(7).color = [1 1 1];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = "Z2*:*Ä¬ÈÏ";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the PrismaticJoint structure array by filling in null values.
smiData.PrismaticJoint(6).Pz.Pos = 0.0;
smiData.PrismaticJoint(6).ID = "";

smiData.PrismaticJoint(1).Pz.Pos = 0;  % m
smiData.PrismaticJoint(1).ID = "[platform-1:-:Y1-1]";

smiData.PrismaticJoint(2).Pz.Pos = 0;  % m
smiData.PrismaticJoint(2).ID = "[Y1-1:-:Z1-1]";

smiData.PrismaticJoint(3).Pz.Pos = 0;  % m
smiData.PrismaticJoint(3).ID = "[platform-1:-:Y2-1]";

smiData.PrismaticJoint(4).Pz.Pos = 0;  % m
smiData.PrismaticJoint(4).ID = "[Y2-1:-:Z2-1]";

smiData.PrismaticJoint(5).Pz.Pos = 0;  % m
smiData.PrismaticJoint(5).ID = "[Z1-1:-:actuator1-1]";

smiData.PrismaticJoint(6).Pz.Pos = 0;  % m
smiData.PrismaticJoint(6).ID = "[Z2-1:-:actuator2-1]";