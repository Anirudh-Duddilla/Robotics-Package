No_Rotatinal_Transformation=str2double(inputdlg('Number of Rotation')); 
syms x y z 
theta=ones(No_Rotatinal_Transformation,1);e=eye(4);r=eye(3);P0=zeros(1,3);T=eye(4);
for i=1:No_Rotatinal_Transformation   
theta(i)=str2double(inputdlg(['Angle of rotation theta in degree ',num2str(i),' ' ]));
W(i)=(inputdlg( [num2str(i), ' Axis of Rotation'] ));
endeffectorX=str2double(inputdlg('Input End Effector Position X '));
endeffectorY=str2double(inputdlg('Input End Effector Position Y '));
endeffectorZ=str2double(inputdlg('Input End Effector Position Z '));

if W(i)==x
    
    w1=1;
    w2=0;
    w3=0;
end
if W(i)==y
    w1=0;        
    w2=1;
    w3=0;
end
if W(i)==z
    w1=0;
    w2=0;
    w3=1;
end

st=sind(theta(i));
ct=cosd(theta(i));
vt=(1-ct);

E1=[w1^2*vt+ct, w1*w2*vt-w3*st, w1*w3*vt+w2*st, endeffectorX; w1*w2*vt+w3*st, w2^2*vt+ct, w2*w3*vt-w1*st, endeffectorY; w1*w3*vt-w2*st, w2*w3*vt+w1*st, w3^2*vt+ct, endeffectorZ; 0, 0, 0, 1];
eval(sprintf('E%d = E1;',i));
e=E1;
eval(sprintf('E%d',i))

P1=e(1:3,4);
eval(sprintf('X%d = P1;',i));
p=P1;
eval(sprintf('X%d',i))

r1=e(1:3,1:3);
eval(sprintf('R%d = r1;',i));
r=r*r1;
eval(sprintf('R%d',i))

T=T*E1;
R=T(1:3,1:3);
P=T(1:3,4);

keyboard();
Q1=rotm2quat(R);
 eval(sprintf('QD%d = Q1;',i));
  q=Q1;
 eval(sprintf('QD%d',i))

  if i==1
   x1=[0 P(1)];
   y1=[0 P(2)];
   z1=[0 P(3)];
   plot3(x1, y1, z1,'O--') 
   else
   x1=[X1(1) T(1,4)];
   y1=[X1(2) T(2,4)];
   z1=[X1(3) T(3,4)];
   plot3(x1, y1, z1,'O--')
   end
   hold on
   grid on
 plotTransforms(P',Q1) 
end

disp(['Translation Matrix from 0 to ',num2str(i)] )
disp(T)