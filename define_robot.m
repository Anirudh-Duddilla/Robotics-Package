clc;
clear variables;
close all;

No_of_Joints = input('enter number of joints');
disp(' ');
i=1;

while (i<=No_of_Joints)
    if i==1
        fprintf(' enter P for prismatic joint,\nenter R for revolute \n');
    end
    fprintf('Is Link %d',i)
    por(i)=input(' prismatic or revolute: ','s');
    if xor(strcmp(por(i),'R'),strcmp(por(i),'P')) ==0
        i=i-1;
        fprintf('Invalid input');
    end
    i=i+1;
end
D = input ('enter Y if you have the DH parameter or N if you do not');
disp ( ' enter values for link i to n ')
if D == Y
    for i=1:No_of_joints
        if por(i)== 'R'
            promt= {' for revolute joint'}
            a(i,1) = input('link length  ')
            d(i,1) = input ( ' Distance')
            Alpha(i,1) = input ('angle in radian ')
            
        else 
            Promt = { 'for prismatic joint'}
            a(i,1) = input(' Link Length - ');
        Alpha(i,1) = input(' Angle alpha in radians - ');
        theta(i,1) = input(' Angle theta in radians - ');
        disp(' ')
    end
    if por(i)=='R'
        DH(i,:) = [0 d(i,1) a(i,1) Alpha(i,1)];
    else
        DH(i,:) = [theta(i,1) 0 a(i,1) Alpha(i,1)];
    end
end
disp(' ')
disp(' Below are the inputed DH Parameters ')

else
for i = 1:No_of_Joints
    if por(i)=='R'
        disp(' For Revolute Joint ')
        linkl(i,1)     = input(' Length of link - ');
        Zdist(i,1)     = input(' Distance between joints along the previous joint axis - ');%Include this in the manual
        Zangle(i,1) = pi/180*input(' Angle between the z axis of consecutive joints in degrees - ');
        disp(' ')
    else
        disp(' For Prismatic Joint ')
        linkl(i,1)  = input(' Length of link - ');
        Zangle(i,1) = pi/180* input(' Angle between the z axis of the consecutive links in degrees - ');
        Xangle(i,1) = pi/180* input(' Angle between the x axis of the consecutive links in degrees - ');
        disp(' ')
    end
    if por(i)=='r'
        DH(i,:) = [0 Zdist(i,1) linkl(i,1) Zangle(i,1)];
    else
        DH(i,:) = [Xangle(i,1) 0 linkl(i,1) Zangle(i,1)];
    end
end
end
    

for k = 1:No_of_Joints
 if por(k)=='R'
    L{k} = Link('d',DH(k,2), 'a', DH(k,3), 'alpha', DH(k,4));
 else
    disp(' As entered link is prismatic please enter its joint limits ')
    disp(' ')
    limitlow(k)   = input(' Lower limit for the joint - ');
    limitupper(k) = input(' Upper limit for the joint - ');
    L{k} = Link('theta',DH(k,1), 'a', DH(k,3), 'alpha', DH(k,4));
    L{k}.qlim = [limitlow(k), limitupper(k)];
    disp(' ')
 end
end
for b = 1:No_of_Joints
    X(b) = L{b};
end
n = 1:No_of_Joints;
m =[X(n)];

R = SerialLink(m);
     
            
        
        
    