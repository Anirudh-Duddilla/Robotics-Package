function [B_m, C_m, G_m]= DynamicsEquation(R)
dims=size(R.links);
n_joints=dims(1,2);

% declare all the symbols for the given robot
syms ml [1,n_joints] real
syms Il [1,n_joints] real
syms a  [1,n_joints] real
syms alphaa [1,n_joints] real
syms d [1,n_joints] real
syms theta [1,n_joints] real
syms l [1,n_joints] real
syms mm [1,n_joints] real
syms Im [1,n_joints] real
syms q [1,n_joints] real
syms kr [1,n_joints] real
syms g real
joint_types=R.config(); 


%defining q vector
for i=1:n_joints
    if joint_types=='R'
        q(1,i)=theta(1,i);
    else
        q(1,i)=d(1,i);
    end
end

%defining gravity vector
g=sym(g*R.gravity);


%create a symbolic DH table for Jacobian Calculation
sym_DH=[a',alphaa',d',theta'];

for i=1:n_joints
    if R.alpha(1,i)==0
        sym_DH(i,2)=sym(zeros(1,1));    
    end
    if R.a(1,i)==0
        sym_DH(i,1)=sym(zeros(1,1));
    end
    if R.d(1,i)==0
        sym_DH(i,3)=sym(zeros(1,1));
    end
    
    if isempty(R.theta)
        continue
    else
        if R.theta(1,i)==0
            sym_DH(i,4)=sym(zeros(1,1));
        end
    end
end

[T_OH,T_all]=forwardKinematics(sym_DH); 

%to find P and Z vectors
P0=sym(zeros(3,1));
Z0=[sym(zeros(2,1));sym(ones(1,1))];
for i=1:n_joints
    if i==1
        Pvec(:,i)=P0;
        Zvec(:,i)=Z0;
    else
        Pvec(:,i)=T_all(1:3,4,i-1);
        Zvec(:,i)=T_all(1:3,3,i-1);
    end
end

PL=sym(zeros(3,n_joints));
for i=1:n_joints
    tempDH=sym_DH(1:i,:);
    tempDH(i,1)=l(1,i);
    [tempT_OH,tempT_all]=forwardKinematics(tempDH);
    PL(:,i)=sym(tempT_OH(1:3,4));
end

PM=sym(zeros(3,n_joints));
ZM=sym(zeros(3,n_joints));
for i=1:n_joints
    if i==1
        PM(:,i)=sym([0;0;0]);
        ZM(:,i)=sym([0;0;1]);
    else
        tempDH=sym_DH(1:i-1,:);
        [tempT_OH,tempT_all]=forwardKinematics(tempDH);
        PM(:,i)=sym(tempT_OH(1:3,4));
        ZM(:,i)=sym(tempT_OH(1:3,3));
    end
end


for i=1:n_joints
    Jpl(:,:,i)=sym(zeros(3,n_joints));
    Jol(:,:,i)=sym(zeros(3,n_joints));
    Jpm(:,:,i)=sym(zeros(3,n_joints));
    Jom(:,:,i)=sym(zeros(3,n_joints));
end
% finding Jpl,Jol, Jpm
for i=1:n_joints
    for j=1:i
        if joint_types(1,i)=='R'
            Jpl(:,j,i)=cross(Zvec(:,j),(PL(:,i)-Pvec(:,j)));
            Jol(:,j,i)=Zvec(:,j);
            Jpm(:,j,i)=cross(Zvec(:,j),(PM(:,i)-Pvec(:,j)));
        end
        
        if joint_types(1,i)=='P'
            Jpl(:,j,i)=Zvec(:,j);
            Jpm(:,j,i)=Zvec(:,j);
        end
    end
end
    
% separate for loop for Jom finding
for i=1:n_joints
    for j=1:i
        if j==i
            Jom(:,j,i)=kr(1,i)*ZM(:,i);
        else
            Jom(:,j,i)=Jol(:,j,i);
        end
    end
end

%to find B
B=sym(zeros(n_joints,n_joints));
for i=1:n_joints
    B=B+(ml(1,i)*Jpl(:,:,i)'*Jpl(:,:,i)+Jol(:,:,i)'*Il(1,i)*Jol(:,:,i)+...
        mm(1,i)*Jpm(:,:,i)'*Jpm(:,:,i)+ Jom(:,:,i)'*Im(1,i)*Jom(:,:,i));
end
% to find C
for i=1:n_joints
    for j=1:n_joints
        for k=1:n_joints
            c(i,j,k)=0.5*(diff(B(i,j),q(1,k))+ diff(B(i,k),q(1,j))-...
                diff(B(j,k),q(1,i)));
        end
    end
end


% finding G term
G=sym(zeros(n_joints,1));
for i=1:n_joints
    G=G+(ml(1,i)*g'*Jpl(:,:,i)+ mm(1,i)*g'*Jpm(:,:,i));
end

%generating equations of motion
syms tau [n_joints,1]
for i=1:n_joints
    if q(1,i)==theta(1,i)
        qdot(i,1)=sym(strcat(char(theta(1,i)),'_dot'));
        qddot(i,1)=sym(strcat(char(theta(1,i)),'_double_dot'));
    else
        qdot(i,1)=sym(strcat(char(d(1,i)),'_dot'));
        qddot(i,1)=sym(strcat(char(d(1,i)),'_double_dot'));
    end
end

C=sym(zeros(n_joints,n_joints));
for i=1:n_joints
    for j=1:n_joints
        tempSum=0;
        for k=1:n_joints 
            tempSum=tempSum+c(i,j,k)*qdot(k,1);
        end
        C(i,j)=tempSum;
    end
end
B_m=B;
C_m=C;
G_m=G;
end

