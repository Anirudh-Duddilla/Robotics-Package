function [T_OH,T_all] = forwardKinematics(DH)
dim=size(DH);
T_OH=eye(4,4);
for i=1:dim(1,1)
    A=[cos(DH(i,4)) -sin(DH(i,4))*cos(DH(i,2)) sin(DH(i,4))*sin(DH(i,2)) DH(i,1)*cos(DH(i,4));...
       sin(DH(i,4)) cos(DH(i,4))*cos(DH(i,2)) -cos(DH(i,4))*sin(DH(i,2)) DH(i,1)*sin(DH(i,4));...
       0 sin(DH(i,2)) cos(DH(i,2)) DH(i,3) ; 0 0 0 1];
    T_OH=T_OH*A;
    T_all(:,:,i)=T_OH;
end
end

