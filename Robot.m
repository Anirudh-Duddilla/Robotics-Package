
global R

global n_links

a1=1; a2=1; a3=1;

l1=0.5; l2=0.5; l3=0.5;

m_l1=50; m_l2=50; m_l3=50;

I_l1=10; I_l2=10; I_l3=10;

k_r1=100; k_r2=100; k_r3=100;

m_m1 = 5; m_m2 = 5; m_m3=5;

I_m1=0.01; I_m2=0.01; I_m3=0.01;

%dh parameters
alpha = [0, 0, 0];
a = [a1, a2, a3]; 
d = [0, 0, 0];
theta = zeros(1,3); 
dh = [theta' d' a' alpha'];


L{1} = Link('d', dh(1,2), 'a', dh(1,3), 'alpha', dh(1,4), 'm',m_l1,...
           'r',[l1,0,0]', 'I',I_l1*eye(3),'G', k_r1, 'Jm',I_m1 );
L{2} = Link('d', dh(2,2), 'a', dh(2,3), 'alpha', dh(2,4), 'm',m_l2,...
           'r',[l2,0,0]', 'I',I_l2*eye(3),'G', k_r2, 'Jm',I_m2 ); 
L{3} = Link('d', dh(3,2), 'a', dh(3,3), 'alpha', dh(3,4), 'm',m_l3,...
           'r',[l3,0,0]', 'I',I_l3*eye(3),'G', k_r3, 'Jm',I_m3 );

R = SerialLink([L{1} L{2} L{3}],'name','3LinkPlanar');
R.gravity=[0;0;-9.8];
n_links=R.n;
