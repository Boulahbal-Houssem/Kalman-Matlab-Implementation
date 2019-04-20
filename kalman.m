function [ Kalmantable, XE ] = kalman( N,T,Z,F,mX0,PX0,Qw,Rv ,H1 ,H2, Hfull)
% This function Implements linear kalman filter
%   @inputs : 
%		N         : numbre of time samples
%		T         : vector of samples (1xN)
%		Z         : postions measurement of robot and amers
%		F 	  : dynamic matrices that describe the système
%		mX0       : expectation of the inital postion		
%		PX0       : expectation of the inital covarience of the postion
%		Qw        : dynamic noise covarience 
%		Rv        : measurement noise covarience
%		H1        : Dynamic when sensor 1 is activated
%		H2        : Dynamic when sensor 2 is activated
%		Hfull     : Dynamic when both sensors are activated
%   @Outputs :		
%		Kalmantable(1,N) : Predicated postion	
%		Kalmantable(2,N) : Predicated covarience of postion	
%		Kalmantable(3,N) : Updated state estimate	
%		Kalmantable(4,N) : Updated covarience estimate	
%		Kalmantable(5,N) : Kalman's gain 	
		
deltaT = 1;
W = chol(Qw)'*randn(6,N);
Kalmantable = cell(5,N);
Source1Vue=zeros(1,N);
Source2Vue=zeros(1,N);
Kalmantable(1,1) ={nan*ones(6,1)};
Kalmantable(2,1) ={nan*ones(6,6)};
Kalmantable(3,1) ={mX0};
Kalmantable(4,1) ={PX0};
Kalmantable(5,1) ={nan};
for i=2:N
	% Prediction
	Xest = cell2mat(Kalmantable(3,i-1))
	Pest = cell2mat(Kalmantable(4,i-1))
	Xpred = F * Xest;
	Ppred = F * Pest * F' + Qw;
	Kalmantable(1,i)= F * Kalmantable(3,i-1); % Xpred iter i
	Kalmantable(2,i)= F * Kalmantable(4,i-1) *F' + Qw ;%Ppred iter i
	% Mise a jour du postion
	% cheking Amers visibility
	if (~isnan(Z(1,i)))
		Source1Vue(i)=1;
	end
	if (~isnan(Z(3,i)))
		Source2Vue(i)=1;
	end
	% Calculating gain
	if (Source1Vue(i)==1)&&(Source2Vue(i)==1)
		H = Hfull;
		R =Rv;
		z= Z(:,i);
	elseif (Source1Vue(i)==1)&&(Source2Vue(i)==0)
		H = H1;
		R =Rv(1:2,1:2);
		z= Z(1:2,i);
	elseif (Source1Vue(i)==0)&&(Source2Vue(i)==1)
	H = H2;
	R =Rv(3:4,3:4)
	z= Z(3:4,i);
	else
	H = zeros([4 6]);
	R =Rv;
	z =zeros([4 1]);
	gain=Ppred * H' * inv(R + H * Ppred * H');
	% update variables
	Kalmantable(1,i)= {Xpred};
	Kalmantable(2,i)= {Ppred};
	Kalmantable(3,i)= {Xpred + gain * ( z - H *Xpred)}
	Kalmantable(4,i)= {Ppred - gain * H * Ppred };
	Kalmantable(5,i) = {gain};
	%XE(i,:) = Xpred + gain * ( z - H *Xpred);
	XE(i,:) = Xpred
	
end
