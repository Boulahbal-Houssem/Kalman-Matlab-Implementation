function [ output_args ] = kalman( N,T,Z,F,Hfull,mX0,PX0,Qw,Rv,X )
% UNTITLED2 Summary of this function goes here
% Detailed explanation goes here
N=50;
W = chol(Qw)'*randn(6,N);
Kalmantable = cell(5,N);

H1=[-1 0 1 0 0 0; 0 -1 0 1 0 0];
H2=[-1 0 0 0 1 0 ; 0 -1 0 0 0 1];
H=[H1;H2]; Hfull=H;
F = [cos(w*deltaT) -sin(w*deltaT) ; sin(w*deltaT) cos(w*deltaT)];
Source1Vue=zeros(1,N);
Source2Vue=zeros(1,N);
Kalmantable(1,1) ={nan*ones(6,1)};
Kalmantable(2,1) ={nan*ones(6,6)};
Kalmantable(3,1) ={mX0};
Kalmantable(4,1) ={PX0};
Kalmantable(5,1) ={nan}
for i=2:N
   % Prediction    
   Kalmantable(1,i)= F * Kalmantable(3,i-1); % Xpred iter i
   Kalmantable(2,i)= F * Kalmantable(4,i-1) *F' + Qw ;%Ppred iter i
   % Mise a jour du postion 
   % cheking Amers visibility
   if (~isnan(Z(1,i)))
     Source1Vue(k)=1;
  end
   if (~isnan(Z(3,i)))
     Source2Vue(k)=1;
   end 
   % Calculating gain
   if (Source1Vue(k)==1)&&(Source2Vue(k)==1)
    gain = Kalmantable(2,i) * Hfull' * inv(Rv + Hfull * Kalmantable(2,i) * Hfull')
    Kalmantable(3,i)= Kalmantable(1,i) + gain * ( Z(:,i) - Hfull * X(i) );
    Kalmantable(4,i)= Kalmantable(2,i) - gain * ( Z(:,i) - Hfull * X(i) );

  elseif (Source1Vue(k)==1)&&(Source2Vue(k)==0)
      gain = Kalmantable(2,i) * H1' * inv(Rv + H1 * Kalmantable(2,i) * H1')
  elseif (Source1Vue(k)==0)&&(Source2Vue(k)==1)
      gain = Kalmantable(2,i) * H2' * inv(Rv + H2 * Kalmantable(2,i) * H2')
   else
      gain = nan;
      
   end
   Kalmantable(5,i)=gain;
    % update variables
       
       
  end
   
   
   
end
    
    
    
end

