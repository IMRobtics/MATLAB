chi_k=atan2(y_0-y_k,x_0-x_k);

if mod(psi_k-chi_k,2*pi)<=pi
psi_k>=chi_k;

else
psi_k<chi_k;

if psi_k>=chi_k
fxkr=fxkrc;
fykr=fykrc;
else
fxkr=fxkrcc;
fykr=fykrcc;
end

%Eq 17.
fkr=fxkri+fykrj;

%Eq 18.
fxkrc  =  B/A(y_k-y_0);
fykrc  = -A/B(x_k-x_0);

%Eq 19.
fxkrcc = -B/A(y_k-y_0);
fykrcc =  A/B(x_k-x_0);

fxkrn(i)=fxkr/abs(fkr);
fykrn(i)=fykr/abs(fkr);

%Eq 20.
fkrn=fxkrni+fykrnj;

r_k=(A^2*(x_k-x_0)^2+B^2*(y_k-y_0)-1)^0.5;

%Eq 21.
    if r_k<=ra
        f_xk_OA=f_xk_des+g*abs(f_k_des)*fxkrn(1/rk-1/ra);
        f_yk_OA=f_yk_des+g*abs(f_k_des)*fykrn(1/rk-1/ra);
    else
        f_xk_OA=f_xk_des;
        f_yk_OA=f_yk_des;
    end

%Eq 22.
    if r_k<=ra
        f_xk_BS=f_xk_vs+((g*abs(f_k_vs)*fxkrn)/(r_k)^2)*(1/r_k-1/r_a);
        f_yk_BS=f_yk_vs+((g*abs(f_k_vs)*fykrn)/(r_k)^2)*(1/r_k-1/r_a);
    else
%Eq 23.
        f_xk_BS=f_xk_vs*exp(-tau*r_k)+f_xk_vs*(1-exp(-tau*r_k));
        f_yk_BS=f_yk_vs*exp(-tau*r_k)+f_yk_vs*(1-exp(-tau*r_k));
    end
   
    if r_m<=r_a
        f_xv_BS=f_xv_des+k_p(x_m-x_v)*(1-exp(-tau*rm));
        f_yv_BS=f_yv_des+k_p(y_m-y_v)*(1-exp(-tau*rm));
    else
        f_xv_BS=f_xv_des;
        f_yv_BS=f_yv_des;
    end

    for i=1:number_of_robots
        x_m_mean(i)=x_k(i)/number_of_robots;
        y_m_mean(i)=y_k(i)/number_of_robots;
    end
    
    for i=1:number_of_robots
        x_m(i)=sum(x_m_mean(i,1:number_of_robots));
        y_m(i)=sum(y_m_mean(i,1:number_of_robots));
    end
    
    
    