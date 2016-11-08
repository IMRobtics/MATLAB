% coulomb - Program to compute Coulomb force between charges
clear all;  help coulomb;    % Clear memory; print header

%@ Initialize variables (e.g., positions of charges, physical constants)
NCharges = input('Enter the number of charges: ');
for iCharge=1:NCharges
  fprintf('----- \n  For charge #%g \n',iCharge);
  r_in = input('Enter position (in m) as [x y]: ');
  x(iCharge) = r_in(1);   % x-component of position
  y(iCharge) = r_in(2);   % y-component of position
  q(iCharge) = input('Enter charge (in C): ');
end
Epsilon0 = 8.85e-12;    % Permittivity of free space (C^2/(N m^2))
Constant = 1/(4*pi*Epsilon0);  % Useful constant

%@ Loop over charges to compute the force on each charge
fprintf('\n\n Forces are: \n\n');
for iCharge = 1:NCharges

  Fx = 0.0;  % Initialize components of total force to zero
  Fy = 0.0;
  
  %@ Loop over other charges to compute force on this charge
  for jCharge = 1:NCharges
	if( iCharge ~= jCharge )  % If iCharge NOT equal to jCharge
		
	  %@ Compute the components of vector distance between two charges
	  xij = x(iCharge) - x(jCharge);
	  yij = y(iCharge) - y(jCharge);
	  Rij = sqrt(xij^2 + yij^2);
	
	  %@ Compute the x and y components of the force between
	  %@ these two charges using Coulomb's law
	  Fx = Fx + Constant*q(iCharge)*q(jCharge)*xij/Rij^3;
	  Fy = Fy + Constant*q(iCharge)*q(jCharge)*yij/Rij^3;
	
	end
  end
  
  %@ Print out the total force on this charge due to the others
  fprintf('Force on charge #%g is: \n',iCharge);
  fprintf(' x-component: %g N \n',Fx);
  fprintf(' y-component: %g N \n',Fy);
end