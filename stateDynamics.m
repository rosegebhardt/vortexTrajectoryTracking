function dzdt = stateDynamics(t,z,u)

parameters

% State variables
z1 = z(1);
z2 = z(2);
phi = z(3);

% System configuration at current time
beta = A*cos(omega*t+phi);
delta = sqrt(1-beta^2)*deltaConstant;
x0 = r0*(delta + 1i*beta);
k = r0*(delta + sqrt(1 - beta^2));

% Inverse mapping
zee = z1 + 1i*z2;
x = 0.5*(zee+sign(z1)*(zee^2-4*k^2)^0.5);
x_image = x0 + r0^2/conj(x - x0);

% Potential derivatives
Gamma_0 = Gamma_v*((abs(x-x0))^2+r0^2)/((abs(x-k))^2) - ...
          4*pi*u0*exp(1i*aoa)*imag(k-x0);
dg_dx = 1 - k^2*x^-2;
d2g_dx2 = 2*k^2*x^-3;
dFv_dx = u0*exp(-1i*aoa) - u0*exp(1i*aoa)*(r0^2/(x-x0)^2) + ...
         Gamma_0/(2i*pi*(x - x0)) - Gamma_v/(2i*pi*(x - x_image));
dHv_dz = dFv_dx/dg_dx - 1i*Gamma_v*d2g_dx2/(4*pi*dg_dx^2);

% Convection velocity
w = conj(dHv_dz);
dzdt = [real(w);imag(w);u];

end