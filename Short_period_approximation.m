% Define symbolic variables
syms V0 g theta0 K_q real
syms X_V X_alpha X_q real
syms Z_V Z_alpha Z_q real
syms M_V M_alpha M_q real
syms X_de X_dt Z_de Z_dt M_de M_dt real

% State matrix (A)
A = [       X_V,              X_alpha,       -g*cos(theta0),         X_q;
       Z_V/V0,            Z_alpha/V0,  -(g*sin(theta0))/V0,    1 + Z_q/V0;
            0,                     0,                    0,             1;
          M_V,               M_alpha,                    0,           M_q ];

% Input matrix (B)
B = [    X_de,    X_dt;
       Z_de/V0, Z_dt/V0;
            0,       0;
         M_de,    M_dt ];

% Similarity transformation
Tinv = [1 0 0 0;0 0 1 0; 0 1 0 0;0 0 0 1];
Anew = Tinv*(A)*inv(Tinv);
Bnew = Tinv*(B);

% Obtain short-period approximation
A_sp = Anew(3:4,3:4);
B_sp = Bnew(3:4,1);