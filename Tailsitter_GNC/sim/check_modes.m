% CHECK_MODES  Linearize and inspect modes
% Run after run_sim confirms trim

model   = 'fixedwing_model';   % your Simulink model name
op      = operspec(model);
op_trim = findop(model, op);
sys     = linearize(model, op_trim);
[A,~,~,~] = ssdata(sys);

disp('=== Eigenvalues ===');
ev = eig(A);
disp(ev);

disp('=== Oscillatory modes ===');
for i = 1:length(ev)
    if imag(ev(i)) > 0
        wn   = abs(ev(i));
        zeta = -real(ev(i)) / wn;
        fprintf('  wn=%.3f rad/s   zeta=%.3f\n', wn, zeta);
    end
end

% State indices for reference:
% 1-3: x,y,z   4:psi  5:phi  6:theta
% 7-9: vx,vy,vz   10-12: wx,wy,wz
% 13:T1  14:T2  15:d1  16:d2