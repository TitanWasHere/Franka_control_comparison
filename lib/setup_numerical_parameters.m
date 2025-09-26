disp('Loading numerical parameters...')

% masses
true_vals.m1 = 4.970684;
true_vals.m2 = 0.646926;
true_vals.m3 = 3.228604;
true_vals.m4 = 3.587895;
true_vals.m5 = 1.225946;
true_vals.m6 = 1.666555;
true_vals.m7 = 7.35522e-01;

% CoMs
true_vals.rc1 = [3.875e-03; 2.081e-03; 0.0];
true_vals.rc2 = [-3.141e-03; -2.872e-02; 3.495e-03];
true_vals.rc3 = [2.7518e-02; 3.9252e-02; -6.6502e-02];
true_vals.rc4 = [-5.317e-02; 1.04419e-01; 2.7454e-02];
true_vals.rc5 = [-1.1953e-02; 4.1065e-02; -3.8437e-02];
true_vals.rc6 = [6.0149e-02; -1.4117e-02; -1.0517e-02];
true_vals.rc7 = [1.0517e-02; -4.252e-03; 6.1597e-02];

% inertia tensors
true_vals.Ic1 = [ 7.0337e-01, -1.3900e-04,  6.7720e-03;
                 -1.3900e-04,  7.0661e-01,  1.9169e-02;
                  6.7720e-03,  1.9169e-02,  9.1170e-03];

true_vals.Ic2 = [ 7.9620e-03, -3.9250e-03,  1.0254e-02;
                 -3.9250e-03,  2.8110e-02,  7.0400e-04;
                  1.0254e-02,  7.0400e-04,  2.5995e-02];

true_vals.Ic3 = [ 3.7242e-02, -4.7610e-03, -1.1396e-02;
                 -4.7610e-03,  3.6155e-02, -1.2805e-02;
                 -1.1396e-02, -1.2805e-02,  1.0830e-02];

true_vals.Ic4 = [ 2.5853e-02,  7.7960e-03, -1.3320e-03;
                  7.7960e-03,  1.9552e-02,  8.6410e-03;
                 -1.3320e-03,  8.6410e-03,  2.8323e-02];

true_vals.Ic5 = [ 3.5549e-02, -2.1170e-03, -4.0370e-03;
                 -2.1170e-03,  2.9474e-02,  2.2900e-04;
                 -4.0370e-03,  2.2900e-04,  8.6270e-03];

true_vals.Ic6 = [ 1.9640e-03,  1.0900e-04, -1.1580e-03;
                  1.0900e-04,  4.3540e-03,  3.4100e-04;
                 -1.1580e-03,  3.4100e-04,  5.4330e-03];

true_vals.Ic7 = [ 1.2516e-02, -4.2800e-04, -1.1960e-03;
                 -4.2800e-04,  1.0027e-02, -7.4100e-04;
                 -1.1960e-03, -7.4100e-04,  4.8150e-03];

% friction
true_vals.fv = [0.0665; 0.1987; 0.0399; 0.2257; 0.1023; -0.0132; 0.0638];
true_vals.fc = [0.2450; 0.1523; 0.1827; 0.3591; 0.2669; 0.1658; 0.2109];
true_vals.fo = [-0.1073; -0.1566; -0.0686; -0.2522; 0.0045; 0.0910; -0.0127];

% gravity
true_vals.g0 = 9.80665;

% true numerical vector
all_rc_true = [true_vals.rc1, true_vals.rc2, true_vals.rc3, true_vals.rc4, true_vals.rc5, true_vals.rc6, true_vals.rc7];
all_I_true = [true_vals.Ic1, true_vals.Ic2, true_vals.Ic3, true_vals.Ic4, true_vals.Ic5, true_vals.Ic6, true_vals.Ic7];
true_params_vec = [
    true_vals.m1; true_vals.m2; true_vals.m3; true_vals.m4; true_vals.m5; true_vals.m6; true_vals.m7;
    all_rc_true(:);   % flatten the 3x7 CoM matrix into a 21x1 vector
    all_I_true(:);    % flatten the 3x21 Inertia matrix into a 63x1 vector
    true_vals.g0;
    true_vals.fv(:);
    true_vals.fc(:);
    true_vals.fo(:)
];

% perturbed values for the controller
perturbed_vals = true_vals;
uncertainty_factor = 1.20;
perturbed_vals.m3 = true_vals.m3 * uncertainty_factor;
perturbed_vals.m5 = true_vals.m5 * uncertainty_factor;
perturbed_vals.m7 = true_vals.m7 * uncertainty_factor;
perturbed_vals.Ic7(3,3) = true_vals.Ic7(3,3) * uncertainty_factor;
perturbed_vals.fv = true_vals.fv * uncertainty_factor;
perturbed_vals.fc = true_vals.fc * uncertainty_factor;
perturbed_vals.fo = true_vals.fo * uncertainty_factor;

% perturbed numerical vector
all_rc_perturbed = [perturbed_vals.rc1, perturbed_vals.rc2, perturbed_vals.rc3, perturbed_vals.rc4, perturbed_vals.rc5, perturbed_vals.rc6, perturbed_vals.rc7];
all_I_perturbed = [perturbed_vals.Ic1, perturbed_vals.Ic2, perturbed_vals.Ic3, perturbed_vals.Ic4, perturbed_vals.Ic5, perturbed_vals.Ic6, perturbed_vals.Ic7];
perturbed_params_vec = [
    perturbed_vals.m1; perturbed_vals.m2; perturbed_vals.m3; perturbed_vals.m4; perturbed_vals.m5; perturbed_vals.m6; perturbed_vals.m7;
    all_rc_perturbed(:);
    all_I_perturbed(:);
    perturbed_vals.g0;
    perturbed_vals.fv(:);
    perturbed_vals.fc(:);
    perturbed_vals.fo(:)
];

disp('Numerical parameter vectors created in the workspace.')
clear uncertainty_factor true_vals perturbed_vals all_rc_true all_I_true all_rc_perturbed all_I_perturbed;