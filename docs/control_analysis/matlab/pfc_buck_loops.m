%% PFC + Buck small-signal models, discretization, and sisotool entry points
%
%   Run section by section (Ctrl+Enter). Section 5 opens sisotool.
%
%   Plant derivations and the reasoning behind every number are in
%     ../README.md          (small-signal, gain limits, discrete PI = section 8)
%     ../../theory/README.md (why the topology is built this way)
%
%   Tested on MATLAB R2021b (Control System Toolbox required).

clear; clc;
s = tf('s');

%% ---------------- Parameters ----------------
L     = 1.566e-3;    % PFC inductor (measured 2026-10-07)
Cbus  = 10e-6;       % bus capacitor
Lb    = 75e-6;       % buck inductor
RLb   = 30e-3;       % buck inductor DCR   -- ASSUMED, replace with measurement
ESR   = 20e-3;       % Cbus ESR            -- ASSUMED, replace with measurement
Ts    = 50e-6;       % 20 kHz control rate
Td    = 1.5*Ts;      % 1 cycle compute + 0.5 cycle ZOH

Vo    = 100;         % output (voltage source in the bench setup)
Pout  = 600;         % operating power

% Current-loop gains (phaselock/main.c)
K     = 7.0;         % CURRENT_LOOP_K, Kp = K/Vbus
fz_i  = 360;         % PI zero [Hz], 0 = pure P.  Professor's value: 360
wz_i  = 2*pi*fz_i;

% Which loop sections 5 and 6 work on.
DESIGN = 'current';  % 'current' = PFC current loop (the professor's +P +I)
                     % 'buck'    = Buck -> Vbus loop

% Operating points over the line cycle. Vref = |vac|+40, floor 140.
% Worst case for the current loop is |vac| ~ 100 -> Vbus = 140 (see ../README.md s2).
Vbus_list = [140 170 190];

fprintf('=== setup ===\n');
fprintf('Ts = %.0f us, Td = %.0f us, L = %.3f mH, K = %.1f, fz = %g Hz\n\n', ...
        Ts*1e6, Td*1e6, L*1e3, K, fz_i);

%% ---------------- 1. PFC current loop (continuous) ----------------
% Plant   G_id = iL/d = Vbus/(sL)
% Control C_i  = Kp(1 + wz/s),  Kp = K/Vbus   -> Vbus cancels out of the loop
%
% So the loop gain is Vbus-INDEPENDENT. That is the whole point of dividing
% Kp by Vbus (../../theory/README.md s3).

if fz_i > 0
    Ci_shape = (1 + wz_i/s);        % PI
else
    Ci_shape = tf(1);               % pure P
end

Ti = (K/(s*L)) * Ci_shape;          % Vbus already cancelled
Ti.InputDelay = Td;                 % exact delay (margin() handles it)

[Gm,Pm,Wcg,Wcp] = margin(Ti);
fprintf('=== 1. PFC current loop (continuous) ===\n');
fprintf('fc = %7.1f Hz   PM = %5.1f deg   GM = %5.1f dB @ %.0f Hz\n', ...
        Wcp/2/pi, Pm, 20*log10(Gm), Wcg/2/pi);

% Compare against pure P so the cost of the zero is explicit
Ti_P = (K/(s*L));  Ti_P.InputDelay = Td;
[~,Pm_P,~,Wcp_P] = margin(Ti_P);
fprintf('pure P :  fc = %7.1f Hz   PM = %5.1f deg\n', Wcp_P/2/pi, Pm_P);
fprintf('PI cost:  %+.1f Hz crossover, %+.1f deg phase margin\n\n', ...
        (Wcp-Wcp_P)/2/pi, Pm-Pm_P);

% Tracking error at the line frequency -- this is what Ki buys you
f_line = 60;  w_line = 2*pi*f_line;
Tl   = freqresp(Ti,  w_line);     Tl   = Tl(1);
Tl_P = freqresp(Ti_P, w_line);    Tl_P = Tl_P(1);
fprintf('60 Hz loop gain   : P = %5.1f   PI = %5.1f\n', abs(Tl_P), abs(Tl));
fprintf('60 Hz track error : P = %5.2f%%  PI = %5.2f%%\n', ...
        100*abs(1/(1+Tl_P)), 100*abs(1/(1+Tl)));
fprintf('60 Hz phase error : P = %+5.2f deg  PI = %+5.2f deg\n\n', ...
        rad2deg(angle(Tl_P/(1+Tl_P))), rad2deg(angle(Tl/(1+Tl))));

%% ---------------- 2. PFC current loop (discrete, exact delay) ----------------
% Better than Pade: c2d('zoh') already contains the half-sample ZOH, and the
% one-cycle compute delay is exactly z^-1. No approximation anywhere.

Vb_nom = 170;
Gid_c  = Vb_nom/(s*L);
Gid_d  = c2d(Gid_c, Ts, 'zoh');          % plant + ZOH
Gid_d  = Gid_d * tf(1,[1 0],Ts);         % * z^-1  (compute delay)

Kp_i   = K/Vb_nom;
if fz_i > 0
    Ci_d = Kp_i * c2d(Ci_shape, Ts, 'tustin');
else
    Ci_d = Kp_i;
end

Ti_d = Ci_d * Gid_d;
[Gmd,Pmd,~,Wcpd] = margin(Ti_d);
fprintf('=== 2. PFC current loop (discrete, Vbus = %g) ===\n', Vb_nom);
fprintf('fc = %7.1f Hz   PM = %5.1f deg   GM = %5.1f dB\n\n', ...
        Wcpd/2/pi, Pmd, 20*log10(Gmd));

%% ---------------- 3. Buck -> Vbus loop ----------------
% ASSUMPTION: the PFC current loop is fast, so i_in into the bus node is an
% ideal current source. Valid below the current-loop crossover; above it, see
% ../README.md s2 for the full bus admittance.
%
%   Cbus*vbus' = i_in - d*iLb
%   Lb*iLb'    = d*vbus - Vo - RLb*iLb        (Vo stiff: 100 V source)
%
%   Gvd = -[D*Vbus + ILb*RLb + s*Lb*ILb] / [s^2*Lb*Cbus + s*RLb*Cbus + D^2]

fprintf('=== 3. Buck -> Vbus plant ===\n');
for Vbus = Vbus_list
    D   = Vo/Vbus;
    ILb = Pout/Vo;

    num = -[Lb*ILb, D*Vbus + ILb*RLb];
    den = [Lb*Cbus, RLb*Cbus, D^2];
    Gvd = tf(num, den);

    wn   = D/sqrt(Lb*Cbus);
    zeta = (RLb/Lb)/(2*wn);
    fprintf(['Vbus=%3g  D=%.3f  DC gain=%7.1f V/duty  ' ...
             'fn=%6.0f Hz  zeta=%.4f  Q=%5.1f (%+.1f dB peak)\n'], ...
            Vbus, D, dcgain(Gvd), wn/2/pi, zeta, 1/(2*zeta), ...
            20*log10(1/(2*zeta)));
end
fprintf('\nNOTE: the sign is negative, so the controller must be MINUS\n');
fprintf('      (or written on 1-d). See CLAUDE.md.\n\n');

% Keep the nominal plant around for section 5
Vbus = Vb_nom;  D = Vo/Vbus;  ILb = Pout/Vo;
Gvd_buck = tf(-[Lb*ILb, D*Vbus + ILb*RLb], [Lb*Cbus, RLb*Cbus, D^2]);
Gvd_buck.InputDelay = Td;

% Highest crossover a PLAIN INTEGRATOR can reach before the resonance peak
% pokes back up through 0 dB.
%   |C*G| at resonance = (Ki/wn) * Q * dcgain   must stay < 1
%   crossover          = Ki * dcgain / 2pi
wn    = D/sqrt(Lb*Cbus);
Qres  = wn*Lb/RLb;
g0    = abs(dcgain(Gvd_buck));
Ki_max = wn/(Qres*g0);
fc_max = Ki_max*g0/(2*pi);

fprintf('resonance: fn = %.0f Hz, Q = %.1f (%+.1f dB peak)\n', ...
        wn/2/pi, Qres, 20*log10(Qres));
fprintf('MAX crossover for a plain integrator = %.0f Hz\n', fc_max);
fprintf('-> the professor marked ~1 kHz. With this damping that is %.0fx too high:\n', ...
        1000/fc_max);
fprintf('   a loop crossing at 1 kHz crosses 0 dB AGAIN at the resonance,\n');
fprintf('   where the phase has already gone past -180 deg.\n');
fprintf('   Fixes: (a) measure the real ESR/DCR, (b) notch at %.0f Hz, (c) lower fc.\n\n', ...
        wn/2/pi);

%% ---------------- 4. c2d -> coefficients for main.c ----------------
% Discrete PI as implemented in firmware:
%     u[n] = u[n-1] + b0*e[n] + b1*e[n-1]
% Back-calculation anti-windup is applied on top (see ../README.md s8).

fprintf('=== 4. discrete current-loop PI, per operating point ===\n');
fprintf('%-8s %-12s %-12s  (u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + KiTs*e[n])\n', ...
        'Vbus', 'Kp', 'Ki*Ts');
for Vbus = Vbus_list
    Kp    = K/Vbus;
    KiTs  = wz_i*K*Ts/Vbus;      % Ki = wz*Kp  ->  Ki*Ts = wz*K*Ts/Vbus
    fprintf('%-8g %-12.6f %-12.6f\n', Vbus, Kp, KiTs);
end
fprintf('\nIn code, compute both from Vbus every cycle, do not hard-code:\n');
fprintf('  Ikp    = CURRENT_LOOP_K / vbus;\n');
fprintf('  Iki_Ts = CURRENT_LOOP_K * (2*PI*%g) * TS / vbus;   // = %.4f/vbus\n\n', ...
        fz_i, wz_i*K*Ts);

%% ---------------- 5. sisotool: root locus + Bode, drag your own poles/zeros ----------------
% Opens Control System Designer. Root locus, open-loop Bode and step response
% are all live-linked -- add a pole or zero and every plot updates as you drag.
%
% In the GUI:
%   right-click any plot -> Add Pole or Zero -> Real / Complex / Integrator
%   right-click          -> Edit Compensator   (type exact values, add a Notch)
%   drag the pink squares (poles) and circles (zeros)
%   the status bar shows PM / GM live
%   Export -> save the compensator to the workspace as C
%
% Delay: sisotool needs a rational plant, so Pade it here. Sections 1-3 used the
% exact delay. Re-check the final design in section 6, which is exact.
%
% Set DESIGN at the top of this file to pick which loop opens.

switch DESIGN

case 'current'
    % ---- PFC current loop: the professor's +P +I --------------------------
    % Plant is a pure integrator Vbus/(sL). No resonance, ONE crossover, so
    % the phase margin sisotool reports is trustworthy here (unlike 'buck').
    %
    %   C(s) = Kp*(s + wz)/s
    %     gain marker  = Kp = K/Vbus        <- I_LOOP_K  in psim_current_pi.c
    %     zero marker  = wz = 2*pi*fz       <- I_LOOP_FZ in psim_current_pi.c
    %
    % Slope is -40 dB/dec below fz, -20 dB/dec above it.
    % Target with the professor's values (K=7, fz=360): fc ~ 783 Hz, PM ~ 44 deg.
    %
    % Drag the zero LEFT  -> better PM, less 60 Hz gain (worse tracking).
    % Drag the zero RIGHT -> more 60 Hz gain, worse PM. 360 Hz is the balance.
    %
    % The /Vbus is invisible here: it cancels against the plant's Vbus, which
    % is the whole point -- this plot does not move as Vbus swings 140..190.
    P_design = pade(Gid_c, 2);
    C_start  = Kp_i * Ci_shape;

case 'buck'
    % ---- Buck -> Vbus -----------------------------------------------------
    % NEGATIVE dc gain, so feed sisotool the loop written on (1-d): a sign flip
    % that keeps the controller positive. Negate again for the firmware.
    % (Same trick as CLAUDE.md "等價寫法是把迴路寫在 1-d 上".)
    %
    % Watch the 3.4 kHz resonance peak, NOT the low-frequency crossover. With
    % multiple 0 dB crossings the reported PM is misleading -- read GM and the
    % "Stable loop" verdict instead.
    P_design = -pade(Gvd_buck, 2);
    C_start  = Ki_max / s;      % conservative: peak sits right at 0 dB

end

sisotool(P_design, C_start)

%% ---------------- 6. Check an exported design, exactly ----------------
% Run AFTER exporting the compensator from sisotool to the workspace as C.
% Redoes the margins with the exact delay (no Pade) and discretizes for the DSP.

if exist('C','var')
    switch DESIGN
    case 'current', P_exact = Gid_c; P_exact.InputDelay = Td;
    case 'buck',    P_exact = -Gvd_buck;
    end

    [Gm2,Pm2,~,Wcp2] = margin(C*P_exact);
    fprintf('=== 6. exported design (%s loop), exact delay ===\n', DESIGN);
    fprintf('fc = %.1f Hz   PM = %.1f deg   GM = %.1f dB\n\n', ...
            Wcp2/2/pi, Pm2, 20*log10(Gm2));

    if strcmp(DESIGN,'current')
        % C = Kp*(s + wz)/s  ->  read the two numbers straight back out
        [zc,~,kc] = zpkdata(C,'v');
        Kp_new = kc;
        wz_new = -zc(1);
        fprintf('paste into psim_current_pi.c / main.c:\n');
        fprintf('  #define I_LOOP_K    %.4f      // was 7.0   (= Kp * Vbus)\n', ...
                Kp_new*Vb_nom);
        fprintf('  #define I_LOOP_FZ   %.1f       // was 360.0 (= wz/2pi)\n\n', ...
                wz_new/2/pi);
        fprintf('  per-cycle, from the measured bus:\n');
        fprintf('    Ikp    = I_LOOP_K / vbus;\n');
        fprintf('    Iki_Ts = I_LOOP_K * (2*PI*I_LOOP_FZ) * TS / vbus;\n');
        fprintf('           = %.4f / vbus\n\n', Kp_new*Vb_nom*wz_new*Ts);
    end

    Cd = c2d(C, Ts, 'tustin');
    [bz,az] = tfdata(Cd,'v');
    fprintf('generic discrete form, Ts = %g us:\n', Ts*1e6);
    fprintf('  u[n] = ');
    for k = 2:numel(az), fprintf('%+.8g*u[n-%d] ', -az(k), k-1); end
    for k = 1:numel(bz), fprintf('%+.8g*e[n-%d] ', bz(k), k-1); end
    if strcmp(DESIGN,'buck')
        fprintf('\n  (remember the MINUS sign for Buck->Vbus)');
    end
    fprintf('\n\n  Add back-calculation anti-windup on top (../README.md s8).\n');
else
    disp('Section 6: export the compensator from sisotool as C, then re-run this cell.');
end
