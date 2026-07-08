%% Ball_On_Plate_Research_Grade_Simulation.m
% Research-grade MATLAB simulation: ball-on-plate robot with four stepper-driven linkages
% Author: ChatGPT-generated upgrade
%
% Main upgrades compared with the visual-only version:
% 1) Nonlinear rolling-ball dynamics with rotational inertia, Coulomb/viscous friction,
%    plate-rate coupling, bounded contact region, disturbance force, and sensor noise.
% 2) Cascaded control: path generator -> trajectory tracking -> LQR-like PD feedforward
%    outer loop -> tilt commands -> four-leg inverse kinematics -> realistic stepper actuator.
% 3) Stepper model includes 2nd-order motor lag, rate/acceleration limits, microstep quantization,
%    holding torque estimate, backlash/deadband, and tracking error.
% 4) Advanced reference path: smooth Lissajous/flower path with time-parametrized velocity and
%    acceleration feedforward.
% 5) Improved 3D dark/neon environment, transparent plate, four-bar-style linkages, actual trail,
%    reference path, live telemetry, diagnostics, and optional MP4 export.
%
% Run directly. Uses core MATLAB graphics only. Control System Toolbox is NOT required.

clear; clc; close all;

%% ============================ User Options ============================
cfg.T             = 34;        % simulation time [s]
cfg.dt            = 0.0125;    % numerical time step [s]
cfg.renderEvery   = 3;         % render every N simulation steps
cfg.makeMP4       = false;     % true = export video
cfg.mp4Name       = 'ball_on_plate_research_grade.mp4';
cfg.randomSeed    = 8;
cfg.showDiagnostics = true;

rng(cfg.randomSeed);

%% ============================ Geometry ================================
geo.plateL        = 4.80;      % plate side length [m, scaled visual]
geo.plateZ0       = 2.65;      % nominal plate center height
geo.baseL         = 6.30;
geo.corner        = geo.plateL/2;
geo.baseCorner    = geo.baseL/2;
geo.ballRadius    = 0.18;
geo.legRadius     = 0.040;
geo.crankLength   = 0.48;
geo.linkColor     = [0.58 0.18 1.00];
geo.motorColor    = [0.015 0.015 0.020];
geo.plateAlpha    = 0.23;

geo.plateXY = [ geo.corner  geo.corner 0;
              -geo.corner  geo.corner 0;
              -geo.corner -geo.corner 0;
               geo.corner -geo.corner 0];
geo.plateAnchors = [ geo.corner  geo.corner 0;
                    -geo.corner  geo.corner 0;
                    -geo.corner -geo.corner 0;
                     geo.corner -geo.corner 0];
geo.baseAnchors  = [ geo.baseCorner  geo.baseCorner 0;
                    -geo.baseCorner  geo.baseCorner 0;
                    -geo.baseCorner -geo.baseCorner 0;
                     geo.baseCorner -geo.baseCorner 0];

%% ============================ Physical Model ==========================
plant.g           = 9.81;
plant.ballMass    = 0.055;     % kg
plant.ballRadius  = geo.ballRadius;
plant.inertiaCoef = 2/5;       % solid sphere: I = 2/5 m r^2
plant.rollFactor  = 1/(1 + plant.inertiaCoef); % acceleration reduction for rolling sphere = 5/7
plant.muVisc      = 0.23;      % viscous rolling damping
plant.muCoul      = 0.035;     % Coulomb rolling resistance equivalent
plant.wallRestitution = 0.42;
plant.wallDamping = 0.72;
plant.maxBallSpeed = 4.5;
plant.sensorSigma = 0.006;     % position sensor noise [m]
plant.disturbAmp  = 0.060;     % disturbance acceleration magnitude

%% ============================ Controller ==============================
ctrl.maxTilt      = deg2rad(12.0);
ctrl.maxTiltRate  = deg2rad(55.0);
ctrl.Kp           = diag([2.55 2.55]);
ctrl.Kd           = diag([2.05 2.05]);
ctrl.Ki           = diag([0.055 0.055]);
ctrl.Kff          = 0.70;      % acceleration feedforward weight
ctrl.intLimit     = 0.75;
ctrl.commandFilterTau = 0.045;

%% ============================ Stepper / Linkage Model =================
act.nLegs         = 4;
act.omegaN        = 34;        % rad/s motor closed-loop natural frequency
act.zeta          = 0.68;      % damping ratio
act.maxSpeed      = deg2rad(520);
act.maxAccel      = deg2rad(4500);
act.microstepDeg  = 0.1125;    % 1/16 microstep of 1.8 degree motor
act.backlashDeg   = 0.18;
act.deadbandDeg   = 0.035;
act.torqueConst   = 0.38;      % Nm/rad-ish visual estimate
act.torqueLimit   = 1.80;
act.crankScale    = 0.80;      % target leg angle sensitivity to vertical platform height
act.nominalAngle  = deg2rad(42);
act.vibrationAmp  = deg2rad(0.18);

%% ============================ Reference Path ==========================
N = floor(cfg.T/cfg.dt)+1;
t = linspace(0,cfg.T,N)';
ref = zeros(N,2); refd = zeros(N,2); refdd = zeros(N,2);
for k = 1:N
    [ref(k,:), refd(k,:), refdd(k,:)] = referenceTrajectory(t(k));
end

%% ============================ State Variables =========================
% Ball state in plate/body coordinates: [x y vx vy]
x = [-1.10; 0.65; 0.0; 0.0];
roll = 0; pitch = 0; rollDot = 0; pitchDot = 0;
rollCmdFilt = 0; pitchCmdFilt = 0;
errInt = [0;0];

legTheta = act.nominalAngle*ones(4,1);
legOmega = zeros(4,1);
legCmd   = legTheta;
lastLegTarget = legTheta;

hist.time = t;
hist.ball = nan(N,2); hist.vel = nan(N,2); hist.ref = ref;
hist.err  = nan(N,1); hist.tilt = nan(N,2); hist.tiltCmd = nan(N,2);
hist.legTheta = nan(N,4); hist.legCmd = nan(N,4); hist.torque = nan(N,4);
hist.energy = nan(N,1); hist.slipIndex = nan(N,1);

%% ============================ Figure Setup ============================
fig = figure('Color',[0.012 0.014 0.022], 'Name','Research-Grade Ball-on-Plate Simulation', ...
    'Units','pixels', 'Position',[70 30 740 1280], 'Renderer','opengl');

ax3 = axes('Parent',fig,'Position',[0.045 0.345 0.91 0.59]);
hold(ax3,'on'); axis(ax3,'equal'); grid(ax3,'off');
axis(ax3,[-4.25 4.25 -4.25 4.25 -0.15 4.35]);
view(ax3,[-42 28]);
set(ax3,'Color',[0.012 0.014 0.022], 'XColor',[0.16 0.23 0.30], ...
    'YColor',[0.16 0.23 0.30], 'ZColor',[0.16 0.23 0.30]);
ax3.XTick=[]; ax3.YTick=[]; ax3.ZTick=[];
camlight(ax3,'headlight'); lighting(ax3,'gouraud'); material(ax3,'dull');

annotation(fig,'textbox',[0.035 0.955 0.93 0.035], 'String','Research-Grade Self-Balancing Ball-on-Plate Robot', ...
    'Color',[0.86 0.97 1.00], 'FontSize',19, 'FontWeight','bold', ...
    'HorizontalAlignment','center','EdgeColor','none');
annotation(fig,'textbox',[0.07 0.925 0.86 0.030], 'String','Nonlinear rolling dynamics  |  cascaded LQR-style tracking  |  inverse kinematics  |  realistic stepper actuation', ...
    'Color',[0.52 0.76 0.92], 'FontSize',10.5, 'HorizontalAlignment','center','EdgeColor','none');
statusText = annotation(fig,'textbox',[0.06 0.875 0.88 0.045], 'String','', ...
    'Color',[0.75 0.94 1.00], 'FontSize',11.5, 'FontName','Consolas', ...
    'HorizontalAlignment','center','EdgeColor','none');

% Base floor and motors
floorPatch = patch(ax3,[-3.55 3.55 3.55 -3.55],[-3.55 -3.55 3.55 3.55],[0 0 0 0], ...
    [0.025 0.028 0.038], 'FaceAlpha',0.72, 'EdgeColor',[0.08 0.12 0.16], 'LineWidth',1.0);
for i=1:4
    drawCube(ax3,geo.baseAnchors(i,:) + [0 0 0.13],[0.52 0.52 0.26],geo.motorColor,0.96,[0.25 0.22 0.36]);
end

% Plate, border, paths, ball
platePatch = patch(ax3,nan,nan,nan,[0.28 0.88 1.00], 'FaceAlpha',geo.plateAlpha, ...
    'EdgeColor',[0.15 0.95 1.00], 'LineWidth',2.1);
edgeLines = gobjects(4,1);
for i=1:4
    edgeLines(i)=plot3(ax3,nan,nan,nan,'Color',[0.05 0.98 1.0],'LineWidth',3.0);
end

ref3 = zeros(N,3);
for k=1:N, ref3(k,:) = localToWorld([ref(k,1),ref(k,2),0.018],0,0,geo.plateZ0); end
refLine = plot3(ax3,ref3(:,1),ref3(:,2),ref3(:,3),'--','Color',[0.12 0.95 1.0],'LineWidth',1.45);
trailLine = plot3(ax3,nan,nan,nan,'Color',[1.00 0.50 0.07],'LineWidth',2.25);
ballSurf = makeSphere(ax3,geo.ballRadius,[1.00 0.45 0.07]);
refDot = plot3(ax3,nan,nan,nan,'o','MarkerSize',7,'MarkerFaceColor',[0.12 0.95 1.0], 'MarkerEdgeColor','none');

legLines = gobjects(4,2); crankLines = gobjects(4,1); jointDots = gobjects(4,3);
for i=1:4
    legLines(i,1)=plot3(ax3,nan,nan,nan,'Color',geo.linkColor,'LineWidth',4.0);
    legLines(i,2)=plot3(ax3,nan,nan,nan,'Color',[0.25 0.60 1.0],'LineWidth',2.2);
    crankLines(i)=plot3(ax3,nan,nan,nan,'Color',[0.93 0.68 1.0],'LineWidth',3.0);
    for j=1:3
        jointDots(i,j)=plot3(ax3,nan,nan,nan,'o','MarkerSize',5.5,'MarkerFaceColor',[0.95 0.80 1.0], 'MarkerEdgeColor','none');
    end
end

% Diagnostics panels
axXY = axes('Parent',fig,'Position',[0.08 0.205 0.38 0.105]); hold(axXY,'on'); grid(axXY,'on');
styleAxis(axXY); title(axXY,'Path Following','Color',[0.88 0.97 1.0],'FontSize',10);
plot(axXY,ref(:,1),ref(:,2),'--','Color',[0.12 0.95 1.0],'LineWidth',1.1);
xyActual = plot(axXY,nan,nan,'Color',[1.0 0.50 0.08],'LineWidth',1.4);
xyDot = plot(axXY,nan,nan,'o','MarkerFaceColor',[1 0.45 0.06],'MarkerEdgeColor','none','MarkerSize',5);
axis(axXY,[-2.0 2.0 -2.0 2.0]); xlabel(axXY,'x'); ylabel(axXY,'y');

axErr = axes('Parent',fig,'Position',[0.56 0.205 0.36 0.105]); hold(axErr,'on'); grid(axErr,'on');
styleAxis(axErr); title(axErr,'Tracking Error','Color',[0.88 0.97 1.0],'FontSize',10);
errLine = plot(axErr,nan,nan,'Color',[1.0 0.50 0.08],'LineWidth',1.3);
axis(axErr,[0 cfg.T 0 1.6]); xlabel(axErr,'time [s]'); ylabel(axErr,'|e|');

axTilt = axes('Parent',fig,'Position',[0.08 0.065 0.38 0.105]); hold(axTilt,'on'); grid(axTilt,'on');
styleAxis(axTilt); title(axTilt,'Plate Roll / Pitch','Color',[0.88 0.97 1.0],'FontSize',10);
rollLine = plot(axTilt,nan,nan,'LineWidth',1.2,'Color',[0.25 0.75 1.0]);
pitchLine = plot(axTilt,nan,nan,'LineWidth',1.2,'Color',[1.0 0.50 0.08]);
axis(axTilt,[0 cfg.T -14 14]); xlabel(axTilt,'time [s]'); ylabel(axTilt,'deg');
legend(axTilt,{'roll','pitch'},'TextColor',[0.8 0.9 1],'Color',[0.02 0.025 0.035], 'Location','southwest');

axStep = axes('Parent',fig,'Position',[0.56 0.065 0.36 0.105]); hold(axStep,'on'); grid(axStep,'on');
styleAxis(axStep); title(axStep,'Stepper Command Tracking','Color',[0.88 0.97 1.0],'FontSize',10);
stepCmdLine = plot(axStep,nan,nan,'--','Color',[0.12 0.95 1.0],'LineWidth',1.1);
stepActLine = plot(axStep,nan,nan,'Color',[1.0 0.50 0.08],'LineWidth',1.3);
axis(axStep,[0 cfg.T 25 65]); xlabel(axStep,'time [s]'); ylabel(axStep,'leg-1 deg');
legend(axStep,{'cmd','actual'},'TextColor',[0.8 0.9 1],'Color',[0.02 0.025 0.035], 'Location','southwest');

if cfg.makeMP4
    vw = VideoWriter(cfg.mp4Name,'MPEG-4');
    vw.FrameRate = round(1/(cfg.dt*cfg.renderEvery));
    vw.Quality = 95; open(vw);
end

%% ============================ Main Simulation Loop ====================
trail = nan(N,3);
for k = 1:N
    tk = t(k);

    % Sensor reading
    yMeas = x(1:2) + plant.sensorSigma*randn(2,1);
    e = ref(k,:)' - yMeas;
    ed = refd(k,:)' - x(3:4);
    errInt = clampVec(errInt + e*cfg.dt, ctrl.intLimit);

    % Outer-loop desired acceleration in plate coordinates
    aDes = ctrl.Kp*e + ctrl.Kd*ed + ctrl.Ki*errInt + ctrl.Kff*refdd(k,:)';
    % Map desired ball acceleration to plate tilt. Small-angle: ax ~= rollFactor*g*pitch, ay ~= -rollFactor*g*roll
    pitchCmd = aDes(1)/(plant.rollFactor*plant.g);
    rollCmd  = -aDes(2)/(plant.rollFactor*plant.g);
    pitchCmd = max(min(pitchCmd,ctrl.maxTilt),-ctrl.maxTilt);
    rollCmd  = max(min(rollCmd, ctrl.maxTilt),-ctrl.maxTilt);

    % Command filtering and tilt-rate limit
    alpha = cfg.dt/(ctrl.commandFilterTau + cfg.dt);
    pitchCmdFilt = pitchCmdFilt + alpha*(pitchCmd - pitchCmdFilt);
    rollCmdFilt  = rollCmdFilt  + alpha*(rollCmd  - rollCmdFilt);
    pitchCmdFilt = rateLimit(pitchCmdFilt, pitch, ctrl.maxTiltRate, cfg.dt);
    rollCmdFilt  = rateLimit(rollCmdFilt,  roll,  ctrl.maxTiltRate, cfg.dt);

    % Four-leg inverse kinematics: target crank angles from desired plate corner heights
    Rcmd = rotmRP(rollCmdFilt,pitchCmdFilt);
    legTargets = zeros(4,1);
    for i=1:4
        pCmd = (Rcmd*geo.plateAnchors(i,:)')' + [0 0 geo.plateZ0];
        dz = pCmd(3) - geo.plateZ0;
        rawTheta = act.nominalAngle + act.crankScale*dz;
        legTargets(i) = quantizeAngle(rawTheta, act.microstepDeg);
    end

    % Backlash/deadband target shaping
    for i=1:4
        if abs(legTargets(i)-lastLegTarget(i)) < deg2rad(act.deadbandDeg)
            legCmd(i) = lastLegTarget(i);
        else
            sgn = sign(legTargets(i)-lastLegTarget(i));
            legCmd(i) = legTargets(i) - sgn*deg2rad(act.backlashDeg);
            lastLegTarget(i) = legTargets(i);
        end
    end

    % Stepper dynamics: saturated acceleration/speed + vibration ripple
    legTorque = zeros(4,1);
    for i=1:4
        thetaErr = legCmd(i) - legTheta(i);
        acc = act.omegaN^2*thetaErr - 2*act.zeta*act.omegaN*legOmega(i);
        acc = max(min(acc,act.maxAccel),-act.maxAccel);
        legOmega(i) = legOmega(i) + acc*cfg.dt;
        legOmega(i) = max(min(legOmega(i),act.maxSpeed),-act.maxSpeed);
        legTheta(i) = legTheta(i) + legOmega(i)*cfg.dt + act.vibrationAmp*sin(2*pi*42*tk + i*0.7)*cfg.dt;
        legTorque(i) = min(act.torqueLimit, abs(act.torqueConst*thetaErr + 0.035*legOmega(i)));
    end

    % Effective plate pose comes from realized leg heights, not instant command
    legHeight = (legTheta - act.nominalAngle)/act.crankScale;
    h1=legHeight(1); h2=legHeight(2); h3=legHeight(3); h4=legHeight(4);
    pitchNew = ((h1+h4) - (h2+h3))/(2*geo.plateL); % x-slope
    rollNew  = ((h1+h2) - (h3+h4))/(2*geo.plateL); % y-slope
    pitchNew = max(min(pitchNew,ctrl.maxTilt*1.05),-ctrl.maxTilt*1.05);
    rollNew  = max(min(rollNew, ctrl.maxTilt*1.05),-ctrl.maxTilt*1.05);
    pitchDot = (pitchNew - pitch)/cfg.dt;
    rollDot  = (rollNew - roll)/cfg.dt;
    pitch = pitchNew; roll = rollNew;

    % Nonlinear rolling ball dynamics on tilted/moving plate
    dist = plant.disturbAmp*[sin(1.7*tk+0.6)+0.35*sin(5.1*tk); cos(1.45*tk)-0.25*sin(4.3*tk)];
    ax = plant.rollFactor*plant.g*sin(pitch) - plant.muVisc*x(3) - plant.muCoul*tanh(8*x(3)) + dist(1) - 0.035*pitchDot;
    ay = -plant.rollFactor*plant.g*sin(roll)  - plant.muVisc*x(4) - plant.muCoul*tanh(8*x(4)) + dist(2) + 0.035*rollDot;

    x(3) = x(3) + ax*cfg.dt;
    x(4) = x(4) + ay*cfg.dt;
    spd = hypot(x(3),x(4));
    if spd > plant.maxBallSpeed
        x(3:4) = x(3:4)*plant.maxBallSpeed/spd;
    end
    x(1) = x(1) + x(3)*cfg.dt;
    x(2) = x(2) + x(4)*cfg.dt;

    % Contact boundary handling
    limit = geo.corner - geo.ballRadius*1.4;
    if abs(x(1)) > limit
        x(1) = sign(x(1))*limit;
        x(3) = -plant.wallRestitution*x(3)*plant.wallDamping;
    end
    if abs(x(2)) > limit
        x(2) = sign(x(2))*limit;
        x(4) = -plant.wallRestitution*x(4)*plant.wallDamping;
    end

    % History
    hist.ball(k,:) = x(1:2)';
    hist.vel(k,:) = x(3:4)';
    hist.err(k) = norm(ref(k,:)'-x(1:2));
    hist.tilt(k,:) = [roll pitch];
    hist.tiltCmd(k,:) = [rollCmdFilt pitchCmdFilt];
    hist.legTheta(k,:) = legTheta';
    hist.legCmd(k,:) = legCmd';
    hist.torque(k,:) = legTorque';
    hist.energy(k) = 0.5*plant.ballMass*(x(3)^2+x(4)^2) + plant.ballMass*plant.g*geo.ballRadius;
    hist.slipIndex(k) = norm([ax ay])/(plant.g*0.8);

    % Rendering
    if mod(k,cfg.renderEvery)==1 || k==N
        R = rotmRP(roll,pitch);
        plateWorld = (R*geo.plateXY')' + [0 0 geo.plateZ0];
        set(platePatch,'XData',plateWorld(:,1),'YData',plateWorld(:,2),'ZData',plateWorld(:,3));
        for ii=1:4
            jj = mod(ii,4)+1;
            set(edgeLines(ii),'XData',[plateWorld(ii,1) plateWorld(jj,1)], ...
                'YData',[plateWorld(ii,2) plateWorld(jj,2)], 'ZData',[plateWorld(ii,3) plateWorld(jj,3)]);
        end

        ballCenter = localToWorld([x(1),x(2),geo.ballRadius+0.025],roll,pitch,geo.plateZ0);
        updateSphere(ballSurf,ballCenter,geo.ballRadius);
        refCenter = localToWorld([ref(k,1),ref(k,2),geo.ballRadius+0.035],roll,pitch,geo.plateZ0);
        set(refDot,'XData',refCenter(1),'YData',refCenter(2),'ZData',refCenter(3));
        trail(k,:) = ballCenter;
        recent = max(1,k-850):k;
        set(trailLine,'XData',trail(recent,1),'YData',trail(recent,2),'ZData',trail(recent,3));

        % update ref path to lie on current plate plane
        skip=1:10:N;
        refNow = zeros(numel(skip),3);
        for q=1:numel(skip)
            kk=skip(q); refNow(q,:) = localToWorld([ref(kk,1),ref(kk,2),0.020],roll,pitch,geo.plateZ0);
        end
        set(refLine,'XData',refNow(:,1),'YData',refNow(:,2),'ZData',refNow(:,3));

        % Linkage display: motor -> crank -> plate anchor + secondary support
        for i=1:4
            base = geo.baseAnchors(i,:) + [0 0 0.23];
            anchor = (R*geo.plateAnchors(i,:)')' + [0 0 geo.plateZ0];
            radial = geo.baseAnchors(i,1:2); radial = radial/(norm(radial)+eps);
            tangent = [-radial(2), radial(1)];
            crankTip = base + [geo.crankLength*cos(legTheta(i))*radial + 0.18*sin(legTheta(i))*tangent, ...
                               geo.crankLength*sin(legTheta(i))+0.15];
            mid = 0.52*crankTip + 0.48*anchor + [0 0 0.12*sin(legTheta(i))];
            set(crankLines(i),'XData',[base(1) crankTip(1)],'YData',[base(2) crankTip(2)],'ZData',[base(3) crankTip(3)]);
            set(legLines(i,1),'XData',[crankTip(1) mid(1) anchor(1)],'YData',[crankTip(2) mid(2) anchor(2)],'ZData',[crankTip(3) mid(3) anchor(3)]);
            set(legLines(i,2),'XData',[base(1) anchor(1)],'YData',[base(2) anchor(2)],'ZData',[base(3) anchor(3)]);
            pts = [base; crankTip; anchor];
            for j=1:3
                set(jointDots(i,j),'XData',pts(j,1),'YData',pts(j,2),'ZData',pts(j,3));
            end
        end

        statusText.String = sprintf('t = %5.2f s   roll = %+6.2f deg   pitch = %+6.2f deg   error = %.3f m   avg torque = %.2f Nm   slip index = %.2f', ...
            tk,rad2deg(roll),rad2deg(pitch),hist.err(k),mean(legTorque),hist.slipIndex(k));

        idx = 1:k;
        set(xyActual,'XData',hist.ball(idx,1),'YData',hist.ball(idx,2));
        set(xyDot,'XData',x(1),'YData',x(2));
        set(errLine,'XData',hist.time(idx),'YData',hist.err(idx));
        set(rollLine,'XData',hist.time(idx),'YData',rad2deg(hist.tilt(idx,1)));
        set(pitchLine,'XData',hist.time(idx),'YData',rad2deg(hist.tilt(idx,2)));
        set(stepCmdLine,'XData',hist.time(idx),'YData',rad2deg(hist.legCmd(idx,1)));
        set(stepActLine,'XData',hist.time(idx),'YData',rad2deg(hist.legTheta(idx,1)));

        drawnow limitrate;
        if cfg.makeMP4
            writeVideo(vw,getframe(fig));
        end
    end
end

if cfg.makeMP4
    close(vw);
    fprintf('MP4 exported: %s\n',cfg.mp4Name);
end

%% ============================ Result Summary ==========================
valid = ~isnan(hist.err);
rmsErr = sqrt(mean(hist.err(valid).^2));
maxErr = max(hist.err(valid));
meanTorque = mean(hist.torque(valid,:), 'all');
maxTiltDeg = max(abs(rad2deg(hist.tilt(valid,:))),[],'all');

fprintf('\n===== Research-Grade Ball-on-Plate Simulation Summary =====\n');
fprintf('RMS tracking error      : %.4f m\n',rmsErr);
fprintf('Maximum tracking error  : %.4f m\n',maxErr);
fprintf('Mean holding torque     : %.4f Nm\n',meanTorque);
fprintf('Maximum plate tilt      : %.3f deg\n',maxTiltDeg);
fprintf('Simulation duration     : %.2f s\n',cfg.T);
fprintf('===========================================================\n');

%% ============================ Local Functions =========================
function [r, rd, rdd] = referenceTrajectory(t)
    % Smooth advanced path: bounded Lissajous + flower modulation.
    A = 1.22; B = 1.05; w = 0.54;
    x = A*sin(w*t) + 0.33*sin(3*w*t + 0.35);
    y = B*sin(2*w*t + pi/3) + 0.25*cos(4*w*t);
    xd = A*w*cos(w*t) + 0.33*3*w*cos(3*w*t + 0.35);
    yd = B*2*w*cos(2*w*t + pi/3) - 0.25*4*w*sin(4*w*t);
    xdd = -A*w^2*sin(w*t) - 0.33*(3*w)^2*sin(3*w*t + 0.35);
    ydd = -B*(2*w)^2*sin(2*w*t + pi/3) - 0.25*(4*w)^2*cos(4*w*t);
    r = [x y]; rd = [xd yd]; rdd = [xdd ydd];
end

function R = rotmRP(roll,pitch)
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R = Ry*Rx;
end

function p = localToWorld(local,roll,pitch,z0)
    R = rotmRP(roll,pitch);
    p = (R*local(:))' + [0 0 z0];
end

function y = clampVec(x,lim)
    y = max(min(x,lim),-lim);
end

function y = rateLimit(cmd,current,maxRate,dt)
    d = cmd-current;
    d = max(min(d,maxRate*dt),-maxRate*dt);
    y = current+d;
end

function q = quantizeAngle(theta,microstepDeg)
    step = deg2rad(microstepDeg);
    q = round(theta/step)*step;
end

function h = makeSphere(ax,r,color)
    [xs,ys,zs] = sphere(32);
    h = surf(ax,r*xs,r*ys,r*zs,'FaceColor',color,'EdgeColor','none','FaceLighting','gouraud');
end

function updateSphere(h,c,r)
    [xs,ys,zs] = sphere(32);
    set(h,'XData',c(1)+r*xs,'YData',c(2)+r*ys,'ZData',c(3)+r*zs);
end

function styleAxis(ax)
    set(ax,'Color',[0.020 0.024 0.036], 'XColor',[0.70 0.82 0.92], 'YColor',[0.70 0.82 0.92], ...
        'GridColor',[0.20 0.29 0.38], 'MinorGridColor',[0.16 0.22 0.30], 'FontSize',8.5);
end

function drawCube(ax,center,scale,faceColor,alphaVal,edgeColor)
    sx=scale(1)/2; sy=scale(2)/2; sz=scale(3)/2;
    V=[-sx -sy -sz; sx -sy -sz; sx sy -sz; -sx sy -sz; -sx -sy sz; sx -sy sz; sx sy sz; -sx sy sz] + center;
    F=[1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax,'Vertices',V,'Faces',F,'FaceColor',faceColor,'FaceAlpha',alphaVal,'EdgeColor',edgeColor,'LineWidth',0.8);
end
