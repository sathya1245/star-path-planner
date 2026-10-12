function star_basic()
% STAR_BASIC  Minimal, self-contained demo of Occupancy Grid Mapping (OGM)
% plus the STAR idea (Stochastic Traffic-Adaptive Replanning):
%
%   OGM  : local free/occupied grid built from detections every tick
%   TEI  : Traffic Entropy Index  = 0.7*HeadingEntropy + 0.3*SpeedVariance
%   HTRF : Heavy-Tailed Risk Field (bivariate Cauchy lobe per obstacle)
%   Adaptive horizon : high TEI -> short horizon (2 s), low TEI -> long (5 s)
%   Contingency portfolio : pre-computed escape maneuvers, hot-swapped in
%                           an emergency (rebuilt only every few ticks)
%
% Base MATLAB only (no toolboxes).  Run with:   star_basic
%
% Road model: straight road along +x, so Frenet s = x and lateral offset d = y.

    %% 1. Parameters ---------------------------------------------------
    P.road_len = 90;   P.road_w = 10;   P.res = 0.5;   % road + grid cell (m)
    P.dt = 0.1;        P.T = 25;                       % timestep, max time (s)
    P.ego_r = 1.0;                                     % ego footprint radius (m)
    P.margin = 1.0;                                    % extra safety gap (m), same in TTC + collision checks
    P.v_nom = 8;  P.v_caut = 5;  P.v_yield = 3;        % speed per state (m/s)
    P.ttc_yield = 4;   P.ttc_brake = 1.5;              % TTC thresholds (s)
    P.tei_caution = 0.7;                               % enter CAUTIOUS above this
    P.tei_lo = 0.2;    P.tei_hi = 1.0;                 % TEI range for horizon map
    P.h_min = 2;       P.h_max = 5;                    % horizon limits (s)
    P.d_cands = [-3.5 -2 0 2 3.5];                     % candidate lateral offsets
    P.vy = 2.0;                                        % max lateral speed (m/s)
    P.sig0 = 1.2;      P.sig_speed = 0.5;              % HTRF base spread / speed gain
    P.w_risk = 20;                                     % HTRF weight in planner cost
    P.portfolio_every = 10;                            % rebuild portfolio every N ticks

    % Obstacles: position [x y], velocity [vx vy], radius, class
    obs = [ mk([30 -4],  [0.2  0.6], 0.9, 'cow'), ...
            mk([45  3],  [0.4 -0.5], 0.5, 'pedestrian'), ...
            mk([55 -1],  [1.0  0.2], 1.0, 'rickshaw'), ...
            mk([38 2.5], [0    0  ], 0.6, 'pushcart') ];

    %% 2. Main closed loop ---------------------------------------------
    ego = struct('x', 0, 'y', 0, 'v', 0);
    portfolio = make_portfolio(ego.y, P);
    prev_state = 'NOMINAL';
    n = round(P.T / P.dt);
    log_v = zeros(1,n);  log_tei = zeros(1,n);  log_H = zeros(1,n);
    lat_plan = [];  lat_swap = [];  min_clear = inf;  last = n;
    figure('Color', 'w');

    for k = 1:n
        t = (k-1) * P.dt;
        obs_t = propagate(obs, t);                 % where obstacles are now

        % --- Perception: occupancy grid map ---
        G = build_ogm(obs_t, ego, P);

        % --- STAR: chaos index -> planning horizon ---
        tei = compute_tei(obs_t, ego);
        H   = adaptive_horizon(tei, P);

        % --- Decision: simple state machine (TTC reactive + TEI preemptive) ---
        ttc = compute_ttc(ego, obs_t, P);
        % (30% hysteresis on the TTC thresholds stops state chattering)
        if     ttc < P.ttc_brake,                                 state = 'EMERGENCY';
        elseif strcmp(prev_state, 'EMERGENCY') && ttc < 1.3*P.ttc_brake, state = 'EMERGENCY';
        elseif ttc < P.ttc_yield,                                 state = 'YIELD';
        elseif strcmp(prev_state, 'YIELD') && ttc < 1.3*P.ttc_yield,     state = 'YIELD';
        elseif tei > P.tei_caution,                               state = 'CAUTIOUS';
        else                                                      state = 'NOMINAL';
        end
        prev_state = state;

        % --- Planning ---
        if strcmp(state, 'EMERGENCY')
            tic;                                   % fast path: pick from portfolio
            [d_t, v_t, path] = hotswap(portfolio, ego, obs_t, G, P);
            lat_swap(end+1) = toc;
        else
            switch state
                case 'NOMINAL',  v_t = P.v_nom;
                case 'CAUTIOUS', v_t = P.v_caut;
                case 'YIELD',    v_t = P.v_yield;
            end
            tic;                                   % normal path: HTRF-weighted search
            [d_t, path] = plan_lateral(ego, obs_t, G, H, P);
            lat_plan(end+1) = toc;
        end

        % Asynchronous decoupling: portfolio rebuilt only every N ticks
        if mod(k, P.portfolio_every) == 0
            portfolio = make_portfolio(ego.y, P);
        end

        % --- Motion (simple longitudinal + lateral-rate-limited model) ---
        a = min(max((v_t - ego.v) * 1.2, -6), 2.5);
        ego.v = max(ego.v + a * P.dt, 0);
        ego.x = ego.x + ego.v * P.dt;
        ego.y = ego.y + min(max(d_t - ego.y, -P.vy*P.dt), P.vy*P.dt);

        % --- Logging + safety metric ---
        log_v(k) = ego.v;  log_tei(k) = tei;  log_H(k) = H;
        for j = 1:numel(obs_t)
            c = norm([ego.x ego.y] - obs_t(j).pos) - obs_t(j).r - P.ego_r;
            min_clear = min(min_clear, c);
        end

        if mod(k, 2) == 0
            draw_frame(G, obs_t, ego, path, state, tei, H, t, P);
            drawnow;
        end
        if ego.x >= P.road_len
            last = k;  break;
        end
    end

    %% 3. Summary ------------------------------------------------------
    fprintf('Finished: x = %.1f m after %.1f s\n', ego.x, last * P.dt);
    fprintf('Minimum clearance to any obstacle: %.2f m (%s)\n', min_clear, ...
        ternary(min_clear > 0, 'no collision', 'COLLISION'));
    fprintf('Normal planner latency : %.2f ms mean (%d calls)\n', 1000*mean(lat_plan), numel(lat_plan));
    if ~isempty(lat_swap)
        fprintf('Hot-swap latency       : %.2f ms mean (%d calls)\n', 1000*mean(lat_swap), numel(lat_swap));
    else
        fprintf('Hot-swap never needed in this run.\n');
    end

    figure('Color', 'w');
    tt = (0:last-1) * P.dt;
    subplot(2,1,1); plot(tt, log_v(1:last), 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Speed (m/s)'); title('Ego speed');
    subplot(2,1,2); yyaxis left;  plot(tt, log_tei(1:last), 'LineWidth', 1.5); ylabel('TEI');
                    yyaxis right; plot(tt, log_H(1:last),   'LineWidth', 1.5); ylabel('Horizon (s)');
    grid on; xlabel('Time (s)'); title('Traffic Entropy Index and adaptive horizon');
end

%% ====================== Occupancy grid mapping ========================
function G = build_ogm(obs_t, ego, P)
% Local grid around the ego: 1 = occupied (obstacle or road edge), 0 = free.
    xs = (ego.x - 10) : P.res : (ego.x + 45);
    ys = (-P.road_w/2 - 1) : P.res : (P.road_w/2 + 1);
    [X, Y] = meshgrid(xs, ys);
    occ = abs(Y) > P.road_w/2;                          % road edges
    for j = 1:numel(obs_t)
        occ = occ | ((X - obs_t(j).pos(1)).^2 + (Y - obs_t(j).pos(2)).^2 <= obs_t(j).r^2);
    end
    rc = ceil(P.ego_r / P.res);                         % inflate by ego size
    occ_inf = conv2(double(occ), ones(2*rc+1), 'same') > 0;
    G = struct('X', X, 'Y', Y, 'occ', double(occ), 'occ_inf', occ_inf, 'res', P.res);
end

function tf = cell_occupied(G, x, y)
    ix = round((x - G.X(1,1)) / G.res) + 1;
    iy = round((y - G.Y(1,1)) / G.res) + 1;
    tf = false;
    if iy >= 1 && iy <= size(G.occ,1) && ix >= 1 && ix <= size(G.occ,2)
        tf = G.occ_inf(iy, ix);
    end
end

%% ================= STAR: TEI, HTRF, adaptive horizon ==================
function tei = compute_tei(obs_t, ego)
% TEI = 0.7*H + 0.3*var(speed), using obstacles within 50 m of the ego.
    ang = [];  spd = [];
    for j = 1:numel(obs_t)
        if norm(obs_t(j).pos - [ego.x ego.y]) <= 50
            s = norm(obs_t(j).vel);
            spd(end+1) = s;                                         %#ok<AGROW>
            if s > 1e-3
                ang(end+1) = atan2(obs_t(j).vel(2), obs_t(j).vel(1)); %#ok<AGROW>
            end
        end
    end
    Hent = 0;                                           % heading entropy (8 bins)
    if ~isempty(ang)
        p = histcounts(ang, linspace(-pi, pi, 9));
        p = p / sum(p);  p = p(p > 0);
        Hent = -sum(p .* log(p));
    end
    v2 = 0;                                             % speed variance
    if numel(spd) > 1, v2 = var(spd); end
    tei = 0.7 * Hent + 0.3 * v2;
end

function H = adaptive_horizon(tei, P)
% High chaos -> short horizon (h_min), structured -> long horizon (h_max).
    f = min(max((tei - P.tei_lo) / (P.tei_hi - P.tei_lo), 0), 1);
    H = P.h_max - f * (P.h_max - P.h_min);
end

function R = htrf_map(X, Y, obs_t, P)
% Sum of bivariate-Cauchy risk lobes, one per obstacle:
%   f(p) = 1/(2*pi*sqrt|S|) * (1 + (p-mu)' * inv(S) * (p-mu))^(-3/2)
% S is aligned with the obstacle heading: wider along it when moving fast,
% wider across it for erratic classes (cow) than for predictable ones.
    R = zeros(size(X));
    for j = 1:numel(obs_t)
        o = obs_t(j);
        spd = norm(o.vel);
        th = 0;  if spd > 1e-3, th = atan2(o.vel(2), o.vel(1)); end
        sa = P.sig0 + P.sig_speed * spd;                % spread along heading
        sc = P.sig0 * class_mult(o.type);               % spread across heading
        Rm = [cos(th) -sin(th); sin(th) cos(th)];
        S  = Rm * diag([sa^2, sc^2]) * Rm';
        Si = inv(S);
        dx = X - o.pos(1);   dy = Y - o.pos(2);
        q  = Si(1,1)*dx.^2 + 2*Si(1,2)*dx.*dy + Si(2,2)*dy.^2;
        R  = R + (1 / (2*pi*sqrt(det(S)))) * (1 + q).^(-1.5);
    end
end

function m = class_mult(type)
    switch type
        case 'cow',        m = 2.5;     % erratic
        case 'pedestrian', m = 1.5;
        otherwise,         m = 1.0;     % vehicles / static
    end
end

%% ======================== Decision + planning =========================
function ttc = compute_ttc(ego, obs_t, P)
% Time-to-collision to the nearest obstacle ahead that overlaps our lane.
    ttc = inf;
    for j = 1:numel(obs_t)
        o = obs_t(j);
        gap = o.pos(1) - ego.x - o.r - P.ego_r;
        lat = abs(o.pos(2) - ego.y);
        if o.pos(1) > ego.x && lat < o.r + P.ego_r + P.margin
            ttc = min(ttc, max(gap, 0) / max(ego.v, 0.5));
        end
    end
end

function pts = lateral_path(ego, d_target, L, n, P)
% Path of n points over distance L, moving laterally toward d_target at
% the same rate limit the ego motion model uses (so plan == execution).
    xs  = linspace(ego.x, ego.x + L, n)';
    tau = (xs - ego.x) / max(ego.v, 2);
    dy  = max(min(d_target - ego.y, P.vy * tau), -P.vy * tau);
    pts = [xs, ego.y + dy];
end

function hit = collides(pts, ego, obs_t, G, P)
% Cheap hard check: predicted obstacle circles + (near-term) grid cells.
    hit = false;  v = max(ego.v, 2);
    for i = 1:size(pts,1)
        tau = (pts(i,1) - ego.x) / v;
        o = propagate(obs_t, tau);
        for j = 1:numel(o)
            if norm(pts(i,:) - o(j).pos) < o(j).r + P.ego_r + P.margin
                hit = true;  return;
            end
        end
        if tau <= 1 && cell_occupied(G, pts(i,1), pts(i,2))
            hit = true;  return;
        end
    end
end

function risk = path_risk(pts, ego, obs_t, P)
% Total HTRF risk along a path, evaluated at predicted obstacle positions.
    risk = 0;  v = max(ego.v, 2);
    for i = 1:size(pts,1)
        o = propagate(obs_t, (pts(i,1) - ego.x) / v);
        risk = risk + htrf_map(pts(i,1), pts(i,2), o, P);
    end
end

function [d_best, path] = plan_lateral(ego, obs_t, G, H, P)
% Try each lateral offset over a TEI-driven horizon; pick the lowest cost.
    L = H * max(ego.v, 2);                              % horizon: seconds -> meters
    best = inf;  d_best = ego.y;  path = lateral_path(ego, ego.y, L, 12, P);
    for d = P.d_cands
        pts  = lateral_path(ego, d, L, 12, P);
        cost = P.w_risk * path_risk(pts, ego, obs_t, P) ...
             + 1000 * collides(pts, ego, obs_t, G, P) ...
             + 0.5 * abs(d) + 1.0 * abs(d - ego.y);
        if cost < best
            best = cost;  d_best = d;  path = pts;
        end
    end
end

%% ================== STAR: contingency portfolio =======================
function pf = make_portfolio(y0, P)
% Escape maneuvers prepared ahead of time (brake in lane / swerve L / swerve R).
    pf = struct('name', {'brake', 'swerve_left', 'swerve_right'}, ...
                'd',    {y0, min(y0 + 3, P.d_cands(end)), max(y0 - 3, P.d_cands(1))}, ...
                'v',    {0, P.v_yield, P.v_yield}, ...
                'pref', {0, 1, 1});
end

function [d_t, v_t, path] = hotswap(pf, ego, obs_t, G, P)
% No search: just a quick collision check of each ready-made maneuver.
    best = inf;  d_t = pf(1).d;  v_t = pf(1).v;  path = lateral_path(ego, d_t, 8, 8, P);
    for i = 1:numel(pf)
        pts  = lateral_path(ego, pf(i).d, 8, 8, P);
        cost = 1000 * collides(pts, ego, obs_t, G, P) + pf(i).pref;
        if cost < best
            best = cost;  d_t = pf(i).d;  v_t = pf(i).v;  path = pts;
        end
    end
end

%% ============================= Helpers ================================
function s = mk(pos, vel, r, type)
    s = struct('pos', pos, 'vel', vel, 'r', r, 'type', type);
end

function out = propagate(obs, t)
    out = obs;
    for j = 1:numel(obs)
        out(j).pos = obs(j).pos + obs(j).vel * t;
    end
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end

function draw_frame(G, obs_t, ego, path, state, tei, H, t, P)
    cla;  hold on;
    R = htrf_map(G.X, G.Y, obs_t, P);                   % risk heat map
    imagesc(G.X(1,:), G.Y(:,1), R);  set(gca, 'YDir', 'normal');  colormap(hot);
    contour(G.X, G.Y, G.occ, [0.5 0.5], 'c', 'LineWidth', 1);   % occupied cells
    th = linspace(0, 2*pi, 30);
    for j = 1:numel(obs_t)
        plot(obs_t(j).pos(1) + obs_t(j).r*cos(th), obs_t(j).pos(2) + obs_t(j).r*sin(th), 'w-', 'LineWidth', 1.5);
    end
    plot(path(:,1), path(:,2), 'g-', 'LineWidth', 2.5);
    plot(ego.x, ego.y, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
    axis equal;  xlim([min(G.X(:)) max(G.X(:))]);  ylim([min(G.Y(:)) max(G.Y(:))]);
    title(sprintf('t=%.1fs | %s | v=%.1f m/s | TEI=%.2f | horizon=%.1fs', t, state, ego.v, tei, H));
    xlabel('x (m)');  ylabel('y (m)');  hold off;
end
