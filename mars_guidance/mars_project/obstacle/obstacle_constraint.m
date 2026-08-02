%% OBSTACLE CONSTRAINT (Paper Eq.6/7/8 + Box I with homotopy)
%  Computes the required altitude F (m) of each obstacle constraint and the
%  trajectory margin rx - F.
%
%  type:  'glide'   - conventional glide-slope constraint, Eq.(6)
%         'relaxed' - relaxed glide-slope constraint, Eq.(7)/Box I
%         'stepwise'- stepwise constraint, Eq.(8)/Eq.(33)
%  delta: homotopy parameter in [0,1]. The full constraint is multiplied by
%         delta (Box I / Eq.33): delta=0 -> no constraint, delta=1 -> full.
%
%  Inputs:
%    r      - position vector [rx; ry; rz] (m)
%    params - struct from mars_params.m
%    type   - constraint type (string)
%    delta  - homotopy parameter (default 1)
%  Outputs:
%    F      - required altitude (m)
%    margin - rx - F (positive = constraint satisfied)
%    d_h    - horizontal distance sqrt(ry^2 + rz^2) (m)

function [F, margin, d_h] = obstacle_constraint(r, params, type, delta)
    if nargin < 4 || isempty(delta)
        delta = 1;
    end
    rx = r(1); ry = r(2); rz = r(3);
    d_h = sqrt(ry.^2 + rz.^2);

    switch type
        case 'glide'
            % Eq.(6): tan(gamma_gs) * sqrt(ry^2+rz^2) <= rx
            F = tan(params.gamma_gs_rad) * d_h;
        case 'relaxed'
            % Eq.(7)/Box I: safe altitude h at radius l2, relaxed cone inside
            l1 = params.l1;
            l2 = params.l2;
            h  = params.h_safe;
            tan_new = h / (l2 - l1);   % tan(gamma_new)
            if d_h <= l1
                F0 = 0;
            elseif d_h < l2
                F0 = (d_h - l1) * tan_new;
            else
                F0 = h;
            end
            F = delta * F0;
        case 'stepwise'
            % Eq.(8)/Eq.(33): chimney of radius l and altitude h
            if d_h <= params.l_step
                F0 = 0;
            else
                F0 = params.h_step;
            end
            F = delta * F0;
        otherwise
            error('Unknown obstacle constraint type: %s', type);
    end
    margin = rx - F;
end
