% 시스템 파라미터 
Rc = 10;         % 옴
Rs = 1;          % 옴
Lc = 412.5e-3;   % H
Km = 6.5308e-5;  % N·m^2 / A^2
Mb = 0.068;      % kg
g = 9.81;        % m/s^2
xb0 = 6e-3;      % m (제어 목표 위치)

% 평형점에서 필요한 전류
i0 = sqrt(2 * Mb * g / Km) * xb0;

% 자코비안 선형화 (평형점 기준 A, B)
A0 = [0 1 0;
      Km*i0^2/(Mb*xb0^3), 0, -Km*i0/(Mb*xb0^2);
      0 0 -(Rc + Rs)/Lc];

B0 = [0; 0; 1/Lc];

% 출력 행렬
C = [1 0 0];

% 제어기
poles = [-10, -12, -14];
K = acker(A0, B0, poles);

% 초기 상태 (공이 위쪽에서 낙하)
x0 = [0.014; 0; 2.001];  % 단위: [m; m/s; A]

% 시뮬레이션 시간
tspan = [0 5];

% 시뮬레이션 실행
[T, X] = ode45(@(t, x) levitation(t, x, Rc, Rs, Lc, Km, Mb, g, xb0, i0, K), tspan, x0);

% 결과 (mm로 변환)
figure;
plot(T, X(:,1)*1000, 'b-', 'LineWidth', 2);
xlabel('Time (sec)');
ylabel('Ball Position (mm)');
title('Electromagnetic Levitation - Jacobian Linearization');
grid on;
xlim([0 5]);
ylim([5 15]);
yticks(5:1:15);
xticks(0:0.5:5);

function dx = levitation(~, x, Rc, Rs, Lc, Km, Mb, g, xb0, i0, K)
    % 상태 변수
    x1 = x(1); % 위치 [m]
    x2 = x(2); % 속도 [m/s]
    x3 = x(3); % 전류 [A]

    % 제어 입력 (평형점 기준으로 보정)
    ref = [xb0; 0; i0];
    u_eq = (Rc + Rs) * i0;
    u = -K * (x - ref) + u_eq;

    % 비선형 시스템 식
    x1_dot = x2;
    x2_dot = -Km * x3^2 / (2 * Mb * x1^2) + g;
    x3_dot = ( - (Rc + Rs) * x3 + u ) / Lc;

    dx = [x1_dot; x2_dot; x3_dot];
end
