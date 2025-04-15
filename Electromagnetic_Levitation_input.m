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

% 초기 상태 
x0 = [0.014; 0; 2.001];  % 단위: [m; m/s; A]

Az = [0 1 0; 0 0 1; 0 0 0];
Bz = [0; 0; 1];

% 제어기
poles = [-10, -12, -14];
K = acker(Az, Bz, poles);

% 출력 행렬
C = [1 0 0];

% 시뮬레이션 시간
tspan = [0 5];

% 시뮬레이션 실행
[T, X] = ode45(@(t, x) levitation(t, x, Rc, Rs, Lc, Km, Mb, g, xb0, i0, K), tspan, x0);

% 결과 (mm로 변환)
figure;
plot(T, X(:,1)*1000, 'r--', 'LineWidth', 2);
xlabel('Time (sec)');
ylabel('Ball Position (mm)');
title('Electromagnetic Levitation - Input Linearization');
grid on;
xlim([0 5]);
ylim([5 15]);
yticks(5:1:15);
xticks(0:0.5:5);

function dx = levitation(~, x, Rc, Rs, Lc, Km, Mb, g, xb0, i0, K, zi )
    x1 = x(1);
    x2 = x(2);
    x3 = x(3); 
    x = [x1; x2; x3];

    % 입출력 궤환 선형화
    z1 = x1;
    z2 = x2;
    z3 = -Km * x3^2 / (2 * Mb * x1^2) + g;
    z = [z1; z2; z3];

    % 평형점
    zi = [xb0;0;0];   

    % 선형 제어기 설계 (가상 입력)
    v = -K * (z - zi);

    % 실제 입력 u 계산 (입력 변환)
    z1 = x1;
    z2 = x2;
    z3 = -Km * x3^2 / (2 * Mb * x1^2) + g;

    alpha = (Km * x3) / (Mb * x1^2) * ( (x2 * x3 / x1) + ((Rc + Rs) * x3 / Lc) );
    beta  = -Km * x3 / (Mb * Lc * x1^2);

    % 실제 제어 입력
    u = (v - alpha) / beta;

    % 비선형 시스템 식
    x1_dot = x2;
    x2_dot = -Km * x3^2 / (2 * Mb * x1^2) + g;
    x3_dot = (1 / Lc) * (- (Rc + Rs) * x3 + u);

    dx = [x1_dot; x2_dot; x3_dot];
end
