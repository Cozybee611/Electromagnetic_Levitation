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

% 입출력 선형화
Az = [0 1 0;
       0 0 1;
       0 0 0];
Bz = [0; 0; 1];

poles_z = [-10 -12 -14];
Kz = acker(Az, Bz, poles_z);

% 초기 상태 (공이 위쪽에서 낙하)
x0 = [0.014; 0; 2.001; 0];  % 단위: [m; m/s; A]

% 시뮬레이션 시간
tspan = [0 5];

% 시뮬레이션 실행
[T, X] = ode45(@(t, x) switching_control(t, x, Rc, Rs, Lc, Km, Mb, g, xb0, i0, K, Kz), tspan, x0);

% 결과 (mm로 변환)
figure;
plot(T, X(:,1)*1000, 'k-', 'LineWidth', 2);
xlabel('Time (sec)');
ylabel('Ball Position (mm)');
title('Electromagnetic Levitation - Jacobian Linearization');
grid on;
xlim([0 5]);
ylim([5 15]);
yticks(5:1:15);
xticks(0:0.5:5);

% 실시간 스위칭 제어 함수
function dx = switching_control(~, x, Rc, Rs, Lc, Km, Mb, g, xb0, i0, K, Kz)
    % 상태 변수
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    e = x(4);
    x = [x1; x2; x3];

    xe = [x1;x2;x3;e];
    ki = -2;
    kj = -10
    e_dot = x1 - xb0; % 적분기

    y = x(1);

    if x(1) > 0.006
        % 자코비안
        ref = [xb0; 0; i0];
        u_eq = (Rc + Rs) * i0;
        u = -K * (x - ref) - kj * e + u_eq;

        % 비선형 시스템 식
        x1_dot = x2;
        x2_dot = -Km * x3^2 / (2 * Mb * x1^2) + g;
        x3_dot = ( - (Rc + Rs) * x3 + u ) / Lc;

        dx = [x1_dot; x2_dot; x3_dot; e_dot];
    else
        % 출력 궤환 선형화
        z1 = x1;
        z2 = x2;
        z3 = -Km * x3^2 / (2 * Mb * x1^2) + g;
        z = [z1; z2; z3];

        % 평형점
        zi = [xb0;0;0];   

        % 선형 제어기 설계
         v  = -Kz * (z - zi) + ki * e;

        % 실제 입력 u 계산 (입력 변환)
        z1 = x1;
        z2 = x2;
        z3 = -Km * x3^2 / (2 * Mb * x1^2) + g;

        alpha = (Km * x3) / (Mb * x1^2) * ( (x2 * x3 / x1) + ((Rc + Rs) * x3 / Lc) );
        beta  = -Km * x3 / (Mb * Lc * x1^2);

        % 실제 제어 입력
        u = (v - alpha - ki * e) / beta;

        % 비선형 시스템 식
        x1_dot = x2;
        x2_dot = -Km * x3^2 / (2 * Mb * x1^2) + g;
        x3_dot = (1 / Lc) * (- (Rc + Rs) * x3 + u);

        dx = [x1_dot; x2_dot; x3_dot; e_dot];
    end
end