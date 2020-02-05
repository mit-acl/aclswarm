%% Pentagonal pyramid

% Pentagon
n = 5;
qAng = linspace(0,360,n+1);        % Desired locations of agents on the unit circle given in angle
qAng(end) = [];
pen = [cosd(qAng); sind(qAng); zeros(1,n)] * 2;

% Add base
Qs = [[0;0;1.7], pen];


figure;
hold on 
for i = 1 : 6
    scatter3(Qs(1,i),Qs(2,i),Qs(3,i), 'fill');
end
axis equal
grid on
view([33,18])



%% Triangular prism


% Equilateral triangle
n = 3;
qAng = linspace(-30,360-30,n+1);        % Desired locations of agents on the unit circle given in angle
qAng(end) = [];
tri = [cosd(qAng); sind(qAng); zeros(1,n)] * 1;

Qs1 = [tri(1,:); zeros(1,3); tri(2,:)];
Qs2 = [tri(1,:); 1.5*ones(1,3); tri(2,:)];

Qs = [Qs1, Qs2];

figure;
hold on 
for i = 1 : 6
    scatter3(Qs(1,i),Qs(2,i),Qs(3,i), 'fill');
end
axis equal
grid on
view([33,18])



%% Slanted plane


Qs = [0  1   2  0 1   2;
      0  0   0  1 1   1;
      0  0.5 1  0 0.5 1];

  
figure;
hold on 
for i = 1 : 6
    scatter3(Qs(1,i),Qs(2,i),Qs(3,i), 'fill');
end
axis equal
grid on
view([-46,20])