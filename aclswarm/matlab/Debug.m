% Plot of current positions
figure;
hold on
for i = 1 : n
    scatter(q(2*i-1), q(2*i),50);
    text(q(2*i-1), q(2*i), num2str(i))
end
hold off
axis equal
title('Current positions')



%%

% Plot of formation points
figure;
hold on
for i = 1 : n
    scatter(qs(1,i), qs(2,i),50);
    text(qs(1,i), qs(2,i), num2str(i))
end
hold off
axis equal
title('Before permutation');




%%

qs1 = qs0 * P;

% Plot of formation points
figure;
hold on
for i = 1 : n
    scatter(qs1(1,i), qs1(2,i),50);
    text(qs1(1,i), qs1(2,i), num2str(i))
end
hold off
axis equal
title('After permutation');




%%

% Plot of formation points

figure;
hold on
for i = 1 : n
    scatter(qs0(1,i), qs0(2,i),50);
    text(qs0(1,i), qs0(2,i), num2str(i))
end
hold off
axis equal

