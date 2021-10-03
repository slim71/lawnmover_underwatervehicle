%% plots init
pos = zeros(size(out.pos.signals.values,1), size(out.pos.signals.values,3));
for i = 1:size(out.pos.signals.values,3)
	pos(:,i) = out.pos.signals.values(:,1,i);
end
pos_true = zeros(size(out.pos_true.signals.values,1), size(out.pos_true.signals.values,3));
for i = 1:size(out.pos_true.signals.values,3)
	pos_true(:,i) = out.pos_true.signals.values(:,1,i);
end

GPS = [];
for i = 1:size(out.GPS.signals.values,3)
	if norm(out.GPS.signals.values(:,1,i),2) < 1e5
		GPS = [GPS, out.GPS.signals.values(:,1,i)];
	end
end
USBL = zeros(size(out.USBL.signals.values,2), size(out.pos.signals.values,1));
for i = 1:size(out.USBL.signals.values,1)
	USBL(:,i) = out.USBL.signals.values(i,:);
end

cov = out.poscov.signals.values;

%% covariance plot + real meas
figure(1)
clf

hold on
for i = 1:2:min(size(out.poscov.signals.values,3),100)
	plotEllipses(pos([2,1],i)', 3*[sqrt(cov(2,2,i)), sqrt(cov(1,1,i))]);
end
plot(pos(2,:), pos(1,:), 'b')
plot(pos_true(2,:),pos_true(1,:), ':r')
plot(GPS(2,:), GPS(1,:), 'g*')
plot(USBL(2,:), USBL(1,:), 'r*')
plot(pos_transp(2),pos_transp(1),'k.', 'MarkerSize',30)
plot(pos(2,:), pos(1,:), 'b')
plot(pos_true(2,:),pos_true(1,:), 'r')
title('covariance plot + real meas')
grid on
xlabel('Est [m]')
ylabel('Nord [m]')
legend('Posizione Stimata','Posizione Vera','GPS','USBL','Posizione Transponder');
axis equal
