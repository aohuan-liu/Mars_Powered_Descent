 %% 正弦曲线测试
 % 打开 MATLAB，运行这个文件
 % 如果能看到一条正弦曲线从 0 画到 10 秒，说明你的 MATLAB 环境没问题
 
 t = 0:0.01:10;
 y = sin(2*pi*t);
 
 figure;
 plot(t, y, 'b-', 'LineWidth', 2);
 grid on;
 xlabel('Time (s)');
 ylabel('Amplitude');
 title('y = sin(2\pi t)');
 ylim([-1.5, 1.5]);
 
 disp('如果看到这条消息，说明脚本跑通了');
