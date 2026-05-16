clear; clc;

sp = serialport("COM5", 115200);
configureTerminator(sp, "CR/LF");
flush(sp);

for k = 1:20
    line = readline(sp);
    disp(line);
end

clear sp;