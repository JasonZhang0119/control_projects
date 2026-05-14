clear; clc;

port = "COM7";
baudrate = 115200;

sp = serialport(port, baudrate);

setDTR(sp, true);
setRTS(sp, false);

configureTerminator(sp, "LF");
sp.Timeout = 3;
flush(sp);

disp("Now press NRST on the board, then press any key in MATLAB.");
pause;

flush(sp);

for k = 1:200
    line = readline(sp);
    fprintf("[%03d] %s\n", k, strtrim(line));
end

clear sp;