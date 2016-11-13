function SongMexicanHatDance(serPort)
% SongMexicanHatDance(serPort)
% Plays a song
% By Brian Wright, University of Pennsylvania, 2016

fwrite(serPort, [140 1 8 66 12 70 12 0 12 65 12 70 12 0 12 65 12 70 12]);
fwrite(serPort, [140 2 8 65 12 70 12 72 12 70 12 69 12 0 12 70 12 72 12]);
fwrite(serPort, [140 3 8 65 12 69 12 0 12 65 12 69 12 0 12 65 12 69 12]);
fwrite(serPort, [140 4 8 65 12 69 12 70 12 69 12 67 12 0 12 69 12 70 12]);
fwrite(serPort, [141 1]);
