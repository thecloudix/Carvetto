POST_NAME = "invinchmul"
UNITS = "INCHES"
INVERSE_TIME_MODE = "YES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.3|0.01]
begin HEADER
"M3[S]"
end
begin FEED_RATE_CHANGE
"(fr)G94 F[F]"
end
begin RAPID_MOVE
"G94 G0[X][Y][Z][F]"
end
begin FEED_MOVE
"G93 G1[X][Y][Z][F]"
end
begin DWELL_MOVE
"G4 P1[F]"
end
begin FOOTER
"M5"
end
