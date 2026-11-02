POST_NAME = "invfiall"
UNITS = "MM"
INVERSE_TIME_MODE = "YES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.1]
VAR INV = [FI|A| Q|1.2|10]
begin HEADER
"M3[S][FI][F]"
end
begin RAPID_MOVE
"G0[X][Y][Z][FI]"
end
begin FEED_MOVE
"G93 G1[X][Y][Z][FI]"
end
begin CW_ARC_MOVE
"G93 G2[X][Y][Z][I][J][F][FI]"
end
begin CCW_ARC_MOVE
"G93 G3[X][Y][Z][I][J][F][FI]"
end
begin DWELL_MOVE
"G4 P1[FI][F]"
end
begin FOOTER
"M5[FI]"
end
