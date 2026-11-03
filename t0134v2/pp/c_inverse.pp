POST_NAME = "inv"
UNITS = "MM"
INVERSE_TIME_MODE = "YES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.1]
VAR DWELL_TIME = [DWELL|A| P|1.2]
begin HEADER
"G90 G21"
"M3[S]"
end
begin TOOLCHANGE
"M5"
"T[T] M6"
end
begin SPINDLE_ON
"M3[S]"
end
begin NEW_SEGMENT
"M3[S]"
end
begin RAPID_MOVE
"G94 G0[X][Y][Z]"
end
begin FEED_MOVE
"G93 G1[X][Y][Z][F]"
end
begin CW_ARC_MOVE
"G93 G2[X][Y][Z][I][J][F]"
end
begin CCW_ARC_MOVE
"G93 G3[X][Y][Z][I][J][F]"
end
begin DWELL_MOVE
"G4[DWELL]"
end
begin FOOTER
"M5"
end
