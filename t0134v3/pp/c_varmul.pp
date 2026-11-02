POST_NAME = "varmul"
UNITS = "INCHES"
VAR SPINDLE_SPEED = [S|A| S|1.1|0.001]
VAR FEED_RATE = [F|C| F|1.1|0.1]
VAR DWELL_TIME = [DWELL|A| P|1.0|1000]
begin HEADER
"M3[S]"
end
begin TOOLCHANGE
"M5"
"M0"
end
begin SPINDLE_ON
"M3[S]"
end
begin NEW_SEGMENT
"M3[S]"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z][F]"
end
begin CW_ARC_MOVE
"G2[X][Y][Z][I][J][F]"
end
begin CCW_ARC_MOVE
"G3[X][Y][Z][I][J][F]"
end
begin DWELL_MOVE
"G4[DWELL]"
end
begin FOOTER
"M5"
end
