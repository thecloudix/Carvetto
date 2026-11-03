POST_NAME = "manual"
UNITS = "MM"
VAR FEED_RATE = [F|C| F|1.0]
VAR DWELL_TIME = [DWELL|A| P|1.2]
begin HEADER
"(start spindle by hand)"
end
begin TOOLCHANGE
"M0 (change tool [T])"
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
"M2"
end
